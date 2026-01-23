#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>
#include <hal/nrf_gpio.h>
#include "spi_bus_share.h"
#include "spi_bus_share_internal.h"
#include "spi_bus_share_config.h"

LOG_MODULE_DECLARE(spi_bus_share, LOG_LEVEL_INF);


extern const struct device *gpio_dev2;
extern const struct device *flash_dev;
extern enum spi_bus_owner current_owner;
extern struct k_mutex bus_mutex;

#ifndef THIS_IS_NRF5340
extern const struct device *spi_dev;
#endif

#if defined(ENABLE_CS_MIRROR) && !defined(THIS_IS_NRF5340)
extern struct gpio_callback cs_mirror_cb_data;
extern bool cs_mirror_enabled;
#endif




int spi_bus_tristate(void)
{
	int ret;

	k_mutex_lock(&bus_mutex, K_FOREVER);

	if (current_owner == SPI_BUS_OWNER_NONE) {
		LOG_WRN("%s bus already tristated", SPI_BUS_TYPE);
		k_mutex_unlock(&bus_mutex);
		return 0;
	}
	LOG_INF("Tristating %s bus pins", SPI_BUS_TYPE);

	#ifndef THIS_IS_BOOTLOADER

	#ifndef THIS_IS_NRF5340
	if (spi_dev && device_is_ready(spi_dev)) {
		LOG_INF("Suspending SPI3 device to disable peripheral...");
		ret = pm_device_action_run(spi_dev, PM_DEVICE_ACTION_SUSPEND);
		if (ret < 0) {
			LOG_ERR("Failed to suspend SPI3 device: %d", ret);
		} else {
			LOG_INF("SPI3 device suspended and disabled");
		}
		k_msleep(10);
	}
	#endif

	if (flash_dev && device_is_ready(flash_dev)) {
		LOG_INF("Suspending flash device to release pins...");
		ret = pm_device_action_run(flash_dev, PM_DEVICE_ACTION_SUSPEND);
		if (ret < 0) {
			LOG_ERR("Failed to suspend flash device: %d", ret);
		}
		LOG_INF("Flash device suspended, peripheral disabled");
		k_msleep(10);
	} else {
		LOG_INF("Flash device not available - configuring pins directly (MCUboot mode)");
	}

	#endif


	#ifdef THIS_IS_NRF9151
	nrf_gpio_cfg_default(SPI_CS_PIN);
	#

	ret = gpio_pin_configure(gpio_dev2, SPI_CS_PIN, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to tristate CS pin: %d", ret);
	}

	#endif

	current_owner = SPI_BUS_OWNER_NONE;
	LOG_INF("%s bus successfully tristated", SPI_BUS_TYPE);

	k_mutex_unlock(&bus_mutex);
	return 0;
}

int spi_bus_reclaim(void)
{

	int ret = 0;
	k_mutex_lock(&bus_mutex, K_FOREVER);

	if (current_owner == SPI_BUS_OWNER_THIS_MCU) {
		LOG_WRN("%s bus already owned by this MCU", SPI_BUS_TYPE);
		k_mutex_unlock(&bus_mutex);
		return 0;
	}

	LOG_INF("Reclaiming %s bus pins", SPI_BUS_TYPE);


	#ifndef THIS_IS_BOOTLOADER
	if (flash_dev && device_is_ready(flash_dev)) {
		LOG_INF("Resuming flash device to reclaim pins...");
		ret = pm_device_action_run(flash_dev, PM_DEVICE_ACTION_RESUME);
		if (ret < 0) {
			LOG_ERR("Failed to resume flash device: %d", ret);
		}
		LOG_INF("Flash device resumed, peripheral re-enabled and reinitialized");
		k_msleep(10);
	} else {
		LOG_INF("Flash device not available - configuring pins directly (MCUboot mode)");
	}

	#ifndef THIS_IS_NRF5340
	if (spi_dev && device_is_ready(spi_dev)) {
		LOG_INF("Resuming SPI3 device to enable peripheral...");
		ret = pm_device_action_run(spi_dev, PM_DEVICE_ACTION_RESUME);
		if (ret < 0) {
			LOG_ERR("Failed to resume SPI3 device: %d", ret);
		} else {
			LOG_INF("SPI3 device resumed and enabled");
		}
		k_msleep(10);
	}
	#endif

	#endif

	#ifdef THIS_IS_NRF9151	
	nrf_gpio_cfg_default(SPI_CS_PIN);


	ret = gpio_pin_configure(gpio_dev2, SPI_CS_PIN, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
	if (ret < 0) {
		LOG_ERR("Failed to configure CS pin: %d", ret);
	}

	
	LOG_INF("Configured CS pins as OUTPUT and set HIGH (inactive)");
	#endif
	current_owner = SPI_BUS_OWNER_THIS_MCU;
	LOG_INF("%s bus successfully reclaimed", SPI_BUS_TYPE);

	k_mutex_unlock(&bus_mutex);
	return ret;
}

/**
 * @brief Configure both REQUEST and GRANT pins as INPUT for idle state
 * Used when this MCU is idle and needs to listen for both REQUEST and GRANT signals (FSM_IDLE_NOT_OWNER)
 */
int configure_pins_as_idle(void)
{
	int ret;

	gpio_pin_interrupt_configure(gpio_dev2, BUS_GRANT_PIN, GPIO_INT_DISABLE);
	gpio_pin_interrupt_configure(gpio_dev2, BUS_REQUEST_PIN, GPIO_INT_DISABLE);

	ret = gpio_pin_configure(gpio_dev2, BUS_REQUEST_PIN, GPIO_INPUT | GPIO_PULL_DOWN);
	if (ret < 0) {
		LOG_ERR("Failed to configure REQUEST as input: %d", ret);
		return ret;
	}

	ret = gpio_pin_configure(gpio_dev2, BUS_GRANT_PIN, GPIO_INPUT | GPIO_PULL_DOWN);
	if (ret < 0) {
		LOG_ERR("Failed to configure GRANT as input: %d", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure(gpio_dev2, BUS_GRANT_PIN, GPIO_INT_EDGE_BOTH);
	if (ret < 0) {
		LOG_ERR("Failed to enable GRANT interrupt: %d", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure(gpio_dev2, BUS_REQUEST_PIN, GPIO_INT_EDGE_BOTH);
	if (ret < 0) {
		LOG_ERR("Failed to enable REQUEST interrupt: %d", ret);
		return ret;
	}

	LOG_DBG("Pins configured: REQUEST=INPUT, GRANT=INPUT (idle mode - listening for both)");
	return 0;
}

/**
 * @brief Configure REQUEST pin as OUTPUT and GRANT pin as INPUT
 * Used when this MCU is actively requesting the bus (FSM_REQUESTING)
 */
int configure_pins_as_requester(void)
{
	int ret;

	gpio_pin_interrupt_configure(gpio_dev2, BUS_GRANT_PIN, GPIO_INT_DISABLE);
	gpio_pin_interrupt_configure(gpio_dev2, BUS_REQUEST_PIN, GPIO_INT_DISABLE);

	ret = gpio_pin_configure(gpio_dev2, BUS_REQUEST_PIN, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure REQUEST as output: %d", ret);
		return ret;
	}

	ret = gpio_pin_configure(gpio_dev2, BUS_GRANT_PIN, GPIO_INPUT | GPIO_PULL_DOWN);
	if (ret < 0) {
		LOG_ERR("Failed to configure GRANT as input: %d", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure(gpio_dev2, BUS_GRANT_PIN, GPIO_INT_EDGE_BOTH);
	if (ret < 0) {
		LOG_ERR("Failed to enable GRANT interrupt: %d", ret);
		return ret;
	}

	LOG_DBG("Pins configured: REQUEST=OUTPUT, GRANT=INPUT (requester mode)");
	return 0;
}

/**
 * @brief Configure REQUEST pin as INPUT and GRANT pin as OUTPUT
 * Used when this MCU owns the bus and can grant to others (FSM_IDLE_OWNER)
 */
int configure_pins_as_owner(void)
{
	int ret;


	gpio_pin_interrupt_configure(gpio_dev2, BUS_GRANT_PIN, GPIO_INT_DISABLE);
	gpio_pin_interrupt_configure(gpio_dev2, BUS_REQUEST_PIN, GPIO_INT_DISABLE);


	ret = gpio_pin_configure(gpio_dev2, BUS_REQUEST_PIN, GPIO_INPUT | GPIO_PULL_DOWN);
	if (ret < 0) {
		LOG_ERR("Failed to configure REQUEST as input: %d", ret);
		return ret;
	}


	ret = gpio_pin_configure(gpio_dev2, BUS_GRANT_PIN, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure GRANT as output: %d", ret);
		return ret;
	}


	ret = gpio_pin_interrupt_configure(gpio_dev2, BUS_REQUEST_PIN, GPIO_INT_EDGE_BOTH);
	if (ret < 0) {
		LOG_ERR("Failed to enable REQUEST interrupt: %d", ret);
		return ret;
	}

	LOG_DBG("Pins configured: REQUEST=INPUT, GRANT=OUTPUT (owner mode)");
	return 0;
}

/**
 * @brief CS Mirror ISR - mirrors nRF5340's pin 15 to pin 0 (nRF9151 only)
 *
 * When nRF5340 owns the bus, pin 0 mirrors pin 15 (nRF5340's CS output).
 * When nRF9151 owns the bus, pin 0 is controlled by the SPI driver (no mirroring).
 */
#if defined(ENABLE_CS_MIRROR) && !defined(THIS_IS_NRF5340)
void cs_mirror_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);

	if (!gpio_dev2 || !cs_mirror_enabled) {
		return;
	}


	if (!(pins & BIT(CS_MIRROR_NRF5340_SOURCE_PIN))) {
		return;
	}


	int source_state = nrf_gpio_pin_read(CS_MIRROR_NRF5340_SOURCE_PIN);
	nrf_gpio_pin_write(CS_MIRROR_TARGET_PIN, source_state);


	// int actual_state = nrf_gpio_pin_read(CS_MIRROR_TARGET_PIN);

	// LOG_INF("CS Mirror: P0.%d=%d -> P0.%d (wrote=%d, actual=%d)",
	//         CS_MIRROR_NRF5340_SOURCE_PIN, source_state,
	//         CS_MIRROR_TARGET_PIN, source_state, actual_state);
}
#else
void cs_mirror_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
}
#endif

/**
 * @brief Enable CS mirroring from nRF5340 (nRF9151 only, when nRF5340 owns bus)
 *
 * Switches pin 0 to mirror pin 15 (controlled by nRF5340).
 * Pin 0 remains as GPIO output, just changes its mirror source.
 */
#if defined(ENABLE_CS_MIRROR) && !defined(THIS_IS_NRF5340)
int cs_mirror_enable(void)
{
	int ret;

	LOG_INF("Enabling CS mirror from nRF5340: P0.%d -> P0.%d",
	        CS_MIRROR_NRF5340_SOURCE_PIN, CS_MIRROR_TARGET_PIN);

	ret = gpio_pin_configure(gpio_dev2, CS_MIRROR_NRF5340_SOURCE_PIN, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure nRF5340 CS source pin as input: %d", ret);
		return ret;
	}


	NRF_GPIO_Type *gpio_port = NRF_P0;
	gpio_port->PIN_CNF[CS_MIRROR_TARGET_PIN] =
		(GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
		(GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
		(GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
		(GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
		(GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

	LOG_INF("Pin 0 configured as GPIO output via register (PIN_CNF[0]=0x%08x)",
	        gpio_port->PIN_CNF[CS_MIRROR_TARGET_PIN]);

	ret = gpio_pin_interrupt_configure(gpio_dev2, CS_MIRROR_NRF5340_SOURCE_PIN, GPIO_INT_EDGE_BOTH);
	if (ret < 0) {
		LOG_ERR("Failed to enable CS mirror interrupt on nRF5340 source: %d", ret);
		return ret;
	}


	int initial_state = nrf_gpio_pin_read(CS_MIRROR_NRF5340_SOURCE_PIN);
	nrf_gpio_pin_write(CS_MIRROR_TARGET_PIN, initial_state);

	cs_mirror_enabled = true;
	LOG_INF("CS mirror enabled - P0.%d now mirrors nRF5340's P0.%d (initial state=%d)",
	        CS_MIRROR_TARGET_PIN, CS_MIRROR_NRF5340_SOURCE_PIN, initial_state);

	return 0;
}
#else
int cs_mirror_enable(void)
{
	return 0;
}
#endif

/**
 * @brief Disable CS mirroring (nRF9151 only, when nRF9151 takes bus ownership)
 *
 * Returns pin 0 control to the SPI driver so nRF9151 can access flash.
 * Stops mirroring from nRF5340's pin 15.
 */
#if defined(ENABLE_CS_MIRROR) && !defined(THIS_IS_NRF5340)
int cs_mirror_disable(void)
{
	int ret;

	if (!cs_mirror_enabled) {
		return 0;
	}

	LOG_INF("Disabling CS mirror - returning P0.%d to SPI driver control", CS_MIRROR_TARGET_PIN);


	ret = gpio_pin_interrupt_configure(gpio_dev2, CS_MIRROR_NRF5340_SOURCE_PIN, GPIO_INT_DISABLE);
	if (ret < 0) {
		LOG_WRN("Failed to disable CS mirror interrupt on nRF5340 source: %d", ret);
	}


	ret = gpio_pin_configure(gpio_dev2, CS_MIRROR_NRF5340_SOURCE_PIN, GPIO_INPUT);
	if (ret < 0) {
		LOG_WRN("Failed to reconfigure nRF5340 CS source pin: %d", ret);
	}

	cs_mirror_enabled = false;


	nrf_gpio_pin_write(CS_MIRROR_TARGET_PIN, 1);

	LOG_INF("CS mirror disabled - Pin 0 set HIGH, nRF9151 SPI driver can now control it");

	return 0;
}
#else
int cs_mirror_disable(void)
{
	return 0;
}
#endif
