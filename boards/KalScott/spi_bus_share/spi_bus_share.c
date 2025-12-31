#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <nrfx_gpiote.h>
#include <hal/nrf_gpio.h>
#include <nrfx.h>
#include <stdlib.h>
#include "spi_bus_share.h"
#include <zephyr/drivers/flash.h>

// Kconfig options (from CONFIG_SPI_BUS_SHARE_* in Kconfig):
// - CONFIG_SPI_BUS_SHARE_BOOTLOADER -> THIS_IS_BOOTLOADER
// - CONFIG_SPI_BUS_SHARE_SHELL -> SPI_BUS_SHARE_SHELL
// - CONFIG_SPI_BUS_SHARE_NRF5340 -> THIS_IS_NRF5340
// - CONFIG_SPI_BUS_SHARE_NRF9151 -> THIS_IS_NRF9151
// - CONFIG_SPI_BUS_CLAIM_ON_NO_MCU -> SPI_BUS_CLAIM_ON_NO_MCU

// Auto-enable SPI_BUS_CLAIM_ON_NO_MCU for bootloader builds

#ifdef THIS_IS_BOOTLOADER
	#ifndef SPI_BUS_CLAIM_ON_NO_MCU
		#define SPI_BUS_CLAIM_ON_NO_MCU
	#endif
#endif


/*
There are two trains of thought for arbitration:

1) Binary Determistic: There is always a bus owner. The owner can grant the bus to the other MCU when requested.
   The other MCU can only request the bus when it is not owned by the first MCU.
   This requires strict adherence to the protocol to avoid deadlocks.

   This option is more deterministic 
   but ultimatly reduces the ability for more communication patterns.
   i.e. REQUEST and GRANT lines are claimed and used for one purpose only.

2) Tristate Listening: This configures the IPC gpios as inputs when idle. This allows extra communcation
	that specifcally helps in scenarios where one MCU is offline or unresponsive. 
	Specifcally this allows a non-owner to listen to requests and grants even when it does not own the bus.
	This preserves handshakeing even when both FSM's are not in synch.

	This however, has a cost of increased complexity & dual role IPC lines (less relaiable).


	For example the binary determistic will requst bus, and if no response is received it just assert ownership.
	The listener, will give up the bus (if not needed / or locked) and if the other MCU asks for the bus
	it will grant it, even though it has already released it. 
*/

//These macros control which type of arbitration / will replace functionality
// BOTH defined = Binary Deterministic
// #define REPLACE_IDLE_WITH_TRISTATE
// #define SPI_BUS_CLAIM_ON_NO_MCU
LOG_MODULE_REGISTER(spi_bus_share, LOG_LEVEL_INF);

#ifdef THIS_IS_NRF5340
	/* nRF5340 configuration */
	#define SPI_SCK_PIN   17
	#define SPI_MOSI_PIN  13
	#define SPI_MISO_PIN  14
	#define SPI_CS_PIN    18

	#define SPI_CS1_PIN   27
	#define SPI_CS2_PIN   15

	#define BUS_REQUEST_PIN  7
	#define BUS_GRANT_PIN    8

	#define INITIAL_FSM_STATE     FSM_IDLE_OWNER
	#define INITIAL_OWNER         SPI_BUS_OWNER_THIS_MCU
	#define SPI_BUS_TYPE          "QSPI"
#else
	/* nRF9151 configuration  */
	#define SPI_SCK_PIN   2
	#define SPI_MOSI_PIN  3
	#define SPI_MISO_PIN  1
	#define SPI_CS_PIN    0

	#define SPI_CS1_PIN   27
	#define SPI_CS2_PIN   15

	#define BUS_REQUEST_PIN  18
	#define BUS_GRANT_PIN    19

	#define INITIAL_FSM_STATE     FSM_IDLE_NOT_OWNER
	#define INITIAL_OWNER         SPI_BUS_OWNER_OTHER_MCU
	#define SPI_BUS_TYPE          "SPI"
	static const struct device *spi_dev;
#endif  


#define BUS_REQUEST_TIMEOUT_MS     120000 
#define BUS_QUICK_GRANT_TIMEOUT_MS 500  
#define BUS_MISSING_MCU_TIMEOUT_MS 2000  
#define BUS_LEASE_TIME_MS          5000  
#define BUS_MAX_HOG_TIME_MS        30000
#define BUS_RELEASE_TIMEOUT_MS     500


enum bus_fsm_state {
	FSM_STARTUP,        
	FSM_IDLE_OWNER,
	FSM_IDLE_NOT_OWNER,
	FSM_REQUESTING,
	FSM_RELEASING,
	FSM_ERROR
};


enum bus_fsm_event {
	EVENT_STARTUP_DISCOVERY,  /* Initial boot - discover bus ownership */
	EVENT_REQUEST_BUS,        /* This MCU wants to request the bus */
	EVENT_RELEASE_BUS,        /* This MCU wants to release the bus */
	EVENT_GRANT_RECEIVED,     /* Other MCU ready for handover (GRANT falling edge HIGH→LOW) */
	EVENT_GRANT_LOST,         /* [DEPRECATED] Not used in level-based protocol */
	EVENT_REQUEST_RECEIVED,   /* Other MCU is requesting the bus (REQUEST pin went HIGH) */
	EVENT_REQUEST_RELEASED,   /* Other MCU released its request (REQUEST pin went LOW) */
	EVENT_TIMEOUT,            /* Timeout waiting for response */
	EVENT_ERROR_RECOVERY      /* Recover from error state */
};

/* FSM Context */
struct bus_fsm_context {
	enum bus_fsm_state state;
	struct k_mutex mutex;
	struct k_sem event_sem;
	enum bus_fsm_event pending_event;
	struct k_work_delayable timeout_work;
	struct k_work_delayable deferred_grant_work;
	bool grant_line_state;
	bool request_line_state;
	uint32_t error_count;
	uint32_t transition_count;
	int64_t bus_acquired_time;
	int64_t bus_lease_expires;
	bool transaction_in_progress;
	bool other_mcu_detected;
};

static const struct device *gpio_dev2;

static const struct device *flash_dev;
static enum spi_bus_owner current_owner = SPI_BUS_OWNER_THIS_MCU;
static struct k_mutex bus_mutex;
static struct k_work_delayable init_work;
static struct bus_fsm_context fsm_ctx;
static struct gpio_callback grant_cb_data;
static struct gpio_callback request_cb_data;
static bool fsm_initialized = false;

static int configure_pins_as_requester(void);
static int configure_pins_as_owner(void);
static int configure_pins_as_idle(void);
static int configure_pin_tristate(uint32_t pin);
static int configure_cs_tristate(void);
static int configure_pins_as_tristate(void);



/**
 * @brief Get string name for FSM state (for logging)
 */
static const char *fsm_state_to_string(enum bus_fsm_state state)
{
	switch (state) {
	case FSM_STARTUP:        return "STARTUP";
	case FSM_IDLE_OWNER:     return "IDLE_OWNER";
	case FSM_IDLE_NOT_OWNER: return "IDLE_NOT_OWNER";
	case FSM_REQUESTING:     return "REQUESTING";
	case FSM_RELEASING:      return "RELEASING";
	case FSM_ERROR:          return "ERROR";
	default:                 return "UNKNOWN";
	}
}

/**
 * @brief Get string name for FSM event (for logging)
 */
static const char *fsm_event_to_string(enum bus_fsm_event event)
{
	switch (event) {
	case EVENT_STARTUP_DISCOVERY: return "STARTUP_DISCOVERY";
	case EVENT_REQUEST_BUS:       return "REQUEST_BUS";
	case EVENT_RELEASE_BUS:       return "RELEASE_BUS";
	case EVENT_GRANT_RECEIVED:    return "GRANT_RECEIVED";
	case EVENT_GRANT_LOST:        return "GRANT_LOST";
	case EVENT_REQUEST_RECEIVED:  return "REQUEST_RECEIVED";
	case EVENT_REQUEST_RELEASED:  return "REQUEST_RELEASED";
	case EVENT_TIMEOUT:           return "TIMEOUT";
	case EVENT_ERROR_RECOVERY:    return "ERROR_RECOVERY";
	default:                      return "UNKNOWN";
	}
}

/**
 * @brief Timeout work handler - handles FSM timeouts
 */
static void fsm_timeout_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	LOG_WRN("FSM timeout in state %s", fsm_state_to_string(fsm_ctx.state));

	k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);
	fsm_ctx.pending_event = EVENT_TIMEOUT;
	k_mutex_unlock(&fsm_ctx.mutex);

	k_sem_give(&fsm_ctx.event_sem);
}

/**
 * @brief Deferred grant work handler - completes handover after lease expires
 * At this point, GRANT is already HIGH. This handler pulls it LOW to signal handover.
 */
static void deferred_grant_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	LOG_INF("Lease expired, completing deferred bus handover");

	k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

	k_mutex_unlock(&fsm_ctx.mutex);
	int ret = spi_bus_tristate();
	k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

	if (ret == 0) {
		k_msleep(5); 
		gpio_pin_set(gpio_dev2, BUS_GRANT_PIN, 0);
		LOG_INF("GRANT pulled LOW - deferred handover complete");

		k_mutex_unlock(&fsm_ctx.mutex);
		#ifdef REPLACE_IDLE_WITH_TRISTATE
		configure_pins_as_tristate();
		#else
		configure_pins_as_idle();
		#endif
		k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

		fsm_ctx.state = FSM_IDLE_NOT_OWNER;
		current_owner = SPI_BUS_OWNER_OTHER_MCU;
	} else {
		LOG_ERR("Failed to tristate bus during deferred grant");
		gpio_pin_set(gpio_dev2, BUS_GRANT_PIN, 0);
		fsm_ctx.state = FSM_ERROR;
		fsm_ctx.error_count++;
	}

	k_mutex_unlock(&fsm_ctx.mutex);
}

/**
 * @brief GPIO ISR callback for BUS_GRANT pin
 * Called when the other MCU changes the grant signal
 *
 * Level-based protocol (NEW):
 * - GRANT LOW  = Owner owns the bus, not granting
 * - GRANT HIGH (held) = Owner is granting but still finishing up work
 * - Falling edge (HIGH→LOW) = Owner ready for handover, grantee should take NOW
 * - No response (stays LOW) = Either error or no MCU present
 *
 * This allows owner to indefinitely hold while finishing critical operations.
 */
static void bus_grant_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	if (!fsm_initialized || !gpio_dev2) {
		return;
	}

	bool grant_state = gpio_pin_get(gpio_dev2, BUS_GRANT_PIN);

	k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);
	bool previous_state = fsm_ctx.grant_line_state;
	fsm_ctx.grant_line_state = grant_state;

	if (!grant_state && previous_state) {
		fsm_ctx.pending_event = EVENT_GRANT_RECEIVED;
		LOG_DBG("ISR: GRANT_RECEIVED (falling edge HIGH→LOW - handover ready)");
	} else if (grant_state && !previous_state) {
		LOG_DBG("ISR: Grant acknowledged (HIGH) - waiting for handover (falling edge)");
	}

	k_mutex_unlock(&fsm_ctx.mutex);

	if (!grant_state && previous_state) {
		k_sem_give(&fsm_ctx.event_sem);
	}
}

/**
 * @brief GPIO ISR callback for BUS_REQUEST pin
 * Called when the other MCU changes the request signal
 */
static void bus_request_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	if (!fsm_initialized || !gpio_dev2) {
		return;
	}

	bool request_state = gpio_pin_get(gpio_dev2, BUS_REQUEST_PIN);

	k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);
	fsm_ctx.request_line_state = request_state;

	if (request_state) {
		fsm_ctx.pending_event = EVENT_REQUEST_RECEIVED;
		LOG_DBG("ISR: REQUEST_RECEIVED");
	} else {

		fsm_ctx.pending_event = EVENT_REQUEST_RELEASED;
		LOG_DBG("ISR: REQUEST_RELEASED");
	}

	k_mutex_unlock(&fsm_ctx.mutex);


	k_sem_give(&fsm_ctx.event_sem);
}

/**
 * @brief Configure both REQUEST and GRANT pins as INPUT for idle state
 * Used when this MCU is idle and needs to listen for both REQUEST and GRANT signals (FSM_IDLE_NOT_OWNER)
 */
static int configure_pins_as_idle(void)
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
static int configure_pins_as_requester(void)
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
static int configure_pins_as_owner(void)
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
 * @brief Post an event to the FSM
 */
static int fsm_post_event(enum bus_fsm_event event)
{
	if (!fsm_initialized) {
		LOG_WRN("FSM not initialized yet, waiting...");
		/* Wait for FSM to be initialized (max 5 seconds) */
		for (int i = 0; i < 100; i++) {
			k_msleep(50);
			if (fsm_initialized) {
				LOG_INF("FSM ready after %d ms", (i+1)*50);
				break;
			}
		}
		if (!fsm_initialized) {
			LOG_ERR("FSM still not initialized after waiting");
			return -EAGAIN;
		}
	}

	k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);
	fsm_ctx.pending_event = event;
	k_mutex_unlock(&fsm_ctx.mutex);

	k_sem_give(&fsm_ctx.event_sem);

	return 0;
}

enum spi_bus_owner spi_bus_get_owner(void)
{
	enum spi_bus_owner owner;

	k_mutex_lock(&bus_mutex, K_FOREVER);
	owner = current_owner;
	k_mutex_unlock(&bus_mutex);

	return owner;
}

/**
 * @brief Core FSM state machine - processes events and transitions states
 */
static int fsm_process_event(enum bus_fsm_event event)
{
	enum bus_fsm_state old_state;
	int ret = 0;

	if (!fsm_initialized || !gpio_dev2) {
		LOG_ERR("FSM not initialized, cannot process event");
		return -EAGAIN;
	}

	k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

	old_state = fsm_ctx.state;

	LOG_INF("FSM: [%s] + [%s]", fsm_state_to_string(old_state), fsm_event_to_string(event));

	switch (fsm_ctx.state) {
	case FSM_STARTUP:

		if (event == EVENT_STARTUP_DISCOVERY) {

			bool other_grant = gpio_pin_get(gpio_dev2, BUS_GRANT_PIN);
			bool other_request = gpio_pin_get(gpio_dev2, BUS_REQUEST_PIN);

			LOG_INF("Startup: GRANT=%d, REQUEST=%d", other_grant, other_request);

			if (other_grant) {
				LOG_INF("Other MCU offering bus at startup - accepting");
				fsm_ctx.other_mcu_detected = true;

				k_mutex_unlock(&fsm_ctx.mutex);
				ret = spi_bus_reclaim();
				k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

				if (ret == 0) {
					k_mutex_unlock(&fsm_ctx.mutex);
					configure_pins_as_owner();
					k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

					fsm_ctx.state = FSM_IDLE_OWNER;
					current_owner = SPI_BUS_OWNER_THIS_MCU;
				} else {
					fsm_ctx.state = FSM_ERROR;
				}

			} else {

				LOG_INF("No grant at startup - requesting bus with short timeout");

				k_mutex_unlock(&fsm_ctx.mutex);
				configure_pins_as_requester();
				k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

				gpio_pin_set(gpio_dev2, BUS_REQUEST_PIN, 1);
				k_work_schedule(&fsm_ctx.timeout_work,
				                K_MSEC(BUS_MISSING_MCU_TIMEOUT_MS));

				fsm_ctx.state = FSM_REQUESTING;
			}

		} else if (event == EVENT_GRANT_RECEIVED) {
			LOG_INF("Grant received during startup");
			fsm_ctx.other_mcu_detected = true;

			k_work_cancel_delayable(&fsm_ctx.timeout_work);

			k_mutex_unlock(&fsm_ctx.mutex);
			ret = spi_bus_reclaim();
			k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

			if (ret == 0) {
				k_mutex_unlock(&fsm_ctx.mutex);
				configure_pins_as_owner();
				k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

				fsm_ctx.state = FSM_IDLE_OWNER;
				current_owner = SPI_BUS_OWNER_THIS_MCU;
			} else {
				gpio_pin_set(gpio_dev2, BUS_REQUEST_PIN, 0);
				fsm_ctx.state = FSM_ERROR;
			}

		} else if (event == EVENT_TIMEOUT) {
#ifdef SPI_BUS_CLAIM_ON_NO_MCU
			LOG_INF("No response during startup - claiming SPI bus");
			fsm_ctx.other_mcu_detected = false;

			gpio_pin_set(gpio_dev2, BUS_REQUEST_PIN, 0);

			k_mutex_unlock(&fsm_ctx.mutex);
			ret = spi_bus_reclaim();
			k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

			if (ret == 0) {
				k_mutex_unlock(&fsm_ctx.mutex);
				configure_pins_as_owner();
				k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

				fsm_ctx.state = FSM_IDLE_OWNER;
				current_owner = SPI_BUS_OWNER_THIS_MCU;
				LOG_INF("SPI bus claimed successfully");
			} else {
				fsm_ctx.state = FSM_ERROR;
				LOG_ERR("Failed to claim SPI bus");
			}
#else
			LOG_INF("No response during startup - other MCU appears absent");
			LOG_INF("Bus will remain idle until explicitly requested");
			fsm_ctx.other_mcu_detected = false;

			gpio_pin_set(gpio_dev2, BUS_REQUEST_PIN, 0);

			fsm_ctx.state = FSM_IDLE_NOT_OWNER;
			current_owner = SPI_BUS_OWNER_NONE;
#endif
		}
		break;

	case FSM_IDLE_OWNER:
		if (event == EVENT_REQUEST_RECEIVED) {

			fsm_ctx.other_mcu_detected = true;

			bool in_transaction = fsm_ctx.transaction_in_progress;

			if (in_transaction) {
				gpio_pin_set(gpio_dev2, BUS_GRANT_PIN, 1);

				LOG_INF("Bus request received but in transaction lock - GRANT HIGH");
				LOG_INF("Bus will be held indefinitely until spi_bus_release_transaction() is called");


			} else {

				LOG_INF("Other MCU requesting bus - granting access");

				gpio_pin_set(gpio_dev2, BUS_GRANT_PIN, 1);
				LOG_DBG("GRANT set HIGH - preparing handover");

				k_mutex_unlock(&fsm_ctx.mutex);
				ret = spi_bus_tristate();
				k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

				if (ret == 0) {
					k_msleep(5);
					gpio_pin_set(gpio_dev2, BUS_GRANT_PIN, 0);
					LOG_DBG("GRANT pulled LOW - handover complete");

					k_mutex_unlock(&fsm_ctx.mutex);
					#ifdef REPLACE_IDLE_WITH_TRISTATE
					configure_pins_as_tristate();
					#else
					configure_pins_as_idle();
					#endif
					k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

					fsm_ctx.state = FSM_IDLE_NOT_OWNER;
					current_owner = SPI_BUS_OWNER_OTHER_MCU;
				} else {
					LOG_ERR("Failed to tristate bus during grant");
					gpio_pin_set(gpio_dev2, BUS_GRANT_PIN, 0);
					fsm_ctx.state = FSM_ERROR;
				}
			}

		} else if (event == EVENT_RELEASE_BUS) {
			LOG_INF("Voluntarily releasing bus");

			k_mutex_unlock(&fsm_ctx.mutex);
			ret = spi_bus_tristate();
			k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

			if (ret == 0) {
				gpio_pin_set(gpio_dev2, BUS_GRANT_PIN, 0);

				k_mutex_unlock(&fsm_ctx.mutex);
				#ifdef REPLACE_IDLE_WITH_TRISTATE
				configure_pins_as_tristate();
				#else
				configure_pins_as_idle();
				#endif
				k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

				fsm_ctx.state = FSM_IDLE_NOT_OWNER;
				current_owner = SPI_BUS_OWNER_OTHER_MCU;
			} else {
				LOG_ERR("Failed to tristate bus during release");
				fsm_ctx.state = FSM_ERROR;
			}
		}
		break;

	case FSM_IDLE_NOT_OWNER:
		if (event == EVENT_REQUEST_BUS) {
			LOG_INF("Requesting bus ownership");

			k_mutex_unlock(&fsm_ctx.mutex);
			configure_pins_as_requester();
			k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

			fsm_ctx.bus_acquired_time = k_uptime_get();

			gpio_pin_set(gpio_dev2, BUS_REQUEST_PIN, 1);

			k_work_schedule(&fsm_ctx.timeout_work, K_MSEC(BUS_REQUEST_TIMEOUT_MS));

			fsm_ctx.state = FSM_REQUESTING;

		} else if (event == EVENT_GRANT_RECEIVED) {
			LOG_INF("Unsolicited bus grant received - accepting");

			k_mutex_unlock(&fsm_ctx.mutex);
			ret = spi_bus_reclaim();
			k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

			if (ret == 0) {
				k_mutex_unlock(&fsm_ctx.mutex);
				configure_pins_as_owner();
				k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

				fsm_ctx.state = FSM_IDLE_OWNER;
				current_owner = SPI_BUS_OWNER_THIS_MCU;
			} else {
				LOG_ERR("Failed to reclaim bus");
				fsm_ctx.state = FSM_ERROR;
			}

		} else if (event == EVENT_REQUEST_RECEIVED) {

			#ifdef SPI_BUS_CLAIM_ON_NO_MCU
			LOG_INF("Other MCU requesting bus while we don't own it - granting");
			LOG_INF("Sending GRANT signal on pin P0.%d (this MCU never owned bus)", BUS_GRANT_PIN);
			fsm_ctx.other_mcu_detected = true;


			ret = gpio_pin_interrupt_configure(gpio_dev2, BUS_GRANT_PIN, GPIO_INT_DISABLE);
			LOG_DBG("GRANT interrupt disabled (ret=%d)", ret);


			ret = gpio_pin_configure(gpio_dev2, BUS_GRANT_PIN, GPIO_OUTPUT_INACTIVE);
			if (ret != 0) {
				LOG_ERR("Failed to reconfigure GRANT as output: %d", ret);
				fsm_ctx.state = FSM_ERROR;
				break;
			}
			LOG_INF("GRANT pin P0.%d reconfigured as OUTPUT (ret=%d)", BUS_GRANT_PIN, ret);

			ret = gpio_pin_set(gpio_dev2, BUS_GRANT_PIN, 1);
			LOG_INF("GRANT set HIGH - acknowledging we don't own bus (ret=%d)", ret);


			int grant_val = gpio_pin_get(gpio_dev2, BUS_GRANT_PIN);
			LOG_INF("GRANT readback value: %d (should be 1)", grant_val);


			k_msleep(5);
			ret = gpio_pin_set(gpio_dev2, BUS_GRANT_PIN, 0);
			LOG_INF("GRANT pulled LOW - handover complete (falling edge sent) (ret=%d)", ret);


			grant_val = gpio_pin_get(gpio_dev2, BUS_GRANT_PIN);
			LOG_INF("GRANT readback value: %d (should be 0)", grant_val);


			ret = gpio_pin_configure(gpio_dev2, BUS_GRANT_PIN, GPIO_INPUT | GPIO_PULL_DOWN);
			if (ret != 0) {
				LOG_ERR("Failed to reconfigure GRANT as input: %d", ret);
				fsm_ctx.state = FSM_ERROR;
				break;
			}
			LOG_DBG("GRANT pin reconfigured back to INPUT");


			ret = gpio_pin_interrupt_configure(gpio_dev2, BUS_GRANT_PIN, GPIO_INT_EDGE_BOTH);
			if (ret != 0) {
				LOG_ERR("Failed to reconfigure GRANT interrupt: %d", ret);
				fsm_ctx.state = FSM_ERROR;
				break;
			}
			LOG_DBG("GRANT interrupt re-enabled");

			LOG_INF("Discovery complete - other MCU detected and granted bus");
			#else
			LOG_WRN("Other MCU requested bus while we don't own it - ignoring (no claim)");
			#endif
		}
		break;

	case FSM_REQUESTING:

		if (event == EVENT_GRANT_RECEIVED) {

			int64_t response_time = k_uptime_get() - fsm_ctx.bus_acquired_time;
			LOG_INF("Bus grant received (response time: %lld ms)", response_time);

			fsm_ctx.other_mcu_detected = true;

			k_work_cancel_delayable(&fsm_ctx.timeout_work);

			gpio_pin_set(gpio_dev2, BUS_REQUEST_PIN, 0);

			k_mutex_unlock(&fsm_ctx.mutex);
			ret = spi_bus_reclaim();
			k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

			if (ret == 0) {
				k_mutex_unlock(&fsm_ctx.mutex);
				configure_pins_as_owner();
				k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

				fsm_ctx.bus_acquired_time = k_uptime_get();

				fsm_ctx.state = FSM_IDLE_OWNER;
				current_owner = SPI_BUS_OWNER_THIS_MCU;
				if (response_time < BUS_QUICK_GRANT_TIMEOUT_MS) {
					LOG_INF("Other MCU responded quickly (%lld ms) - available",
					        response_time);
				} else {
					LOG_INF("Other MCU responded slowly (%lld ms) - was busy",
					        response_time);
				}
			} else {
				LOG_ERR("Failed to reclaim bus after grant");
				gpio_pin_set(gpio_dev2, BUS_REQUEST_PIN, 0);
				fsm_ctx.state = FSM_ERROR;
			}

		} else if (event == EVENT_TIMEOUT) {
			int64_t wait_time = k_uptime_get() - fsm_ctx.bus_acquired_time;

			if (!fsm_ctx.other_mcu_detected && wait_time >= BUS_MISSING_MCU_TIMEOUT_MS) {
#ifdef SPI_BUS_CLAIM_ON_NO_MCU
				LOG_INF("No grant after %lld ms - claiming SPI bus", wait_time);

				gpio_pin_set(gpio_dev2, BUS_REQUEST_PIN, 0);

				k_mutex_unlock(&fsm_ctx.mutex);
				ret = spi_bus_reclaim();
				k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

				if (ret == 0) {
					k_mutex_unlock(&fsm_ctx.mutex);
					configure_pins_as_owner();
					k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

					fsm_ctx.state = FSM_IDLE_OWNER;
					current_owner = SPI_BUS_OWNER_THIS_MCU;
					LOG_INF("SPI bus claimed successfully");
				} else {
					fsm_ctx.state = FSM_ERROR;
					LOG_ERR("Failed to claim SPI bus");
				}
#else
				LOG_WRN("No grant after %lld ms - other MCU appears absent, remaining idle",
				        wait_time);
				LOG_INF("Bus will remain idle (tristated) until other MCU is detected");

				gpio_pin_set(gpio_dev2, BUS_REQUEST_PIN, 0);
				fsm_ctx.state = FSM_IDLE_NOT_OWNER;
				current_owner = SPI_BUS_OWNER_NONE;
#endif
			} else if (fsm_ctx.other_mcu_detected) {
				LOG_ERR("Timeout waiting for bus grant from detected MCU");
				gpio_pin_set(gpio_dev2, BUS_REQUEST_PIN, 0);
				fsm_ctx.state = FSM_ERROR;
				fsm_ctx.error_count++;
				ret = -ETIMEDOUT;
			} else {
				LOG_WRN("Timeout in REQUESTING state but conditions unclear");
				gpio_pin_set(gpio_dev2, BUS_REQUEST_PIN, 0);
				fsm_ctx.state = FSM_IDLE_NOT_OWNER;
			}

		} else if (event == EVENT_GRANT_LOST || event == EVENT_RELEASE_BUS) {
			LOG_WRN("Bus request cancelled or denied");

			k_work_cancel_delayable(&fsm_ctx.timeout_work);
			gpio_pin_set(gpio_dev2, BUS_REQUEST_PIN, 0);
			fsm_ctx.state = FSM_IDLE_NOT_OWNER;
		}
		break;

	case FSM_RELEASING:
		if (event == EVENT_GRANT_LOST) {
			gpio_pin_set(gpio_dev2, BUS_REQUEST_PIN, 0);
			fsm_ctx.state = FSM_IDLE_NOT_OWNER;
			current_owner = SPI_BUS_OWNER_OTHER_MCU;
		}
		break;

	case FSM_ERROR:
		if (event == EVENT_ERROR_RECOVERY) {
			LOG_INF("Attempting error recovery");

			gpio_pin_set(gpio_dev2, BUS_REQUEST_PIN, 0);
			k_work_cancel_delayable(&fsm_ctx.timeout_work);
			bool grant = gpio_pin_get(gpio_dev2, BUS_GRANT_PIN);

			if (grant) {
				k_mutex_unlock(&fsm_ctx.mutex);
				ret = spi_bus_reclaim();
				k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

				if (ret == 0) {
					fsm_ctx.state = FSM_IDLE_OWNER;
					current_owner = SPI_BUS_OWNER_THIS_MCU;
				}
			} else {
				k_mutex_unlock(&fsm_ctx.mutex);
				ret = spi_bus_tristate();
				k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

				fsm_ctx.state = FSM_IDLE_NOT_OWNER;
				current_owner = SPI_BUS_OWNER_OTHER_MCU;
			}
		}
		break;

	default:
		LOG_ERR("Unknown FSM state: %d", fsm_ctx.state);
		ret = -EINVAL;
		break;
	}

	if (fsm_ctx.state != old_state) {
		fsm_ctx.transition_count++;
		LOG_INF("FSM: [%s] -> [%s]", fsm_state_to_string(old_state),
		        fsm_state_to_string(fsm_ctx.state));
	}

	k_mutex_unlock(&fsm_ctx.mutex);

	return ret;
}

/**
 * @brief FSM processing thread - waits for events and processes them
 */
static void fsm_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	LOG_INF("FSM thread started, waiting for initialization...");
	while (!fsm_initialized) {
		k_sleep(K_MSEC(100));
	}

	LOG_INF("FSM thread ready to process events");

	while (1) {

		k_sem_take(&fsm_ctx.event_sem, K_FOREVER);

		enum bus_fsm_event event;

		k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);
		event = fsm_ctx.pending_event;
		k_mutex_unlock(&fsm_ctx.mutex);

		fsm_process_event(event);
	}
}

K_THREAD_DEFINE(fsm_thread_id, 2048, fsm_thread, NULL, NULL, NULL, 5, 0, 0);

static void spi_bus_share_init_work_handler(struct k_work *work)
{
	int ret;

	ARG_UNUSED(work);

	LOG_INF("Initializing SPI bus sharing module");

	gpio_dev2 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	if (!device_is_ready(gpio_dev2)) {
		LOG_ERR("GPIO device not ready");
		return;
	}

	#ifndef THIS_IS_NRF5340
	spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi3));
	if (!device_is_ready(spi_dev)) {
		LOG_WRN("SPI device not ready");
	}
	#endif

	flash_dev = DEVICE_DT_GET(DT_NODELABEL(flash_ext));
	if (!device_is_ready(flash_dev)) {
		LOG_WRN("Flash device not ready - will skip power management in MCUboot");
		flash_dev = NULL;  
	} else {
		LOG_INF("Flash device ready");
	}

	fsm_ctx.state = FSM_STARTUP;
	fsm_ctx.grant_line_state = false;
	fsm_ctx.request_line_state = false;
	fsm_ctx.error_count = 0;
	fsm_ctx.transition_count = 0;
	fsm_ctx.bus_acquired_time = 0;
	fsm_ctx.bus_lease_expires = 0;
	fsm_ctx.transaction_in_progress = false;
	fsm_ctx.other_mcu_detected = false;

	gpio_init_callback(&grant_cb_data, bus_grant_isr, BIT(BUS_GRANT_PIN));
	gpio_init_callback(&request_cb_data, bus_request_isr, BIT(BUS_REQUEST_PIN));

	ret = gpio_add_callback(gpio_dev2, &grant_cb_data);
	if (ret < 0) {
		LOG_ERR("Failed to add GRANT callback: %d", ret);
		return;
	}

	ret = gpio_add_callback(gpio_dev2, &request_cb_data);
	if (ret < 0) {
		LOG_ERR("Failed to add REQUEST callback: %d", ret);
		return;
	}

	LOG_INF("Starting in STARTUP state - configuring SPI pins");

	spi_bus_tristate();

	#ifdef REPLACE_IDLE_WITH_TRISTATE
	configure_pins_as_tristate();
	#else
	configure_pins_as_idle();
	#endif


	current_owner = SPI_BUS_OWNER_NONE;

	fsm_initialized = true;

	LOG_INF("=== %s Bus Sharing Module Initialized ===", SPI_BUS_TYPE);
#ifdef THIS_IS_NRF5340
	LOG_INF("Build: nRF5340 (QSPI)");
#else
	LOG_INF("Build: nRF9151 (SPI)");
#endif
	LOG_INF("%s pins - SCK: P0.%d, MOSI: P0.%d, MISO: P0.%d",
	        SPI_BUS_TYPE, SPI_SCK_PIN, SPI_MOSI_PIN, SPI_MISO_PIN);
	LOG_INF("CS pins - CS0: P0.%d, CS1: P0.%d, CS2: P0.%d",
	        SPI_CS_PIN, SPI_CS1_PIN, SPI_CS2_PIN);
	LOG_INF("Control pins - REQUEST: P0.%d, GRANT: P0.%d",
	        BUS_REQUEST_PIN, BUS_GRANT_PIN);
	LOG_INF("FSM initialized in state: %s", fsm_state_to_string(fsm_ctx.state));
	LOG_INF("Starting auto-discovery...");

	k_sleep(K_MSEC(100));
	fsm_post_event(EVENT_STARTUP_DISCOVERY);
}

int spi_bus_share_init(void)
{
	k_mutex_init(&fsm_ctx.mutex);
	k_sem_init(&fsm_ctx.event_sem, 0, 1);
	k_work_init_delayable(&fsm_ctx.timeout_work, fsm_timeout_handler);
	k_work_init_delayable(&fsm_ctx.deferred_grant_work, deferred_grant_handler);

	k_mutex_init(&bus_mutex);

	k_work_init_delayable(&init_work, spi_bus_share_init_work_handler);

	k_work_schedule(&init_work, K_NO_WAIT);

	LOG_INF("SPI bus sharing module will initialize.");

	return 0;
}

#ifndef THIS_IS_BOOTLOADER
SYS_INIT(spi_bus_share_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif
/**
 * @brief Request bus ownership via FSM
 */
int spi_bus_request(void)
{
	LOG_INF("API: Requesting bus ownership");

	return fsm_post_event(EVENT_REQUEST_BUS);
}

/**
 * @brief Release bus ownership via FSM
 */
int spi_bus_release(void)
{
	LOG_INF("API: Releasing bus ownership");

	return fsm_post_event(EVENT_RELEASE_BUS);
}

/**
 * @brief Recover from FSM error state
 */
int spi_bus_error_recovery(void)
{
	LOG_INF("API: Initiating error recovery");

	return fsm_post_event(EVENT_ERROR_RECOVERY);
}


/**
 * @brief Cleanup/destructor - shutdown FSM thread and release GPIO resources
 *
 * This function is used during bootloader→application handoff on nRF5340.
 * It cleanly shuts down the FSM thread, removes ISR callbacks, and resets
 * GPIO pins to INPUT state. This releases SECURE mode hardware control
 * so the NON-SECURE application can reconfigure the pins.
 *
 * @return 0 on success, negative errno code on failure
 */
int spi_bus_shutdown(void)
{
	LOG_INF("Shutting down SPI bus share module...");

	if (!fsm_initialized || !gpio_dev2) {
		LOG_WRN("Module not initialized, nothing to shut down");
		return 0;
	}

	/* Stop the FSM thread by aborting it */
	k_thread_abort(fsm_thread_id);
	LOG_INF("  FSM thread aborted");

	/* Remove GPIO interrupt callbacks */
	int ret = gpio_remove_callback(gpio_dev2, &grant_cb_data);
	if (ret < 0) {
		LOG_WRN("Failed to remove GRANT callback: %d", ret);
	}

	ret = gpio_remove_callback(gpio_dev2, &request_cb_data);
	if (ret < 0) {
		LOG_WRN("Failed to remove REQUEST callback: %d", ret);
	}
	LOG_INF("  GPIO callbacks removed");

	/* Disable interrupts on both pins */
	gpio_pin_interrupt_configure(gpio_dev2, BUS_REQUEST_PIN, GPIO_INT_DISABLE);
	gpio_pin_interrupt_configure(gpio_dev2, BUS_GRANT_PIN, GPIO_INT_DISABLE);
	LOG_INF("  GPIO interrupts disabled");

	/* Reset both pins to INPUT (tristate) to release hardware control */
	#if defined(THIS_IS_NRF5340) && !defined(THIS_IS_BOOTLOADER)
	/* Use direct register writes for nRF5340 application */
	NRF_GPIO_Type *gpio = NRF_P0_NS;
	gpio->PIN_CNF[BUS_REQUEST_PIN] = 0;
	gpio->OUT &= ~(1U << BUS_REQUEST_PIN);
	gpio->PIN_CNF[BUS_GRANT_PIN] = 0;
	gpio->OUT &= ~(1U << BUS_GRANT_PIN);
	LOG_INF("  Pins P0.%d and P0.%d reset via direct HW access", BUS_REQUEST_PIN, BUS_GRANT_PIN);
	#else
	/* Use Zephyr API for other platforms */
	gpio_pin_configure(gpio_dev2, BUS_REQUEST_PIN, GPIO_INPUT);
	gpio_pin_configure(gpio_dev2, BUS_GRANT_PIN, GPIO_INPUT);
	LOG_INF("  Pins P0.%d and P0.%d reset via Zephyr API", BUS_REQUEST_PIN, BUS_GRANT_PIN);
	#endif

	/* Cancel any pending work items */
	k_work_cancel_delayable(&fsm_ctx.timeout_work);
	k_work_cancel_delayable(&fsm_ctx.deferred_grant_work);
	LOG_INF("  Pending work items cancelled");

	/* Mark as uninitialized */
	fsm_initialized = false;

	LOG_INF("SPI bus share shutdown complete - HW resources released");
	return 0;
}

/**
 * @brief Get current FSM state
 */
int spi_bus_get_fsm_state(void)
{
	int state;

	k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);
	state = (int)fsm_ctx.state;
	k_mutex_unlock(&fsm_ctx.mutex);

	return state;
}

/**
 * @brief Get FSM statistics
 */
int spi_bus_get_fsm_stats(uint32_t *error_count, uint32_t *transition_count)
{
	k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

	if (error_count) {
		*error_count = fsm_ctx.error_count;
	}

	if (transition_count) {
		*transition_count = fsm_ctx.transition_count;
	}

	k_mutex_unlock(&fsm_ctx.mutex);

	return 0;
}

/**
 * @brief Acquire bus for transaction (blocking)
 *
 * Blocks indefinitely until bus is acquired.
 * use_transaction_lock controls whether bus yields immediately or holds indefinitely.
 */
int spi_bus_acquire_blocking(bool use_transaction_lock)
{
	int ret;
	enum bus_fsm_state state;

	LOG_INF("API: Acquiring bus (transaction_lock=%d)", use_transaction_lock);

	state = (enum bus_fsm_state)spi_bus_get_fsm_state();
	if (state == FSM_IDLE_OWNER) {
		LOG_INF("Already own the bus");

		if (use_transaction_lock) {
			k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);
			fsm_ctx.transaction_in_progress = true;
			fsm_ctx.bus_lease_expires = INT64_MAX; 
			k_mutex_unlock(&fsm_ctx.mutex);
		} else {
			k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);
			fsm_ctx.transaction_in_progress = false;
			fsm_ctx.bus_lease_expires = k_uptime_get(); 
			k_mutex_unlock(&fsm_ctx.mutex);
		}

		return 0;
	}

	ret = spi_bus_request();
	if (ret < 0) {
		return ret;
	}

	while (1) {
		k_sleep(K_MSEC(50));

		state = (enum bus_fsm_state)spi_bus_get_fsm_state();

		if (state == FSM_IDLE_OWNER) {
			LOG_INF("Bus acquired successfully");

			if (use_transaction_lock) {
				k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);
				fsm_ctx.transaction_in_progress = true;
				fsm_ctx.bus_lease_expires = INT64_MAX; 
				k_mutex_unlock(&fsm_ctx.mutex);
				LOG_INF("Transaction lock active - bus held indefinitely until release");
			} else {
	
				k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);
				fsm_ctx.transaction_in_progress = false;
				fsm_ctx.bus_lease_expires = k_uptime_get(); 
				k_mutex_unlock(&fsm_ctx.mutex);
				LOG_INF("Quick mode - bus will yield immediately on request");
			}

			return 0;

		} else if (state == FSM_ERROR) {
			LOG_ERR("Bus acquisition failed - FSM in error state");
			return -EIO;
		}

	}
}

/**
 * @brief Release bus after transaction complete
 *
 * If GRANT is HIGH (other MCU waiting), this will trigger immediate handover.
 */
int spi_bus_release_transaction(void)
{
	int ret = 0;
	bool grant_high;

	LOG_INF("API: Releasing transaction lock");

	k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);
	fsm_ctx.transaction_in_progress = false;
	fsm_ctx.bus_lease_expires = k_uptime_get(); 

	grant_high = (gpio_pin_get(gpio_dev2, BUS_GRANT_PIN) == 1);
	k_mutex_unlock(&fsm_ctx.mutex);

	if (grant_high) {

		LOG_INF("Other MCU waiting - completing handover");

		ret = spi_bus_tristate();
		if (ret == 0) {
	
			k_msleep(5); 
			gpio_pin_set(gpio_dev2, BUS_GRANT_PIN, 0);
			LOG_INF("GRANT pulled LOW - handover complete");


			configure_pins_as_idle();

			k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);
			fsm_ctx.state = FSM_IDLE_NOT_OWNER;
			k_mutex_unlock(&fsm_ctx.mutex);

			current_owner = SPI_BUS_OWNER_OTHER_MCU;
		} else {
			LOG_ERR("Failed to tristate bus during transaction release");
			gpio_pin_set(gpio_dev2, BUS_GRANT_PIN, 0);

			k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);
			fsm_ctx.state = FSM_ERROR;
			fsm_ctx.error_count++;
			k_mutex_unlock(&fsm_ctx.mutex);

			return ret;
		}
	} else {
		LOG_INF("No MCU waiting - transaction lock released, keeping bus ownership");
	}

	return 0;
}


/*
======================================================================================
======================================================================================
======================================================================================
======================================================================================
PIN & DEVICE CONFIGURATION
======================================================================================
======================================================================================
======================================================================================
*/

static int configure_pin_output(uint32_t pin)
{
	int ret;

	nrf_gpio_cfg_default(pin);

	ret = gpio_pin_configure(gpio_dev2, pin,  GPIO_OUTPUT | GPIO_ACTIVE_LOW);
	if (ret < 0) {
		LOG_ERR("Failed to configure pin %d as output: %d", pin, ret);
		return ret;
	}

	LOG_DBG("Pin %d configured as output", pin);
	return 0;
}

static int configure_cs_output(void)
{
	int ret;

	ret = configure_pin_output(SPI_CS_PIN);
	if (ret < 0) return ret;
	ret = configure_pin_output(SPI_CS1_PIN);
	if (ret < 0) return ret;
	ret = configure_pin_output(SPI_CS2_PIN);
	if (ret < 0) return ret;

	return 0;
}

/**
 * @brief Configure a pin as high-impedance input (tristate)
 */
static int configure_pin_tristate(uint32_t pin)
{
	int ret;

	nrf_gpio_cfg_default(pin);

	ret = gpio_pin_configure(gpio_dev2, pin, GPIO_INPUT | GPIO_PULL_DOWN);
	if (ret < 0) {
		LOG_ERR("Failed to configure pin %d as tristate: %d", pin, ret);
		return ret;
	}

	LOG_DBG("Pin %d configured as tristate", pin);
	return 0;
}

/**
 * @brief Configure all pins as(tristate)
 */
static int configure_pins_as_tristate(void)
{
	int ret;

	ret = configure_pin_tristate(SPI_SCK_PIN);
	if (ret < 0) return ret;

	ret = configure_pin_tristate(SPI_MOSI_PIN);
	if (ret < 0) return ret;

	ret = configure_pin_tristate(SPI_MISO_PIN);
	if (ret < 0) return ret;

	ret = configure_cs_tristate();
	if (ret < 0) return ret;

	return 0;
}

/**
 * @brief Configure CS pins as high-impedance to prevent bus conflicts
 */
static int configure_cs_tristate(void)
{
	int ret;

	gpio_pin_set(gpio_dev2, SPI_CS_PIN, 1);
	gpio_pin_set(gpio_dev2, SPI_CS1_PIN, 1);
	gpio_pin_set(gpio_dev2, SPI_CS2_PIN, 1);

	k_msleep(1); 

	ret = configure_pin_tristate(SPI_CS_PIN);
	if (ret < 0) return ret;

	ret = configure_pin_tristate(SPI_CS1_PIN);
	if (ret < 0) return ret;

	ret = configure_pin_tristate(SPI_CS2_PIN);
	if (ret < 0) return ret;

	return 0;
}

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
	ret = configure_pins_as_tristate();
	if (ret < 0) {
		LOG_ERR("Failed to configure pins as tristate");
		k_mutex_unlock(&bus_mutex);
	}

	current_owner = SPI_BUS_OWNER_NONE;
	LOG_INF("%s bus successfully tristated", SPI_BUS_TYPE);

	k_mutex_unlock(&bus_mutex);
	return 0;
}

int spi_bus_reclaim(void)
{
	int ret;

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
		/* Continue without flash PM - just configure pins */
	}

	#endif
	LOG_INF("Manually reconfiguring CS pins");
	ret = configure_cs_output();
	if (ret < 0) {
		LOG_ERR("Failed to configure CS pins");
		k_mutex_unlock(&bus_mutex);
	}
	ret = gpio_pin_set(gpio_dev2, SPI_CS_PIN, 0);
	ret = gpio_pin_set(gpio_dev2, SPI_CS1_PIN, 0);
	ret = gpio_pin_set(gpio_dev2, SPI_CS2_PIN, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set CS pins: %d", ret);
		k_mutex_unlock(&bus_mutex);
	}

	LOG_INF("CS pins configured as outputs (high)");

	current_owner = SPI_BUS_OWNER_THIS_MCU;
	LOG_INF("%s bus successfully reclaimed", SPI_BUS_TYPE);

	k_mutex_unlock(&bus_mutex);
	return 0;
}



/*
======================================================================================
======================================================================================
======================================================================================
======================================================================================
COMMANDS
======================================================================================
======================================================================================
======================================================================================
*/

#ifdef SPI_BUS_SHARE_SHELL
static int cmd_spi_tristate(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Tristating SPI bus...");

	int ret = spi_bus_tristate();
	if (ret == 0) {
		shell_print(sh, "SUCCESS: SPI bus pins are now in high-impedance state");
		shell_print(sh, "The other MCU can now safely access the external memory");
	} else {
		shell_error(sh, "FAILED: Could not tristate SPI bus (error %d)", ret);
	}

	return ret;
}

static int cmd_spi_reclaim(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Reclaiming SPI bus...");

	int ret = spi_bus_reclaim();
	if (ret == 0) {
		shell_print(sh, "SUCCESS: SPI bus reclaimed");
		shell_print(sh, "This MCU can now access the external memory");
	} else {
		shell_error(sh, "FAILED: Could not reclaim SPI bus (error %d)", ret);
	}

	return ret;
}

static int cmd_spi_owner(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	enum spi_bus_owner owner = spi_bus_get_owner();

	shell_print(sh, "Current SPI bus owner:");

	switch (owner) {
	case SPI_BUS_OWNER_THIS_MCU:
		shell_print(sh, "  THIS_MCU - This microprocessor owns the bus");
		shell_print(sh, "  SPI operations are enabled");
		break;
	case SPI_BUS_OWNER_OTHER_MCU:
		shell_print(sh, "  OTHER_MCU - Other microprocessor owns the bus");
		shell_print(sh, "  SPI operations should not be performed");
		break;
	case SPI_BUS_OWNER_NONE:
		shell_print(sh, "  NONE - Bus is tristated (high-impedance)");
		shell_print(sh, "  No MCU currently owns the bus");
		break;
	default:
		shell_error(sh, "  UNKNOWN - Invalid owner state");
		break;
	}

	return 0;
}

static int cmd_spi_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "SPI Bus Sharing Status");
	shell_print(sh, "======================");
	shell_print(sh, "");


	shell_print(sh, "Pin Configuration:");
	shell_print(sh, "  SCK  (Clock):      P0.%d", SPI_SCK_PIN);
	shell_print(sh, "  MOSI (Master Out): P0.%d", SPI_MOSI_PIN);
	shell_print(sh, "  MISO (Master In):  P0.%d", SPI_MISO_PIN);
	shell_print(sh, "  CS0  (Flash):      P0.%d", SPI_CS_PIN);
	shell_print(sh, "  CS1:               P0.%d", SPI_CS1_PIN);
	shell_print(sh, "  CS2:               P0.%d", SPI_CS2_PIN);
	shell_print(sh, "");


	enum spi_bus_owner owner = spi_bus_get_owner();
	shell_print(sh, "Current Bus Owner:");

	switch (owner) {
	case SPI_BUS_OWNER_THIS_MCU:
		shell_print(sh, "  THIS_MCU (SPI enabled)");
		break;
	case SPI_BUS_OWNER_OTHER_MCU:
		shell_print(sh, "  OTHER_MCU (SPI disabled)");
		break;
	case SPI_BUS_OWNER_NONE:
		shell_print(sh, "  NONE (Bus tristated)");
		break;
	default:
		shell_print(sh, "  UNKNOWN");
		break;
	}

	return 0;
}

static int cmd_spi_test_pins(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Testing SPI Pin States");
	shell_print(sh, "======================");
	shell_print(sh, "");

	shell_print(sh, "Pin Configurations:");


	NRF_GPIO_Type *gpio_port = NRF_P0_NS;


	uint32_t sck_cnf = gpio_port->PIN_CNF[SPI_SCK_PIN];
	shell_print(sh, "  SCK  (P0.%d): CNF=0x%08X", SPI_SCK_PIN, sck_cnf);
	shell_print(sh, "    DIR=%s, INPUT=%s, PULL=%s, DRIVE=%s, SENSE=%s",
		(sck_cnf & GPIO_PIN_CNF_DIR_Msk) ? "Output" : "Input",
		(sck_cnf & GPIO_PIN_CNF_INPUT_Msk) ? "Disconnected" : "Connected",
		((sck_cnf & GPIO_PIN_CNF_PULL_Msk) >> GPIO_PIN_CNF_PULL_Pos) == 0 ? "Disabled" :
		((sck_cnf & GPIO_PIN_CNF_PULL_Msk) >> GPIO_PIN_CNF_PULL_Pos) == 1 ? "Pulldown" : "Pullup",
		((sck_cnf & GPIO_PIN_CNF_DRIVE_Msk) >> GPIO_PIN_CNF_DRIVE_Pos) == 0 ? "S0S1" : "Other",
		((sck_cnf & GPIO_PIN_CNF_SENSE_Msk) >> GPIO_PIN_CNF_SENSE_Pos) == 0 ? "Disabled" : "Enabled");


	uint32_t mosi_cnf = gpio_port->PIN_CNF[SPI_MOSI_PIN];
	shell_print(sh, "  MOSI (P0.%d): CNF=0x%08X", SPI_MOSI_PIN, mosi_cnf);
	shell_print(sh, "    DIR=%s, INPUT=%s, PULL=%s",
		(mosi_cnf & GPIO_PIN_CNF_DIR_Msk) ? "Output" : "Input",
		(mosi_cnf & GPIO_PIN_CNF_INPUT_Msk) ? "Disconnected" : "Connected",
		((mosi_cnf & GPIO_PIN_CNF_PULL_Msk) >> GPIO_PIN_CNF_PULL_Pos) == 0 ? "Disabled" :
		((mosi_cnf & GPIO_PIN_CNF_PULL_Msk) >> GPIO_PIN_CNF_PULL_Pos) == 1 ? "Pulldown" : "Pullup");


	uint32_t miso_cnf = gpio_port->PIN_CNF[SPI_MISO_PIN];
	shell_print(sh, "  MISO (P0.%d): CNF=0x%08X", SPI_MISO_PIN, miso_cnf);
	shell_print(sh, "    DIR=%s, INPUT=%s, PULL=%s",
		(miso_cnf & GPIO_PIN_CNF_DIR_Msk) ? "Output" : "Input",
		(miso_cnf & GPIO_PIN_CNF_INPUT_Msk) ? "Disconnected" : "Connected",
		((miso_cnf & GPIO_PIN_CNF_PULL_Msk) >> GPIO_PIN_CNF_PULL_Pos) == 0 ? "Disabled" :
		((miso_cnf & GPIO_PIN_CNF_PULL_Msk) >> GPIO_PIN_CNF_PULL_Pos) == 1 ? "Pulldown" : "Pullup");

	uint32_t cs_cnf = gpio_port->PIN_CNF[SPI_CS_PIN];
	shell_print(sh, "  CS0  (P0.%d): CNF=0x%08X", SPI_CS_PIN, cs_cnf);
	shell_print(sh, "    DIR=%s, INPUT=%s, PULL=%s",
		(cs_cnf & GPIO_PIN_CNF_DIR_Msk) ? "Output" : "Input",
		(cs_cnf & GPIO_PIN_CNF_INPUT_Msk) ? "Disconnected" : "Connected",
		((cs_cnf & GPIO_PIN_CNF_PULL_Msk) >> GPIO_PIN_CNF_PULL_Pos) == 0 ? "Disabled" :
		((cs_cnf & GPIO_PIN_CNF_PULL_Msk) >> GPIO_PIN_CNF_PULL_Pos) == 1 ? "Pulldown" : "Pullup");

	shell_print(sh, "");
	shell_print(sh, "Tristate Check:");
	shell_print(sh, "  A tristated pin should show: DIR=Input, PULL=Pulldown (or Disabled)");
	shell_print(sh, "  An active pin shows: Connected to peripheral or DIR=Output");

	return 0;
}

static int cmd_bus_request(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Requesting bus ownership via FSM...");

	int ret = spi_bus_request();
	if (ret == 0) {
		shell_print(sh, "Request posted to FSM");
		shell_print(sh, "Use 'bus_fsm_status' to check state");
	} else {
		shell_error(sh, "Failed to post request: %d", ret);
	}

	return ret;
}

static int cmd_bus_release(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Releasing bus via FSM...");

	int ret = spi_bus_release();
	if (ret == 0) {
		shell_print(sh, "Release posted to FSM");
		shell_print(sh, "Use 'bus_fsm_status' to check state");
	} else {
		shell_error(sh, "Failed to post release: %d", ret);
	}

	return ret;
}

static int cmd_bus_fsm_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	uint32_t errors, transitions;

	shell_print(sh, "SPI Bus FSM Status");
	shell_print(sh, "==================");
	shell_print(sh, "");

	int state = spi_bus_get_fsm_state();
	shell_print(sh, "Current FSM State: %s", fsm_state_to_string((enum bus_fsm_state)state));
	shell_print(sh, "");

	bool req_state = gpio_pin_get(gpio_dev2, BUS_REQUEST_PIN);
	bool grant_state = gpio_pin_get(gpio_dev2, BUS_GRANT_PIN);

	shell_print(sh, "GPIO Pin States:");
	shell_print(sh, "  BUS_REQUEST (P0.%d): %s", BUS_REQUEST_PIN, req_state ? "HIGH" : "LOW");
	shell_print(sh, "  BUS_GRANT   (P0.%d): %s", BUS_GRANT_PIN, grant_state ? "HIGH" : "LOW");
	shell_print(sh, "");

	k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);
	bool other_detected = fsm_ctx.other_mcu_detected;
	bool in_transaction = fsm_ctx.transaction_in_progress;
	k_mutex_unlock(&fsm_ctx.mutex);

	shell_print(sh, "Bus Status:");
	shell_print(sh, "  Other MCU detected:   %s", other_detected ? "YES" : "NO");
	shell_print(sh, "  Transaction lock:     %s", in_transaction ? "YES (holding indefinitely)" : "NO");
	if (in_transaction && grant_state) {
		shell_print(sh, "  GRANT line:           HIGH (other MCU waiting)");
		shell_print(sh, "  Status:               Holding bus, will release on bus_release_transaction");
	} else if (in_transaction) {
		shell_print(sh, "  GRANT line:           LOW (no MCU waiting)");
		shell_print(sh, "  Status:               Transaction lock active, no contention");
	} else if (grant_state) {
		shell_print(sh, "  GRANT line:           HIGH (WARNING: unexpected state)");
	}
	shell_print(sh, "");

	spi_bus_get_fsm_stats(&errors, &transitions);
	shell_print(sh, "Statistics:");
	shell_print(sh, "  Total transitions: %u", transitions);
	shell_print(sh, "  Error count:       %u", errors);
	shell_print(sh, "");

	shell_print(sh, "FSM State Descriptions:");
	shell_print(sh, "  STARTUP:        Auto-discovering bus ownership");
	shell_print(sh, "  IDLE_OWNER:     This MCU owns the bus");
	shell_print(sh, "  IDLE_NOT_OWNER: Other MCU owns the bus");
	shell_print(sh, "  REQUESTING:     Waiting for grant from other MCU");
	shell_print(sh, "  RELEASING:      Releasing bus to other MCU");
	shell_print(sh, "  ERROR:          Protocol error - use 'bus_recover'");

	return 0;
}

static int cmd_bus_recover(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Initiating FSM error recovery...");

	int ret = spi_bus_error_recovery();
	if (ret == 0) {
		shell_print(sh, "Recovery posted to FSM");
		shell_print(sh, "Use 'bus_fsm_status' to check result");
	} else {
		shell_error(sh, "Failed to post recovery: %d", ret);
	}

	return ret;
}

static int cmd_bus_simulate_grant(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (argc != 2) {
		shell_error(sh, "Usage: bus_simulate_grant <0|1>");
		shell_print(sh, "  0 = Other MCU denies/revokes access (GRANT_LOST)");
		shell_print(sh, "  1 = Other MCU grants access (GRANT_RECEIVED)");
		return -EINVAL;
	}

	int value = atoi(argv[1]);
	if (value != 0 && value != 1) {
		shell_error(sh, "Invalid value. Use 0 or 1");
		return -EINVAL;
	}

	shell_print(sh, "");
	shell_print(sh, "=== SINGLE MCU TEST MODE ===");
	shell_print(sh, "Simulating other MCU %s bus grant...",
	            value ? "GRANTING" : "DENYING/REVOKING");
	shell_print(sh, "");

	int ret;
	if (value) {
		ret = fsm_post_event(EVENT_GRANT_RECEIVED);
		shell_print(sh, "Posted EVENT_GRANT_RECEIVED to FSM");
	} else {
		ret = fsm_post_event(EVENT_GRANT_LOST);
		shell_print(sh, "Posted EVENT_GRANT_LOST to FSM");
	}

	if (ret == 0) {
		shell_print(sh, "Event posted successfully");
		shell_print(sh, "Use 'bus_fsm_status' to verify state transition");
	} else {
		shell_error(sh, "Failed to post event: %d", ret);
	}

	shell_print(sh, "");

	return ret;
}

static int cmd_bus_simulate_request(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (argc != 2) {
		shell_error(sh, "Usage: bus_simulate_request <0|1>");
		shell_print(sh, "  0 = Other MCU released its request (REQUEST_RELEASED)");
		shell_print(sh, "  1 = Other MCU is requesting the bus (REQUEST_RECEIVED)");
		return -EINVAL;
	}

	int value = atoi(argv[1]);
	if (value != 0 && value != 1) {
		shell_error(sh, "Invalid value. Use 0 or 1");
		return -EINVAL;
	}

	shell_print(sh, "");
	shell_print(sh, "=== SINGLE MCU TEST MODE ===");
	shell_print(sh, "Simulating other MCU %s bus request...",
	            value ? "REQUESTING" : "RELEASING REQUEST");
	shell_print(sh, "");

	int ret;
	if (value) {
		ret = fsm_post_event(EVENT_REQUEST_RECEIVED);
		shell_print(sh, "Posted EVENT_REQUEST_RECEIVED to FSM");
	} else {
		ret = fsm_post_event(EVENT_REQUEST_RELEASED);
		shell_print(sh, "Posted EVENT_REQUEST_RELEASED to FSM");
	}

	if (ret == 0) {
		shell_print(sh, "Event posted successfully");
		shell_print(sh, "Use 'bus_fsm_status' to verify state transition");
	} else {
		shell_error(sh, "Failed to post event: %d", ret);
	}

	shell_print(sh, "");

	return ret;
}

static int cmd_bus_acquire(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	if (argc != 2) {
		shell_error(sh, "Usage: bus_acquire <quick|lock>");
		shell_print(sh, "  quick = Acquire bus, yield immediately on request (no transaction lock)");
		shell_print(sh, "  lock  = Acquire bus with transaction lock (hold indefinitely)");
		return -EINVAL;
	}

	bool use_transaction_lock;

	if (strcmp(argv[1], "quick") == 0) {
		use_transaction_lock = false;
		shell_print(sh, "Acquiring bus in QUICK mode (will yield immediately)...");
	} else if (strcmp(argv[1], "lock") == 0) {
		use_transaction_lock = true;
		shell_print(sh, "Acquiring bus with TRANSACTION LOCK (will hold indefinitely)...");
	} else {
		shell_error(sh, "Invalid mode. Use 'quick' or 'lock'");
		return -EINVAL;
	}

	shell_print(sh, "");
	shell_print(sh, "This command will block until bus is acquired...");
	shell_print(sh, "");

	int ret = spi_bus_acquire_blocking(use_transaction_lock);

	if (ret == 0) {
		shell_print(sh, "SUCCESS: Bus acquired");
		if (use_transaction_lock) {
			shell_print(sh, "Transaction lock active - bus will NOT yield to other MCU");
			shell_print(sh, "Use 'bus_release_transaction' to release the lock and handover");
		} else {
			shell_print(sh, "Quick mode - bus will yield immediately if other MCU requests");
		}
	} else {
		shell_error(sh, "FAILED: Could not acquire bus (error %d)", ret);
	}

	return ret;
}

static int cmd_bus_release_transaction(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Releasing transaction lock...");

	int ret = spi_bus_release_transaction();

	if (ret == 0) {
		shell_print(sh, "SUCCESS: Transaction lock released");
		shell_print(sh, "If other MCU was waiting, handover is now complete");
		shell_print(sh, "Use 'bus_fsm_status' to check current state");
	} else {
		shell_error(sh, "FAILED: Could not release transaction (error %d)", ret);
	}

	return ret;
}

SHELL_CMD_REGISTER(spi_tristate, NULL, "Release SPI bus (tristate pins)", cmd_spi_tristate);
SHELL_CMD_REGISTER(spi_reclaim, NULL, "Reclaim SPI bus for this MCU", cmd_spi_reclaim);
SHELL_CMD_REGISTER(spi_owner, NULL, "Show current SPI bus owner", cmd_spi_owner);
SHELL_CMD_REGISTER(spi_status, NULL, "Show SPI bus sharing status", cmd_spi_status);
SHELL_CMD_REGISTER(spi_test_pins, NULL, "Test and show actual pin states", cmd_spi_test_pins);

SHELL_CMD_REGISTER(bus_request, NULL, "Request bus ownership (FSM)", cmd_bus_request);
SHELL_CMD_REGISTER(bus_release, NULL, "Release bus ownership (FSM)", cmd_bus_release);
SHELL_CMD_REGISTER(bus_acquire, NULL, "Acquire bus (blocking) - quick or lock mode", cmd_bus_acquire);
SHELL_CMD_REGISTER(bus_release_transaction, NULL, "Release transaction lock and complete handover", cmd_bus_release_transaction);
SHELL_CMD_REGISTER(bus_fsm_status, NULL, "Show FSM status and statistics", cmd_bus_fsm_status);
SHELL_CMD_REGISTER(bus_recover, NULL, "Recover from FSM error state", cmd_bus_recover);
SHELL_CMD_REGISTER(bus_simulate_grant, NULL, "TEST: Simulate other MCU grant signal", cmd_bus_simulate_grant);
SHELL_CMD_REGISTER(bus_simulate_request, NULL, "TEST: Simulate other MCU request signal", cmd_bus_simulate_request);


#endif