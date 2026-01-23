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
#include "spi_bus_share_internal.h"
#include "spi_bus_share_config.h"
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/pinctrl.h>
#include <hal/nrf_spim.h>

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

#if defined(ENABLE_CS_MIRROR) && !defined(THIS_IS_NRF5340)
	#define CS_MIRROR_ON_BECOME_OWNER() cs_mirror_disable()
	#define CS_MIRROR_ON_BECOME_NOT_OWNER() cs_mirror_enable()
#else
	#define CS_MIRROR_ON_BECOME_OWNER() do {} while (0)
	#define CS_MIRROR_ON_BECOME_NOT_OWNER() do {} while (0)
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

#ifndef THIS_IS_NRF5340
const struct device *spi_dev;
#endif


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

const struct device *gpio_dev2;

const struct device *flash_dev;
enum spi_bus_owner current_owner = SPI_BUS_OWNER_THIS_MCU;
struct k_mutex bus_mutex;
static struct bus_fsm_context fsm_ctx;
static struct gpio_callback grant_cb_data;
static struct gpio_callback request_cb_data;
static bool fsm_initialized = false;

#if defined(ENABLE_CS_MIRROR) && !defined(THIS_IS_NRF5340)
struct gpio_callback cs_mirror_cb_data;
bool cs_mirror_enabled = false;
#endif



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
	
		configure_pins_as_idle();
		k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

		fsm_ctx.state = FSM_IDLE_NOT_OWNER;
		current_owner = SPI_BUS_OWNER_OTHER_MCU;

		CS_MIRROR_ON_BECOME_NOT_OWNER();
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

					CS_MIRROR_ON_BECOME_OWNER();

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

				CS_MIRROR_ON_BECOME_OWNER();

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

				CS_MIRROR_ON_BECOME_OWNER();
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
					
					configure_pins_as_idle();
					k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

					fsm_ctx.state = FSM_IDLE_NOT_OWNER;
					current_owner = SPI_BUS_OWNER_OTHER_MCU;

					CS_MIRROR_ON_BECOME_NOT_OWNER();
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
			
				configure_pins_as_idle();
	
				k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

				fsm_ctx.state = FSM_IDLE_NOT_OWNER;
				current_owner = SPI_BUS_OWNER_OTHER_MCU;

				CS_MIRROR_ON_BECOME_NOT_OWNER();
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

				CS_MIRROR_ON_BECOME_OWNER();

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

				CS_MIRROR_ON_BECOME_OWNER();
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

					CS_MIRROR_ON_BECOME_OWNER();
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

			CS_MIRROR_ON_BECOME_NOT_OWNER();
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

					CS_MIRROR_ON_BECOME_OWNER();
				}
			} else {
				k_mutex_unlock(&fsm_ctx.mutex);
				ret = spi_bus_tristate();
				k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);

				fsm_ctx.state = FSM_IDLE_NOT_OWNER;
				current_owner = SPI_BUS_OWNER_OTHER_MCU;

				CS_MIRROR_ON_BECOME_NOT_OWNER();
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

#ifdef THIS_IS_BOOTLOADER
K_THREAD_STACK_DEFINE(fsm_thread_stack, 2048);
static struct k_thread fsm_thread_data;
static k_tid_t fsm_thread_id;
#else
K_THREAD_DEFINE(fsm_thread_id, 2048, fsm_thread, NULL, NULL, NULL, 5, 0, 0);
#endif

int spi_bus_share_init(void)
{
	int ret;

	#ifdef THIS_IS_NRF9151
		LOG_INF("nRF9151 build - using SPI bus (delayed for sync)");
		k_sleep(K_MSEC(BOOT_DELAY_MS));
	#else
		LOG_INF("nRF5340 build - using QSPI bus");
	#endif


	k_mutex_init(&fsm_ctx.mutex);
	k_sem_init(&fsm_ctx.event_sem, 0, 1);
	k_work_init_delayable(&fsm_ctx.timeout_work, fsm_timeout_handler);
	k_work_init_delayable(&fsm_ctx.deferred_grant_work, deferred_grant_handler);

	k_mutex_init(&bus_mutex);

	LOG_INF("Initializing SPI bus sharing module");

	gpio_dev2 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	if (!device_is_ready(gpio_dev2)) {
		LOG_ERR("GPIO device not ready");
		return -ENODEV;
	}

	#ifndef THIS_IS_NRF5340
	spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi3));
	if (!device_is_ready(spi_dev)) {
		LOG_WRN("SPI device not ready");
	}
	#endif

	flash_dev = DEVICE_DT_GET(DT_NODELABEL(flash_ext));
	if (!device_is_ready(flash_dev)) {
		LOG_WRN("Flash device not ready - attempting to initialize...");
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
		return ret;
	}

	ret = gpio_add_callback(gpio_dev2, &request_cb_data);
	if (ret < 0) {
		LOG_ERR("Failed to add REQUEST callback: %d", ret);
		return ret;
	}

	#if defined(ENABLE_CS_MIRROR) && !defined(THIS_IS_NRF5340)

	gpio_init_callback(&cs_mirror_cb_data, cs_mirror_isr, BIT(CS_MIRROR_NRF5340_SOURCE_PIN));
	ret = gpio_add_callback(gpio_dev2, &cs_mirror_cb_data);
	if (ret < 0) {
		LOG_ERR("Failed to add CS_MIRROR callback: %d", ret);
		return ret;
	}
	LOG_INF("CS mirror callback registered for P0.%d (nRF5340 source)",
	        CS_MIRROR_NRF5340_SOURCE_PIN);
	#endif

	LOG_INF("Starting in STARTUP state - configuring SPI pins");

	spi_bus_tristate();

	configure_pins_as_idle();


	current_owner = SPI_BUS_OWNER_NONE;

#ifdef THIS_IS_BOOTLOADER
	/* Manually create and start FSM thread for bootloader */
	LOG_INF("Creating FSM thread manually for bootloader...");
	fsm_thread_id = k_thread_create(&fsm_thread_data, fsm_thread_stack,
	                                K_THREAD_STACK_SIZEOF(fsm_thread_stack),
	                                fsm_thread, NULL, NULL, NULL,
	                                5, 0, K_NO_WAIT);
	k_thread_name_set(fsm_thread_id, "fsm_thread");
	LOG_INF("FSM thread created with tid %p", fsm_thread_id);
#endif

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

	return 0;
}

#ifndef THIS_IS_BOOTLOADER
SYS_INIT(spi_bus_share_init, POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY);
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

	k_thread_abort(fsm_thread_id);
	LOG_INF("  FSM thread aborted");

	int ret = gpio_remove_callback(gpio_dev2, &grant_cb_data);
	if (ret < 0) {
		LOG_WRN("Failed to remove GRANT callback: %d", ret);
	}

	ret = gpio_remove_callback(gpio_dev2, &request_cb_data);
	if (ret < 0) {
		LOG_WRN("Failed to remove REQUEST callback: %d", ret);
	}
	LOG_INF("  GPIO callbacks removed");

	gpio_pin_interrupt_configure(gpio_dev2, BUS_REQUEST_PIN, GPIO_INT_DISABLE);
	gpio_pin_interrupt_configure(gpio_dev2, BUS_GRANT_PIN, GPIO_INT_DISABLE);
	LOG_INF("  GPIO interrupts disabled");

	#if defined(THIS_IS_NRF5340) && !defined(THIS_IS_BOOTLOADER)
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

	k_work_cancel_delayable(&fsm_ctx.timeout_work);
	k_work_cancel_delayable(&fsm_ctx.deferred_grant_work);
	LOG_INF("  Pending work items cancelled");

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
ACCESSOR FUNCTIONS FOR SHELL
======================================================================================
======================================================================================
======================================================================================
*/

const struct device *spi_bus_get_gpio_dev(void)
{
	return gpio_dev2;
}

const struct device *spi_bus_get_spi_dev(void)
{
#ifndef THIS_IS_NRF5340
	return spi_dev;
#else
	return NULL;
#endif
}

const char *spi_bus_get_type(void)
{
	return SPI_BUS_TYPE;
}

uint8_t spi_bus_get_sck_pin(void)
{
	return SPI_SCK_PIN;
}

uint8_t spi_bus_get_mosi_pin(void)
{
	return SPI_MOSI_PIN;
}

uint8_t spi_bus_get_miso_pin(void)
{
	return SPI_MISO_PIN;
}

uint8_t spi_bus_get_cs_pin(void)
{
	return SPI_CS_PIN;
}

uint8_t spi_bus_get_cs1_pin(void)
{
	return SPI_CS1_PIN;
}

uint8_t spi_bus_get_cs2_pin(void)
{
	return SPI_CS2_PIN;
}

uint8_t spi_bus_get_bus_request_pin(void)
{
	return BUS_REQUEST_PIN;
}

uint8_t spi_bus_get_bus_grant_pin(void)
{
	return BUS_GRANT_PIN;
}

const char *spi_bus_fsm_state_to_string(int state)
{
	return fsm_state_to_string((enum bus_fsm_state)state);
}

void spi_bus_get_transaction_state(bool *other_mcu_detected, bool *transaction_in_progress)
{
	k_mutex_lock(&fsm_ctx.mutex, K_FOREVER);
	if (other_mcu_detected) {
		*other_mcu_detected = fsm_ctx.other_mcu_detected;
	}
	if (transaction_in_progress) {
		*transaction_in_progress = fsm_ctx.transaction_in_progress;
	}
	k_mutex_unlock(&fsm_ctx.mutex);
}

int spi_bus_simulate_grant_event(int value)
{
	int ret;
	if (value) {
		ret = fsm_post_event(EVENT_GRANT_RECEIVED);
		LOG_INF("Posted EVENT_GRANT_RECEIVED to FSM");
	} else {
		ret = fsm_post_event(EVENT_GRANT_LOST);
		LOG_INF("Posted EVENT_GRANT_LOST to FSM");
	}
	return ret;
}

int spi_bus_simulate_request_event(int value)
{
	int ret;
	if (value) {
		ret = fsm_post_event(EVENT_REQUEST_RECEIVED);
		LOG_INF("Posted EVENT_REQUEST_RECEIVED to FSM");
	} else {
		ret = fsm_post_event(EVENT_REQUEST_RELEASED);
		LOG_INF("Posted EVENT_REQUEST_RELEASED to FSM");
	}
	return ret;
}
