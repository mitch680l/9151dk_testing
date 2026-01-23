#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <hal/nrf_gpio.h>
#include <stdlib.h>
#include "spi_bus_share.h"
#include "spi_bus_share_internal.h"

LOG_MODULE_REGISTER(spi_bus_shell, LOG_LEVEL_INF);

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
	shell_print(sh, "  SCK  (Clock):      P0.%d", spi_bus_get_sck_pin());
	shell_print(sh, "  MOSI (Master Out): P0.%d", spi_bus_get_mosi_pin());
	shell_print(sh, "  MISO (Master In):  P0.%d", spi_bus_get_miso_pin());
	shell_print(sh, "  CS0  (Flash):      P0.%d", spi_bus_get_cs_pin());
	shell_print(sh, "  CS1:               P0.%d", spi_bus_get_cs1_pin());
	shell_print(sh, "  CS2:               P0.%d", spi_bus_get_cs2_pin());
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

	shell_print(sh, "========================================");
	shell_print(sh, "  SPI Pin Electrical State Analysis");
	shell_print(sh, "========================================");
	shell_print(sh, "");

	NRF_GPIO_Type *gpio_port = NRF_P0_NS;
	const struct device *gpio_dev2 = spi_bus_get_gpio_dev();

	const struct {
		uint8_t pin;
		const char *name;
	} pins[] = {
		{spi_bus_get_sck_pin(), "SCK"},
		{spi_bus_get_mosi_pin(), "MOSI"},
		{spi_bus_get_miso_pin(), "MISO"},
		{spi_bus_get_cs_pin(), "CS0"},
		{spi_bus_get_cs1_pin(), "CS1"},
		{spi_bus_get_cs2_pin(), "CS2"},
		{spi_bus_get_bus_request_pin(), "BUS_REQ"},
		{spi_bus_get_bus_grant_pin(), "BUS_GNT"}
	};

	for (int i = 0; i < 8; i++) {
		uint8_t pin = pins[i].pin;
		uint32_t cnf = gpio_port->PIN_CNF[pin];

		shell_print(sh, "--- P0.%d (%s) ---", pin, pins[i].name);
		shell_print(sh, "  Register: PIN_CNF[%d] = 0x%08X", pin, cnf);

		uint32_t dir = (cnf & GPIO_PIN_CNF_DIR_Msk) >> GPIO_PIN_CNF_DIR_Pos;
		uint32_t input = (cnf & GPIO_PIN_CNF_INPUT_Msk) >> GPIO_PIN_CNF_INPUT_Pos;
		uint32_t pull = (cnf & GPIO_PIN_CNF_PULL_Msk) >> GPIO_PIN_CNF_PULL_Pos;
		uint32_t drive = (cnf & GPIO_PIN_CNF_DRIVE_Msk) >> GPIO_PIN_CNF_DRIVE_Pos;
		uint32_t sense = (cnf & GPIO_PIN_CNF_SENSE_Msk) >> GPIO_PIN_CNF_SENSE_Pos;

		shell_print(sh, "  Configuration:");
		shell_print(sh, "    DIR    [bit 0]     = %d (%s)", dir, dir ? "Output" : "Input");
		shell_print(sh, "    INPUT  [bit 1]     = %d (%s)", input, input ? "Disconnected" : "Connected");
		shell_print(sh, "    PULL   [bits 2-3]  = %d (%s)", pull,
			pull == 0 ? "Disabled" : pull == 1 ? "Pulldown" : pull == 3 ? "Pullup" : "Reserved");
		shell_print(sh, "    DRIVE  [bits 8-10] = %d (%s)", drive,
			drive == 0 ? "S0S1 (Std)" : drive == 1 ? "H0S1" : drive == 2 ? "S0H1" :
			drive == 3 ? "H0H1" : drive == 4 ? "D0S1" : drive == 5 ? "D0H1" :
			drive == 6 ? "S0D1" : drive == 7 ? "H0D1" : "Unknown");
		shell_print(sh, "    SENSE  [bits 16-17]= %d (%s)", sense,
			sense == 0 ? "Disabled" : sense == 2 ? "High" : sense == 3 ? "Low" : "Reserved");

		uint32_t in_state = (gpio_port->IN >> pin) & 0x1;
		uint32_t out_state = (gpio_port->OUT >> pin) & 0x1;

		shell_print(sh, "  Electrical State:");
		shell_print(sh, "    IN  register [bit %d] = %d (%s)", pin, in_state,
			in_state ? "HIGH (VDD)" : "LOW (GND)");
		shell_print(sh, "    OUT register [bit %d] = %d (%s)", pin, out_state,
			out_state ? "HIGH (driving VDD)" : "LOW (driving GND)");

		int zephyr_read = gpio_pin_get(gpio_dev2, pin);
		shell_print(sh, "    Zephyr API read      = %d", zephyr_read);

		shell_print(sh, "  Analysis:");
		if (dir == 1) {
			shell_print(sh, "    Mode: OUTPUT - Pin is actively driven by MCU");
			shell_print(sh, "    Drive strength: %s",
				drive == 0 ? "Standard (0.5mA)" :
				drive == 1 ? "High drive 0 (5mA sink)" :
				drive == 3 ? "High drive (5mA)" : "Enhanced");
			shell_print(sh, "    Output value: %s (OUT=%d matches IN=%d? %s)",
				out_state ? "HIGH" : "LOW", out_state, in_state,
				(out_state == in_state) ? "YES" : "NO - possible contention!");
		} else {
			shell_print(sh, "    Mode: INPUT - Pin is reading external signal");
			if (input == 1) {
				shell_print(sh, "    Buffer: DISCONNECTED (high-Z, tristated)");
				shell_print(sh, "    Physical state: Floating (undefined voltage)");
			} else {
				shell_print(sh, "    Buffer: CONNECTED (reading pin state)");
			}

			if (pull == 1) {
				shell_print(sh, "    Pull: PULLDOWN enabled (~13k to GND)");
				shell_print(sh, "    Expected: LOW unless externally driven HIGH");
				if (in_state == 1) {
					shell_print(sh, "    >>> Pin is HIGH - external signal present!");
				}
			} else if (pull == 3) {
				shell_print(sh, "    Pull: PULLUP enabled (~13k to VDD)");
				shell_print(sh, "    Expected: HIGH unless externally driven LOW");
				if (in_state == 0) {
					shell_print(sh, "    >>> Pin is LOW - external signal present!");
				}
			} else {
				shell_print(sh, "    Pull: DISABLED");
				shell_print(sh, "    WARNING: Floating input (undefined state)");
			}
		}
		shell_print(sh, "");
	}

	shell_print(sh, "========================================");
	shell_print(sh, "  SPI Peripheral State");
	shell_print(sh, "========================================");

	const struct device *spi_dev = spi_bus_get_spi_dev();
	//const char *bus_type = spi_bus_get_type();

	#ifndef THIS_IS_NRF5340
	if (spi_dev && device_is_ready(spi_dev)) {
		shell_print(sh, "SPI3 Device: READY");

		NRF_SPIM_Type *spim = NRF_SPIM3_NS;

		shell_print(sh, "SPIM3 Registers:");
		shell_print(sh, "  ENABLE = 0x%08X (%s)", spim->ENABLE,
			spim->ENABLE == 7 ? "ENABLED" : "DISABLED");
		shell_print(sh, "  FREQUENCY = 0x%08X", spim->FREQUENCY);
		shell_print(sh, "  PSEL.SCK  = 0x%08X (pin %d, %s)", spim->PSEL.SCK,
			spim->PSEL.SCK & 0x1F,
			(spim->PSEL.SCK & 0x80000000) ? "DISCONNECTED" : "CONNECTED");
		shell_print(sh, "  PSEL.MOSI = 0x%08X (pin %d, %s)", spim->PSEL.MOSI,
			spim->PSEL.MOSI & 0x1F,
			(spim->PSEL.MOSI & 0x80000000) ? "DISCONNECTED" : "CONNECTED");
		shell_print(sh, "  PSEL.MISO = 0x%08X (pin %d, %s)", spim->PSEL.MISO,
			spim->PSEL.MISO & 0x1F,
			(spim->PSEL.MISO & 0x80000000) ? "DISCONNECTED" : "CONNECTED");

		if (spim->ENABLE == 7) {
			shell_print(sh, "  >>> SPI peripheral is ACTIVE (owns pins)");
		} else {
			shell_print(sh, "  >>> SPI peripheral is DISABLED (pins released)");
		}
	} else {
		shell_print(sh, "SPI3 Device: NOT READY or unavailable");
	}
	#else
	shell_print(sh, "QSPI Peripheral: (nRF5340 - not implemented in this test)");
	#endif

	shell_print(sh, "");
	shell_print(sh, "========================================");
	shell_print(sh, "  GPIO Port Registers (Global)");
	shell_print(sh, "========================================");
	shell_print(sh, "  IN  = 0x%08X (all pins input state)", gpio_port->IN);
	shell_print(sh, "  OUT = 0x%08X (all pins output latch)", gpio_port->OUT);
	shell_print(sh, "");

	shell_print(sh, "========================================");
	shell_print(sh, "  Summary & Interpretation");
	shell_print(sh, "========================================");
	shell_print(sh, "TRISTATED pins should show:");
	shell_print(sh, "  - DIR=Input, INPUT=Disconnected or Connected with PULL");
	shell_print(sh, "  - SPI peripheral PSEL registers show DISCONNECTED");
	shell_print(sh, "");
	shell_print(sh, "ACTIVE (owned) pins should show:");
	shell_print(sh, "  - SPI peripheral ENABLE=7 (enabled)");
	shell_print(sh, "  - PSEL registers point to correct pins (CONNECTED)");
	shell_print(sh, "  - GPIO DIR may be Input (peripheral controls it)");
	shell_print(sh, "");

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
	const struct device *gpio_dev2 = spi_bus_get_gpio_dev();

	shell_print(sh, "SPI Bus FSM Status");
	shell_print(sh, "==================");
	shell_print(sh, "");

	int state = spi_bus_get_fsm_state();
	shell_print(sh, "Current FSM State: %s", spi_bus_fsm_state_to_string(state));
	shell_print(sh, "");

	bool req_state = gpio_pin_get(gpio_dev2, spi_bus_get_bus_request_pin());
	bool grant_state = gpio_pin_get(gpio_dev2, spi_bus_get_bus_grant_pin());

	shell_print(sh, "GPIO Pin States:");
	shell_print(sh, "  BUS_REQUEST (P0.%d): %s", spi_bus_get_bus_request_pin(), req_state ? "HIGH" : "LOW");
	shell_print(sh, "  BUS_GRANT   (P0.%d): %s", spi_bus_get_bus_grant_pin(), grant_state ? "HIGH" : "LOW");
	shell_print(sh, "");

	bool other_detected, in_transaction;
	spi_bus_get_transaction_state(&other_detected, &in_transaction);

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

	int ret = spi_bus_simulate_grant_event(value);

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

	int ret = spi_bus_simulate_request_event(value);

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

#endif /* SPI_BUS_SHARE_SHELL */
