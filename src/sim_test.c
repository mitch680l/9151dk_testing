#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include "sim_test.h"

LOG_MODULE_REGISTER(sim_test, LOG_LEVEL_INF);

#define SIM_DETECT_PIN 24
#define SIM_SELECT_PIN 25

static const struct device *gpio_dev;
static struct gpio_callback sim_detect_cb_data;

bool sim_test_passed = false;


static void sim_detect_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	int val = gpio_pin_get(gpio_dev, SIM_DETECT_PIN);

	if (val < 0) {
		LOG_ERR("Failed to read SIM detect pin: %d", val);
		return;
	}

	if (val) {
		LOG_INF("SIM card in = HIGH");
	} else {
		LOG_INF("SIM card out = LOW");
	}
}


static void sim_test_read(void)
{
	int val = gpio_pin_get(gpio_dev, SIM_DETECT_PIN);

	if (val < 0) {
		LOG_ERR("Failed to read SIM detect pin: %d", val);
		return;
	}

	if (val) {
		LOG_INF("SIM detect pin P0.24: HIGH (SIM card inserted)");
	} else {
		LOG_INF("SIM detect pin P0.24: LOW (SIM card removed)");
	}
}


static int cmd_sim_read(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int val = gpio_pin_get(gpio_dev, SIM_DETECT_PIN);

	if (val < 0) {
		shell_error(sh, "Failed to read SIM detect pin: %d", val);
		return -EIO;
	}

	if (val) {
		shell_print(sh, "SIM detect pin P0.24: HIGH (SIM card inserted)");
	} else {
		shell_print(sh, "SIM detect pin P0.24: LOW (SIM card removed)");
	}

	return 0;
}


static int cmd_sim_select(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: SIM_SELECT <0|1>");
		return -EINVAL;
	}

	int sim_num = atoi(argv[1]);

	if (sim_num != 0 && sim_num != 1) {
		shell_error(sh, "Invalid SIM number. Must be 0 or 1");
		return -EINVAL;
	}

	shell_print(sh, "Selecting SIM %d", sim_num);

	int ret = gpio_pin_set(gpio_dev, SIM_SELECT_PIN, sim_num);
	if (ret < 0) {
		shell_error(sh, "Failed to set SIM select pin: %d", ret);
		LOG_ERR("Failed to set SIM select pin P0.25: %d", ret);
		return -EIO;
	}

	LOG_INF("SIM %d selected - P0.25 set to %s", sim_num, sim_num ? "HIGH" : "LOW");

	return 0;
}


void sim_test_case(void)
{
	int sim_select_val;
	int sim_detect_val;

	sim_test_passed = false;

	LOG_INF("=== Starting SIM Test Case ===");
	LOG_INF("Test assumes: SIM_SELECT = 1 (HIGH), SIM card plugged into device");


	sim_select_val = gpio_pin_get(gpio_dev, SIM_SELECT_PIN);
	if (sim_select_val < 0) {
		LOG_ERR("Failed to read SIM select pin: %d", sim_select_val);
		LOG_ERR("Test FAILED - Cannot read SIM select pin");
		return;
	}

	if (sim_select_val != 1) {
		LOG_WRN("SIM_SELECT is not HIGH, setting to 1...");
		int ret = gpio_pin_set(gpio_dev, SIM_SELECT_PIN, 1);
		if (ret < 0) {
			LOG_ERR("Failed to set SIM select pin: %d", ret);
			LOG_ERR("Test FAILED - Cannot set SIM select pin");
			return;
		}
		LOG_INF("SIM_SELECT set to HIGH");
	} else {
		LOG_INF("SIM_SELECT confirmed: HIGH");
	}


	sim_detect_val = gpio_pin_get(gpio_dev, SIM_DETECT_PIN);
	if (sim_detect_val < 0) {
		LOG_ERR("Failed to read SIM detect pin: %d", sim_detect_val);
		LOG_ERR("Test FAILED - Cannot read SIM detect pin");
		return;
	}

	LOG_INF("SIM detect pin P0.24: %s", sim_detect_val ? "HIGH" : "LOW");


	if (sim_detect_val == 1) {
		LOG_INF("Test PASSED - SIM card detected (HIGH)");
		sim_test_passed = true;
	} else {
		LOG_ERR("Test FAILED - SIM card not detected (LOW)");
		LOG_ERR("Please ensure SIM card is properly inserted into the device");
		sim_test_passed = false;
	}

	LOG_INF("=== SIM Test Case Complete: %s ===", sim_test_passed ? "PASSED" : "FAILED");
}


static int cmd_sim_test_case(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Running SIM test case...");
	sim_test_case();

	if (sim_test_passed) {
		shell_print(sh, "Result: PASSED");
	} else {
		shell_print(sh, "Result: FAILED");
	}

	return 0;
}


static int sim_test_init(void)
{
	int ret;

	gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	if (!device_is_ready(gpio_dev)) {
		LOG_ERR("GPIO device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure(gpio_dev, SIM_SELECT_PIN, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure SIM select pin: %d", ret);
		return ret;
	}
	LOG_INF("SIM select pin P0.25 initialized to HIGH (SIM 1 default)");


	ret = gpio_pin_configure(gpio_dev, SIM_DETECT_PIN, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure SIM detect pin: %d", ret);
		return ret;
	}


	ret = gpio_pin_interrupt_configure(gpio_dev, SIM_DETECT_PIN, GPIO_INT_EDGE_BOTH);
	if (ret < 0) {
		LOG_ERR("Failed to configure SIM detect interrupt: %d", ret);
		return ret;
	}


	gpio_init_callback(&sim_detect_cb_data, sim_detect_callback, BIT(SIM_DETECT_PIN));
	ret = gpio_add_callback(gpio_dev, &sim_detect_cb_data);
	if (ret < 0) {
		LOG_ERR("Failed to add SIM detect callback: %d", ret);
		return ret;
	}

	LOG_INF("SIM test module initialized - P0.24 (detect input), P0.25 (select output)");

	sim_test_read();

	return 0;
}

SYS_INIT(sim_test_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);


SHELL_CMD_REGISTER(sim_read, NULL, "Read SIM card detect state", cmd_sim_read);
SHELL_CMD_REGISTER(sim_select, NULL, "Select SIM card (0 or 1)", cmd_sim_select);
SHELL_CMD_REGISTER(sim_test_case, NULL, "Run SIM test case (assumes SIM 1 and card inserted)", cmd_sim_test_case);
