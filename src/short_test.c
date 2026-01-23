#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/shell/shell.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "short_test.h"


#define SHORT_TEST_USE_PULLDOWN

static const struct shell *active_shell;

#if defined(CONFIG_SOC_NRF5340_CPUAPP)
    #define MCU_IS_NRF5340 1
    #define MCU_NAME "nRF5340"
    #define GPIO0_BASE 0x50842500UL
    #define GPIO1_BASE 0x50842800UL
#elif defined(CONFIG_SOC_SERIES_NRF91X) || defined(CONFIG_SOC_NRF9151_LACA)
    #define MCU_IS_NRF9151 1
    #define MCU_NAME "nRF9151"
    #if defined(CONFIG_TRUSTED_EXECUTION_NONSECURE)
        #define GPIO0_BASE 0x40842500UL
    #else
        #define GPIO0_BASE 0x50842500UL
    #endif
#else
    #error "Unsupported MCU - must be nRF5340 or nRF9151"
#endif

#if defined(MCU_IS_NRF5340)
#include "uart_usb_bridge.h"
#endif

#define GPIO_OUT_OFFSET      0x004
#define GPIO_OUTSET_OFFSET   0x008
#define GPIO_OUTCLR_OFFSET   0x00C
#define GPIO_IN_OFFSET       0x010
#define GPIO_DIR_OFFSET      0x014
#define GPIO_DIRSET_OFFSET   0x018
#define GPIO_DIRCLR_OFFSET   0x01C
#define GPIO_PIN_CNF_OFFSET  0x200

#define PIN_CNF_DIR_INPUT    (0UL << 0)
#define PIN_CNF_DIR_OUTPUT   (1UL << 0)
#define PIN_CNF_INPUT_CONN   (0UL << 1)
#define PIN_CNF_INPUT_DISC   (1UL << 1)
#define PIN_CNF_PULL_NONE    (0UL << 2)
#define PIN_CNF_PULL_DOWN    (1UL << 2)
#define PIN_CNF_PULL_UP      (3UL << 2)

#define REG32(addr) (*(volatile uint32_t *)(addr))

#define GPIO0_OUT      REG32(GPIO0_BASE + GPIO_OUT_OFFSET)
#define GPIO0_OUTSET   REG32(GPIO0_BASE + GPIO_OUTSET_OFFSET)
#define GPIO0_OUTCLR   REG32(GPIO0_BASE + GPIO_OUTCLR_OFFSET)
#define GPIO0_IN       REG32(GPIO0_BASE + GPIO_IN_OFFSET)
#define GPIO0_DIR      REG32(GPIO0_BASE + GPIO_DIR_OFFSET)
#define GPIO0_DIRSET   REG32(GPIO0_BASE + GPIO_DIRSET_OFFSET)
#define GPIO0_DIRCLR   REG32(GPIO0_BASE + GPIO_DIRCLR_OFFSET)
#define GPIO0_PIN_CNF(n) REG32(GPIO0_BASE + GPIO_PIN_CNF_OFFSET + ((n) * 4))

#if defined(MCU_IS_NRF5340)
#define GPIO1_OUT      REG32(GPIO1_BASE + GPIO_OUT_OFFSET)
#define GPIO1_OUTSET   REG32(GPIO1_BASE + GPIO_OUTSET_OFFSET)
#define GPIO1_OUTCLR   REG32(GPIO1_BASE + GPIO_OUTCLR_OFFSET)
#define GPIO1_IN       REG32(GPIO1_BASE + GPIO_IN_OFFSET)
#define GPIO1_DIR      REG32(GPIO1_BASE + GPIO_DIR_OFFSET)
#define GPIO1_DIRSET   REG32(GPIO1_BASE + GPIO_DIRSET_OFFSET)
#define GPIO1_DIRCLR   REG32(GPIO1_BASE + GPIO_DIRCLR_OFFSET)
#define GPIO1_PIN_CNF(n) REG32(GPIO1_BASE + GPIO_PIN_CNF_OFFSET + ((n) * 4))
#endif

#define GPIO0_PIN_COUNT  32

#if defined(MCU_IS_NRF5340)
    #define GPIO1_PIN_COUNT  16
#else
    #define GPIO1_PIN_COUNT  0
#endif

static void cfg_pin_input_nopull(int port, int pin)
{
    uint32_t cnf = PIN_CNF_DIR_INPUT | PIN_CNF_INPUT_CONN | PIN_CNF_PULL_NONE;
    if (port == 0) {
        GPIO0_PIN_CNF(pin) = cnf;
    }
#if defined(MCU_IS_NRF5340)
    else {
        GPIO1_PIN_CNF(pin) = cnf;
    }
#endif
}

static void cfg_pin_input_pulldown(int port, int pin)
{
    uint32_t cnf = PIN_CNF_DIR_INPUT | PIN_CNF_INPUT_CONN | PIN_CNF_PULL_DOWN;
    if (port == 0) {
        GPIO0_PIN_CNF(pin) = cnf;
    }
#if defined(MCU_IS_NRF5340)
    else {
        GPIO1_PIN_CNF(pin) = cnf;
    }
#endif
}

static void cfg_pin_input(int port, int pin)
{
#if defined(SHORT_TEST_USE_PULLDOWN)
    cfg_pin_input_pulldown(port, pin);
#else
    cfg_pin_input_nopull(port, pin);
#endif
}

static void cfg_pin_output(int port, int pin)
{
    uint32_t cnf = PIN_CNF_DIR_OUTPUT | PIN_CNF_INPUT_CONN | PIN_CNF_PULL_NONE;
    if (port == 0) {
        GPIO0_PIN_CNF(pin) = cnf;
    }
#if defined(MCU_IS_NRF5340)
    else {
        GPIO1_PIN_CNF(pin) = cnf;
    }
#endif
}

static void set_pin_high(int port, int pin)
{
    if (port == 0) {
        GPIO0_OUTSET = (1UL << pin);
    }
#if defined(MCU_IS_NRF5340)
    else {
        GPIO1_OUTSET = (1UL << pin);
    }
#endif
}

static void set_pin_low(int port, int pin)
{
    if (port == 0) {
        GPIO0_OUTCLR = (1UL << pin);
    }
#if defined(MCU_IS_NRF5340)
    else {
        GPIO1_OUTCLR = (1UL << pin);
    }
#endif
}

static uint32_t read_port(int port)
{
    if (port == 0) {
        return GPIO0_IN;
    }
#if defined(MCU_IS_NRF5340)
    else {
        return GPIO1_IN & 0xFFFF;
    }
#endif
    return 0;
}

static uint32_t read_gpio_port_ex(int port, uint8_t pin_count, int skip_pin)
{
    for (uint8_t pin = 0; pin < pin_count; pin++) {
        if (skip_pin >= 0 && pin == skip_pin) {
            continue;
        }
        cfg_pin_input(port, pin);
    }

    k_busy_wait(10);

    uint32_t states = read_port(port);

    if (pin_count < 32) {
        states &= ((1UL << pin_count) - 1);
    }

    return states;
}

static uint32_t read_gpio_port(int port, uint8_t pin_count)
{
    return read_gpio_port_ex(port, pin_count, -1);
}

/* Baselines for no-pull mode */
#if defined(MCU_IS_NRF9151)
#define BASELINE_NOPULL_P0 0xCC303FC0UL
#else
#define BASELINE_NOPULL_P0 0xE1859700UL
#define BASELINE_NOPULL_P1 0x0000FFF7UL
#endif

/* Baselines for pull-down mode */
#if defined(MCU_IS_NRF9151)
#define BASELINE_PULLDOWN_P0 0x003038C0UL
#else
#define BASELINE_PULLDOWN_P0 0x00000200UL
#define BASELINE_PULLDOWN_P1 0x00000004UL
#endif

/* Select active baselines based on mode */
#if defined(SHORT_TEST_USE_PULLDOWN)
#define BASELINE_P0 BASELINE_PULLDOWN_P0
#if defined(MCU_IS_NRF5340)
#define BASELINE_P1 BASELINE_PULLDOWN_P1
#endif
#else
#define BASELINE_P0 BASELINE_NOPULL_P0
#if defined(MCU_IS_NRF5340)
#define BASELINE_P1 BASELINE_NOPULL_P1
#endif
#endif

static void format_pin_states(uint32_t states, uint8_t pin_count, char *buf, size_t buf_size)
{
    int pos = 0;

    for (int i = pin_count - 1; i >= 0 && pos < buf_size - 1; i--) {
        buf[pos++] = (states & (1UL << i)) ? '1' : '0';
        if (i > 0 && (i % 8) == 0 && pos < buf_size - 1) {
            buf[pos++] = ' ';
        }
    }
    buf[pos] = '\0';
}

static void format_drive_states(uint32_t states, uint8_t pin_count, uint32_t baseline,
                                 int driven_pin, char *buf, size_t buf_size)
{
    int pos = 0;

    for (int i = pin_count - 1; i >= 0 && pos < buf_size - 1; i--) {
        int is_high = (states & (1UL << i)) != 0;
        int is_baseline = (baseline & (1UL << i)) != 0;
        int is_driven = (i == driven_pin);

        if (is_driven) {
            buf[pos++] = 'D';
        } else if (is_baseline && is_high) {
            buf[pos++] = 'X';
        } else if (is_baseline && !is_high) {
            buf[pos++] = '?';
        } else if (is_high) {
            buf[pos++] = '1';
        } else {
            buf[pos++] = '0';
        }

        if (i > 0 && (i % 8) == 0 && pos < buf_size - 1) {
            buf[pos++] = ' ';
        }
    }
    buf[pos] = '\0';
}

int short_test_read_pins(const char *context)
{
    uint32_t gpio0_states;
    char pin_str[64];
    const char *ctx = (context && context[0]) ? context : "shell";

    for (int pin = 0; pin < GPIO0_PIN_COUNT; pin++) {
        cfg_pin_input(0, pin);
    }

#if defined(MCU_IS_NRF5340)
    for (int pin = 0; pin < GPIO1_PIN_COUNT; pin++) {
        cfg_pin_input(1, pin);
    }
#endif

    k_busy_wait(10);

    gpio0_states = GPIO0_IN;
    format_drive_states(gpio0_states, GPIO0_PIN_COUNT, BASELINE_P0, -1, pin_str, sizeof(pin_str));
    printk("[READOUT] %s %s P0: %s\n", ctx, MCU_NAME, pin_str);

#if defined(MCU_IS_NRF5340)
    uint32_t gpio1_states;
    gpio1_states = GPIO1_IN & 0xFFFF;
    format_drive_states(gpio1_states, GPIO1_PIN_COUNT, BASELINE_P1, -1, pin_str, sizeof(pin_str));
    printk("[READOUT] %s %s P1: %s\n", ctx, MCU_NAME, pin_str);
#endif

    return 0;
}

#if defined(MCU_IS_NRF5340)
int short_test_request_remote_pins(const char *context)
{
    char cmd[64];
    const char *ctx = (context && context[0]) ? context : "shell";

    snprintf(cmd, sizeof(cmd), "short_test read_pins %s\r\n", ctx);

    int ret = uart_bridge_send(cmd, strlen(cmd));
    if (ret < 0) {
        printk("[ERROR] Failed to send command to nRF9151: %d\n", ret);
        return ret;
    }

    return 0;
}

int short_test_read_all_pins(void)
{
    int ret;

    ret = short_test_read_pins("shell");
    if (ret != 0) {
        return ret;
    }

    ret = short_test_request_remote_pins("shell");
    if (ret != 0) {
        return ret;
    }

    return 0;
}

int short_test_request_remote_drive(int port, int pin)
{
    char cmd[64];

    snprintf(cmd, sizeof(cmd), "short_test drive_pin %d %d\r\n", port, pin);

    int ret = uart_bridge_send(cmd, strlen(cmd));
    if (ret < 0) {
        printk("[ERROR] Failed to send drive command to nRF9151: %d\n", ret);
        return ret;
    }

    return 0;
}
#endif

static void drive_pin_start(int port, int pin)
{
    cfg_pin_output(port, pin);
    set_pin_high(port, pin);
    k_busy_wait(100);
}

static void drive_pin_stop(int port, int pin)
{
    set_pin_low(port, pin);
    cfg_pin_input(port, pin);
}

static void read_and_print_local_while_driving(int port, int pin)
{
    uint32_t gpio0_states;
    char pin_str[64];
    int skip_p0 = (port == 0) ? pin : -1;
    int skip_p1 = (port == 1) ? pin : -1;
    int driven_p0 = (port == 0) ? pin : -1;
    int driven_p1 = (port == 1) ? pin : -1;

    gpio0_states = read_gpio_port_ex(0, GPIO0_PIN_COUNT, skip_p0);
    format_drive_states(gpio0_states, GPIO0_PIN_COUNT, BASELINE_P0, driven_p0, pin_str, sizeof(pin_str));
    if (active_shell) {
        shell_fprintf(active_shell, SHELL_NORMAL, "[DRIVE] %s P%d.%02d HIGH P0: %s\n", MCU_NAME, port, pin, pin_str);
    } else {
        printk("[DRIVE] %s P%d.%02d HIGH P0: %s\n", MCU_NAME, port, pin, pin_str);
    }

#if defined(MCU_IS_NRF5340)
    uint32_t gpio1_states;
    gpio1_states = read_gpio_port_ex(1, GPIO1_PIN_COUNT, skip_p1);
    format_drive_states(gpio1_states, GPIO1_PIN_COUNT, BASELINE_P1, driven_p1, pin_str, sizeof(pin_str));
    if (active_shell) {
        shell_fprintf(active_shell, SHELL_NORMAL, "[DRIVE] %s P%d.%02d HIGH P1: %s\n", MCU_NAME, port, pin, pin_str);
    } else {
        printk("[DRIVE] %s P%d.%02d HIGH P1: %s\n", MCU_NAME, port, pin, pin_str);
    }
#endif
}

static void drive_single_pin_test(int port, int pin)
{
    drive_pin_start(port, pin);
    read_and_print_local_while_driving(port, pin);
    drive_pin_stop(port, pin);
}

int short_test_drive_pin(int port, int pin)
{
    int max_pins = (port == 0) ? GPIO0_PIN_COUNT : GPIO1_PIN_COUNT;

    if (port < 0 || port > 1) {
        printk("[ERROR] Invalid port: %d\n", port);
        return -EINVAL;
    }

#if !defined(MCU_IS_NRF5340)
    if (port == 1) {
        printk("[ERROR] GPIO1 not available on %s\n", MCU_NAME);
        return -EINVAL;
    }
#endif

    if (pin < 0 || pin >= max_pins) {
        printk("[ERROR] Invalid pin: %d (max %d for port %d)\n", pin, max_pins - 1, port);
        return -EINVAL;
    }

    char ctx[32];
    snprintf(ctx, sizeof(ctx), "%s_P%d.%02d", MCU_NAME, port, pin);

    drive_pin_start(port, pin);
    read_and_print_local_while_driving(port, pin);

#if defined(MCU_IS_NRF5340)
    short_test_request_remote_pins(ctx);
#endif

    drive_pin_stop(port, pin);

    return 0;
}

int short_test_drive_all_local(void)
{
    for (int pin = 0; pin < GPIO0_PIN_COUNT; pin++) {
        drive_single_pin_test(0, pin);
    }

#if defined(MCU_IS_NRF5340)
    for (int pin = 0; pin < GPIO1_PIN_COUNT; pin++) {
        drive_single_pin_test(1, pin);
    }
#endif

    return 0;
}

#if defined(MCU_IS_NRF5340)
int short_test_drive_all_with_remote(void)
{
    char ctx[32];

    for (int pin = 0; pin < GPIO0_PIN_COUNT; pin++) {
        snprintf(ctx, sizeof(ctx), "%s_P0.%02d", MCU_NAME, pin);
        drive_pin_start(0, pin);
        read_and_print_local_while_driving(0, pin);
        short_test_request_remote_pins(ctx);
        k_busy_wait(50000);
        drive_pin_stop(0, pin);
    }

    for (int pin = 0; pin < GPIO1_PIN_COUNT; pin++) {
        snprintf(ctx, sizeof(ctx), "%s_P1.%02d", MCU_NAME, pin);
        drive_pin_start(1, pin);
        read_and_print_local_while_driving(1, pin);
        short_test_request_remote_pins(ctx);
        k_busy_wait(50000);
        drive_pin_stop(1, pin);
    }

    uart_bridge_send("short_test drive_all\r\n", 22);

    return 0;
}
#endif

static int cmd_read_pins(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(sh);
    const char *context = (argc >= 2) ? argv[1] : "shell";
    short_test_read_pins(context);
    return 0;
}

static int cmd_drive_pin(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 3) {
        shell_print(sh, "Usage: short_test drive_pin <port> <pin>");
        return -EINVAL;
    }
    int port = atoi(argv[1]);
    int pin = atoi(argv[2]);
    short_test_drive_pin(port, pin);
    return 0;
}

static int cmd_drive_all(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    active_shell = sh;
    short_test_drive_all_local();
    active_shell = NULL;
    return 0;
}

static int cmd_set_pin(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 4) {
        shell_print(sh, "Usage: short_test set <port> <pin> <0|1>");
        return -EINVAL;
    }
    int port = atoi(argv[1]);
    int pin = atoi(argv[2]);
    int state = atoi(argv[3]);

    int max_pins = (port == 0) ? GPIO0_PIN_COUNT : GPIO1_PIN_COUNT;
    if (port < 0 || port > 1 || pin < 0 || pin >= max_pins) {
        shell_print(sh, "Invalid port/pin");
        return -EINVAL;
    }

    cfg_pin_output(port, pin);
    if (state) {
        set_pin_high(port, pin);
        shell_print(sh, "P%d.%02d -> HIGH", port, pin);
    } else {
        set_pin_low(port, pin);
        shell_print(sh, "P%d.%02d -> LOW", port, pin);
    }
    return 0;
}

static int cmd_release_pin(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 3) {
        shell_print(sh, "Usage: short_test release <port> <pin>");
        return -EINVAL;
    }
    int port = atoi(argv[1]);
    int pin = atoi(argv[2]);

    int max_pins = (port == 0) ? GPIO0_PIN_COUNT : GPIO1_PIN_COUNT;
    if (port < 0 || port > 1 || pin < 0 || pin >= max_pins) {
        shell_print(sh, "Invalid port/pin");
        return -EINVAL;
    }

    cfg_pin_input(port, pin);
    shell_print(sh, "P%d.%02d -> INPUT", port, pin);
    return 0;
}

#if defined(MCU_IS_NRF5340)
static int cmd_read_remote(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    short_test_request_remote_pins("shell");
    return 0;
}

static int cmd_read_all(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    short_test_read_all_pins();
    return 0;
}

static int cmd_drive_all_full(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    active_shell = sh;
    short_test_drive_all_with_remote();
    active_shell = NULL;
    return 0;
}
#endif

SHELL_STATIC_SUBCMD_SET_CREATE(sub_short_test,
    SHELL_CMD(read_pins, NULL, "Read GPIO pins on this MCU", cmd_read_pins),
    SHELL_CMD(drive_pin, NULL, "Drive single pin high and read all: <port> <pin>", cmd_drive_pin),
    SHELL_CMD(drive_all, NULL, "Drive each local pin and read", cmd_drive_all),
    SHELL_CMD(set, NULL, "Set pin state: <port> <pin> <0|1>", cmd_set_pin),
    SHELL_CMD(release, NULL, "Release pin to input: <port> <pin>", cmd_release_pin),
#if defined(MCU_IS_NRF5340)
    SHELL_CMD(read_remote, NULL, "Request GPIO pin read from nRF9151", cmd_read_remote),
    SHELL_CMD(read_all, NULL, "Read GPIO pins on both MCUs", cmd_read_all),
    SHELL_CMD(drive_all_full, NULL, "Drive test on both MCUs with cross-read", cmd_drive_all_full),
#endif
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(short_test, &sub_short_test, "Short test commands", NULL);
