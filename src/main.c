#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "lte_helper.h"

LOG_MODULE_REGISTER(main);

int main(void)
{
        printk("nRF9151 test running\n");
        //modem_configure();
        while (1) {
        // LOG_INF("6");
        // //printk("running\n");
        // k_msleep(1000);
        // LOG_INF("7");
        // k_msleep(1000);
        // LOG_INF("8");
        // k_msleep(1000);
        // LOG_INF("9");
        // k_msleep(1000);
        // LOG_INF("10");
        k_msleep(1000);
        }
        return 0;
}
