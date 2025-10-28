#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>


LOG_MODULE_REGISTER(main);

int main(void)
{
        printk("nRF9151 test running\n");
        while (1) {
        LOG_INF("1");
        //printk("running\n");
        k_msleep(10);
        LOG_INF("2");
        k_msleep(10);
        LOG_INF("3");
        k_msleep(10);
        LOG_INF("4");
        k_msleep(10);
        LOG_INF("5");
        }
        return 0;
}
