#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>

#define SPI_DEV_LABEL "SPI2_MCU_LINK"

const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi2));


static const struct spi_config spi_cfg = {
    .frequency = 1000000,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    .slave = 0,
    .cs = NULL,
};

static void spi_test_thread(void)
{
    if (!device_is_ready(spi_dev)) {
        printk("SPI2 device not ready\n");
        return;
    }

    uint8_t counter = 0;
    uint8_t tx_buf[4];
    uint8_t rx_buf[4];

    struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
    struct spi_buf rx = { .buf = rx_buf, .len = sizeof(rx_buf) };
    struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

    while (1) {
        tx_buf[0] = 0xA5;
        tx_buf[1] = 0x5A;
        tx_buf[2] = counter++;
        tx_buf[3] = 0xFF;

        int ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
        if (ret == 0) {
            printk("SPI2 TX: %02X %02X %02X %02X | RX: %02X %02X %02X %02X\n",
                   tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3],
                   rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
        } else {
            printk("SPI transceive failed: %d\n", ret);
        }

        k_sleep(K_MSEC(500));
    }
}

//K_THREAD_DEFINE(spi_test_tid, 1024, spi_test_thread, NULL, NULL, NULL,7, 0, 0);
