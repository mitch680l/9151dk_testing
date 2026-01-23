#ifndef SPI_BUS_SHARE_INTERNAL_H
#define SPI_BUS_SHARE_INTERNAL_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include "spi_bus_share.h"

/**
 * @brief Internal header for SPI bus sharing module
 *
 * This header exposes internal functions and data needed by the shell commands.
 * Only include this header in spi_bus_share.c and spi_bus_share_shell.c
 */

/**
 * @brief Get GPIO device
 */
const struct device *spi_bus_get_gpio_dev(void);

/**
 * @brief Get SPI device
 */
const struct device *spi_bus_get_spi_dev(void);

/**
 * @brief Get bus type string
 */
const char *spi_bus_get_type(void);

/**
 * @brief Get pin numbers
 */
uint8_t spi_bus_get_sck_pin(void);
uint8_t spi_bus_get_mosi_pin(void);
uint8_t spi_bus_get_miso_pin(void);
uint8_t spi_bus_get_cs_pin(void);
uint8_t spi_bus_get_cs1_pin(void);
uint8_t spi_bus_get_cs2_pin(void);
uint8_t spi_bus_get_bus_request_pin(void);
uint8_t spi_bus_get_bus_grant_pin(void);

/**
 * @brief Get FSM state as string
 */
const char *spi_bus_fsm_state_to_string(int state);

/**
 * @brief Get transaction state
 */
void spi_bus_get_transaction_state(bool *other_mcu_detected, bool *transaction_in_progress);

/**
 * @brief Simulate grant event (for testing)
 */
int spi_bus_simulate_grant_event(int value);

/**
 * @brief Simulate request event (for testing)
 */
int spi_bus_simulate_request_event(int value);

/**
 * @brief Configure both REQUEST and GRANT pins as INPUT for idle state
 */
int configure_pins_as_idle(void);

/**
 * @brief Configure REQUEST pin as OUTPUT and GRANT pin as INPUT
 */
int configure_pins_as_requester(void);

/**
 * @brief Configure REQUEST pin as INPUT and GRANT pin as OUTPUT
 */
int configure_pins_as_owner(void);

/**
 * @brief CS Mirror ISR callback (nRF9151 only)
 */
void cs_mirror_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

/**
 * @brief Enable CS mirroring (nRF9151 only)
 */
int cs_mirror_enable(void);

/**
 * @brief Disable CS mirroring (nRF9151 only)
 */
int cs_mirror_disable(void);

#endif /* SPI_BUS_SHARE_INTERNAL_H */
