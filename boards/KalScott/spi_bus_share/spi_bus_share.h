#ifndef SPI_BUS_SHARE_H
#define SPI_BUS_SHARE_H

#include <zephyr/kernel.h>

/**
 * @brief SPI bus owner enumeration
 *
 * Defines which microprocessor currently owns the SPI bus
 */
enum spi_bus_owner {
	SPI_BUS_OWNER_THIS_MCU = 0,  /* This microprocessor owns the bus */
	SPI_BUS_OWNER_OTHER_MCU = 1, /* Other microprocessor owns the bus */
	SPI_BUS_OWNER_NONE = 2       /* Bus is tristated, no owner */
};

/**
 * @brief Release SPI bus pins and configure them as high-impedance
 *
 * This function tristates all SPI bus pins (SCK, MOSI, MISO, CS) to allow
 * another microprocessor to control the shared external memory.
 *
 * @return 0 on success, negative errno code on failure
 */
int spi_bus_tristate(void);

/**
 * @brief Reclaim SPI bus pins and configure them for SPI operation
 *
 * This function reconfigures the SPI bus pins from high-impedance state
 * back to their normal SPI function, allowing this microprocessor to
 * access the shared external memory.
 *
 * @return 0 on success, negative errno code on failure
 */
int spi_bus_reclaim(void);

/**
 * @brief Get the current owner of the SPI bus
 *
 * @return Current bus owner (SPI_BUS_OWNER_THIS_MCU, SPI_BUS_OWNER_OTHER_MCU, or SPI_BUS_OWNER_NONE)
 */
enum spi_bus_owner spi_bus_get_owner(void);

/**
 * @brief Initialize the SPI bus sharing module
 *
 * This function initializes the GPIO pins and state tracking for
 * SPI bus sharing between microprocessors.
 *
 * @return 0 on success, negative errno code on failure
 */
int spi_bus_share_init(void);

/**
 * @brief Request bus ownership via FSM
 *
 * Posts a REQUEST_BUS event to the FSM. The FSM will assert the BUS_REQUEST
 * GPIO line and wait for the other MCU to grant access.
 *
 * Level-based protocol:
 * - Requester asserts REQUEST (HIGH)
 * - Owner sets GRANT HIGH (acknowledging, but may still be busy)
 * - Owner completes work, tristates bus, pulls GRANT LOW (handover ready)
 * - Requester detects falling edge, takes ownership
 *
 * This allows owner to indefinitely block during critical operations.
 * This is non-blocking - use spi_bus_get_fsm_state() to check status.
 *
 * @return 0 on success (request posted), negative errno code on failure
 */
int spi_bus_request(void);

/**
 * @brief Release bus ownership via FSM
 *
 * Posts a RELEASE_BUS event to the FSM. The FSM will tristate the SPI pins
 * and deassert BUS_REQUEST to give the bus to the other MCU.
 * This is non-blocking - use spi_bus_get_fsm_state() to check status.
 *
 * @return 0 on success (release posted), negative errno code on failure
 */
int spi_bus_release(void);

/**
 * @brief Recover from FSM error state
 *
 * Attempts to recover the FSM from an error state by checking the
 * BUS_GRANT line and reconfiguring accordingly.
 *
 * @return 0 on success (recovery posted), negative errno code on failure
 */
int spi_bus_error_recovery(void);

/**
 * @brief Get current FSM state
 *
 * Returns the current state of the bus arbitration FSM.
 * States: 0=IDLE_OWNER, 1=IDLE_NOT_OWNER, 2=REQUESTING, 3=RELEASING, 4=ERROR
 *
 * @return Current FSM state (0-4)
 */
int spi_bus_get_fsm_state(void);

/**
 * @brief Get FSM statistics
 *
 * Retrieves error count and state transition count from the FSM.
 *
 * @param error_count Pointer to store error count (can be NULL)
 * @param transition_count Pointer to store transition count (can be NULL)
 * @return 0 on success
 */
int spi_bus_get_fsm_stats(uint32_t *error_count, uint32_t *transition_count);

/**
 * @brief Acquire bus for transaction (blocking)
 *
 * Requests bus ownership and blocks until granted.
 * Two modes of operation:
 *
 * Mode 1: use_transaction_lock = false (Quick handoff mode)
 * - Blocks until bus is acquired
 * - No lease time - other MCU can request immediately
 * - Bus given up quickly/instantly when requested
 * - Use for quick operations where you want to yield immediately
 *
 * Mode 2: use_transaction_lock = true (Transaction lock mode)
 * - Blocks until bus is acquired
 * - Sets transaction lock to hold bus during critical operations
 * - If other MCU requests during transaction:
 *   1. This MCU sets GRANT HIGH (acknowledging request)
 *   2. This MCU continues holding GRANT HIGH indefinitely
 *   3. Only when spi_bus_release_transaction() is called does handover occur
 * - Use for critical operations that must complete atomically
 *
 * @param use_transaction_lock If true, blocks indefinitely until spi_bus_release_transaction()
 * @return 0 on success, negative errno on failure
 */
int spi_bus_acquire_blocking(bool use_transaction_lock);

/**
 * @brief Release bus after transaction complete
 *
 * Clears transaction flag and allows other MCU to request bus.
 *
 * With level-based protocol:
 * - If another MCU is waiting (GRANT is HIGH), this triggers the handover:
 *   1. Completes deferred grant work
 *   2. Tristates SPI bus
 *   3. Pulls GRANT LOW (falling edge signals handover ready)
 *   4. Other MCU detects falling edge and takes ownership
 * - If no MCU waiting, just clears the transaction flag
 *
 * Does not immediately release ownership - just makes it available for request.
 *
 * @return 0 on success, negative errno on failure
 */
int spi_bus_release_transaction(void);

int spi_bus_shutdown(void);

#endif /* SPI_BUS_SHARE_H */
