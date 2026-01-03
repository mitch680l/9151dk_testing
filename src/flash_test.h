#ifndef FLASH_TEST_H
#define FLASH_TEST_H

#include <zephyr/kernel.h>

/**
 * @brief Run flash write/read test on mcuboot_secondary partition
 *
 * This test writes a known pattern to the mcuboot_secondary partition
 * in external flash and reads it back to verify the write operation.
 *
 * @return 0 on success, negative errno on failure
 */
int flash_test_run(void);

#endif /* FLASH_TEST_H */
