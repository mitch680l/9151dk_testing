/*
 * Flash Test Module for mcuboot_secondary partition
 *
 * This module provides functionality to test writing to and reading from
 * the mcuboot_secondary partition in external flash.
 */

#include "flash_test.h"
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/storage/flash_map.h>
#include <string.h>

LOG_MODULE_REGISTER(flash_test, LOG_LEVEL_INF);

#define TEST_PATTERN_SIZE 256
#define TEST_WRITE_SIZE 4096  /* Write 4KB chunks */
#define TEST_TOTAL_SIZE (16 * 1024)  /* Test with 16KB total */

/**
 * @brief Run comprehensive flash write/read test
 */
int flash_test_run(void)
{
    const struct flash_area *fa;
    int err;
    uint8_t write_buf[TEST_PATTERN_SIZE];
    uint8_t read_buf[TEST_PATTERN_SIZE];

    LOG_INF("=== Starting Flash Test on mcuboot_secondary ===");

    /* Open the mcuboot_secondary partition */
    err = flash_area_open(FIXED_PARTITION_ID(mcuboot_secondary), &fa);
    if (err) {
        LOG_ERR("Failed to open mcuboot_secondary partition: %d", err);
        return err;
    }

    LOG_INF("Partition opened successfully:");
    LOG_INF("  Device: %s", fa->fa_dev->name);
    LOG_INF("  Offset: 0x%lx", (unsigned long)fa->fa_off);
    LOG_INF("  Size: 0x%zx (%zu bytes)", fa->fa_size, fa->fa_size);

    /* Erase the test region */
    LOG_INF("Erasing test region (first %d bytes)...", TEST_TOTAL_SIZE);
    err = flash_area_erase(fa, 0, TEST_TOTAL_SIZE);
    if (err) {
        LOG_ERR("Flash erase failed: %d", err);
        flash_area_close(fa);
        return err;
    }
    LOG_INF("Erase complete");

    /* Verify erased state (should be 0xFF) */
    LOG_INF("Verifying erased state...");
    err = flash_area_read(fa, 0, read_buf, TEST_PATTERN_SIZE);
    if (err) {
        LOG_ERR("Flash read after erase failed: %d", err);
        flash_area_close(fa);
        return err;
    }

    bool erase_ok = true;
    for (int i = 0; i < TEST_PATTERN_SIZE; i++) {
        if (read_buf[i] != 0xFF) {
            LOG_WRN("Byte %d not erased: 0x%02x (expected 0xFF)", i, read_buf[i]);
            erase_ok = false;
            if (i > 10) break;  /* Don't spam too many messages */
        }
    }

    if (erase_ok) {
        LOG_INF("Erase verification PASSED - all bytes are 0xFF");
    } else {
        LOG_ERR("Erase verification FAILED - some bytes not 0xFF");
    }

    /* Create test pattern - repeating sequence with offset markers */
    LOG_INF("Creating test pattern...");
    for (int i = 0; i < TEST_PATTERN_SIZE; i++) {
        write_buf[i] = (uint8_t)(i & 0xFF);
    }

    /* Write pattern to multiple locations */
    LOG_INF("Writing test pattern to flash...");
    for (int offset = 0; offset < TEST_TOTAL_SIZE; offset += TEST_PATTERN_SIZE) {
        /* Modify pattern slightly for each write to verify location */
        write_buf[0] = (uint8_t)(offset >> 8);
        write_buf[1] = (uint8_t)(offset & 0xFF);

        err = flash_area_write(fa, offset, write_buf, TEST_PATTERN_SIZE);
        if (err) {
            LOG_ERR("Flash write failed at offset 0x%x: %d", offset, err);
            flash_area_close(fa);
            return err;
        }

        if (offset % 1024 == 0) {
            LOG_INF("  Written %d / %d bytes", offset + TEST_PATTERN_SIZE, TEST_TOTAL_SIZE);
        }
    }
    LOG_INF("Write complete");

    /* Read back and verify */
    LOG_INF("Reading back and verifying data...");
    int errors = 0;
    for (int offset = 0; offset < TEST_TOTAL_SIZE; offset += TEST_PATTERN_SIZE) {
        err = flash_area_read(fa, offset, read_buf, TEST_PATTERN_SIZE);
        if (err) {
            LOG_ERR("Flash read failed at offset 0x%x: %d", offset, err);
            flash_area_close(fa);
            return err;
        }

        /* Recreate expected pattern for this offset */
        write_buf[0] = (uint8_t)(offset >> 8);
        write_buf[1] = (uint8_t)(offset & 0xFF);
        for (int i = 2; i < TEST_PATTERN_SIZE; i++) {
            write_buf[i] = (uint8_t)(i & 0xFF);
        }

        /* Compare */
        if (memcmp(read_buf, write_buf, TEST_PATTERN_SIZE) != 0) {
            LOG_ERR("Data mismatch at offset 0x%x", offset);
            for (int i = 0; i < TEST_PATTERN_SIZE; i++) {
                if (read_buf[i] != write_buf[i]) {
                    LOG_ERR("  Byte %d: expected 0x%02x, got 0x%02x",
                           i, write_buf[i], read_buf[i]);
                    errors++;
                    if (errors > 10) break;  /* Limit error output */
                }
            }
            if (errors > 10) break;
        }
    }

    if (errors == 0) {
        LOG_INF("Verification PASSED - all data matches!");
    } else {
        LOG_ERR("Verification FAILED - %d errors found", errors);
    }

    /* Display first 32 bytes for inspection */
    err = flash_area_read(fa, 0, read_buf, 32);
    if (err == 0) {
        LOG_INF("First 32 bytes of flash:");
        LOG_HEXDUMP_INF(read_buf, 32, "Flash content:");
    }

    /* Test MCUboot magic pattern write/read */
    LOG_INF("Testing MCUboot magic pattern...");
    uint32_t mcuboot_magic = 0x96f3b83d;
    uint32_t read_magic;

    /* Erase first page again for clean test */
    err = flash_area_erase(fa, 0, 4096);
    if (err) {
        LOG_ERR("Failed to erase for MCUboot test: %d", err);
    } else {
        err = flash_area_write(fa, 0, &mcuboot_magic, sizeof(mcuboot_magic));
        if (err) {
            LOG_ERR("Failed to write MCUboot magic: %d", err);
        } else {
            err = flash_area_read(fa, 0, &read_magic, sizeof(read_magic));
            if (err) {
                LOG_ERR("Failed to read MCUboot magic: %d", err);
            } else {
                if (read_magic == mcuboot_magic) {
                    LOG_INF("MCUboot magic test PASSED (0x%08x)", read_magic);
                } else {
                    LOG_ERR("MCUboot magic test FAILED: wrote 0x%08x, read 0x%08x",
                           mcuboot_magic, read_magic);
                }
            }
        }
    }

    flash_area_close(fa);

    LOG_INF("=== Flash Test Complete ===");
    if (errors == 0) {
        LOG_INF("Result: SUCCESS - Flash read/write working correctly");
        return 0;
    } else {
        LOG_ERR("Result: FAILURE - Flash read/write has issues");
        return -EIO;
    }
}

/**
 * @brief Shell command to run flash test
 */
static int cmd_flash_test(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    shell_print(sh, "Starting flash test...");

    int err = flash_test_run();
    if (err) {
        shell_error(sh, "Flash test failed with error: %d", err);
        return err;
    }

    shell_print(sh, "Flash test completed successfully!");
    return 0;
}

/**
 * @brief Shell command to dump flash content
 */
static int cmd_flash_dump(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_error(sh, "Usage: flashtest dump <bytes>");
        shell_print(sh, "Example: flashtest dump 256");
        return -EINVAL;
    }

    int bytes = atoi(argv[1]);
    if (bytes <= 0 || bytes > 4096) {
        shell_error(sh, "Bytes must be between 1 and 4096");
        return -EINVAL;
    }

    const struct flash_area *fa;
    int err = flash_area_open(FIXED_PARTITION_ID(mcuboot_secondary), &fa);
    if (err) {
        shell_error(sh, "Failed to open partition: %d", err);
        return err;
    }

    uint8_t *buf = k_malloc(bytes);
    if (!buf) {
        shell_error(sh, "Failed to allocate buffer");
        flash_area_close(fa);
        return -ENOMEM;
    }

    err = flash_area_read(fa, 0, buf, bytes);
    if (err) {
        shell_error(sh, "Failed to read flash: %d", err);
        k_free(buf);
        flash_area_close(fa);
        return err;
    }

    shell_print(sh, "First %d bytes of mcuboot_secondary:", bytes);
    shell_hexdump(sh, buf, bytes);

    k_free(buf);
    flash_area_close(fa);

    return 0;
}

/**
 * @brief Shell command to erase flash partition
 */
static int cmd_flash_erase(const struct shell *sh, size_t argc, char **argv)
{
    shell_print(sh, "Erasing mcuboot_secondary partition...");

    const struct flash_area *fa;
    int err = flash_area_open(FIXED_PARTITION_ID(mcuboot_secondary), &fa);
    if (err) {
        shell_error(sh, "Failed to open partition: %d", err);
        return err;
    }

    shell_print(sh, "Erasing %zu bytes...", fa->fa_size);
    err = flash_area_erase(fa, 0, fa->fa_size);
    if (err) {
        shell_error(sh, "Erase failed: %d", err);
        flash_area_close(fa);
        return err;
    }

    flash_area_close(fa);
    shell_print(sh, "Erase complete");

    return 0;
}

/* Shell command tree */
SHELL_STATIC_SUBCMD_SET_CREATE(flashtest_cmds,
    SHELL_CMD_ARG(run, NULL, "Run flash write/read test", cmd_flash_test, 1, 0),
    SHELL_CMD_ARG(dump, NULL, "Dump flash content\nUsage: flashtest dump <bytes>",
                  cmd_flash_dump, 2, 0),
    SHELL_CMD_ARG(erase, NULL, "Erase mcuboot_secondary partition",
                  cmd_flash_erase, 1, 0),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(flashtest, &flashtest_cmds, "Flash test commands", NULL);
