#include "spi_bus_share.h"
#include <zephyr/kernel.h>
#include <bootutil/image.h>
#include <bootutil/bootutil.h>
#include <bootutil/bootutil_public.h>
#include <zephyr/logging/log.h>
#include "bootutil/fault_injection_hardening.h" 
/* Flag to track if SPI bus has been initialized and acquired */
static bool spi_bus_initialized = false;

LOG_MODULE_REGISTER(mcu_spi_init, LOG_LEVEL_INF);
// /**
//  * @brief Hook called when MCUboot reads an image header
//  *
//  * This is called for each image slot during boot validation.
//  * We use this to initialize and acquire the SPI bus before any flash operations.
//  */
// int boot_read_image_header_hook(int img_index, int slot,
//                                  struct image_header *img_header)
//  {
//     return BOOT_HOOK_REGULAR;
// }

// /**
//  * @brief Hook called after an image has been validated
//  *
//  * This hook is called after MCUboot validates an image's signature and integrity.
//  */
// fih_ret boot_image_check_hook(int img_index, int slot)
// {
//     /* No special handling needed for image validation */
//     LOG_INF("Inside boot_image_check_hook");
//     FIH_RET(FIH_BOOT_HOOK_REGULAR);
// }

// /**
//  * @brief Hook called after a region has been copied during image swap
//  *
//  * This hook is called after MCUboot copies a region of flash during the swap process.
//  */
// int boot_copy_region_post_hook(int img_index, const struct flash_area *area,
//                                 size_t size)
// {
//     //LOG_INF("Inside boot_copy_region_post_hook");
//     /* No special handling needed after copy operations */
//     return 0;
// }

// /**
//  * @brief Hook called when performing a firmware update
//  *
//  * This hook is called before MCUboot performs the update operation.
//  */
// int boot_perform_update_hook(int img_index, struct image_header *img_head,
//                               const struct flash_area *area)
// {
//     LOG_INF("Inside boot_perform_update_hook");
//     /* No special handling needed for updates */
//     return 0;
// }

// /**
//  * @brief Hook called when reading swap state from primary slot
//  *
//  * This hook allows customization of how swap state is read from the primary slot.
//  */
// int boot_read_swap_state_primary_slot_hook(int image_index,
//                                             struct boot_swap_state *state)
// {
//     LOG_INF("Inside boot_read_swap_state_primary_slot_hook");
//     /* Use default swap state handling */
//     return BOOT_HOOK_REGULAR;
// }

// /**
//  * @brief Hook called after a serial upload completes
//  *
//  * This hook is called after MCUboot completes a firmware upload via serial interface.
//  */
// int boot_serial_uploaded_hook(int img_index, const struct flash_area *area,
//                                    size_t size)
// {
//     LOG_INF("Inside boot_serial_uploaded_hook");
//     /* No special handling needed for serial uploads */
//     return BOOT_HOOK_REGULAR;
// }

// /**
//  * @brief Hook called when a reset is requested via serial interface
//  *
//  * This hook is called when MCUboot receives a reset request via serial.
//  */
// int boot_reset_request_hook(bool force)
// {
//     LOG_INF("Inside boot_reset_request_hook");
//     /* No special handling needed for reset requests */
//     return 0;
// }


fih_ret boot_go_hook(struct boot_rsp *rsp) {
    int ret;

    if (!spi_bus_initialized) {
        LOG_INF("Acquiring SPI bus for external flash access");

        ret = spi_bus_share_init();
        if (ret < 0) {
            LOG_ERR("Failed to init SPI bus share: %d", ret);
            FIH_RET(FIH_FAILURE);
        }

        // Give FSM time to complete startup discovery
        LOG_INF("Waiting for FSM startup discovery to complete...");
        k_sleep(K_MSEC(3000));  // Wait up to 3 seconds for discovery

        LOG_INF("Requesting bus ownership from other MCU...");
        ret = spi_bus_acquire_blocking(true);
        if (ret < 0) {
            LOG_ERR("Failed to acquire SPI bus: %d (other MCU may not be responding)", ret);
            FIH_RET(FIH_FAILURE);
        }

        spi_bus_initialized = true;
        LOG_INF("SPI bus acquired successfully - MCUboot has access to external flash");
    }

    LOG_INF("=== boot_go_hook ready - proceeding to boot ===");

    spi_bus_shutdown();
    FIH_RET(FIH_BOOT_HOOK_REGULAR);
}