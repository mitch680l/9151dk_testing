#include "spi_bus_share.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/init.h>
#include <bootutil/image.h>
#include <bootutil/bootutil.h>
#include <bootutil/bootutil_public.h>
#include <zephyr/logging/log.h>
#include "bootutil/fault_injection_hardening.h"

LOG_MODULE_REGISTER(mcu_spi_init, LOG_LEVEL_INF);

static int spi_bus_bootloader_early_init(void)
{
    int ret;
    int wait_loops = 0;
    const int max_wait_loops = 100;  

    LOG_INF("=== Bootloader early SPI bus init ===");

    
    ret = spi_bus_share_init();
    if (ret < 0) {
        LOG_ERR("Failed to init SPI bus share: %d", ret);
        sys_reboot(SYS_REBOOT_COLD);
        return ret;
    }

    LOG_INF("Yielding to FSM thread...");
    k_sleep(K_MSEC(500));

    /* FSM states (from spi_bus_share.c):
     * 0 = FSM_STARTUP
     * 1 = FSM_IDLE_OWNER
     * 2 = FSM_IDLE_NOT_OWNER
     * 3 = FSM_REQUESTING
     * 4 = FSM_RELEASING
     * 5 = FSM_ERROR
     */
    #define FSM_STATE_STARTUP        0
    #define FSM_STATE_IDLE_OWNER     1
    #define FSM_STATE_IDLE_NOT_OWNER 2
    #define FSM_STATE_REQUESTING     3
    #define FSM_STATE_ERROR          5


    const int requesting_timeout_loops = 10;  
    int requesting_wait = 0;

    LOG_INF("Waiting for FSM startup discovery...");
    while (wait_loops < max_wait_loops) {
        k_sleep(K_MSEC(50));
        wait_loops++;

        int state = spi_bus_get_fsm_state();
        if (state == FSM_STATE_IDLE_OWNER) {
            LOG_INF("FSM ready as owner after %d ms", wait_loops * 100);
            break;
        } else if (state == FSM_STATE_IDLE_NOT_OWNER) {
            LOG_INF("FSM in NOT_OWNER state, requesting bus...");
            break;
        } else if (state == FSM_STATE_REQUESTING) {
            requesting_wait++;
            if (requesting_wait >= requesting_timeout_loops) {
                LOG_WRN("Other MCU not responding after %d ms - simulating grant",
                        requesting_wait * 100);
                spi_bus_simulate_grant_event(1);
            }
        } else if (state == FSM_STATE_ERROR) {
            LOG_ERR("FSM entered error state during startup");
            sys_reboot(SYS_REBOOT_COLD);
            return -EIO;
        }
    }

    if (wait_loops >= max_wait_loops) {
        LOG_ERR("Timeout waiting for FSM discovery");
        sys_reboot(SYS_REBOOT_COLD);
        return -ETIMEDOUT;
    }


    ret = spi_bus_acquire_blocking(true);
    if (ret != 0) {
        LOG_ERR("Failed to acquire SPI bus: %d - rebooting", ret);
        sys_reboot(SYS_REBOOT_COLD);
        return ret;
    }

    LOG_INF("=== SPI bus acquired - flash driver can now init ===");
    return 0;
}



SYS_INIT(spi_bus_bootloader_early_init, POST_KERNEL , 5);

/**
 * @brief Hook called when MCUboot reads an image header
 *
 * This is called for each image slot during boot validation.
 * We use this to initialize and acquire the SPI bus before any flash operations.
 */
int boot_read_image_header_hook(int img_index, int slot,
                                 struct image_header *img_header)
 {
    return BOOT_HOOK_REGULAR;
}

/**
 * @brief Hook called after an image has been validated
 *
 * This hook is called after MCUboot validates an image's signature and integrity.
 */
fih_ret boot_image_check_hook(int img_index, int slot)
{

    LOG_INF("Inside boot_image_check_hook");
    FIH_RET(FIH_BOOT_HOOK_REGULAR);
}

/**
 * @brief Hook called after a region has been copied during image swap
 *
 * This hook is called after MCUboot copies a region of flash during the swap process.
 */
int boot_copy_region_post_hook(int img_index, const struct flash_area *area,
                                size_t size)
{

    return 0;
}

/**
 * @brief Hook called when performing a firmware update
 *
 * This hook is called before MCUboot performs the update operation.
 */
int boot_perform_update_hook(int img_index, struct image_header *img_head,
                              const struct flash_area *area)
{
    LOG_INF("Inside boot_perform_update_hook");
    return 0;
}

/**
 * @brief Hook called when reading swap state from primary slot
 *
 * This hook allows customization of how swap state is read from the primary slot.
 */
int boot_read_swap_state_primary_slot_hook(int image_index,
                                            struct boot_swap_state *state)
{
    LOG_INF("Inside boot_read_swap_state_primary_slot_hook");
    return BOOT_HOOK_REGULAR;
}

/**
 * @brief Hook called after a serial upload completes
 *
 * This hook is called after MCUboot completes a firmware upload via serial interface.
 */
int boot_serial_uploaded_hook(int img_index, const struct flash_area *area,
                                   size_t size)
{
    LOG_INF("Inside boot_serial_uploaded_hook");
    return BOOT_HOOK_REGULAR;
}

/**
 * @brief Hook called when a reset is requested via serial interface
 *
 * This hook is called when MCUboot receives a reset request via serial.
 */
int boot_reset_request_hook(bool force)
{
    LOG_INF("Inside boot_reset_request_hook");
    return 0;
}


fih_ret boot_go_hook(struct boot_rsp *rsp) {
    // int ret;

    // if (!spi_bus_initialized) {
    //     LOG_INF("Acquiring SPI bus for external flash access");

    //     ret = spi_bus_share_init();
    //     if (ret < 0) {
    //         LOG_ERR("Failed to init SPI bus share: %d", ret);
    //         FIH_RET(FIH_FAILURE);
    //     }


    //     LOG_INF("Waiting for FSM startup discovery to complete...");
    //     k_sleep(K_MSEC(3000));

    //     LOG_INF("Requesting bus ownership from other MCU...");
    //     ret = spi_bus_acquire_blocking(true);
    //     if (ret < 0) {
    //         LOG_ERR("Failed to acquire SPI bus: %d (other MCU may not be responding)", ret);
    //         FIH_RET(FIH_FAILURE);
    //     }

    //     spi_bus_initialized = true;
    //     LOG_INF("SPI bus acquired successfully - MCUboot has access to external flash");
    // }

    LOG_INF("=== boot_go_hook ready - proceeding to boot ===");

    spi_bus_shutdown();
    FIH_RET(FIH_BOOT_HOOK_REGULAR);
}

