/*
 * Copyright (c) 2019-2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "fota.h"

#include <nrf_socket.h>



fota_callback_t state_callback = NULL;
struct k_work fota_work;

LOG_MODULE_REGISTER(fota, LOG_LEVEL_INF);

/**
 * @brief Set FOTA state and notify callback
 */
void set_state(enum fota_state new_state, int error)
{
    if (current_state == new_state) {
        return;
    }

    current_state = new_state;
    
    LOG_INF("FOTA state changed to %d (error: %d)", new_state, error);

    if (state_callback) {
        state_callback(new_state, error);
    }
}



/**
 * @brief FOTA download event handler
 */
void fota_dl_handler(const struct fota_download_evt *evt)
{
    switch (evt->id) {
    case FOTA_DOWNLOAD_EVT_ERROR:
        LOG_ERR("FOTA download error");
        set_state(FOTA_CONNECTED, -EIO);
        break;
    case FOTA_DOWNLOAD_EVT_FINISHED:
        LOG_INF("FOTA download finished");
        set_state(FOTA_READY_TO_APPLY, 0);
        fota_apply_update();
        break;
    case FOTA_DOWNLOAD_EVT_PROGRESS:
        LOG_INF("FOTA download progress: %d%%", evt->progress);
        break;
    default:
        break;
    }
}


/**
 * @brief Download firmware from configured host
 */
int download_firmware(void)
{
    int err;
    

    err = fota_download_init(fota_dl_handler);
    if (err) {
        LOG_ERR("fota_download_init() failed, err %d", err);
        return err;
    }

    
    LOG_INF("Starting firmware download from %s%s", ota_config.server_addr, firmware_filename);
    err = fota_download_start(ota_config.server_addr, firmware_filename, atoi(ota_config.cert_tag), 0, 0);
    if (err) {
        LOG_ERR("fota_download_start() failed, err %d", err);
        return err;
    }

    
    
    return 0;
}

/**
 * @brief FOTA work callback function
 */
void fota_work_cb(struct k_work *work)
{
    int err;

    ARG_UNUSED(work);

    switch (current_state) {
    case FOTA_DOWNLOADING:
        err = download_firmware();
        if (err) {
            set_state(FOTA_CONNECTED, err);
        }
        break;
    case FOTA_APPLYING:
        LOG_INF("Applying firmware update - rebooting...");
        lte_lc_power_off();
        sys_reboot(SYS_REBOOT_WARM);
        break;
    default:
        break;
    }
}

/**
 * @brief Initialize FOTA subsystem
 */
int fota_init(fota_callback_t callback)
{
    state_callback = callback;
    boot_write_img_confirmed();
    k_work_init(&fota_work, fota_work_cb);
    return 0;
}

/**
 * @brief Check FOTA server for updates
 */
int check_fota_server(void)
{
    switch (current_state) {
    case FOTA_IDLE:
        break;
        
    case FOTA_CONNECTED:
        set_state(FOTA_DOWNLOADING, 0);
        LOG_INF("Checking for FOTA updates...");
        k_work_submit(&fota_work);
        break;
        
    case FOTA_DOWNLOADING:
        LOG_INF("FOTA download already in progress");
        return -EBUSY;
        
    case FOTA_READY_TO_APPLY:
        LOG_INF("FOTA update ready - call fota_apply_update()");
        return 0;
        
    case FOTA_APPLYING:
        LOG_INF("FOTA update being applied");
        return -EBUSY;
        
    default:
        LOG_ERR("Invalid FOTA state: %d", current_state);
        set_state(FOTA_IDLE, -EINVAL);
        return -EINVAL;
    }

    return 0;
}

/**
 * @brief Apply FOTA update
 */
int fota_apply_update(void)
{
    if (current_state != FOTA_READY_TO_APPLY) {
        return -EPERM;
    }

    set_state(FOTA_APPLYING, 0);
    k_work_submit(&fota_work);
    return 0;
}

/**
 * @brief Get current FOTA state
 */
enum fota_state fota_get_state(void)
{
    return current_state;
}

/**
 * @brief Cancel FOTA operation
 */
int fota_cancel(void)
{
    switch (current_state) {
    case FOTA_DOWNLOADING:
        fota_download_cancel();
        set_state(FOTA_CONNECTED, 0);
        break;
    case FOTA_READY_TO_APPLY:
        set_state(FOTA_CONNECTED, 0);
        break;
    default:
        return -EPERM;
    }

    return 0;
}