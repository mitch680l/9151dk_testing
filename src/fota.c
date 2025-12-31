/*
 * Copyright (c) 2019-2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "fota.h"

#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <string.h>
#include <nrf_socket.h>



fota_callback_t state_callback = NULL;
enum fota_state current_state = FOTA_IDLE;
/* OTA configuration - initialized with default values */
struct ota_config_t ota_config = {
    .server_addr = "44.209.5.53",  
    .cert_tag = "-1"  
};

const char *firmware_filename = "/app_update.bin";
struct k_work fota_work;
static char custom_filename[128] = {0};

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

/**
 * @brief Set FOTA server configuration
 */
void fota_set_server(const char *server_addr, const char *cert_tag)
{
    ota_config.server_addr = server_addr;
    ota_config.cert_tag = cert_tag;
    LOG_INF("FOTA server set to: %s (cert: %s)", server_addr, cert_tag);
}

/**
 * @brief Set custom firmware filename for download
 */
void fota_set_filename(const char *filename)
{
    strncpy(custom_filename, filename, sizeof(custom_filename) - 1);
    custom_filename[sizeof(custom_filename) - 1] = '\0';
    firmware_filename = custom_filename;
    LOG_INF("FOTA filename set to: %s", firmware_filename);
}

/**
 * @brief Shell command to start FOTA download with custom filename
 */
static int cmd_fota_download(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_error(sh, "Usage: fota download <filename>");
        shell_print(sh, "Example: fota download /app_update.bin");
        return -EINVAL;
    }

    /* Set the custom filename */
    fota_set_filename(argv[1]);

    /* Check current state */
    if (current_state != FOTA_IDLE && current_state != FOTA_CONNECTED) {
        shell_error(sh, "FOTA already in progress (state: %d)", current_state);
        return -EBUSY;
    }

    /* Transition to connected state if idle */
    if (current_state == FOTA_IDLE) {
        set_state(FOTA_CONNECTED, 0);
    }

    /* Start the download */
    shell_print(sh, "Starting FOTA download: %s", firmware_filename);
    int err = check_fota_server();
    if (err) {
        shell_error(sh, "Failed to start FOTA download: %d", err);
        return err;
    }

    shell_print(sh, "FOTA download initiated successfully");
    return 0;
}

/**
 * @brief Shell command to set FOTA server
 */
static int cmd_fota_set_server(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_error(sh, "Usage: fota server <server_url> [cert_tag]");
        shell_print(sh, "Example: fota server https://example.com/fota 16842753");
        return -EINVAL;
    }

    const char *cert = (argc >= 3) ? argv[2] : ota_config.cert_tag;
    fota_set_server(argv[1], cert);

    shell_print(sh, "FOTA server configured");
    return 0;
}

/**
 * @brief Shell command to check FOTA status
 */
static int cmd_fota_status(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    const char *state_str[] = {
        "IDLE",
        "CONNECTED",
        "DOWNLOADING",
        "READY_TO_APPLY",
        "APPLYING"
    };

    shell_print(sh, "FOTA Status:");
    shell_print(sh, "  State: %s (%d)", state_str[current_state], current_state);
    shell_print(sh, "  Server: %s", ota_config.server_addr);
    shell_print(sh, "  Cert Tag: %s", ota_config.cert_tag);
    shell_print(sh, "  Filename: %s", firmware_filename);

    return 0;
}

/**
 * @brief Shell command to apply FOTA update
 */
static int cmd_fota_apply(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    int err = fota_apply_update();
    if (err) {
        shell_error(sh, "Failed to apply update: %d", err);
        return err;
    }

    shell_print(sh, "Applying update and rebooting...");
    return 0;
}

/**
 * @brief Shell command to cancel FOTA operation
 */
static int cmd_fota_cancel_shell(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    int err = fota_cancel();
    if (err) {
        shell_error(sh, "Failed to cancel FOTA: %d", err);
        return err;
    }

    shell_print(sh, "FOTA operation cancelled");
    return 0;
}

/* Shell command tree */
SHELL_STATIC_SUBCMD_SET_CREATE(fota_cmds,
    SHELL_CMD_ARG(download, NULL, "Download firmware from server\nUsage: fota download <filename>",
                  cmd_fota_download, 2, 0),
    SHELL_CMD_ARG(server, NULL, "Set FOTA server\nUsage: fota server <url> [cert_tag]",
                  cmd_fota_set_server, 2, 1),
    SHELL_CMD_ARG(status, NULL, "Show FOTA status", cmd_fota_status, 1, 0),
    SHELL_CMD_ARG(apply, NULL, "Apply downloaded update", cmd_fota_apply, 1, 0),
    SHELL_CMD_ARG(cancel, NULL, "Cancel FOTA operation", cmd_fota_cancel_shell, 1, 0),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(fota, &fota_cmds, "FOTA commands", NULL);