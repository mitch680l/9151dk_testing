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

LOG_MODULE_REGISTER(ota, LOG_LEVEL_INF);

#ifndef THIS_IS_5340
static download_callback_t download_callback = NULL;


struct ota_config_t ota_config = {
    .server_addr = "44.209.5.53",
    .cert_tag = "-1"
};


static enum download_state current_state = DOWNLOAD_IDLE;


#define DL_BUF_SIZE 2048
static struct download_client dl_client;
static struct stream_flash_ctx stream_ctx;
static uint8_t dl_buf[DL_BUF_SIZE];
static const struct flash_area *target_flash_area = NULL;
static bool dl_client_initialized = false;


static enum {
    TARGET_9151,
    TARGET_5340
} current_target;

static char file_path_buf[CONFIG_DOWNLOAD_CLIENT_MAX_FILENAME_SIZE];

/**
 * @brief Set download state and notify callback
 */
static void set_download_state(enum download_state new_state, int error)
{
    if (current_state == new_state) {
        return;
    }

    current_state = new_state;

    LOG_INF("Download state changed to %d (error: %d)", new_state, error);

    if (download_callback) {
        download_callback(new_state, error);
    }
}

static bool first_fragment_received = true;

/**
 * @brief Download client callback - handles fragments for both 9151 and 5340
 */
static int dl_client_callback(const struct download_client_evt *event)
{
    int err = 0;

    switch (event->id) {
    case DOWNLOAD_CLIENT_EVT_FRAGMENT: {
         if (first_fragment_received) {
            const uint8_t *buf = (const uint8_t *)event->fragment.buf;
            LOG_INF("First fragment (%d bytes)", event->fragment.len);
            LOG_HEXDUMP_INF(buf, MIN(256, event->fragment.len), "First 256 bytes:");

            if (event->fragment.len > 0 && buf[0] == '<') {
                LOG_INF("Content appears to be HTML/text:");
                for (int i = 0; i < MIN(event->fragment.len, 512); i += 64) {
                    int chunk_len = MIN(64, event->fragment.len - i);
                    LOG_HEXDUMP_INF(&buf[i], chunk_len, "");
                }
            }

            first_fragment_received = false;
        }


        err = stream_flash_buffered_write(&stream_ctx, event->fragment.buf,
                                         event->fragment.len, false);
        if (err) {
            LOG_ERR("stream_flash_buffered_write failed: %d", err);
            set_download_state(DOWNLOAD_ERROR, err);
            return err;
        }

        LOG_INF("Downloaded fragment: %d bytes", event->fragment.len);
        break;
    }

    case DOWNLOAD_CLIENT_EVT_DONE:
        LOG_INF("Finalizing download, flushing buffers...");
        err = stream_flash_buffered_write(&stream_ctx, NULL, 0, true);
        if (err) {
            LOG_ERR("stream_flash finalize failed: %d", err);
            set_download_state(DOWNLOAD_ERROR, err);
            return err;
        }
        LOG_INF("Stream flash finalized successfully");

        if (target_flash_area) {
            uint8_t verify_buf[32];
            err = flash_area_read(target_flash_area, 0, verify_buf, sizeof(verify_buf));
            if (err == 0) {
                LOG_INF("First 32 bytes after write:");
                LOG_HEXDUMP_INF(verify_buf, sizeof(verify_buf), "Flash content:");
                uint32_t magic = *(uint32_t *)verify_buf;
                if (magic == 0x96f3b83d) {
                    LOG_INF("MCUboot magic verified in flash!");
                } else {
                    LOG_WRN("MCUboot magic NOT found in flash! Got 0x%08x", magic);
                }
            }

            flash_area_close(target_flash_area);
            target_flash_area = NULL;
        }

        if (current_target == TARGET_9151) {
            LOG_INF("nRF9151 firmware download complete");
            LOG_INF("Firmware stored in mcuboot_secondary partition");
        } else {
            LOG_INF("nRF5340 firmware download complete");
            LOG_INF("Firmware stored in mcuboot_secondary_5340 partition");
        }

        set_download_state(DOWNLOAD_COMPLETE, 0);
        break;

    case DOWNLOAD_CLIENT_EVT_ERROR:
        LOG_ERR("Download client error: %d", event->error);

        if (target_flash_area) {
            flash_area_close(target_flash_area);
            target_flash_area = NULL;
        }

        set_download_state(DOWNLOAD_ERROR, event->error);
        break;

    default:
        break;
    }

    return err;
}

/**
 * @brief Common download function for both 9151 and 5340
 */
static int ota_download_common(const char *filename, int partition_id,
                               enum download_state target_type)
{
    int err;
    struct download_client_cfg dl_cfg;

    if (current_state == DOWNLOAD_IN_PROGRESS) {
        LOG_ERR("Download already in progress");
        return -EBUSY;
    }

    current_target = target_type;

    first_fragment_received = true;

    LOG_INF("Downloading firmware: %s", filename);

    err = flash_area_open(partition_id, &target_flash_area);
    if (err) {
        LOG_ERR("Failed to open flash partition: %d", err);
        return err;
    }

    LOG_INF("Partition size: %zu bytes at offset 0x%lx",
            target_flash_area->fa_size, target_flash_area->fa_off);


    err = stream_flash_init(&stream_ctx, target_flash_area->fa_dev, dl_buf,
                           sizeof(dl_buf), 0, 
                           target_flash_area->fa_size, NULL);
    if (err) {
        LOG_ERR("stream_flash_init failed: %d", err);
        flash_area_close(target_flash_area);
        target_flash_area = NULL;
        return err;
    }
    LOG_INF("stream_flash initialized with 0 offset (partition-relative)");

    LOG_INF("Erasing partition...");
    err = flash_area_erase(target_flash_area, 0, target_flash_area->fa_size);
    if (err) {
        LOG_ERR("flash_area_erase failed: %d", err);
        flash_area_close(target_flash_area);
        target_flash_area = NULL;
        return err;
    }

    const char *file_path = filename;
    if (file_path[0] == '/') {
        file_path++;
    }

    snprintf(file_path_buf, sizeof(file_path_buf), "/%s", file_path);

    LOG_INF("Host: %s, File: %s", ota_config.server_addr, file_path_buf);

    memset(&dl_cfg, 0, sizeof(dl_cfg));
    dl_cfg.pdn_id = 0;
    dl_cfg.frag_size_override = 0;
    dl_cfg.family = 0;  
    dl_cfg.sec_tag_count = 0;
    dl_cfg.sec_tag_list = NULL;

    int sec_tag = atoi(ota_config.cert_tag);
    if (sec_tag >= 0) {
        static sec_tag_t sec_tag_list[1];
        sec_tag_list[0] = sec_tag;
        dl_cfg.sec_tag_list = sec_tag_list;
        dl_cfg.sec_tag_count = 1;
    }

    LOG_INF("Calling download_client_get:");
    LOG_INF("  Host: '%s'", ota_config.server_addr);
    LOG_INF("  File: '%s' (len=%d)", file_path_buf, strlen(file_path_buf));

    err = download_client_get(&dl_client, ota_config.server_addr, &dl_cfg, file_path_buf, 0);
    if (err) {
        LOG_ERR("download_client_get failed: %d", err);
        flash_area_close(target_flash_area);
        target_flash_area = NULL;
        return err;
    }

    set_download_state(DOWNLOAD_IN_PROGRESS, 0);
    LOG_INF("Firmware download started");

    return 0;
}

/**
 * @brief Initialize OTA subsystem
 */
void ota_init(download_callback_t callback)
{
    int err;

    download_callback = callback;
    boot_write_img_confirmed();

    if (!dl_client_initialized) {
        err = download_client_init(&dl_client, dl_client_callback);
        if (err) {
            LOG_ERR("download_client_init failed: %d", err);
            return;
        }
        dl_client_initialized = true;
    }

    LOG_INF("OTA subsystem initialized");
}

/**
 * @brief Set OTA server configuration
 */
void ota_set_server(const char *server_addr, const char *cert_tag)
{
    ota_config.server_addr = server_addr;
    ota_config.cert_tag = cert_tag;
    LOG_INF("OTA server set to: %s (cert: %s)", server_addr, cert_tag);
}

/**
 * @brief Download firmware for nRF9151
 */
int ota_download_9151(const char *filename)
{
    LOG_INF("Starting nRF9151 download");
    return ota_download_common(filename,
                               FIXED_PARTITION_ID(mcuboot_secondary),
                               TARGET_9151);
}

/**
 * @brief Download firmware for nRF5340
 */
int ota_download_5340(const char *filename)
{
    LOG_INF("Starting nRF5340 download");
    return ota_download_common(filename,
                               FIXED_PARTITION_ID(mcuboot_secondary_5340),
                               TARGET_5340);
}

/**
 * @brief Download firmware for nRF5340 network core
 */
int ota_download_5340net(const char *filename)
{
    LOG_INF("Starting nRF5340 network download");
    return ota_download_common(filename,
                               FIXED_PARTITION_ID(mcuboot_secondary_5340_1),
                               TARGET_5340);
}

/**
 * @brief Cancel OTA operation
 */
int ota_cancel(void)
{
    if (current_state != DOWNLOAD_IN_PROGRESS) {
        return -EPERM;
    }


    download_client_disconnect(&dl_client);

    if (target_flash_area) {
        flash_area_close(target_flash_area);
        target_flash_area = NULL;
    }

    set_download_state(DOWNLOAD_IDLE, 0);
    LOG_INF("OTA operation cancelled");

    return 0;
}

#endif /* !THIS_IS_5340 */

/**
 * @brief Apply the downloaded firmware update (general function)
 */
int ota_apply(void)
{
    int err;

#ifdef THIS_IS_5340
    LOG_INF("Requesting MCUboot upgrade for nRF5340...");
#else
    LOG_INF("Requesting MCUboot upgrade for nRF9151...");
#endif

    int confirmed = boot_is_img_confirmed();
    LOG_INF("Current image confirmed status: %d", confirmed);

    err = boot_request_upgrade(BOOT_UPGRADE_TEST);
    if (err) {
        LOG_ERR("boot_request_upgrade failed: %d", err);
        return err;
    }

    LOG_INF("Upgrade scheduled. Rebooting in 2 seconds...");
    k_sleep(K_SECONDS(2));
    sys_reboot(SYS_REBOOT_WARM);

    return 0;
}

#ifndef THIS_IS_5340
/**
 * @brief Shell command to download nRF9151 firmware
 */
static int cmd_ota_download_9151(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_error(sh, "Usage: ota download9151 <filename>");
        shell_print(sh, "Example: ota download9151 firmware_storage/app_update.bin");
        return -EINVAL;
    }

    int err = ota_download_9151(argv[1]);
    if (err) {
        shell_error(sh, "Failed to start download: %d", err);
        return err;
    }

    shell_print(sh, "nRF9151 download started");
    return 0;
}

/**
 * @brief Shell command to download nRF5340 firmware
 */
static int cmd_ota_download_5340(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_error(sh, "Usage: ota download5340 <filename>");
        shell_print(sh, "Example: ota download5340 firmware_storage/nrf5340_app.bin");
        return -EINVAL;
    }

    int err = ota_download_5340(argv[1]);
    if (err) {
        shell_error(sh, "Failed to start download: %d", err);
        return err;
    }

    shell_print(sh, "nRF5340 download started");
    return 0;
}

/**
 * @brief Shell command to download 5340 network core
 */

static int cmd_ota_download_5340net (const struct shell *sh, size_t argc, char **argv) {
    if (argc < 2) {
        shell_error(sh, "Usage: ota download5340net <filename>");
        shell_print(sh, "Example: ota download5430net firmware_storage/nrf5340_net.bin");
    }

    int err = ota_download_5340net(argv[1]);
    if(err) {
        shell_error(sh, "Failed to start download: %d", err);
        return err;
    }
    shell_print(sh, "nRF5340net download started");
    return 0;
}


/**
 * @brief Shell command to set OTA server
 */
static int cmd_ota_set_server(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_error(sh, "Usage: ota server <server_url> [cert_tag]");
        shell_print(sh, "Example: ota server 44.209.5.53 -1");
        return -EINVAL;
    }

    const char *cert = (argc >= 3) ? argv[2] : ota_config.cert_tag;
    ota_set_server(argv[1], cert);

    shell_print(sh, "OTA server configured");
    return 0;
}

/**
 * @brief Shell command to check OTA status
 */
static int cmd_ota_status(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    const char *state_str[] = {
        "IDLE",
        "IN_PROGRESS",
        "COMPLETE",
        "ERROR"
    };

    shell_print(sh, "OTA Status:");
    shell_print(sh, "  State: %s (%d)", state_str[current_state], current_state);
    shell_print(sh, "  Server: %s", ota_config.server_addr);
    shell_print(sh, "  Cert Tag: %s", ota_config.cert_tag);

    return 0;
}

#endif /* !THIS_IS_5340 */

/**
 * @brief Shell command to verify downloaded nRF9151 image
 */
static int cmd_ota_verify_9151(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    const struct flash_area *fa;
    #ifdef THIS_IS_5340

    int err = flash_area_open(FIXED_PARTITION_ID(ext_flash_reserved), &fa);
    if (err) {
        shell_error(sh, "Failed to open secondary partition: %d", err);
        return err;
    }
    shell_print(sh, "=== nRF9151 Image Verification (ext_flash_reserved) ===");

    #else

    int err = flash_area_open(FIXED_PARTITION_ID(mcuboot_secondary), &fa);
    if (err) {
        shell_error(sh, "Failed to open secondary partition: %d", err);
        return err;
    }
    shell_print(sh, "=== nRF9151 Image Verification (mcuboot_secondary) ===");
    #endif

    shell_print(sh, "Partition info:");
    shell_print(sh, "  Address: 0x%08lx", fa->fa_off);
    shell_print(sh, "  Size: %zu bytes (0x%zx)", fa->fa_size, fa->fa_size);

    uint8_t header[512];
    err = flash_area_read(fa, 0, header, sizeof(header));
    if (err) {
        shell_error(sh, "Failed to read partition: %d", err);
        flash_area_close(fa);
        return err;
    }

    shell_print(sh, "\nFirst 32 bytes:");
    shell_hexdump(sh, header, 32);

    uint32_t magic = *(uint32_t *)&header[0];
    shell_print(sh, "\n--- MCUboot Header Analysis ---");
    shell_print(sh, "Magic: 0x%08x %s", magic,
                (magic == 0x96f3b83d) ? "[VALID]" : "[INVALID - Expected 0x96f3b83d]");

    if (magic == 0x96f3b83d) {
        uint32_t load_addr = *(uint32_t *)&header[4];
        uint16_t hdr_size = *(uint16_t *)&header[8];
        uint16_t protect_tlv_size = *(uint16_t *)&header[10];
        uint32_t img_size = *(uint32_t *)&header[12];
        uint32_t flags = *(uint32_t *)&header[16];
        uint8_t ver_major = header[20];
        uint8_t ver_minor = header[21];
        uint16_t ver_revision = *(uint16_t *)&header[22];
        uint32_t ver_build = *(uint32_t *)&header[24];

        shell_print(sh, "Load Address: 0x%08x", load_addr);
        shell_print(sh, "Header Size: %u bytes", hdr_size);
        shell_print(sh, "Protected TLV Size: %u bytes", protect_tlv_size);
        shell_print(sh, "Image Size: %u bytes (0x%x)", img_size, img_size);
        shell_print(sh, "Flags: 0x%08x", flags);
        shell_print(sh, "Version: %u.%u.%u+%u",
                    ver_major, ver_minor, ver_revision, ver_build);


        if (img_size > fa->fa_size) {
            shell_error(sh, "WARNING: Image size (%u) exceeds partition size (%zu)!",
                       img_size, fa->fa_size);
        }

        /* Read and analyze TLV area */
        uint32_t tlv_offset = hdr_size + img_size;
        shell_print(sh, "\n--- TLV Area Analysis ---");
        shell_print(sh, "TLV offset: 0x%x (%u bytes from start)", tlv_offset, tlv_offset);

        if (tlv_offset + 4 > fa->fa_size) {
            shell_error(sh, "ERROR: TLV offset exceeds partition size!");
        } else {
            uint8_t tlv_header[256];
            size_t tlv_read_size = MIN(sizeof(tlv_header), fa->fa_size - tlv_offset);

            err = flash_area_read(fa, tlv_offset, tlv_header, tlv_read_size);
            if (err) {
                shell_error(sh, "Failed to read TLV area: %d", err);
            } else {
                uint16_t tlv_magic = *(uint16_t *)&tlv_header[0];
                uint16_t tlv_total = *(uint16_t *)&tlv_header[2];

                shell_print(sh, "TLV Magic: 0x%04x %s", tlv_magic,
                           (tlv_magic == 0x6907) ? "[VALID - IMAGE_TLV_INFO_MAGIC]" :
                           (tlv_magic == 0x6908) ? "[VALID - IMAGE_TLV_PROT_INFO_MAGIC]" :
                           "[INVALID]");
                shell_print(sh, "TLV Total Size: %u bytes", tlv_total);

                if (tlv_magic == 0x6907 || tlv_magic == 0x6908) {
                    /* Parse TLV entries */
                    shell_print(sh, "\nTLV Entries:");
                    uint16_t offset = 4; /* After TLV info header */
                    bool found_signature = false;
                    bool found_hash = false;

                    while (offset + 4 <= tlv_read_size && offset < tlv_total) {
                        uint16_t tlv_type = *(uint16_t *)&tlv_header[offset];
                        uint16_t tlv_len = *(uint16_t *)&tlv_header[offset + 2];

                        const char *type_name = "UNKNOWN";
                        if (tlv_type == 0x01) { type_name = "KEYHASH"; }
                        else if (tlv_type == 0x10) { type_name = "SHA256"; found_hash = true; }
                        else if (tlv_type == 0x22) { type_name = "ECDSA_SIG"; found_signature = true; }
                        else if (tlv_type == 0x20) { type_name = "RSA2048_PSS"; found_signature = true; }
                        else if (tlv_type == 0x24) { type_name = "ED25519"; found_signature = true; }
                        else if (tlv_type == 0x40) { type_name = "DEPENDENCY"; }

                        shell_print(sh, "  Type: 0x%04x (%s), Length: %u bytes",
                                   tlv_type, type_name, tlv_len);

                        offset += 4 + tlv_len;
                    }

                    shell_print(sh, "\n--- Validation Summary ---");
                    if (found_hash) {
                        shell_print(sh, "[OK] Image hash (SHA256) present");
                    } else {
                        shell_error(sh, "[FAIL] No image hash found!");
                    }

                    if (found_signature) {
                        shell_print(sh, "[OK] Image signature present");
                    } else {
                        shell_error(sh, "[FAIL] No signature found - MCUboot will REJECT unsigned image!");
                    }

                    if (found_hash && found_signature) {
                        shell_print(sh, "\n[VERDICT: Image is properly signed and should be accepted]");
                    } else {
                        shell_error(sh, "\n[VERDICT: Image is MISSING signature - MCUboot will REJECT]");
                    }
                } else {
                    shell_error(sh, "Invalid TLV magic - cannot parse TLV entries");
                    shell_print(sh, "First 64 bytes of TLV area:");
                    shell_hexdump(sh, tlv_header, MIN(64, tlv_read_size));
                }
            }
        }
    } else {
        shell_error(sh, "\n[VERDICT: Image header INVALID - MCUboot will reject this image]");

 
        if (header[0] == '<' || header[0] == '{') {
            shell_error(sh, "ERROR: Downloaded file appears to be HTML/JSON, not a binary image!");
            shell_print(sh, "First 128 bytes as text:");
            for (int i = 0; i < MIN(128, sizeof(header)); i++) {
                if (header[i] >= 32 && header[i] < 127) {
                    shell_print(sh, "%c", header[i]);
                }
            }
        }
    }

    flash_area_close(fa);
    return 0;
}

/**
 * @brief Shell command to verify downloaded nRF5340 image
 */
static int cmd_ota_verify_5340(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    const struct flash_area *fa;
    #ifdef THIS_IS_5340
    int err = flash_area_open(FIXED_PARTITION_ID(mcuboot_secondary), &fa);
    if (err) {
        shell_error(sh, "Failed to open secondary partition: %d", err);
        return err;
    }
    shell_print(sh, "=== nRF5340 Image Verification (mcuboot_secondary) ===");
    #else
    int err = flash_area_open(FIXED_PARTITION_ID(mcuboot_secondary_5340), &fa);
    if (err) {
        shell_error(sh, "Failed to open secondary partition: %d", err);
        return err;
    }
    shell_print(sh, "=== nRF5340 Image Verification (mcuboot_secondary_5340) ===");
    #endif

    shell_print(sh, "Partition info:");
    shell_print(sh, "  Address: 0x%08lx", fa->fa_off);
    shell_print(sh, "  Size: %zu bytes (0x%zx)", fa->fa_size, fa->fa_size);

    uint8_t header[512];
    err = flash_area_read(fa, 0, header, sizeof(header));
    if (err) {
        shell_error(sh, "Failed to read partition: %d", err);
        flash_area_close(fa);
        return err;
    }

    shell_print(sh, "\nFirst 32 bytes:");
    shell_hexdump(sh, header, 32);

    uint32_t magic = *(uint32_t *)&header[0];
    shell_print(sh, "\n--- MCUboot Header Analysis ---");
    shell_print(sh, "Magic: 0x%08x %s", magic,
                (magic == 0x96f3b83d) ? "[VALID]" : "[INVALID - Expected 0x96f3b83d]");

    if (magic == 0x96f3b83d) {
        uint32_t load_addr = *(uint32_t *)&header[4];
        uint16_t hdr_size = *(uint16_t *)&header[8];
        uint16_t protect_tlv_size = *(uint16_t *)&header[10];
        uint32_t img_size = *(uint32_t *)&header[12];
        uint32_t flags = *(uint32_t *)&header[16];
        uint8_t ver_major = header[20];
        uint8_t ver_minor = header[21];
        uint16_t ver_revision = *(uint16_t *)&header[22];
        uint32_t ver_build = *(uint32_t *)&header[24];

        shell_print(sh, "Load Address: 0x%08x", load_addr);
        shell_print(sh, "Header Size: %u bytes", hdr_size);
        shell_print(sh, "Protected TLV Size: %u bytes", protect_tlv_size);
        shell_print(sh, "Image Size: %u bytes (0x%x)", img_size, img_size);
        shell_print(sh, "Flags: 0x%08x", flags);
        shell_print(sh, "Version: %u.%u.%u+%u",
                    ver_major, ver_minor, ver_revision, ver_build);

        if (img_size > fa->fa_size) {
            shell_error(sh, "WARNING: Image size (%u) exceeds partition size (%zu)!",
                       img_size, fa->fa_size);
        }

        /* Read and analyze TLV area */
        uint32_t tlv_offset = hdr_size + img_size;
        shell_print(sh, "\n--- TLV Area Analysis ---");
        shell_print(sh, "TLV offset: 0x%x (%u bytes from start)", tlv_offset, tlv_offset);

        if (tlv_offset + 4 > fa->fa_size) {
            shell_error(sh, "ERROR: TLV offset exceeds partition size!");
        } else {
            uint8_t tlv_header[256];
            size_t tlv_read_size = MIN(sizeof(tlv_header), fa->fa_size - tlv_offset);

            err = flash_area_read(fa, tlv_offset, tlv_header, tlv_read_size);
            if (err) {
                shell_error(sh, "Failed to read TLV area: %d", err);
            } else {
                uint16_t tlv_magic = *(uint16_t *)&tlv_header[0];
                uint16_t tlv_total = *(uint16_t *)&tlv_header[2];

                shell_print(sh, "TLV Magic: 0x%04x %s", tlv_magic,
                           (tlv_magic == 0x6907) ? "[VALID - IMAGE_TLV_INFO_MAGIC]" :
                           (tlv_magic == 0x6908) ? "[VALID - IMAGE_TLV_PROT_INFO_MAGIC]" :
                           "[INVALID]");
                shell_print(sh, "TLV Total Size: %u bytes", tlv_total);

                if (tlv_magic == 0x6907 || tlv_magic == 0x6908) {
                    /* Parse TLV entries */
                    shell_print(sh, "\nTLV Entries:");
                    uint16_t offset = 4; /* After TLV info header */
                    bool found_signature = false;
                    bool found_hash = false;

                    while (offset + 4 <= tlv_read_size && offset < tlv_total) {
                        uint16_t tlv_type = *(uint16_t *)&tlv_header[offset];
                        uint16_t tlv_len = *(uint16_t *)&tlv_header[offset + 2];

                        const char *type_name = "UNKNOWN";
                        if (tlv_type == 0x01) { type_name = "KEYHASH"; }
                        else if (tlv_type == 0x10) { type_name = "SHA256"; found_hash = true; }
                        else if (tlv_type == 0x22) { type_name = "ECDSA_SIG"; found_signature = true; }
                        else if (tlv_type == 0x20) { type_name = "RSA2048_PSS"; found_signature = true; }
                        else if (tlv_type == 0x24) { type_name = "ED25519"; found_signature = true; }
                        else if (tlv_type == 0x40) { type_name = "DEPENDENCY"; }

                        shell_print(sh, "  Type: 0x%04x (%s), Length: %u bytes",
                                   tlv_type, type_name, tlv_len);

                        offset += 4 + tlv_len;
                    }

                    shell_print(sh, "\n--- Validation Summary ---");
                    if (found_hash) {
                        shell_print(sh, "[OK] Image hash (SHA256) present");
                    } else {
                        shell_error(sh, "[FAIL] No image hash found!");
                    }

                    if (found_signature) {
                        shell_print(sh, "[OK] Image signature present");
                    } else {
                        shell_error(sh, "[FAIL] No signature found - MCUboot will REJECT unsigned image!");
                    }

                    if (found_hash && found_signature) {
                        shell_print(sh, "\n[VERDICT: Image is properly signed and should be accepted]");
                    } else {
                        shell_error(sh, "\n[VERDICT: Image is MISSING signature - MCUboot will REJECT]");
                    }
                } else {
                    shell_error(sh, "Invalid TLV magic - cannot parse TLV entries");
                    shell_print(sh, "First 64 bytes of TLV area:");
                    shell_hexdump(sh, tlv_header, MIN(64, tlv_read_size));
                }
            }
        }
    } else {
        shell_error(sh, "\n[VERDICT: Image header INVALID - MCUboot will reject this image]");

        if (header[0] == '<' || header[0] == '{') {
            shell_error(sh, "ERROR: Downloaded file appears to be HTML/JSON, not a binary image!");
            shell_print(sh, "First 128 bytes as text:");
            for (int i = 0; i < MIN(128, sizeof(header)); i++) {
                if (header[i] >= 32 && header[i] < 127) {
                    shell_print(sh, "%c", header[i]);
                }
            }
        }
    }

    flash_area_close(fa);
    return 0;
}

/**
 * @brief Shell command to apply nRF9151 firmware update
 */
static int cmd_ota_apply_9151(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

#ifdef THIS_IS_5340
    shell_error(sh, "Error: nRF9151 update not supported on nRF5340");
    LOG_ERR("nRF9151 update not supported on nRF5340");
    return -ENOTSUP;
#else
    shell_print(sh, "Applying nRF9151 firmware update...");

    int err = ota_apply();
    if (err) {
        shell_error(sh, "Failed to apply update: %d", err);
        return err;
    }

    return 0;  /* Never reached due to reboot */
#endif
}

/**
 * @brief Shell command to apply nRF5340 firmware update
 */
static int cmd_ota_apply_5340(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

#ifdef THIS_IS_5340
    shell_print(sh, "Applying nRF5340 firmware update...");

    int err = ota_apply();
    if (err) {
        shell_error(sh, "Failed to apply update: %d", err);
        return err;
    }

    return 0;
#else
    shell_print(sh, "triggered 5340 update process");
    return 0;
#endif
}

#ifndef THIS_IS_5340
/**
 * @brief Shell command to cancel OTA operation
 */
static int cmd_ota_cancel_shell(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    int err = ota_cancel();
    if (err) {
        shell_error(sh, "Failed to cancel OTA: %d", err);
        return err;
    }

    shell_print(sh, "OTA operation cancelled");
    return 0;
}
#endif /* !THIS_IS_5340 */


#ifdef THIS_IS_5340

SHELL_STATIC_SUBCMD_SET_CREATE(ota_cmds,
    SHELL_CMD_ARG(verify9151, NULL, "Verify downloaded nRF9151 image",
                  cmd_ota_verify_9151, 1, 0),
    SHELL_CMD_ARG(verify5340, NULL, "Verify downloaded nRF5340 image",
                  cmd_ota_verify_5340, 1, 0),
    SHELL_CMD_ARG(apply9151, NULL, "Apply nRF9151 firmware update",
                  cmd_ota_apply_9151, 1, 0),
    SHELL_CMD_ARG(apply5340, NULL, "Apply nRF5340 firmware update",
                  cmd_ota_apply_5340, 1, 0),
    SHELL_SUBCMD_SET_END
);
#else

SHELL_STATIC_SUBCMD_SET_CREATE(ota_cmds,
    SHELL_CMD_ARG(download9151, NULL, "Download nRF9151 firmware\nUsage: ota download9151 <filename>",
                  cmd_ota_download_9151, 2, 0),
    SHELL_CMD_ARG(download5340, NULL, "Download nRF5340 firmware\nUsage: ota download5340 <filename>",
                  cmd_ota_download_5340, 2, 0),
    SHELL_CMD_ARG(download5340net,NULL,"Download nRF5340 network firmware\nUsage: ota download5340net <filename>",cmd_ota_download_5340net,2,0),
    SHELL_CMD_ARG(verify9151, NULL, "Verify downloaded nRF9151 image",
                  cmd_ota_verify_9151, 1, 0),
    SHELL_CMD_ARG(verify5340, NULL, "Verify downloaded nRF5340 image",
                  cmd_ota_verify_5340, 1, 0),
    SHELL_CMD_ARG(apply9151, NULL, "Apply nRF9151 firmware update and reboot",
                  cmd_ota_apply_9151, 1, 0),
    SHELL_CMD_ARG(apply5340, NULL, "Apply nRF5340 firmware update",
                  cmd_ota_apply_5340, 1, 0),
    SHELL_CMD_ARG(server, NULL, "Set OTA server\nUsage: ota server <url> [cert_tag]",
                  cmd_ota_set_server, 2, 1),
    SHELL_CMD_ARG(status, NULL, "Show OTA status", cmd_ota_status, 1, 0),
    SHELL_CMD_ARG(cancel, NULL, "Cancel OTA operation", cmd_ota_cancel_shell, 1, 0),
    SHELL_SUBCMD_SET_END
);
#endif

SHELL_CMD_REGISTER(ota, &ota_cmds, "OTA commands", NULL);