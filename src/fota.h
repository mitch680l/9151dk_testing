#ifndef FOTA_H
#define FOTA_H

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/storage/flash_map.h>

#ifndef THIS_IS_5340
/* Download functionality only needed on nRF9151 */
#include <net/download_client.h>
#include <zephyr/storage/stream_flash.h>
#include <zephyr/net/tls_credentials.h>

/* Download state */
enum download_state {
    DOWNLOAD_IDLE,
    DOWNLOAD_IN_PROGRESS,
    DOWNLOAD_COMPLETE,
    DOWNLOAD_ERROR
};

/* Download callback type */
typedef void (*download_callback_t)(enum download_state state, int error);

struct ota_config_t {
    const char *server_addr;
    const char *cert_tag;
};

extern struct ota_config_t ota_config;

void ota_init(download_callback_t callback);
void ota_set_server(const char *server_addr, const char *cert_tag);
int ota_download_9151(const char *filename);
int ota_download_5340(const char *filename);
int ota_cancel(void);

#endif /* !THIS_IS_5340 */

/* Functions available on both 9151 and 5340 */
int ota_apply(void);

#endif /* FOTA_H */