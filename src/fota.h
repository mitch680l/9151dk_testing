#ifndef FOTA_H
#define FOTA_H

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/sys/reboot.h>
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <modem/modem_key_mgmt.h>
#include <net/fota_download.h>
#include <dfu/dfu_target_mcuboot.h>

/* FOTA state machine states */
enum fota_state {
    FOTA_IDLE,
    FOTA_CONNECTED,
    FOTA_DOWNLOADING,
    FOTA_READY_TO_APPLY,
    FOTA_APPLYING
};

typedef void (*fota_callback_t)(enum fota_state new_state, int error);

extern enum fota_state current_state;
extern fota_callback_t state_callback;

struct ota_config_t {
    const char *server_addr;
    const char *cert_tag;
};

extern struct ota_config_t ota_config;
extern const char *firmware_filename;




void fota_work_cb(struct k_work *work);
void fota_dl_handler(const struct fota_download_evt *evt);
int modem_configure_and_connect(void);
int download_firmware(void);
void set_state(enum fota_state new_state, int error);
int fota_init(fota_callback_t callback);
int fota_apply_update(void);
int check_fota_server(void);
int fota_cancel(void);
enum fota_state fota_get_state(void);
void fota_set_server(const char *server_addr, const char *cert_tag);
void fota_set_filename(const char *filename);


#endif /* FOTA_H */