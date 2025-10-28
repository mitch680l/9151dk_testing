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


typedef void (*fota_callback_t)(enum fota_state new_state, int error);

extern enum fota_state current_state;
extern fota_callback_t state_callback;




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
int fota_apply_update(void);


#endif /* FOTA_H */