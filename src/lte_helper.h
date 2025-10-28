#ifndef LTE_HELPER_H
#define LTE_HELPER_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h> 
#include <zephyr/device.h>     
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>   
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/net/socket.h>
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <dk_buttons_and_leds.h>
#include <nrf_modem_gnss.h>
#include <nrf_modem_at.h>
#include <zephyr/net/mqtt.h>
#include <modem/modem_info.h>
/*Operator Conversion Data*/

extern struct k_mutex json_mutex;

typedef struct {
    const char *mccmnc;
    const char *name;
} OperatorEntry;

static const OperatorEntry operator_table[] = {
    {"302720", "Canada - Rogers"},
    {"302610", "Canada - Bell"},
    {"302220", "Canada - Telus"},
    {"310260", "United States - T-Mobile"},
    {"310410", "United States - AT&T"},
    {"311480", "United States - Verizon"},
    {"312530", "United States - Dish"},
    {"313100", "United States - FirstNet"},
    {"334020", "Mexico - Telcel"},
    {"334030", "Mexico - AT&T Mexico"},
    {"334050", "Mexico - Movistar"},
};


extern struct k_sem lte_connected;
extern bool update_lte_info;
extern bool publish_lte_info;


int setup_default_apn();
void lte_handler(const struct lte_lc_evt *const evt);
void pack_lte_data(void);
const char *lookup_operator_name(const char *mccmnc);
int modem_configure(void);
#endif