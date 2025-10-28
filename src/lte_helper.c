#include "lte_helper.h"
#include "fota.h"
#include <modem/pdn.h>
LOG_MODULE_REGISTER(lte, LOG_LEVEL_INF);


K_MUTEX_DEFINE(json_mutex);
K_SEM_DEFINE(lte_connected, 0, 1);


/**
 * @brief LTE event handler
 */

 
void lte_handler(const struct lte_lc_evt *const evt)
{
    switch (evt->type) {
    case LTE_LC_EVT_NW_REG_STATUS:
        if ((evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME) ||
            (evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING)) {
            LOG_INF("Network registration status: %s",
                    evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ?
                    "Connected - home network" : "Connected - roaming");
            k_sem_give(&lte_connected);
            if (current_state == FOTA_IDLE) {
                set_state(FOTA_CONNECTED, 0);
            }
        } else {
            if (current_state == FOTA_CONNECTED) {
                LOG_INF("Disconnected from LTE network while in FOTA_CONNECTED state");
                set_state(FOTA_IDLE, 0);
            }
        }
        break;

    case LTE_LC_EVT_RRC_UPDATE:
        LOG_INF("RRC mode: %s", evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ? "Connected" : "Idle");
        if (evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED) {
            if (current_state == FOTA_IDLE) {
                set_state(FOTA_CONNECTED, 0);
            }
        } else {
            if (current_state == FOTA_DOWNLOADING) {
                set_state(FOTA_IDLE, 0);
            }
        }
        break;

    case LTE_LC_EVT_CELL_UPDATE:
        LOG_INF("LTE cell changed: Cell ID: %d, Tracking area: %d",
                evt->cell.id, evt->cell.tac);
        break;

    case LTE_LC_EVT_LTE_MODE_UPDATE:
        switch (evt->lte_mode) {
        case LTE_LC_LTE_MODE_LTEM:
            LOG_INF("LTE mode updated: LTE-M");
            break;
        case LTE_LC_LTE_MODE_NBIOT:
            LOG_INF("LTE mode updated: NB-IoT");
            break;
        case LTE_LC_LTE_MODE_NONE:
            LOG_INF("LTE mode updated: None (off)");
            if (current_state == FOTA_CONNECTED || current_state == FOTA_DOWNLOADING) {
                set_state(FOTA_IDLE, 0);
            }
            break;
        default:
            LOG_INF("LTE mode updated: Unknown");
            break;
        }
        break;

    default:
        break;
    }
}



/**
 * @brief Configure and connect to LTE network
 */
int modem_configure(void)
{
    int err;

    LOG_INF("Starting modem configuration...");

    err = nrf_modem_lib_init();
    if (err) {
        LOG_ERR("Failed to initialize the modem library, error: %d", err);
        return err;
    }


    err = fota_init(set_state);
    if (err) {
        LOG_ERR("FOTA init failed: %d", err);
        return err;
    }
   
    provision_all_tls_credentials();

    lte_lc_system_mode_set(LTE_LC_SYSTEM_MODE_LTEM, LTE_LC_SYSTEM_MODE_PREFER_AUTO);
   
    lte_lc_psm_req(false);
    
    lte_lc_func_mode_set(LTE_LC_FUNC_MODE_NORMAL);
  
    err = lte_lc_connect_async(lte_handler);
    if (err) {
        LOG_ERR("Error in lte_lc_connect_async, error: %d", err);
        return err;
    }
   
    err = k_sem_take(&lte_connected, K_FOREVER); // 30 second timeout
    if (err) {
        LOG_ERR("LTE connection timeout after 30 seconds");
        return err;
    }
    LOG_INF("Connected to LTE network successfully");

    return 0;
}