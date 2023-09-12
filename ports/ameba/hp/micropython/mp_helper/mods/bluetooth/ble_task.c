/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include "ble_task.h"

#include "extmod/modbluetooth.h"

#include <os_msg.h>
#include <os_task.h>
#include <gap.h>
#include <gap_le.h>
#include <app_msg.h>
#include <stdio.h>
#include <basic_types.h>
#include <gap_msg.h>

#include <trace_app.h>
#include <string.h>
#include <gap_adv.h>
#include <gap_bond_le.h>
#include <profile_server.h>
#include <bas.h>
#include <gap_conn_le.h>
#include "app_flags.h"
// #include "os_sync.h"
// #include "os_timer.h"
// #include "os_sched.h"
#include "ftl_app.h"

/*============================================================================*
 *                              Macros
 *============================================================================*/
#define APP_TASK_PRIORITY             1         // !< Task priorities
#define APP_TASK_STACK_SIZE           256 * 4   // !<  Task stack size
#define MAX_NUMBER_OF_GAP_MESSAGE     0x20      // !<  GAP message queue size
#define MAX_NUMBER_OF_IO_MESSAGE      0x20      // !<  IO message queue size
#define MAX_NUMBER_OF_EVENT_MESSAGE   (MAX_NUMBER_OF_GAP_MESSAGE + MAX_NUMBER_OF_IO_MESSAGE)    // !< Event message queue size

/*============================================================================*
 *                              Variables
 *============================================================================*/
void *ble_task_handle = NULL;   // !< BLE Task handle
void *ble_evt_queue_handle = NULL;  // !< Event queue handle
void *ble_io_queue_handle = NULL;   // !< IO queue handle

static T_GAP_DEV_STATE ble_gap_dev_state = { 0, 0, 0, 0 }; /**< GAP device state */
static T_GAP_CONN_STATE ble_gap_conn_state = GAP_CONN_STATE_DISCONNECTED; /**< GAP connection state */

static void ble_task(void *p_param);
static void ble_handle_io_msg(T_IO_MSG io_msg);
static void ble_handle_gap_msg(T_IO_MSG *p_gap_msg);
static void ble_handle_dev_state_evt(T_GAP_DEV_STATE new_state, uint16_t cause);
static void ble_handle_conn_state_evt(uint8_t conn_id, T_GAP_CONN_STATE new_state, uint16_t disc_cause);
static void ble_handle_conn_mtu_info_evt(uint8_t conn_id, uint16_t mtu_size);
static void ble_handle_conn_param_update_evt(uint8_t conn_id, uint8_t status, uint16_t cause);
static void ble_handle_authen_state_evt(uint8_t conn_id, uint8_t new_state, uint16_t cause);

void ble_task_init(void) {
    // similar to https://github.com/ambiot/ambd_sdk/blob/be744260b9230cf01eafcc741d0bd4504cf8c59e/component/common/bluetooth/realtek/sdk/example/ble_peripheral/app_task.c#L99
    os_task_create(&ble_task_handle, "ble", ble_task, 0, APP_TASK_STACK_SIZE,
        APP_TASK_PRIORITY);
}

void ble_task_deinit(void) {
    // similar to https://github.com/ambiot/ambd_sdk/blob/be744260b9230cf01eafcc741d0bd4504cf8c59e/component/common/bluetooth/realtek/sdk/example/ble_peripheral/app_task.c#L105
    if (ble_task_handle) {
        os_task_delete(ble_task_handle);
    }
    if (ble_io_queue_handle) {
        os_msg_queue_delete(ble_io_queue_handle);
    }
    if (ble_evt_queue_handle) {
        os_msg_queue_delete(ble_evt_queue_handle);
    }
    ble_io_queue_handle = NULL;
    ble_evt_queue_handle = NULL;
    ble_task_handle = NULL;

    ble_gap_dev_state.gap_init_state = 0;
    ble_gap_dev_state.gap_adv_sub_state = 0;
    ble_gap_dev_state.gap_adv_state = 0;
    ble_gap_dev_state.gap_scan_state = 0;
    ble_gap_dev_state.gap_conn_state = 0;
}

/**
 * @brief        App task to handle events & messages
 * @param[in]    p_param    Parameters sending to the task
 * @return       void
 */
static void ble_task(void *p_param) {
    // similar to https://github.com/ambiot/ambd_sdk/blob/be744260b9230cf01eafcc741d0bd4504cf8c59e/component/common/bluetooth/realtek/sdk/example/ble_peripheral/app_task.c#L65
    (void)p_param;
    uint8_t event;
    os_msg_queue_create(&ble_io_queue_handle, MAX_NUMBER_OF_IO_MESSAGE, sizeof(T_IO_MSG));
    os_msg_queue_create(&ble_evt_queue_handle, MAX_NUMBER_OF_EVENT_MESSAGE, sizeof(uint8_t));

    gap_start_bt_stack(ble_evt_queue_handle, ble_io_queue_handle, MAX_NUMBER_OF_GAP_MESSAGE);

    while (true) {
        if (os_msg_recv(ble_evt_queue_handle, &event, 0xFFFFFFFF) == true) {
            if (event == EVENT_IO_TO_APP) {
                T_IO_MSG io_msg;
                if (os_msg_recv(ble_io_queue_handle, &io_msg, 0) == true) {
                    ble_handle_io_msg(io_msg);
                }
            } else {
                gap_handle_msg(event);
            }
        }
    }
}

/**
 * @brief    All the application messages are pre-handled in this function
 * @note     All the IO MSGs are sent to this function, then the event handling
 *           function shall be called according to the MSG type.
 * @param[in] io_msg  IO message data
 * @return   void
 */
static void ble_handle_io_msg(T_IO_MSG io_msg) {
    // similar to https://github.com/ambiot/ambd_sdk/blob/be744260b9230cf01eafcc741d0bd4504cf8c59e/component/common/bluetooth/realtek/sdk/example/ble_peripheral/peripheral_app.c#L167
    uint16_t msg_type = io_msg.type;

    switch (msg_type)
    {
        case IO_MSG_TYPE_BT_STATUS: {
            ble_handle_gap_msg(&io_msg);
        }
        break;
        default:
            break;
    }
}

/**
 * @brief    All the BT GAP MSG are pre-handled in this function.
 * @note     Then the event handling function shall be called according to the
 *           subtype of T_IO_MSG
 * @param[in] p_gap_msg Pointer to GAP msg
 * @return   void
 */
static void ble_handle_gap_msg(T_IO_MSG *p_gap_msg) {
    // similar to https://github.com/ambiot/ambd_sdk/blob/be744260b9230cf01eafcc741d0bd4504cf8c59e/component/common/bluetooth/realtek/sdk/example/ble_peripheral/peripheral_app.c#L443
    T_LE_GAP_MSG gap_msg;
    uint8_t conn_id;
    memcpy(&gap_msg, &p_gap_msg->u.param, sizeof(p_gap_msg->u.param));

    APP_PRINT_TRACE2("%s: subtype %d", __func__, p_gap_msg->subtype);
    switch (p_gap_msg->subtype)
    {
        case GAP_MSG_LE_DEV_STATE_CHANGE: {
            ble_handle_dev_state_evt(gap_msg.msg_data.gap_dev_state_change.new_state,
                gap_msg.msg_data.gap_dev_state_change.cause);
        }
        break;

        case GAP_MSG_LE_CONN_STATE_CHANGE: {
            ble_handle_conn_state_evt(gap_msg.msg_data.gap_conn_state_change.conn_id,
                (T_GAP_CONN_STATE)gap_msg.msg_data.gap_conn_state_change.new_state,
                gap_msg.msg_data.gap_conn_state_change.disc_cause);
        }
        break;

        case GAP_MSG_LE_CONN_MTU_INFO: {
            ble_handle_conn_mtu_info_evt(gap_msg.msg_data.gap_conn_mtu_info.conn_id,
                gap_msg.msg_data.gap_conn_mtu_info.mtu_size);
        }
        break;

        case GAP_MSG_LE_CONN_PARAM_UPDATE: {
            ble_handle_conn_param_update_evt(gap_msg.msg_data.gap_conn_param_update.conn_id,
                gap_msg.msg_data.gap_conn_param_update.status,
                gap_msg.msg_data.gap_conn_param_update.cause);
        }
        break;

        case GAP_MSG_LE_AUTHEN_STATE_CHANGE: {
            ble_handle_authen_state_evt(gap_msg.msg_data.gap_authen_state.conn_id,
                gap_msg.msg_data.gap_authen_state.new_state,
                gap_msg.msg_data.gap_authen_state.status);
        }
        break;

        case GAP_MSG_LE_BOND_JUST_WORK: {
            conn_id = gap_msg.msg_data.gap_bond_just_work_conf.conn_id;
            le_bond_just_work_confirm(conn_id, GAP_CFM_CAUSE_ACCEPT);
            APP_PRINT_INFO0("GAP_MSG_LE_BOND_JUST_WORK");
        }
        break;

        case GAP_MSG_LE_BOND_PASSKEY_DISPLAY: {
            uint32_t display_value = 0;
            conn_id = gap_msg.msg_data.gap_bond_passkey_display.conn_id;
            le_bond_get_display_key(conn_id, &display_value);
            APP_PRINT_INFO1("GAP_MSG_LE_BOND_PASSKEY_DISPLAY:passkey %d", display_value);
            le_bond_passkey_display_confirm(conn_id, GAP_CFM_CAUSE_ACCEPT);
            printf("GAP_MSG_LE_BOND_PASSKEY_DISPLAY:passkey %06d\r\n", display_value);
        }
        break;

        case GAP_MSG_LE_BOND_USER_CONFIRMATION: {
            uint32_t display_value = 0;
            conn_id = gap_msg.msg_data.gap_bond_user_conf.conn_id;
            le_bond_get_display_key(conn_id, &display_value);
            APP_PRINT_INFO1("GAP_MSG_LE_BOND_USER_CONFIRMATION: passkey %d", display_value);
            printf("GAP_MSG_LE_BOND_USER_CONFIRMATION: passkey %06d\r\n", display_value);
            // le_bond_user_confirm(conn_id, GAP_CFM_CAUSE_ACCEPT);
        }
        break;

        case GAP_MSG_LE_BOND_PASSKEY_INPUT: {
            // uint32_t passkey = 888888;
            conn_id = gap_msg.msg_data.gap_bond_passkey_input.conn_id;
            APP_PRINT_INFO1("GAP_MSG_LE_BOND_PASSKEY_INPUT: conn_id %d", conn_id);
            // le_bond_passkey_input_confirm(conn_id, passkey, GAP_CFM_CAUSE_ACCEPT);
            printf("GAP_MSG_LE_BOND_PASSKEY_INPUT: conn_id %d\r\n", conn_id);
        }
        break;
            #if F_BT_LE_SMP_OOB_SUPPORT
        case GAP_MSG_LE_BOND_OOB_INPUT: {
            uint8_t oob_data[GAP_OOB_LEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            conn_id = gap_msg.msg_data.gap_bond_oob_input.conn_id;
            APP_PRINT_INFO0("GAP_MSG_LE_BOND_OOB_INPUT");
            le_bond_set_param(GAP_PARAM_BOND_OOB_DATA, GAP_OOB_LEN, oob_data);
            le_bond_oob_input_confirm(conn_id, GAP_CFM_CAUSE_ACCEPT);
        }
        break;
            #endif
        default:
            APP_PRINT_ERROR2("%s: unknown subtype %d", __func__, p_gap_msg->subtype);
            break;
    }
}

/**
 * Callback for gap le to notify app.
 *
 * @param[in] cb_type callback msy type.
 * @param[in] p_cb_data point to callback data.
 * @retval Success of operation.
 */
T_APP_RESULT
ble_gap_callback(uint8_t cb_type, void* p_cb_data)
{
    T_APP_RESULT result = APP_RESULT_SUCCESS;
    T_LE_CB_DATA* p_data = (T_LE_CB_DATA*)p_cb_data;

    switch (cb_type) {
    case GAP_MSG_LE_DATA_LEN_CHANGE_INFO:
        // Logf(LogLevel, Debug,
        //     "GAP_MSG_LE_DATA_LEN_CHANGE_INFO: conn_id %d, tx octets 0x%x, "
        //     "max_tx_time 0x%x",
        //     p_data->p_le_data_len_change_info->conn_id,
        //     p_data->p_le_data_len_change_info->max_tx_octets,
        //     p_data->p_le_data_len_change_info->max_tx_time);
        break;

    case GAP_MSG_LE_MODIFY_WHITE_LIST:
        // Logf(LogLevel, Debug, "GAP_MSG_LE_MODIFY_WHITE_LIST: operation %d, cause 0x%x",
        //     p_data->p_le_modify_white_list_rsp->operation,
        //     p_data->p_le_modify_white_list_rsp->cause);
        break;
#if (LEGACY_ADV_CONCURRENT == 1)
    case GAP_MSG_LE_ADV_UPDATE_PARAM:
        APP_PRINT_INFO1("GAP_MSG_LE_ADV_UPDATE_PARAM: cause 0x%x",
                      p_data->p_le_adv_update_param_rsp->cause);
        if (p_data->p_le_adv_update_param_rsp->cause == 0) {
            if (lac_adapter.start_stop_flag == true)
                break;
            T_GAP_CAUSE ret = GAP_CAUSE_SUCCESS;
#if BT_VENDOR_CMD_ONE_SHOT_SUPPORT
            ret = le_vendor_one_shot_adv();
#endif
            if (ret != GAP_CAUSE_SUCCESS) {
                printf("le_vendor_one_shot_adv fail! ret = 0x%x\r\n", ret);
            }
        } else
            printf("GAP_MSG_LE_ADV_UPDATE_PARAM: cause 0x%x\r\n", p_data->p_le_adv_update_param_rsp->cause);
        break;
#endif
    case GAP_MSG_LE_BOND_MODIFY_INFO:
        // Logf(LogLevel, Debug, "GAP_MSG_LE_BOND_MODIFY_INFO: type 0x%x",
        //     p_data->p_le_bond_modify_info->type);
        // privacy_handle_bond_modify_msg(p_data->p_le_bond_modify_info->type,
        //     p_data->p_le_bond_modify_info->p_entry,
        //     true);
        break;
    default:
        // APP_PRINT_ERROR1("app_gap_callback: unhandled cb_type 0x%x", cb_type);
        break;
    }
    return result;
}

/**
 * @brief    Handle msg GAP_MSG_LE_DEV_STATE_CHANGE
 * @note     All the gap device state events are pre-handled in this function.
 *           Then the event handling function shall be called according to the new_state
 * @param[in] new_state  New gap device state
 * @param[in] cause GAP device state change cause
 * @return   void
 */
static void ble_handle_dev_state_evt(T_GAP_DEV_STATE new_state, uint16_t cause) {
    APP_PRINT_INFO4("%s: init state %d, adv state %d, cause 0x%x",
        __func__, new_state.gap_init_state, new_state.gap_adv_state, cause);
    if (ble_gap_dev_state.gap_init_state != new_state.gap_init_state) {
        if (new_state.gap_init_state == GAP_INIT_STATE_STACK_READY) {
            APP_PRINT_INFO0("GAP stack ready");
            printf("\n\r[BLE peripheral] GAP stack ready\n\r");
            uint8_t bt_addr[6] = {0};
            /*stack ready*/
            gap_get_param(GAP_PARAM_BD_ADDR, bt_addr);
            printf("local bd addr: 0x%02x:%02x:%02x:%02x:%02x:%02x\r\n", \
                bt_addr[5], bt_addr[4], bt_addr[3], bt_addr[2], bt_addr[1], bt_addr[0]);
// #if (LEGACY_ADV_CONCURRENT == 1)	//Do not auto start ADV, need legacy_adv_concurrent_init for configuration
// #if (F_BT_LE_USE_RANDOM_ADDR == 1)
//             memcpy(local_public_addr, bt_addr, 6);
// #endif
// #else
//             le_adv_start();
// #endif
        }
    }

    if (ble_gap_dev_state.gap_adv_state != new_state.gap_adv_state) {
        if (new_state.gap_adv_state == GAP_ADV_STATE_IDLE) {
            if (new_state.gap_adv_sub_state == GAP_ADV_TO_IDLE_CAUSE_CONN) {
                APP_PRINT_INFO0("GAP adv stoped: because connection created");
                printf("\n\rGAP adv stoped: because connection created\n\r");
            } else {
                APP_PRINT_INFO0("GAP adv stoped");
                printf("\n\rGAP adv stopped\n\r");
            }
        } else if (new_state.gap_adv_state == GAP_ADV_STATE_ADVERTISING) {
            APP_PRINT_INFO0("GAP adv start");
            printf("\n\rGAP adv start\n\r");
        }
    }

    ble_gap_dev_state = new_state;
}

/**
 * @brief    Handle msg GAP_MSG_LE_CONN_STATE_CHANGE
 * @note     All the gap conn state events are pre-handled in this function.
 *           Then the event handling function shall be called according to the new_state
 * @param[in] conn_id Connection ID
 * @param[in] new_state  New gap connection state
 * @param[in] disc_cause Use this cause when new_state is GAP_CONN_STATE_DISCONNECTED
 * @return   void
 */
static void ble_handle_conn_state_evt(uint8_t conn_id, T_GAP_CONN_STATE new_state, uint16_t disc_cause) {
    uint16_t conn_interval;
    uint16_t conn_latency;
    uint16_t conn_supervision_timeout;
    uint8_t remote_bd[6];
    T_GAP_REMOTE_ADDR_TYPE remote_bd_type;

    le_get_conn_param(GAP_PARAM_CONN_INTERVAL, &conn_interval, conn_id);
    le_get_conn_param(GAP_PARAM_CONN_LATENCY, &conn_latency, conn_id);
    le_get_conn_param(GAP_PARAM_CONN_TIMEOUT, &conn_supervision_timeout, conn_id);
    le_get_conn_addr(conn_id, remote_bd, (void *)&remote_bd_type);

    APP_PRINT_INFO5("%s: conn_id %d old_state %d new_state %d, disc_cause 0x%x",
        __func__, conn_id, ble_gap_conn_state, new_state, disc_cause);
    switch (new_state)
    {
        case GAP_CONN_STATE_DISCONNECTED: {
            if ((disc_cause != (HCI_ERR | HCI_ERR_REMOTE_USER_TERMINATE))
                && (disc_cause != (HCI_ERR | HCI_ERR_LOCAL_HOST_TERMINATE))) {
                APP_PRINT_ERROR2("%s: connection lost cause 0x%x", __func__, disc_cause);
            }
//             printf("\n\r[BLE peripheral] BT Disconnected, cause 0x%x, start ADV\n\r", disc_cause);
// #if (LEGACY_ADV_CONCURRENT == 1)
//             legacy_adv_concurrent_start();
// #else
//             le_adv_start();
// #endif
            mp_bluetooth_gap_on_connected_disconnected(MP_BLUETOOTH_IRQ_CENTRAL_DISCONNECT, conn_id, remote_bd_type, remote_bd);
        }
        break;

        case GAP_CONN_STATE_CONNECTED: {
            APP_PRINT_INFO5("GAP_CONN_STATE_CONNECTED:remote_bd %s, remote_addr_type %d, conn_interval 0x%x, conn_latency 0x%x, conn_supervision_timeout 0x%x",
                TRACE_BDADDR(remote_bd), remote_bd_type,
                conn_interval, conn_latency, conn_supervision_timeout);
            printf("\n\r[BLE peripheral] BT Connected\n\r");
            #if (LEGACY_ADV_CONCURRENT == 1)
            legacy_adv_concurrent_stop();
            #else
            // Do nothing, stack auto stop ADV
            #endif
            mp_bluetooth_gap_on_connected_disconnected(MP_BLUETOOTH_IRQ_CENTRAL_CONNECT, conn_id, remote_bd_type, remote_bd);
        }
        break;

        default:
            break;
    }
    ble_gap_conn_state = new_state;
}

/**
 * @brief    Handle msg GAP_MSG_LE_CONN_MTU_INFO
 * @note     This msg is used to inform APP that exchange mtu procedure is completed.
 * @param[in] conn_id Connection ID
 * @param[in] mtu_size  New mtu size
 * @return   void
 */
static void ble_handle_conn_mtu_info_evt(uint8_t conn_id, uint16_t mtu_size) {
    APP_PRINT_INFO3("%s: conn_id %d, mtu_size %d", __func__, conn_id, mtu_size);
}

/**
 * @brief    Handle msg GAP_MSG_LE_CONN_PARAM_UPDATE
 * @note     All the connection parameter update change  events are pre-handled in this function.
 * @param[in] conn_id Connection ID
 * @param[in] status  New update state
 * @param[in] cause Use this cause when status is GAP_CONN_PARAM_UPDATE_STATUS_FAIL
 * @return   void
 */
static void ble_handle_conn_param_update_evt(uint8_t conn_id, uint8_t status, uint16_t cause) {
    switch (status)
    {
        case GAP_CONN_PARAM_UPDATE_STATUS_SUCCESS: {
            uint16_t conn_interval;
            uint16_t conn_slave_latency;
            uint16_t conn_supervision_timeout;

            le_get_conn_param(GAP_PARAM_CONN_INTERVAL, &conn_interval, conn_id);
            le_get_conn_param(GAP_PARAM_CONN_LATENCY, &conn_slave_latency, conn_id);
            le_get_conn_param(GAP_PARAM_CONN_TIMEOUT, &conn_supervision_timeout, conn_id);
            APP_PRINT_INFO4("%s update success:conn_interval 0x%x, conn_slave_latency 0x%x, conn_supervision_timeout 0x%x",
                __func__, conn_interval, conn_slave_latency, conn_supervision_timeout);
            printf("%s update success:conn_interval 0x%x, conn_slave_latency 0x%x, conn_supervision_timeout 0x%x\r\n",
                __func__, conn_interval, conn_slave_latency, conn_supervision_timeout);
        }
        break;

        case GAP_CONN_PARAM_UPDATE_STATUS_FAIL: {
            APP_PRINT_ERROR2("%s update failed: cause 0x%x", __func__, cause);
            printf("%s update failed: cause 0x%x\r\n", __func__, cause);
        }
        break;

        case GAP_CONN_PARAM_UPDATE_STATUS_PENDING: {
            APP_PRINT_INFO1("%s update pending.", __func__);
            printf("\n\r%s update pending: conn_id %d\r\n", __func__, conn_id);
        }
        break;

        default:
            break;
    }
}

/**
 * @brief    Handle msg GAP_MSG_LE_AUTHEN_STATE_CHANGE
 * @note     All the gap authentication state events are pre-handled in this function.
 *           Then the event handling function shall be called according to the new_state
 * @param[in] conn_id Connection ID
 * @param[in] new_state  New authentication state
 * @param[in] cause Use this cause when new_state is GAP_AUTHEN_STATE_COMPLETE
 * @return   void
 */
void ble_handle_authen_state_evt(uint8_t conn_id, uint8_t new_state, uint16_t cause) {
    APP_PRINT_INFO3("%s:conn_id %d, cause 0x%x", __func__, conn_id, cause);

    switch (new_state)
    {
        case GAP_AUTHEN_STATE_STARTED: {
            APP_PRINT_INFO1("%s: GAP_AUTHEN_STATE_STARTED", __func__);
        }
        break;

        case GAP_AUTHEN_STATE_COMPLETE: {
            if (cause == GAP_SUCCESS) {
                printf("Pair success\r\n");
                APP_PRINT_INFO1("%s: GAP_AUTHEN_STATE_COMPLETE pair success", __func__);

            } else {
                printf("Pair failed: cause 0x%x\r\n", cause);
                APP_PRINT_INFO1("%s: GAP_AUTHEN_STATE_COMPLETE pair failed", __func__);
            }
        }
        break;

        default: {
            APP_PRINT_ERROR2("%s: unknown newstate %d", __func__, new_state);
        }
        break;
    }
}
