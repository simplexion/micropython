/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include "ble_task.h"

#include <os_msg.h>
#include <os_task.h>
#include <gap.h>
#include <gap_le.h>
#include <app_msg.h>
#include <stdio.h>
#include <basic_types.h>
#include <gap_msg.h>

#include <stdio.h>
#include <trace_app.h>
#include <string.h>
#include <gap.h>
#include <gap_adv.h>
#include <gap_bond_le.h>
#include <profile_server.h>
#include <gap_msg.h>
#include <simple_ble_service.h>
#include <bas.h>
#include <app_msg.h>
#include "peripheral_app.h"
#include <gap_conn_le.h>
#include "platform_stdlib.h"
#include "ble_peripheral_at_cmd.h"
#include "app_flags.h"
#include "os_msg.h"
#include "os_sync.h"
#include "os_task.h"
#include "os_timer.h"
#include "os_sched.h"
#include "vendor_cmd_bt.h"
#include "ftl_app.h"

/*============================================================================*
 *                              Macros
 *============================================================================*/
#define APP_TASK_PRIORITY             1         //!< Task priorities
#define APP_TASK_STACK_SIZE           256 * 4   //!<  Task stack size
#define MAX_NUMBER_OF_GAP_MESSAGE     0x20      //!<  GAP message queue size
#define MAX_NUMBER_OF_IO_MESSAGE      0x20      //!<  IO message queue size
#define MAX_NUMBER_OF_EVENT_MESSAGE   (MAX_NUMBER_OF_GAP_MESSAGE + MAX_NUMBER_OF_IO_MESSAGE)    //!< Event message queue size

/*============================================================================*
 *                              Variables
 *============================================================================*/
void *ble_task_handle = NULL;   //!< BLE Task handle
void *ble_evt_queue_handle = NULL;  //!< Event queue handle
void *ble_io_queue_handle = NULL;   //!< IO queue handle

extern T_GAP_DEV_STATE gap_dev_state;

static void ble_task(void *p_param);
static void ble_handle_io_msg(T_IO_MSG io_msg);
static void ble_handle_gap_msg(T_IO_MSG *p_gap_msg);
static void ble_handle_dev_state_evt(T_GAP_DEV_STATE new_state, uint16_t cause);

void ble_task_init(void) {
    os_task_create(&ble_task_handle, "ble", ble_task, 0, APP_TASK_STACK_SIZE,
                   APP_TASK_PRIORITY);
}

/**
 * @brief        App task to handle events & messages
 * @param[in]    p_param    Parameters sending to the task
 * @return       void
 */
static void ble_task(void *p_param)
{
    // similar to https://github.com/ambiot/ambd_sdk/blob/be744260b9230cf01eafcc741d0bd4504cf8c59e/component/common/bluetooth/realtek/sdk/example/ble_peripheral/app_task.c#L65
    (void)p_param;
    uint8_t event;
    os_msg_queue_create(&ble_io_queue_handle, MAX_NUMBER_OF_IO_MESSAGE, sizeof(T_IO_MSG));
    os_msg_queue_create(&ble_evt_queue_handle, MAX_NUMBER_OF_EVENT_MESSAGE, sizeof(uint8_t));

    gap_start_bt_stack(ble_evt_queue_handle, ble_io_queue_handle, MAX_NUMBER_OF_GAP_MESSAGE);

    driver_init();
    while (true)
    {
        if (os_msg_recv(ble_evt_queue_handle, &event, 0xFFFFFFFF) == true)
        {
            if (event == EVENT_IO_TO_APP)
            {
                T_IO_MSG io_msg;
                if (os_msg_recv(ble_io_queue_handle, &io_msg, 0) == true)
                {
                    ble_handle_io_msg(io_msg);
                }
            }
            else
            {
                gap_handle_msg(event);
            }
        }
    }
}

void app_handle_gap_msg(T_IO_MSG  *p_gap_msg);
/**
 * @brief    All the application messages are pre-handled in this function
 * @note     All the IO MSGs are sent to this function, then the event handling
 *           function shall be called according to the MSG type.
 * @param[in] io_msg  IO message data
 * @return   void
 */
static void ble_handle_io_msg(T_IO_MSG io_msg)
{
    uint16_t msg_type = io_msg.type;

    switch (msg_type)
    {
    case IO_MSG_TYPE_BT_STATUS:
        {
            ble_handle_gap_msg(&io_msg);
        }
        break;
//     case IO_MSG_TYPE_AT_CMD:
//         {
//             uint16_t subtype = io_msg.subtype;
//             void *arg = io_msg.u.buf;
//             ble_peripheral_app_handle_at_cmd(subtype, arg);
//         }
//         break;
//     case IO_MSG_TYPE_QDECODE:
//         {
//             if (io_msg.subtype == 0) {
//                 le_adv_stop();
//             } else if (io_msg.subtype == 1) {
//                 le_adv_start();
// #if (LEGACY_ADV_CONCURRENT == 1)
//             } else if (io_msg.subtype == 2) {
//                 if (lac_adapter.start_stop_flag == true)
//                     break;
//                 T_GAP_CAUSE ret = GAP_CAUSE_SUCCESS;
//                 ret = le_adv_update_param();
//                 if (ret != GAP_CAUSE_SUCCESS) {
//                     printf("le_adv_update_param fail! ret = 0x%x\r\n", ret);
//                 }
// #endif
//             }
//         }
//         break;
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
static void ble_handle_gap_msg(T_IO_MSG *p_gap_msg)
{
    T_LE_GAP_MSG gap_msg;
    uint8_t conn_id;
    memcpy(&gap_msg, &p_gap_msg->u.param, sizeof(p_gap_msg->u.param));

    APP_PRINT_TRACE1("app_handle_gap_msg: subtype %d", p_gap_msg->subtype);
    switch (p_gap_msg->subtype)
    {
    case GAP_MSG_LE_DEV_STATE_CHANGE:
        {
            ble_handle_dev_state_evt(gap_msg.msg_data.gap_dev_state_change.new_state,
                                     gap_msg.msg_data.gap_dev_state_change.cause);
        }
        break;

    case GAP_MSG_LE_CONN_STATE_CHANGE:
        {
            app_handle_conn_state_evt(gap_msg.msg_data.gap_conn_state_change.conn_id,
                                      (T_GAP_CONN_STATE)gap_msg.msg_data.gap_conn_state_change.new_state,
                                      gap_msg.msg_data.gap_conn_state_change.disc_cause);
        }
        break;

    case GAP_MSG_LE_CONN_MTU_INFO:
        {
            app_handle_conn_mtu_info_evt(gap_msg.msg_data.gap_conn_mtu_info.conn_id,
                                         gap_msg.msg_data.gap_conn_mtu_info.mtu_size);
        }
        break;

    case GAP_MSG_LE_CONN_PARAM_UPDATE:
        {
            app_handle_conn_param_update_evt(gap_msg.msg_data.gap_conn_param_update.conn_id,
                                             gap_msg.msg_data.gap_conn_param_update.status,
                                             gap_msg.msg_data.gap_conn_param_update.cause);
        }
        break;

    case GAP_MSG_LE_AUTHEN_STATE_CHANGE:
        {
            app_handle_authen_state_evt(gap_msg.msg_data.gap_authen_state.conn_id,
                                        gap_msg.msg_data.gap_authen_state.new_state,
                                        gap_msg.msg_data.gap_authen_state.status);
        }
        break;

    case GAP_MSG_LE_BOND_JUST_WORK:
        {
            conn_id = gap_msg.msg_data.gap_bond_just_work_conf.conn_id;
            le_bond_just_work_confirm(conn_id, GAP_CFM_CAUSE_ACCEPT);
            APP_PRINT_INFO0("GAP_MSG_LE_BOND_JUST_WORK");
        }
        break;

    case GAP_MSG_LE_BOND_PASSKEY_DISPLAY:
        {
            uint32_t display_value = 0;
            conn_id = gap_msg.msg_data.gap_bond_passkey_display.conn_id;
            le_bond_get_display_key(conn_id, &display_value);
            APP_PRINT_INFO1("GAP_MSG_LE_BOND_PASSKEY_DISPLAY:passkey %d", display_value);
            le_bond_passkey_display_confirm(conn_id, GAP_CFM_CAUSE_ACCEPT);
            printf("GAP_MSG_LE_BOND_PASSKEY_DISPLAY:passkey %06d\r\n", display_value);
        }
        break;

    case GAP_MSG_LE_BOND_USER_CONFIRMATION:
        {
            uint32_t display_value = 0;
            conn_id = gap_msg.msg_data.gap_bond_user_conf.conn_id;
            le_bond_get_display_key(conn_id, &display_value);
            APP_PRINT_INFO1("GAP_MSG_LE_BOND_USER_CONFIRMATION: passkey %d", display_value);
            printf("GAP_MSG_LE_BOND_USER_CONFIRMATION: passkey %06d\r\n", display_value);
            //le_bond_user_confirm(conn_id, GAP_CFM_CAUSE_ACCEPT);
        }
        break;

    case GAP_MSG_LE_BOND_PASSKEY_INPUT:
        {
            //uint32_t passkey = 888888;
            conn_id = gap_msg.msg_data.gap_bond_passkey_input.conn_id;
            APP_PRINT_INFO1("GAP_MSG_LE_BOND_PASSKEY_INPUT: conn_id %d", conn_id);
            //le_bond_passkey_input_confirm(conn_id, passkey, GAP_CFM_CAUSE_ACCEPT);
		    printf("GAP_MSG_LE_BOND_PASSKEY_INPUT: conn_id %d\r\n", conn_id);
        }
        break;
#if F_BT_LE_SMP_OOB_SUPPORT
    case GAP_MSG_LE_BOND_OOB_INPUT:
        {
            uint8_t oob_data[GAP_OOB_LEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            conn_id = gap_msg.msg_data.gap_bond_oob_input.conn_id;
            APP_PRINT_INFO0("GAP_MSG_LE_BOND_OOB_INPUT");
            le_bond_set_param(GAP_PARAM_BOND_OOB_DATA, GAP_OOB_LEN, oob_data);
            le_bond_oob_input_confirm(conn_id, GAP_CFM_CAUSE_ACCEPT);
        }
        break;
#endif
    default:
        APP_PRINT_ERROR1("app_handle_gap_msg: unknown subtype %d", p_gap_msg->subtype);
        break;
    }
}

/**
 * @brief    Handle msg GAP_MSG_LE_DEV_STATE_CHANGE
 * @note     All the gap device state events are pre-handled in this function.
 *           Then the event handling function shall be called according to the new_state
 * @param[in] new_state  New gap device state
 * @param[in] cause GAP device state change cause
 * @return   void
 */
static void ble_handle_dev_state_evt(T_GAP_DEV_STATE new_state, uint16_t cause)
{
    APP_PRINT_INFO3("app_handle_dev_state_evt: init state %d, adv state %d, cause 0x%x",
                    new_state.gap_init_state, new_state.gap_adv_state, cause);
    if (gap_dev_state.gap_init_state != new_state.gap_init_state)
    {
        if (new_state.gap_init_state == GAP_INIT_STATE_STACK_READY)
        {
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

    if (gap_dev_state.gap_adv_state != new_state.gap_adv_state)
    {
        if (new_state.gap_adv_state == GAP_ADV_STATE_IDLE)
        {
            if (new_state.gap_adv_sub_state == GAP_ADV_TO_IDLE_CAUSE_CONN)
            {
                APP_PRINT_INFO0("GAP adv stoped: because connection created");
				printf("\n\rGAP adv stoped: because connection created\n\r");
            }
            else
            {
                APP_PRINT_INFO0("GAP adv stoped");
				printf("\n\rGAP adv stopped\n\r");
            }
        }
        else if (new_state.gap_adv_state == GAP_ADV_STATE_ADVERTISING)
        {
            APP_PRINT_INFO0("GAP adv start");
			printf("\n\rGAP adv start\n\r");
        }
    }

    gap_dev_state = new_state;
}
