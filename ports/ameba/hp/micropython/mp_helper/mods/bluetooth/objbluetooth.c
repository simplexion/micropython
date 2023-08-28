/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Chester Tseng
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// MP include
#include "objbluetooth.h"
#include "ble_task.h"
#include "servicehelper.h"

#include "../wireless/objwlan.h"

#include "extmod/modbluetooth.h"

#include <gap.h>
#include <gap_adv.h>
#include <gap_bond_le.h>
// #include <gap_lib/gap_config.h>
#include <gap_msg.h>
#include <gap_privacy.h>
#include <profile_server.h>

#include <os_sched.h>

enum {
    MP_BLUETOOTH_AMEBA_BLE_STATE_OFF,
    MP_BLUETOOTH_AMEBA_BLE_STATE_ACTIVE,
    MP_BLUETOOTH_AMEBA_BLE_STATE_SUSPENDED,
};

enum {
    MP_BLUETOOTH_AMEBA_GAP_SCAN_STATE_INACTIVE,
    MP_BLUETOOTH_AMEBA_GAP_SCAN_STATE_DEACTIVATING,
    MP_BLUETOOTH_AMEBA_GAP_SCAN_STATE_ACTIVE,
};

extern T_GAP_DEV_STATE gap_dev_state;
STATIC volatile int mp_bluetooth_ameba_ble_state = MP_BLUETOOTH_AMEBA_BLE_STATE_OFF;

#define MP_BLUETOOTH_MAX_SERVICES (8)

typedef struct _mp_bluetooth_ameba_root_pointers_t {
    // Characteristic (and descriptor) value storage.
    mp_gatts_db_t gatts_db;

    // Pending service definitions.
    uint8_t n_services;
    void *semaphore_send;
    struct {
        T_SERVER_ID id;
        uint8_t n_attributes;
        Attribute_t *attributes;
    } services[MP_BLUETOOTH_MAX_SERVICES];
} mp_bluetooth_ameba_root_pointers_t;


int mp_bluetooth_init(void) {
    // Clean up if necessary.
    mp_bluetooth_deinit();

    // Allocate memory for state.
    MP_STATE_PORT(bluetooth_ameba_root_pointers) = m_new0(mp_bluetooth_ameba_root_pointers_t, 1);
    mp_bluetooth_gatts_db_create(&MP_STATE_PORT(bluetooth_ameba_root_pointers)->gatts_db);

    /* according to
     * https://github.com/simplexion/ambd_sdk/blob/1a8f7ca7583c10301bf3170f60af99280559c431/component/common/bluetooth/realtek/sdk/inc/stack/bte.h#L22
     * and
     * https://github.com/simplexion/ambd_sdk/blob/1a8f7ca7583c10301bf3170f60af99280559c431/component/common/bluetooth/realtek/sdk/example/ble_peripheral/ble_app_main.c#L287C5-L287C17
     * , and similar to the zephyr port
     */
    if (mp_bluetooth_ameba_ble_state == MP_BLUETOOTH_AMEBA_BLE_STATE_OFF) {

        if (! mp_bluetooth_is_active()) {
            // T_GAP_DEV_STATE new_state;

            // for some reason bluetooth requires active wifi
            wlan_init0();

            bt_trace_init();

            // bt_stack_config_init(); unrolled
            gap_config_max_le_link_num(APP_MAX_LINKS);
            gap_config_max_le_paired_device(APP_MAX_LINKS);

            bte_init();

            // board_init(); not applicable

            le_gap_init(APP_MAX_LINKS);

            /* equivalent to app_le_gap_init(); will be performed at a later time
             * since this requires advertisement data etc., which will be provided
             * in Python code
             */

            /* equivalent to app_le_profile_init(); will be performed at a later time
             * since this requires a complete list of all services, which will be provided
             * in Python code
             */

            // pwr_mgr_init(); not applicable

            ble_task_init(); // diy
            // app_task_init(); // piggyback on vendor example. this results in
            // advertisements starting too soon.

            // /*Wait BT init complete*/
            // do {
            //     os_delay(100);
            // } while(! mp_bluetooth_is_active());
        }
    }

    mp_bluetooth_ameba_ble_state = MP_BLUETOOTH_AMEBA_BLE_STATE_ACTIVE;

    return 0;
}

void mp_bluetooth_deinit(void) {
    // according to https://github.com/simplexion/ambd_sdk/blob/1a8f7ca7583c10301bf3170f60af99280559c431/component/common/bluetooth/realtek/sdk/example/ble_peripheral/ble_app_main.c#L323
    
    if (mp_bluetooth_ameba_ble_state == MP_BLUETOOTH_AMEBA_BLE_STATE_OFF
        || mp_bluetooth_ameba_ble_state == MP_BLUETOOTH_AMEBA_BLE_STATE_SUSPENDED) {
        return;
    }

    mp_bluetooth_gap_advertise_stop();

    app_task_deinit();
    T_GAP_DEV_STATE state;
	le_get_gap_param(GAP_PARAM_DEV_STATE , &state);
	if (state.gap_init_state != GAP_INIT_STATE_STACK_READY) {
		// printf("[BLE Peripheral]BT Stack is not running\n\r");
	}
#if F_BT_DEINIT
	else {
		bte_deinit();
		bt_trace_uninit();
		memset(&gap_dev_state, 0, sizeof(T_GAP_DEV_STATE));
		// printf("[BLE Peripheral]BT Stack deinitalized\n\r");
	}
#endif

    mp_bluetooth_ameba_ble_state = MP_BLUETOOTH_AMEBA_BLE_STATE_SUSPENDED;

    MP_STATE_PORT(bluetooth_ameba_root_pointers) = NULL;
}

bool mp_bluetooth_is_active(void) {
    T_GAP_DEV_STATE state;
	le_get_gap_param(GAP_PARAM_DEV_STATE , &state);
    return (state.gap_init_state == GAP_INIT_STATE_STACK_READY);
}

void mp_bluetooth_get_current_address(uint8_t *addr_type, uint8_t *addr) {

}

void mp_bluetooth_set_address_mode(uint8_t addr_mode) {
    // TODO: implement
}

size_t mp_bluetooth_gap_get_device_name(const uint8_t **buf) {
    static T_LOCAL_NAME device_name;
    if (flash_load_local_name(&device_name) == 0) {
        if (device_name.local_name[0] != 0xff) {
            *buf = (uint8_t *)device_name.local_name;
        }
    }
    *buf = NULL;
    return strlen((const char *) device_name.local_name);
}

int mp_bluetooth_gap_set_device_name(const uint8_t *buf, size_t len) {
    T_LOCAL_NAME device_name;
    if (len + 1 > sizeof(device_name.local_name)) {
        return MP_EINVAL;
    }

    memcpy(device_name.local_name, buf, len);
    device_name.local_name[len] = '\0';
    return flash_save_local_name(&device_name);
}

extern T_APP_RESULT app_gap_callback(uint8_t cb_type, void *p_cb_data);

int mp_bluetooth_gap_advertise_start(bool connectable, int32_t interval_us, const uint8_t *adv_data, size_t adv_data_len, const uint8_t *sr_data, size_t sr_data_len) {
    // according to https://github.com/simplexion/ambd_sdk/blob/1a8f7ca7583c10301bf3170f60af99280559c431/component/common/bluetooth/realtek/sdk/example/ble_peripheral/ble_app_main.c#L111
    
    /* Device name and device appearance */
    // uint8_t  device_name[GAP_DEVICE_NAME_LEN] = "BLE_PERIPHERAL";
    // uint16_t appearance = GAP_GATT_APPEARANCE_UNKNOWN;
    uint8_t  slave_init_mtu_req = false;


    /* Advertising parameters */
    uint8_t  adv_evt_type = GAP_ADTYPE_ADV_IND;
    uint8_t  adv_direct_type = GAP_REMOTE_ADDR_LE_PUBLIC;
    uint8_t  adv_direct_addr[GAP_BD_ADDR_LEN] = {0};
    uint8_t  adv_chann_map = GAP_ADVCHAN_ALL;
    uint8_t  adv_filter_policy = GAP_ADV_FILTER_ANY;
    // (units of 625us, 160=100ms)
    uint16_t adv_int_min = interval_us/625;
    uint16_t adv_int_max = interval_us/625;

    /* GAP Bond Manager parameters */
    uint8_t  auth_pair_mode = GAP_PAIRING_MODE_PAIRABLE;
    uint16_t auth_flags = GAP_AUTHEN_BIT_BONDING_FLAG;
    uint8_t  auth_io_cap = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
#if F_BT_LE_SMP_OOB_SUPPORT
    uint8_t  auth_oob = false;
#endif
    uint8_t  auth_use_fix_passkey = false;
    uint32_t auth_fix_passkey = 0;
    uint8_t  auth_sec_req_enable = false;
    uint16_t auth_sec_req_flags = GAP_AUTHEN_BIT_BONDING_FLAG;

    /* Set device name and device appearance */
    // le_set_gap_param(GAP_PARAM_DEVICE_NAME, GAP_DEVICE_NAME_LEN, device_name);
    // le_set_gap_param(GAP_PARAM_APPEARANCE, sizeof(appearance), &appearance);
    le_set_gap_param(GAP_PARAM_SLAVE_INIT_GATT_MTU_REQ, sizeof(slave_init_mtu_req),
                     &slave_init_mtu_req);

    /* Set advertising parameters */
    le_adv_set_param(GAP_PARAM_ADV_EVENT_TYPE, sizeof(adv_evt_type), &adv_evt_type);
    le_adv_set_param(GAP_PARAM_ADV_DIRECT_ADDR_TYPE, sizeof(adv_direct_type), &adv_direct_type);
    le_adv_set_param(GAP_PARAM_ADV_DIRECT_ADDR, sizeof(adv_direct_addr), adv_direct_addr);
    le_adv_set_param(GAP_PARAM_ADV_CHANNEL_MAP, sizeof(adv_chann_map), &adv_chann_map);
    le_adv_set_param(GAP_PARAM_ADV_FILTER_POLICY, sizeof(adv_filter_policy), &adv_filter_policy);
    le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MIN, sizeof(adv_int_min), &adv_int_min);
    le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MAX, sizeof(adv_int_max), &adv_int_max);
    if (adv_data != NULL) {
        le_adv_set_param(GAP_PARAM_ADV_DATA, adv_data_len, (void *)adv_data);
    }
    if (sr_data != NULL) {
        le_adv_set_param(GAP_PARAM_SCAN_RSP_DATA, sr_data_len, (void *)sr_data);
    }

    /* Setup the GAP Bond Manager */
    gap_set_param(GAP_PARAM_BOND_PAIRING_MODE, sizeof(auth_pair_mode), &auth_pair_mode);
    gap_set_param(GAP_PARAM_BOND_AUTHEN_REQUIREMENTS_FLAGS, sizeof(auth_flags), &auth_flags);
    gap_set_param(GAP_PARAM_BOND_IO_CAPABILITIES, sizeof(auth_io_cap), &auth_io_cap);
#if F_BT_LE_SMP_OOB_SUPPORT
    gap_set_param(GAP_PARAM_BOND_OOB_ENABLED, sizeof(auth_oob), &auth_oob);
#endif
    le_bond_set_param(GAP_PARAM_BOND_FIXED_PASSKEY, sizeof(auth_fix_passkey), &auth_fix_passkey);
    le_bond_set_param(GAP_PARAM_BOND_FIXED_PASSKEY_ENABLE, sizeof(auth_use_fix_passkey),
                      &auth_use_fix_passkey);
    le_bond_set_param(GAP_PARAM_BOND_SEC_REQ_ENABLE, sizeof(auth_sec_req_enable), &auth_sec_req_enable);
    le_bond_set_param(GAP_PARAM_BOND_SEC_REQ_REQUIREMENT, sizeof(auth_sec_req_flags),
                      &auth_sec_req_flags);

    /* register gap message callback */
    le_register_app_cb(app_gap_callback); // piggyback on vendor example
    
    le_adv_start();

    return 0;
}

void mp_bluetooth_gap_advertise_stop(void) {
    printf("adv stop\r\n");
    le_adv_stop();
}

int mp_bluetooth_gatts_register_service_begin(bool append) {
    return MP_EOPNOTSUPP;
}

int mp_bluetooth_gatts_register_service_end(void) {
    return MP_EOPNOTSUPP;
}

int mp_bluetooth_gatts_register_service(mp_obj_bluetooth_uuid_t *service_uuid, mp_obj_bluetooth_uuid_t **characteristic_uuids, uint16_t *characteristic_flags, mp_obj_bluetooth_uuid_t **descriptor_uuids, uint16_t *descriptor_flags, uint8_t *num_descriptors, uint16_t *handles, size_t num_characteristics) {
    return MP_EOPNOTSUPP;
}

int mp_bluetooth_gap_disconnect(uint16_t conn_handle) {
    return MP_EOPNOTSUPP;
}

int mp_bluetooth_gatts_read(uint16_t value_handle, const uint8_t **value, size_t *value_len) {
    return MP_EOPNOTSUPP;
}

int mp_bluetooth_gatts_write(uint16_t value_handle, const uint8_t *value, size_t value_len, bool send_update) {
    return MP_EOPNOTSUPP;
}

int mp_bluetooth_gatts_notify_indicate(uint16_t conn_handle, uint16_t value_handle, int gatts_op, const uint8_t *value, size_t value_len) {
    return MP_EOPNOTSUPP;
}

int mp_bluetooth_gatts_set_buffer(uint16_t value_handle, size_t len, bool append) {
    return MP_EOPNOTSUPP;
}

int mp_bluetooth_get_preferred_mtu(void) {
    return MP_EOPNOTSUPP;
}

int mp_bluetooth_set_preferred_mtu(uint16_t mtu) {
    return MP_EOPNOTSUPP;
}

MP_REGISTER_ROOT_POINTER(struct _mp_bluetooth_ameba_root_pointers_t *bluetooth_ameba_root_pointers);
