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
#include "servicehelper.h"

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


    if (mp_bluetooth_ameba_ble_state == MP_BLUETOOTH_AMEBA_BLE_STATE_OFF) {
        T_GAP_DEV_STATE new_state;

        bt_trace_init();
        gap_config_max_le_link_num(APP_MAX_LINKS);
        gap_config_max_le_paired_device(APP_MAX_LINKS);
        bte_init();
        le_gap_init(APP_MAX_LINKS);

        bt_coex_init();

        /*Wait BT init complete*/
        do {
            os_delay(100);
            le_get_gap_param(GAP_PARAM_DEV_STATE , &new_state);
        }while(new_state.gap_init_state != GAP_INIT_STATE_STACK_READY);
    }

    mp_bluetooth_ameba_ble_state = MP_BLUETOOTH_AMEBA_BLE_STATE_ACTIVE;

    return 0;
}

void mp_bluetooth_deinit(void) {

}

bool mp_bluetooth_is_active(void) {
    return false;
}

void mp_bluetooth_get_current_address(uint8_t *addr_type, uint8_t *addr) {

}

void mp_bluetooth_set_address_mode(uint8_t addr_mode) {
    // TODO: implement
}

size_t mp_bluetooth_gap_get_device_name(const uint8_t **buf) {
    return MP_EOPNOTSUPP;
}

int mp_bluetooth_gap_set_device_name(const uint8_t *buf, size_t len) {
    return 0;
}

int mp_bluetooth_gap_advertise_start(bool connectable, int32_t interval_us, const uint8_t *adv_data, size_t adv_data_len, const uint8_t *sr_data, size_t sr_data_len) {
    return MP_EOPNOTSUPP;
}

void mp_bluetooth_gap_advertise_stop(void) {

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
