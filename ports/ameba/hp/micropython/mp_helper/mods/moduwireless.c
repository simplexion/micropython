/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Chester Tseng
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

/*****************************************************************************
 *                              Header includes
 * ***************************************************************************/
#include "wireless/objwlan.h"

/*****************************************************************************
 *                              External variables
 * ***************************************************************************/

void modwireless_init(void) {
    //wlan_init0();
}

void modwireless_deinit(void) {
    wlan_deinit0();
}

STATIC const mp_map_elem_t wireless_module_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__),     MP_OBJ_NEW_QSTR(MP_QSTR_wireless) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_WLAN),         MP_OBJ_FROM_PTR(&wlan_type) },
};
STATIC MP_DEFINE_CONST_DICT(wireless_module_globals, wireless_module_globals_table);

const mp_obj_module_t mp_module_uwireless = {
    .base    = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&wireless_module_globals,
};

MP_REGISTER_EXTENSIBLE_MODULE(MP_QSTR_wireless, mp_module_uwireless);
