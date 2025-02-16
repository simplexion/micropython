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

#include "objrtc.h"
#include "rtl8721d_rtc.h"

// singleton RTC object
STATIC rtc_obj_t rtc_obj = {
    .base.type = &rtc_type,
};

void rtc_init0(void) {
    rtc_init();
}

STATIC void rtc_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    rtc_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print, "RTC()");
}

STATIC mp_obj_t rtc_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 0, 0, false);

    // return constant object
    return (mp_obj_t)&rtc_obj;
}

STATIC mp_obj_t rtc_datetime(mp_uint_t n_args, const mp_obj_t *args) {
    static uint32_t seconds_ymd_backup = 0;
    RTC_TimeTypeDef RTC_TimeStruct;

    if (n_args == 1) {
        // RTL872x RTC can only count up to 512 days
        // Get updated time and reset RTC day counter to 0
        RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
        int days = RTC_TimeStruct.RTC_Days;
        RTC_TimeStruct.RTC_Days = 0;
        RTC_SetTime(RTC_Format_BIN, &RTC_TimeStruct);
        int hours = RTC_TimeStruct.RTC_Hours;
        int mins = RTC_TimeStruct.RTC_Minutes;
        int secs = RTC_TimeStruct.RTC_Seconds;

        clock_t seconds_elapsed = days * NUM_SECS_IN_DAY;
        clock_t total_seconds = seconds_elapsed + seconds_ymd_backup + hours * 3600 + mins * 60 + secs;
        timeutils_struct_time_t tm;
        timeutils_seconds_since_2000_to_struct_time(total_seconds, &tm);

        // Update backup copy of year month date in seconds
        seconds_ymd_backup += seconds_elapsed;

        mp_obj_t tuple[8] = {
            mp_obj_new_int(tm.tm_year),
            mp_obj_new_int(tm.tm_mon),
            mp_obj_new_int(tm.tm_mday),
            mp_obj_new_int(tm.tm_wday),
            mp_obj_new_int(tm.tm_hour),
            mp_obj_new_int(tm.tm_min),
            mp_obj_new_int(tm.tm_sec),
            mp_obj_new_int(total_seconds)
        };

        return mp_obj_new_tuple(8, tuple);
    } else {
        // Set time
        mp_obj_t *items;
        mp_obj_get_array_fixed_n(args[1], 8, &items);

        uint32_t year = mp_obj_get_int(items[0]);
        uint32_t month = mp_obj_get_int(items[1]);
        uint32_t day = mp_obj_get_int(items[2]);
        uint32_t hour = mp_obj_get_int(items[4]);
        uint32_t mins = mp_obj_get_int(items[5]);
        uint32_t secs = mp_obj_get_int(items[6]);

        seconds_ymd_backup = ((clock_t)timeutils_seconds_since_2000(year, month, day, 0, 0, 0));

        // RTL872x RTC can only count up to 512 days, so use it to only count day offsets
        RTC_TimeStruct.RTC_H12_PMAM = RTC_H12_AM;
        RTC_TimeStruct.RTC_Days = 0;
        RTC_TimeStruct.RTC_Hours = hour;
        RTC_TimeStruct.RTC_Minutes = mins;
        RTC_TimeStruct.RTC_Seconds = secs;
        RTC_SetTime(RTC_Format_BIN, &RTC_TimeStruct);

        return mp_const_none;
    }
}

STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(rtc_datetime_obj, 1, 2, rtc_datetime);

STATIC const mp_map_elem_t rtc_locals_dict_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR_datetime),  MP_OBJ_FROM_PTR(&rtc_datetime_obj) },
};
STATIC MP_DEFINE_CONST_DICT(rtc_locals_dict, rtc_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    rtc_type,
    MP_QSTR_RTC,
    MP_TYPE_FLAG_NONE,
    make_new, rtc_make_new,
    print, rtc_print,
    locals_dict, &rtc_locals_dict
    );