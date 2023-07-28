/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Chester Tseng
 * Copyright (c) 2023 Markus Blechschmidt (Simplexion GmbH)
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

#include "micropython_task.h"

#include "modmachine.h"
#include "mphalport.h"

#include "py/builtin.h"
#include "py/compile.h"
#include "py/gc.h"
#include "py/mperrno.h"
#include "py/stackctrl.h"
#include "shared/runtime/pyexec.h"
#include "shared/readline/readline.h"

#include "ameba_soc.h"
#include "osdep_service.h"
#include <stdarg.h>
#include "strproc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "objsdcard.h"

TaskHandle_t mp_main_task_handle;

// Allocate memory for the MicroPython GC heap.
static char mpHeap[MP_HEAP_SIZE];

void micropython_task(void* arg) {

soft_reset:
    uart_repl_init();
    mp_stack_ctrl_init();

#if MICROPY_ENABLE_GC
    gc_init(mpHeap, mpHeap + sizeof(mpHeap));
#endif

    // Init MP runtime
    mp_init();
    mp_obj_list_append(mp_sys_path, MP_OBJ_NEW_QSTR(MP_QSTR__slash_lib));

    // Initialise sub-systems.
    readline_init0();
    modmachine_init();

    // Set up file system
    int stat  = interpret_sd_status(SD_Init());
    if (stat != 0 ) {
        pyexec_frozen_module("_boot.py", false);
    } else {
        pyexec_frozen_module("_boot_sd.py", false);
    }

    // Execute user scripts.
    int ret = pyexec_file_if_exists("boot.py");
    if (ret & PYEXEC_FORCED_EXIT) {
        goto soft_reset_exit;
    }
    if (pyexec_mode_kind == PYEXEC_MODE_FRIENDLY_REPL) {
        ret = pyexec_file_if_exists("main.py");
        if (ret & PYEXEC_FORCED_EXIT) {
            goto soft_reset_exit;
        }
    }

    for ( ; ; ) {
        if (pyexec_mode_kind == PYEXEC_MODE_RAW_REPL) {
            if (pyexec_raw_repl() != 0)
                break;
        } else {
            if (pyexec_friendly_repl() != 0) 
                break;
        }
        //osThreadYield();
    }

soft_reset_exit:

    gc_sweep_all();
    mp_hal_stdout_tx_str("MPY: soft reboot\r\n");
    mp_deinit();
    goto soft_reset;

}

void micropython_task_init(void) {
    if(
        xTaskCreate(
            micropython_task,
            MICROPY_TASK_NAME,
            MICROPY_TASK_STACK_DEPTH,
            NULL,
            MICROPY_TASK_PRIORITY,
            &mp_main_task_handle)
        != pdPASS
    ) {
        printf("\n\r%s xTaskCreate(init_thread) failed", __FUNCTION__);
    }
}

// Handle uncaught exceptions (should never be reached in a correct C implementation).
void nlr_jump_fail(void *val) {
    for (;;) {
    }
}
