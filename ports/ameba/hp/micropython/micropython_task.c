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

TaskHandle_t mp_main_task_handle;

// Allocate memory for the MicroPython GC heap.
static char heap[MP_HEAP_SIZE];

void micropython_task(void* arg) {

soft_reset:
    uart_repl_init();
    mp_stack_ctrl_init();

#if MICROPY_ENABLE_GC
    gc_init(heap, heap + sizeof(heap));
#endif

    // Initialize MP runtime.
    mp_init();

    // Initialize sub-systems.
    readline_init0();
    modmachine_init();

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

    // // Start a normal REPL; will exit when ctrl-D is entered on a blank line.
    // pyexec_friendly_repl();


soft_reset_exit:

    // Deinitialise the runtime.
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

// There is no filesystem so stat'ing returns nothing.
mp_import_stat_t mp_import_stat(const char *path) {
    return MP_IMPORT_STAT_NO_EXIST;
}

// There is no filesystem so opening a file raises an exception.
mp_lexer_t *mp_lexer_new_from_file(const char *filename) {
    mp_raise_OSError(MP_ENOENT);
}
