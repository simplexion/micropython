/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Chester Tseng
 * Copyright (c) 2023 Simplexion GmbH
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

#include "mphalport.h"

#include "micropython_task.h"

#include <unistd.h>
#include "py/mpstate.h"
#include "py/mpconfig.h"
#include "py/runtime.h"
#include "py/gc.h"
#include "py/ringbuf.h"
#include "py/stream.h"
#include "shared/runtime/interrupt_char.h"

#include "serial_api.h"
#include "osdep_service.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#define USE_RINGBUF 1

serial_t    uartobj;

STATIC uint8_t uart_ringbuf_array[256];
ringbuf_t   stdin_ringbuf = {uart_ringbuf_array, sizeof(uart_ringbuf_array), 0, 0};

/* LOGUART pins: */
#define UART_TX    PA_7
#define UART_RX    PA_8

void uart_repl_handler(uint32_t id, SerialIrq event) {
    gc_lock();

    ringbuf_t *handlerRingBuf = (ringbuf_t *)id;
    
    if (event == RxIrq) {
//        while(serial_readable(&uartobj)) {
            int repl_buf = serial_getc(&uartobj);
            if (repl_buf == mp_interrupt_char) {
                mp_sched_keyboard_interrupt();
            } else {
                ringbuf_put(handlerRingBuf, (uint8_t)repl_buf);
            }
//        }
    }
    gc_unlock();
}

void uart_repl_init() {
    //init repl on UART and enable uart rx interrupt for receiving incoming data and handle keyboard interrupt
    serial_init(&uartobj,UART_TX,UART_RX);
    serial_baud(&uartobj,115200);
    serial_format(&uartobj, 8, ParityNone, 1);
    serial_irq_handler(&uartobj, uart_repl_handler, (uint32_t)&stdin_ringbuf);
    serial_irq_set(&uartobj, RxIrq, 1);
}


void uart_send_string(serial_t *uartobj, const char *pstr)
{
    unsigned int i=0;

    while (*(pstr+i) != 0) {
        serial_putc(uartobj, *(pstr+i));
        i++;
    }
}


void uart_send_string_with_length(serial_t *uartobj, const char *pstr, size_t len)
{
    for (uint32_t i = 0; i < len; ++i) {
        serial_putc(uartobj, pstr[i]);
    }
}

/////////////////////////////////////
//       HAL STDIO TX & RX         //
/////////////////////////////////////

// Receive single character, blocking until one is available.
int mp_hal_stdin_rx_chr(void) {
    #if USE_RINGBUF
    for (;;) {
        int c = ringbuf_get(&stdin_ringbuf);
        if (c != -1) {
            return c;
        }
        MICROPY_EVENT_POLL_HOOK
    }
    #else
    int c = serial_getc(&uartobj);
    if (c == mp_interrupt_char) {
        mp_sched_keyboard_interrupt();
    } else {
        return c;
    }
    #endif
}

int mp_hal_stdin_rx_readable(void) {
    #if USE_RINGBUF
    return ringbuf_avail(&stdin_ringbuf);
    #else
    return serial_readable(&uartobj);
    #endif
}

// Send the string of given length.
void mp_hal_stdout_tx_strn(const char *str, size_t len) {
    uart_send_string_with_length(&uartobj, str, len);
}

void mp_hal_stdout_tx_chr(char c) {
    serial_putc(&uartobj, (int)c);
}

uintptr_t mp_hal_stdio_poll(uintptr_t poll_flags) {
#if USE_RINGBUF
    uintptr_t ret = 0;
    if ((poll_flags & MP_STREAM_POLL_RD) && ringbuf_peek(&stdin_ringbuf) != -1) {
        ret |= MP_STREAM_POLL_RD;
    }
    return ret;
#else
    poll_flags = poll_flags;
    return MP_STREAM_POLL_RD;
#endif
}


///////////////////////////////
//       Delay & Time        //
///////////////////////////////
void mp_hal_delay_ms(uint32_t ms) {
    const TickType_t xDelay = ms / portTICK_PERIOD_MS;
    vTaskDelay( xDelay );
    // osDelay(ms); //RTOS delay
}

void mp_hal_delay_us(uint32_t us) {
    DelayUs(us); // asm NOP
}

uint64_t mp_hal_ticks_cpu(void) {
    return SYSTIMER_TickGet(); // resolution is 31us
}

uint64_t mp_hal_ticks_us(void) {
    return SYSTIMER_TickGet() * 31ULL;
}

uint32_t mp_hal_ticks_ms(void) {
    return rtw_get_current_time();
}

// Wake up the main task if it is sleeping
void mp_hal_wake_main_task_from_isr(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(mp_main_task_handle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR(pdTRUE);
    }
}
