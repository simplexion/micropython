#include <unistd.h>
#include "py/mpconfig.h"

#include "ameba_soc.h"

// Receive single character, blocking until one is available.
int mp_hal_stdin_rx_chr(void) {
    int c;

    while(! LOGUART_Readable()) {
        taskYIELD ();
    }

    do {
        c = LOGUART_GetChar(_FALSE);
    } while(c < 0);

    return c;
}

// Send the string of given length.
void mp_hal_stdout_tx_strn(const char *str, mp_uint_t len) {
    for(unsigned u=0; u< len; u++) {
        LOGUART_PutChar(str[u]);
    }
}
