// #include "ff.h"
// #include <fatfs_ext/inc/ff_driver.h>
// #include <sdcard.h>
// #include "flash_api.h"
// #include <flash_fatfs.h>
#include "py/mpconfig.h"
#include "py/runtime.h"
#include "py/stream.h"
#include "sd.h" // sd card driver with sdio interface

bool init_sdcard(void);
int interpret_sd_status(SD_RESULT result);

extern const mp_obj_type_t sdcard_type;

typedef struct {
    mp_obj_base_t base;
    size_t        block_size;
    size_t        start;
    size_t        len;
} sdcard_obj_t;