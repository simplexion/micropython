#include <stdint.h>
#include "mpconfigboard.h"

// Python internal features
#define MICROPY_ENABLE_COMPILER                 (1)
#define MICROPY_ENABLE_GC                       (1)
#define MICROPY_HELPER_REPL                     (1)
#define MICROPY_ENABLE_EXTERNAL_IMPORT          (1)
#define MICROPY_ERROR_REPORTING                 (MICROPY_ERROR_REPORTING_TERSE)
#define MICROPY_FLOAT_IMPL                      (MICROPY_FLOAT_IMPL_FLOAT)
#define MICROPY_LONGINT_IMPL                    (MICROPY_LONGINT_IMPL_MPZ)
#define MICROPY_ENABLE_SCHEDULER                (1)
#define MICROPY_KBD_EXCEPTION                   (1)
#define MICROPY_NLR_SETJMP                      (1)
#define MICROPY_ENABLE_FINALISER                (1)

// Fine control over Python builtins, classes, modules, etc.
#define MICROPY_PY_OS                           (1)
#define MICROPY_PY_IO                           (1)
#define MICROPY_PY_SYS                          (1)
#define MICROPY_PY_TIME                         (1)
#define MICROPY_PY_MACHINE                      (1)
#define MICROPY_PY_LWIP                         (1)
#define MICROPY_PY_ASYNC_AWAIT                  (0)
#define MICROPY_PY_BUILTINS_SET                 (0)
#define MICROPY_PY_ATTRTUPLE                    (0)
#define MICROPY_PY_COLLECTIONS                  (0)
#define MICROPY_PY_MATH                         (0)
#define MICROPY_PY_STRUCT                       (0)

// File System Settings
#define MICROPY_HW_ENABLE_INTERNAL_FLASH_STORAGE (1)
#define MICROPY_VFS                             (1)
#define MICROPY_READER_VFS                      (MICROPY_VFS)
#define MICROPY_FATFS_ENABLE_LFN                (1)
#define MICROPY_FATFS_LFN_CODE_PAGE             437 /* 1=SFN/ANSI 437=LFN/U.S.(OEM) */
#define MICROPY_FATFS_RPATH                     (2)
#if MICROPY_HW_ENABLE_INTERNAL_FLASH_STORAGE
#define MICROPY_FATFS_MAX_SS                    (4096)  // default for flash
#else
#define MICROPY_FATFS_MAX_SS                    (512)   // default for SD card
#endif
#define MICROPY_FATFS_USE_LABEL                 (1)

#define MICROPY_BEGIN_ATOMIC_SECTION()          ({vPortEnterCritical(); 1;})
#define MICROPY_END_ATOMIC_SECTION(state)       vPortExitCritical()

// Type definitions for the specific machine.
typedef intptr_t mp_int_t; // must be pointer size
typedef uintptr_t mp_uint_t; // must be pointer size
typedef long mp_off_t;

#define MICROPY_EVENT_POLL_HOOK \
    do { \
        mp_handle_pending(true); \
    } while(0); \

// Heap size for Garbage Collector
#define MP_HEAP_SIZE                        (64 * 1024)

#define MICROPY_TASK_NAME                   ((const char*)"micropython_main_task")
#define MICROPY_TASK_STACK_DEPTH            (((20 * 1024) + 512) / sizeof(StackType_t))
#define MICROPY_TASK_PRIORITY               (tskIDLE_PRIORITY + 1) // 3 for Realtime, the highest priority

// We need to provide a declaration/definition of alloca().
#include <alloca.h>

#define MP_STATE_PORT MP_STATE_VM
