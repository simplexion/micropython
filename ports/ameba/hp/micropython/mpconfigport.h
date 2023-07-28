#include <stdint.h>
#include "mpconfigboard.h"

#ifndef MICROPY_CONFIG_ROM_LEVEL
#define MICROPY_CONFIG_ROM_LEVEL                (MICROPY_CONFIG_ROM_LEVEL_BASIC_FEATURES)
#endif

#define MP_HAL_CLEAN_DCACHE

// Python internal features
#define MICROPY_EMIT_THUMB                      (1)
#define MICROPY_EMIT_INLINE_THUMB               (1)
#define MICROPY_NLR_SETJMP                      (1)
#define MICROPY_BEGIN_ATOMIC_SECTION()          ({vPortEnterCritical(); 1;})
#define MICROPY_END_ATOMIC_SECTION(state)       vPortExitCritical()

/// Module and parser
#define MICROPY_ENABLE_COMPILER                 (1)
#define MICROPY_ENABLE_FINALISER                (1)
#define MICROPY_PERSISTENT_CODE_LOAD            (1)
#define MICROPY_ENABLE_EXTERNAL_IMPORT          (1)
#define MICROPY_QSTR_BYTES_IN_HASH              (1)
#define MICROPY_CPYTHON_COMPAT                  (1)
#define MICROPY_COMP_MODULE_CONST               (1)
#define MICROPY_COMP_CONST                      (1)
#define MICROPY_COMP_DOUBLE_TUPLE_ASSIGN        (1)
#define MICROPY_COMP_TRIPLE_TUPLE_ASSIGN        (0)
#define MICROPY_PY___FILE__                     (1)

/// System and runtime
#define MICROPY_ENABLE_SCHEDULER                (1)
#define MICROPY_ENABLE_GC                       (1)
#define MICROPY_PY_GC                           (1)
#define MICROPY_KBD_EXCEPTION                   (1)
#define MICROPY_PY_OS                           (1)
#define MICROPY_PY_IO                           (1)
#define MICROPY_PY_IO_IOBASE                    (1)
#define MICROPY_PY_IO_FILEIO                    (1)
#define MICROPY_STREAMS_NON_BLOCK               (1)
#define MICROPY_PY_THREAD                       (0)
#define MICROPY_REPL_EVENT_DRIVEN               (0)
#define MICROPY_PY_BUILTINS_EXECFILE            (1)
#define MICROPY_PY_BUILTINS_TIMEOUTERROR        (1)
#define MICROPY_PY_BUILTINS_INPUT               (0)
#define MICROPY_PY_ERRNO                        (1)
#define MICROPY_PY_SYS                          (1)
#define MICROPY_PY_SYS_PLATFORM                 "Realtek Ameba"
#define MICROPY_HW_PORT_VERSION                 "1.0.2"
#define MICROPY_PY_SYS_MODULES                  (1)
#define MICROPY_PY_SYS_EXIT                     (1)
#define MICROPY_PY_SYS_STDIO_BUFFER             (1)
#define MICROPY_PY_SYS_STDFILES                 (0)
#define MICROPY_PY_WEBREPL_DELAY                (20)

/// Help
#define MICROPY_HELPER_REPL                     (1)
#define MICROPY_PY_BUILTINS_HELP                (1)
#define MICROPY_PY_BUILTINS_HELP_TEXT           ameba_mp_help_text
#define MICROPY_PY_BUILTINS_HELP_MODULES        (0)
#define MICROPY_ENABLE_DOC_STRING               (1)
#define MICROPY_PY_MICROPYTHON_MEM_INFO         (MICROPY_ENABLE_GC)
#define MICROPY_ENABLE_SOURCE_LINE              (1)
#define MICROPY_ERROR_REPORTING                 (MICROPY_ERROR_REPORTING_DETAILED)
#define MICROPY_ENABLE_EMERGENCY_EXCEPTION_BUF  (1)
#define MICROPY_REPL_AUTO_INDENT                (1)

/// Algorithms and data structures
#define MICROPY_FLOAT_IMPL                      (MICROPY_FLOAT_IMPL_FLOAT)
#define MICROPY_LONGINT_IMPL                    (MICROPY_LONGINT_IMPL_MPZ)
#define MICROPY_PY_BUILTINS_BYTEARRAY           (1)
#define MICROPY_PY_BUILTINS_MEMORYVIEW          (1)
#define MICROPY_PY_BUILTINS_ENUMERATE           (1)
#define MICROPY_PY_BUILTINS_FROZENSET           (1)
#define MICROPY_PY_BUILTINS_REVERSED            (1)
#define MICROPY_PY_BUILTINS_SET                 (1)
#define MICROPY_PY_BUILTINS_SLICE               (1)
#define MICROPY_PY_BUILTINS_PROPERTY            (1)
#define MICROPY_PY_FSTRINGS                     (1)
#define MICROPY_PY_ARRAY                        (1)
#define MICROPY_PY_ATTRTUPLE                    (1)
#define MICROPY_PY_STRUCT                       (1)
#define MICROPY_PY_COLLECTIONS                  (1)
#define MICROPY_PY_UCTYPES                      (1)
#define MICROPY_PY_SELECT                       (1)

/// APIs
#define MICROPY_PY_TIME                         (1)
#define MICROPY_PY_MACHINE                      (1)
#define MICROPY_PY_LWIP                         (1)
#define MICROPY_PY_HEAPQ                        (1)
#define MICROPY_PY_MATH                         (1)
#define MICROPY_PY_JSON                         (1)
#define MICROPY_PY_BINASCII                     (1)
#define MICROPY_PY_BUILTINS_BYTES_HEX           (1)
#define MICROPY_PY_RE                           (1)
#define MICROPY_PY_RE_MATCH_GROUPS              (1)
#define MICROPY_PY_RE_MATCH_SPAN_START_END      (1)

/// File System
#define MICROPY_ALLOC_PATH_MAX                  (128)
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
