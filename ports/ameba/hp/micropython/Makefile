include $(MAKE_INCLUDE_GEN)

include ../../../../py/mkenv.mk

# select the board to build for, the default board is AMEBA_AMB23
BOARD ?= AMEBA_AMB23
BOARD_DIR ?= boards/$(BOARD)

ifeq ($(wildcard $(BOARD_DIR)/.),)
$(error Invalid BOARD specified: $(BOARD_DIR))
endif

include $(BOARD_DIR)/mpconfigboard.mk

# declare before py.mk
FROZEN_MANIFEST ?= boards/manifest.py

CHIP := AMEBAD

CROSS = 1

# qstr definitions (must come before including py.mk)
QSTR_DEFS = qstrdefsport.h

# MicroPython feature configurations
MICROPY_ROM_TEXT_COMPRESSION ?= 1

# include py core make definitions
include $(TOP)/py/py.mk
include $(TOP)/extmod/extmod.mk

GIT_SUBMODULES += \
	lib/ambd_sdk \
	lib/micropython-lib \

CROSS_COMPILE ?= arm-none-eabi-

INC += $(IFLAGS)
INC += -I$(TOP)
INC += -I$(BUILD)
INC += -I$(BOARD_DIR)
INC += -I$(BASEDIR)/component/common/drivers/sdio/realtek/sdio_host/inc/
INC += -Imp_helper
INC += -Imp_helper/mods
INC += -Imp_helper/mods/network
INC += -Imp_helper/mods/machine

BAD_FLAGS = \
	-MP \
	-MMD \
	--save-temps \
	-Wundef \
	-Wextra

CFLAGS += $(filter-out $(BAD_FLAGS),$(GLOBAL_CFLAGS))
CFLAGS += $(INC) $(COPT)
CFLAGS += -D$(CHIP)
CFLAGS += -D$(BOARD)
CFLAGS += -DMICROPYTHON_RTL8721D
CFLAGS += -DCONFIG_PLATFORM_8721D

# Flags for optional C++ source code
CXXFLAGS += $(CPPFLAGS)

LIBS =

SRC_C = \
	micropython_task.c \
	mphalport.c \
	shared/libc/printf.c \
	shared/libc/string0.c \
	shared/readline/readline.c \
	shared/runtime/interrupt_char.c \
	shared/runtime/gchelper_generic.c \
	shared/runtime/pyexec.c \
	shared/runtime/stdout_helpers.c \
	shared/netutils/netutils.c \
	shared/timeutils/timeutils.c \

SRC_UPY_C = \
	mp_helper/gccollect.c \
	mp_helper/diskio.c \
	mp_helper/help.c \
	mp_helper/mods/modmachine.c \
	mp_helper/mods/machine/objflash.c \
	mp_helper/mods/machine/objpin.c \
	mp_helper/mods/machine/objrtc.c \
	mp_helper/mods/machine/objsdcard.c \
	mp_helper/mods/machine/objuart.c \
	mp_helper/mods/moduwireless.c \
	mp_helper/mods/modussl.c \
	mp_helper/mods/wireless/objwlan.c \
	$(BOARD_DIR)/pins.c \

SRC_C += $(SRC_UPY_C)

SRC_QSTR += shared/readline/readline.c shared/runtime/pyexec.c
SRC_QSTR += $(SRC_UPY_C)

OBJ += $(PY_O)
OBJ += $(addprefix $(BUILD)/, $(SRC_C:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(SRC_CXX:.cpp=.o))

all: $(BUILD)/micropython.o

# $(BUILD)/_frozen_mpy.c: $(TOP)/tests/frozen/frozentest.mpy $(BUILD)/genhdr/qstrdefs.generated.h
# 	$(ECHO) "MISC freezing bytecode"
# 	$(Q)$(TOP)/tools/mpy-tool.py -f -q $(BUILD)/genhdr/qstrdefs.preprocessed.h -mlongint-impl=none $< > $@

# simply link all MicroPython objcts to one relocatable object 
$(BUILD)/micropython.o: $(OBJ)
	$(Q)$(LD) -relocatable -o $@ $^ $(LIBS)

include $(TOP)/py/mkrules.mk
