include $(MAKE_INCLUDE_GEN)

include ../../../../py/mkenv.mk

CROSS = 1

# qstr definitions (must come before including py.mk)
QSTR_DEFS = qstrdefsport.h

# MicroPython feature configurations
MICROPY_ROM_TEXT_COMPRESSION ?= 1

# include py core make definitions
include $(TOP)/py/py.mk
include $(TOP)/extmod/extmod.mk

CROSS_COMPILE ?= arm-none-eabi-

INC += $(IFLAGS)
INC += -I$(TOP)
INC += -I$(BUILD)

CFLAGS += $(INC) $(GLOBAL_CFLAGS) $(COPT)

# CSUPEROPT = -Os # save some code space

# # Tune for Debugging or Optimization
# CFLAGS += -g  # always include debug info in the ELF
# ifeq ($(DEBUG), 1)
# CFLAGS += -O0
# else
# CFLAGS += -Os -DNDEBUG
# CFLAGS += -fdata-sections -ffunction-sections
# endif

# Flags for optional C++ source code
CXXFLAGS += $(CPPFLAGS)

LIBS =

SRC_C = \
	main.c \
	mphalport.c \
	shared/libc/printf.c \
	shared/readline/readline.c \
	shared/runtime/pyexec.c \
	shared/runtime/stdout_helpers.c \
	$(BUILD)/_frozen_mpy.c \

SRC_C += shared/libc/string0.c

SRC_QSTR += shared/readline/readline.c shared/runtime/pyexec.c

OBJ += $(PY_CORE_O)
OBJ += $(addprefix $(BUILD)/, $(SRC_C:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(SRC_CXX:.cpp=.o))

all: $(BUILD)/micropython.o

$(BUILD)/_frozen_mpy.c: $(TOP)/tests/frozen/frozentest.mpy $(BUILD)/genhdr/qstrdefs.generated.h
	$(ECHO) "MISC freezing bytecode"
	$(Q)$(TOP)/tools/mpy-tool.py -f -q $(BUILD)/genhdr/qstrdefs.preprocessed.h -mlongint-impl=none $< > $@

$(BUILD)/micropython.o: $(OBJ)
	$(ECHO) "LINK $@"
	$(Q)$(LD) -r -o $@ $^ $(LIBS)
	$(Q)$(SIZE) $@

include $(TOP)/py/mkrules.mk
