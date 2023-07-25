
include $(MAKE_INCLUDE_GEN)

.PHONY: all clean

#*****************************************************************************#
#                               VARIABLES	                              #
#*****************************************************************************#
DIR = $(SRCDIR)

MODULE_IFLAGS = -I$(DIR)./

vpath %.c $(DIR)
#*****************************************************************************#
#                               Source FILE LIST                              #
#*****************************************************************************#
CSRC = $(DIR)/main.c

#*****************************************************************************#
#                               Object FILE LIST                              #
#*****************************************************************************#
OBJS = $(notdir $(CSRC:.c=.o)) micropython.o

#*****************************************************************************#
#                        RULES TO GENERATE TARGETS                            #
#*****************************************************************************#
all: CLEAN_OBJS CORE_TARGETS COPY_RAM_OBJS

micropython.o:
	@# crude hack to circumvent build error
	rm -rf ../micropython/build
	make -C ../micropython build/micropython.o
	cp ../micropython/build/micropython.o ./

	@if [ "$(COMPILEOS)" != "$(DARWIN_OS)" ]; then \
		find ../micropython -type f -regex '.*\(\.i\|\.d\|\.s\|\.su\|\.o\)' |xargs -i rm {}; \
	else \
		find ../micropython -type f -regex '.*\(\.i\|\.d\|\.s\|\.su\|\.o\)' |xargs -I rm {}; \
	fi
	
#*****************************************************************************#
#              GENERATE OBJECT FILE 
#*****************************************************************************#
CORE_TARGETS: $(OBJS)

#*****************************************************************************#
#                        RULES TO CLEAN TARGETS                               #
#*****************************************************************************#
clean: CLEAN_OBJS
	$(REMOVE) *.o
	$(REMOVE) *.i
	$(REMOVE) *.s
	$(REMOVE) *.d

-include $(DEPS)	
