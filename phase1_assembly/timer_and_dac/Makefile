################################################################################
#
# Makefile for a ARM Cortex M4 based CCA
#
################################################################################

################################################################################

# C Source Files (minus the .c):

# Assembler Files (minus the .s):
ASM_PIECES= vectors startup application

################################################################################

CC=arm-none-eabi-gcc
AS=arm-none-eabi-as
GDB=arm-none-eabi-gdb

REMOVE=rm -f
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
SIZE=arm-none-eabi-size

DEVICE = k60n512
TARGET = timers

CPU_FLAGS = -mcpu=cortex-m4 -mthumb

OPT_LEVEL = 0

CFLAGS  = -Dgcc $(CPU_FLAGS) -c -g -Wall -MD -O${OPT_LEVEL}
CFLAGS += -D$(DEVICE) -DDEVICE=$(DEVICE)

LD_FLAGS = -Xlinker -Map=$(TARGET).map

SCRIPT = k60n512.ld

ASM_FILES= $(ASM_PIECES:%=%.s)
ASM_O_FILES=$(ASM_PIECES:%=%.o)

C_FILES= $(C_PIECES:%=%.c)
C_O_FILES= $(C_PIECES:%=%.o)

SRCS= $(ASM_FILES) $(C_FILES)
OBJS= $(ASM_O_FILES) $(C_O_FILES)

all: $(TARGET).axf

clean: clean_list

rebuild: clean all

debug:
	$(GDB) --eval-command="target remote localhost:3333" $(TARGET).axf

tui:
	$(GDB)tui $(TARGET).axf

ddd:
	ddd --debugger $(GDB) $(TARGET).axf

openocd:
	@echo Launching openOCD...
	@openocd -s /usr/local/share/openocd/scripts -f interface/osbdm.cfg \
    -f board/twr-k60n512.cfg

$(TARGET).axf: $(OBJS)
	@echo
	$(CC) $(CPU_FLAGS) $(OBJS) $(LIBS) $(LD_FLAGS) $(MAP) -o $(TARGET).axf -T $(SCRIPT)
	@echo
	$(OBJCOPY) -O srec ${@} ${@:.axf=.hex}
	${OBJCOPY} -O binary ${@} ${@:.axf=.bin}
	${OBJDUMP} -DS ${@} > ${@:.axf=.dump}
	@echo
	@$(SIZE) ${OBJS}
	@echo -e "\e[32m"
	@$(SIZE) $(TARGET).axf
	@echo -e "\e[0m"
	@$(CC) --version
ifeq ($(OPT_LEVEL), 0)
	@echo -e "\e[31mWARNING:\e[0m Optimization Disabled"
endif
	mkdir -p $(TARGET)
	mv *.[od] $(TARGET)

clean_list:
	$(REMOVE) *.o
	$(REMOVE) *.d
	$(REMOVE) *.axf
	$(REMOVE) $(TARGET).hex
	$(REMOVE) $(TARGET).bin
	$(REMOVE) $(TARGET).map
	$(REMOVE) $(TARGET).dump

ifneq (${MAKECMDGOALS},clean)
-include ${wildcard *.d}
endif

