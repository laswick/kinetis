################################################################################
#
# Makefile for the Freescale Kinetis K60 / ARM Cortex-M4
#
################################################################################

# Name of project/output file:

TARGET = demoGpio

# List your asm files here (minus the .s):

ASM_PIECES = startcode

# List your c files here (minus the .c):

C_PIECES = hardware gpio demoGpio

# Define Hardware Platform

PLATFORM = FREESCALE_K60N512_TOWER_HW

PATH :=/opt/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_EABI/bin:${PATH}
CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
GDB = arm-none-eabi-gdb
OBJDUMP = arm-none-eabi-objdump
SZ = arm-none-eabi-size

ASM_FLAGS = -g
ASM_FILES = ${ASM_PIECES:%=%.s}
ASM_O_FILES = ${ASM_FILES:%.s=%.o}

OPT_LEVEL = 1

C_FLAGS  = -Wall -c -ggdb -MD -O${OPT_LEVEL} -D${PLATFORM} -mlong-calls -ffunction-sections -fdata-sections
C_FILES = ${C_PIECES:%=%.c}
C_O_FILES = ${C_FILES:%.c=%.o}

O_FILES = ${ASM_O_FILES} ${C_O_FILES}

CPU_FLAGS = -mcpu=cortex-m4 -mthumb

LD_SCRIPT = linkerscript.ld

LD_FLAGS = -nostartfiles -Wl,-Map=${TARGET}.map,--gc-sections

LIBPATH = /opt/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_EABI/arm-none-eabi/lib/thumb2

#LIBS  = ${LIBPATH}/libc.a

all: ${TARGET}.axf
	@${OBJDUMP} -DS ${TARGET}.axf >| ${TARGET}.out.s
	@ln -fs ${TARGET}.out.s out.s
	@ln -fs ${TARGET}.axf out.axf
	@echo
	@echo Executable: ${TARGET}.axf, sym-linked to out.axf
	@echo
	@echo Disassembly Listing: ${TARGET}.out.s, sym-linked to out.s
	@echo
	@${CC} --version

${TARGET}.axf: ${O_FILES}
	@echo
	${CC} ${O_FILES} ${LIBS} -T ${LD_SCRIPT} ${LD_FLAGS} -o ${TARGET}.axf
	${SZ} ${TARGET}.axf

%.o: %.s
	${AS} ${ASM_FLAGS} ${CPU_FLAGS} -o $@ $<

%.o: %.c
	${CC} ${C_FLAGS} ${CPU_FLAGS} -o $@ $<

clean:
	@echo
	@echo Cleaning up...
	@echo
	rm -f *.o
	rm -f *.d
	rm -f ${TARGET}.axf
	rm -f ${TARGET}.out.s
	rm -f out.axf
	rm -f out.s
	rm -f ${TARGET}.map

openocd:
	@echo
	@echo Launching openOCD...
	@echo
	@openocd -s /usr/local/share/openocd/scripts -f interface/osbdm.cfg \
    -f board/twr-k60n512.cfg

gdb:
	@echo
	@echo Launching GDB...
	@echo
	${GDB} --eval-command="target remote localhost:3333" out.axf

