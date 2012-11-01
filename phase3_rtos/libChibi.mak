################################################################################
#
# Makefile for the Freescale Kinetis K60 / ARM Cortex-M4
#
################################################################################

CHIBIOS_DIR = ../../ChibiOS

# Name of project/output file:

TARGET = libChibi
OBJDIR = libChibi_obj

# List your asm files here (minus the .s):

ASM_PIECES =

# List your c files here (minus the .c):

vpath %.c $(CHIBIOS_DIR)/os/kernel/src
C_PIECES += chsys chdebug chlists chvt chschd chthreads chdynamic chregistry
C_PIECES += chsem chmtx chcond chevents chmsg chmboxes chqueues chmemcore
C_PIECES += chheap chmempools

vpath %.c $(CHIBIOS_DIR)/os/ports/GCC/ARMCMx
C_PIECES += chcore chcore_v7m

vpath %.c $(CHIBIOS_DIR)/os/ports/common/ARMCMx
C_PIECES += nvic

# Define Hardware Platform

PLATFORM = FREESCALE_K60N512_TOWER_HW

PATH :=/opt/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_EABI/bin:${PATH}
CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
GDB = arm-none-eabi-gdb
OBJDUMP = arm-none-eabi-objdump
AR = arm-none-eabi-ar

CHIBIOS_INCLUDES += -I${CHIBIOS_DIR}/os/kernel/include
CHIBIOS_INCLUDES += -I${CHIBIOS_DIR}/os/ports/common/ARMCMx
CHIBIOS_INCLUDES += -I${CHIBIOS_DIR}/os/ports/GCC/ARMCMx

ASM_FLAGS = -g
ASM_FILES = ${ASM_PIECES:%=%.s}
ASM_O_FILES = ${ASM_FILES:%.s=${OBJDIR}/%.o}

OPT_LEVEL = 0

C_FLAGS = -Wall -c -g -O${OPT_LEVEL} -D${PLATFORM}
C_FLAGS += -I. ${CHIBIOS_INCLUDES}
C_FLAGS += -fomit-frame-pointer -falign-functions=16 -ffunction-sections
C_FLAGS += -fdata-sections -fno-common -Wextra -Wstrict-prototypes
C_FLAGS += -DCORTEX_USE_FPU=FALSE
C_FLAGS += -DTHUMB_PRESENT -mno-thumb-interwork -DTHUMB_NO_INTERWORKING
C_FILES = ${C_PIECES:%=%.c}
C_O_FILES = ${C_FILES:%.c=${OBJDIR}/%.o}

O_FILES = ${ASM_O_FILES} ${C_O_FILES}

CPU_FLAGS = -mcpu=cortex-m4 -mthumb

LD_SCRIPT = linkerscript.ld

# nostartfiles prevents the toolchain from including startup routines.
LD_FLAGS = -nostartfiles -Map=${TARGET}.map

LIBPATH = /opt/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_EABI/arm-none-eabi/lib/thumb2

LIBS  = ${LIBPATH}/libc.a
LIBS += /opt/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_EABI/lib/gcc/arm-none-eabi/4.6.1/thumb2/libgcc.a

all: ${TARGET}.a

${TARGET}.a: ${OBJDIR} ${O_FILES}
	@echo
	rm -f $@
	${AR} crs $@ ${O_FILES}

${OBJDIR}/%.o: %.s
	${AS} ${ASM_FLAGS} ${CPU_FLAGS} -o $@ $<

${OBJDIR}/%.o: %.c
	${CC} ${C_FLAGS} ${CPU_FLAGS} -o $@ $<

${OBJDIR}:
	mkdir ${OBJDIR}

clean:
	@echo
	@echo Cleaning up...
	@echo
	rm -rf ${OBJDIR}
	rm -f ${TARGET}.a
