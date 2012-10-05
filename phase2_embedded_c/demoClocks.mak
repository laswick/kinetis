################################################################################
#
# Makefile for the Freescale Kinetis K60 / ARM Cortex-M4
#
################################################################################

# Name of project/output file:

TARGET = clocks_demo

# List your asm files here (minus the .s):

ASM_PIECES = startcode

# List your c files here (minus the .c):

C_PIECES = hardware gpio clocks demoClocks

# Define Hardware Platform

PLATFORM = FREESCALE_K60N512_TOWER_HW

PATH :=/opt/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_EABI/bin:${PATH}
CC = arm-none-eabi-gcc              # Compiler
AS = arm-none-eabi-as               # Assembler
LD = arm-none-eabi-ld               # Linker
GDB = arm-none-eabi-gdb             # Debugger
OBJDUMP = arm-none-eabi-objdump     # Object file (disassembler output)

ASM_FLAGS = -g
ASM_FILES = ${ASM_PIECES:%=%.s}
ASM_O_FILES = ${ASM_FILES:%.s=%.o}

OPT_LEVEL = 0

C_FLAGS = -Wall -c -g -O${OPT_LEVEL} -D${PLATFORM}
C_FILES = ${C_PIECES:%=%.c}
C_O_FILES = ${C_FILES:%.c=%.o}

O_FILES = ${ASM_O_FILES} ${C_O_FILES}

CPU_FLAGS = -mcpu=cortex-m4 -mthumb

LD_SCRIPT = linkerscript.ld

# nostartfiles flag prevents the toolchain from including it's default startup
# routines. 

LD_FLAGS = -nostartfiles -Map=${TARGET}.map

# path to the thumb-2 assembly instruction set library. Thumb 2 is an
# instruction set for ARM programs. It uses 16 and 32 bit instructions.
LIBPATH = /opt/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_EABI/arm-none-eabi/lib/thumb2

LIBS  = ${LIBPATH}/libc.a

# Target : Dependencies

# Running the makefile, create the executable for loading (axf)
# OBJDUMP (create the dissassembler of the object file)
all: ${TARGET}.axf
    # D = disassemble all sections of the code
    # S = intermix the dissassembly with source code.
	@${OBJDUMP} -DS ${TARGET}.axf >| ${TARGET}.out.s
	@ln -fs ${TARGET}.out.s out.s
	@ln -fs ${TARGET}.axf out.axf       # f = force, s = symbolic
	@echo
	@echo Executable: ${TARGET}.axf, sym-linked to out.axf
	@echo
	@echo Disassembly Listing: ${TARGET}.out.s, sym-linked to out.s
	@echo
	@${CC} --version

# Create the microcontroller AXF file from the object files using
# LINKER
${TARGET}.axf: ${O_FILES}
	@echo
	${LD} ${O_FILES} ${LIBS} -T ${LD_SCRIPT} ${LD_FLAGS} -o ${TARGET}.axf

# Create the object files from the assembly files using
# ASSEMBLER
%.o: %.s
	${AS} ${ASM_FLAGS} ${CPU_FLAGS} -o $@ $<

# Create the object files from the C files using
# COMPILER
%.o: %.c
	${CC} ${C_FLAGS} ${CPU_FLAGS} -o $@ $<

clean:
	@echo
	@echo Cleaning up...
	@echo
	rm -f *.o
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

