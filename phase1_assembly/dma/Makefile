#
# Makefile for the Freescale Kinetis K60 / ARM Cortex-M4
#
# Rob Laswick
# May 15 2012
#

# Name of project/output file:

TARGET = demo

# List your asm files here (minus the .s):

ASM_PIECES = startcode application

PATH :=/opt/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_EABI/bin:${PATH}
AS = arm-none-eabi-gcc
LD = arm-none-eabi-ld
GDB = arm-none-eabi-gdb
OBJDUMP = arm-none-eabi-objdump

ASM_FLAGS = -g -x assembler-with-cpp
ASM_FILES = ${ASM_PIECES:%=%.s}
ASM_O_FILES = ${ASM_FILES:%.s=%.o}

CPU_FLAGS = -mcpu=cortex-m4 -mthumb

LD_SCRIPT = linkerscript.ld
LD_FLAGS = -Map=${TARGET}.map

all: ${TARGET}.axf
	@${OBJDUMP} -DSsh ${TARGET}.axf >| ${TARGET}.out.s
	@ln -fs ${TARGET}.out.s out.s
	@ln -fs ${TARGET}.axf out.axf
	@echo
	@echo Executable: ${TARGET}.axf, sym-linked to out.axf
	@echo
	@echo Disassembly Listing: ${TARGET}.out.s, sym-linked to out.s
	@echo
	@${AS} --version

${TARGET}.axf: ${ASM_O_FILES}
	${LD} ${ASM_O_FILES} -T ${LD_SCRIPT} ${LD_FLAGS} -o ${TARGET}.axf

-include ${ASM_FILES:.s=.d}

%.d: %.s
	@set -e; rm -f $@; \
    ${AS} -MM ${ASM_FLAGS} ${CPU_FLAGS} $< > $@.$$$$; \
    sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
    rm -f $@.$$$$

%.o: %.s
	${AS} -c ${ASM_FLAGS} ${CPU_FLAGS} $*.s -o $*.o

clean:
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
	@echo Launching openOCD...
	@openocd -s /usr/local/share/openocd/scripts -f interface/osbdm.cfg \
    -f board/twr-k60n512.cfg

gdb:
	${GDB} --eval-command="target remote localhost:3333" out.axf

