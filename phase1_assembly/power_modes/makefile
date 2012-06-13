#
# Makefile for the Freescale Kinetis K60 / ARM Cortex-M4
#
# Rob Laswick
# May 15 2012
#

# Name of project/output file:

TARGET = pwrDemo

# List your asm files here (minus the .s):

ASM_PIECES = startcode application

PATH :=/opt/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_GNU_Linux/bin:${PATH}
AS = arm-none-linux-gnueabi-as
LD = arm-none-linux-gnueabi-ld
GDB = arm-none-linux-gnueabi-gdb
OBJDUMP = arm-none-linux-gnueabi-objdump

ASM_FLAGS = -g
ASM_FILES = ${ASM_PIECES:%=%.s}
ASM_O_FILES = ${ASM_FILES:%.s=%.o}

CPU_FLAGS = -mcpu=cortex-m4 -mthumb

LD_SCRIPT = linkerscript.ld
LD_FLAGS = -Map=${TARGET}.map

all: ${TARGET}.axf
	@${OBJDUMP} -DS ${TARGET}.axf >| ${TARGET}.out.s
	@ln -fs ${TARGET}.out.s out.s
	@ln -fs ${TARGET}.axf out.axf
	@echo Executable: ${TARGET}.axf, sym-linked to out.axf
	@echo Disassembly Listing: ${TARGET}.out.s, sym-linked to out.s
	@echo Build Success.

${TARGET}.axf: ${ASM_O_FILES}
	@${LD} ${ASM_O_FILES} -T ${LD_SCRIPT} ${LD_FLAGS} -o ${TARGET}.axf

%.o: %.s
	@${AS} ${ASM_FLAGS} ${CPU_FLAGS} -o $@ $<
	@echo Compiling: $<

clean:
	rm -f *.o
	rm -f ${TARGET}.axf
	rm -f ${TARGET}.out.s
	rm -f out.axf
	rm -f out.s
	rm -f ${TARGET}.map
	@echo Clean.

openocd:
	@echo Launching openOCD...
	@openocd -s /usr/local/share/openocd/scripts -f interface/osbdm.cfg \
    -f board/twr-k60n512.cfg

gdb:
	@echo Launching openOCD...
	${GDB} --eval-command="target remote localhost:3333" out.axf

