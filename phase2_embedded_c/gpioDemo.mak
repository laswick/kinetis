C_FILES = gpioDemo

ARGS = TARGET=gpioDemo C_PIECES=${C_FILES} OPT_LEVEL=0

all:
	make -f Makefile ${ARGS}

clean:
	make -f Makefile clean ${ARGS}

openocd:
	make -f Makefile openocd ${ARGS}

gdb:
	make -f Makefile gdb ${ARGS}

