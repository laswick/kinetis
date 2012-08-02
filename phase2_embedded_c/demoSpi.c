/*******************************************************************************
*
* demoSpi.c
*
* Shaun Weise
* 2012 06 28
*
*******************************************************************************
* This demo demonstrates basic SPI functionality. SPI2 is connected to a
* digilent PmodCls, a 16x2 lcd character display. It features SPI, I2C and UART
* interfaces so its perfect for testing out these interfaces on new platforms.
* Write character data to it and it will display the characters, no
* initialization required. There are special control sequences to control the
* cursor, current line, etc.
*******************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include "kinetis.h"
#include "hardware.h"
#include "globalDefs.h"

/* My name, in a character array. I like seeing my name. */
char shaun[] = { 'S','h','a','u','n' };

/* PmodCls Serial Commands */
char scrCmdClear[]      = {0x1B, '[','0','j' };
char scrCmdReset[]      = {0x1B, '[','0','*' };
char scrCmdDispMode16[] = {0x1B, '[','0','h' };
char scrCmdDispMode40[] = {0x1B, '[','1','h' };
char scrCmdGotoLine1[]  = {0x1B, '[','0',';','0','0','H' };
char scrCmdGotoLine2[]  = {0x1B, '[','1',';','0','0','H' };
char scrCmdScrollL1[]   = {0x1B, '[','0','1','@' };
char scrCmdScrollR1[]   = {0x1B, '[','0','1','A' };
char scrCmdDispEnBklghtOn[]  = {0x1B, '[','3','e' }; /* Screwed up */

static void delay(void)
{
    volatile uint32_t time = 0x0003ffff;
    while (time)
        --time;
}

int main(void)
{
    int fd;
    int i;
    char str[10];
    spiWriteRead_t wr;

    /* OPEN the spi2 device. Check to make sure you are given a 'good'
     * file descriptor, ie. not -1. The file descriptor is required
     * for any future POSIX system call. */
    fd = open("spi2", 0, 0);
    if (fd==-1) {
        assert(0);
    }

    /* Configure spi2 using IOCTL. There are many ways to use IOCTL
     * commands. You can pass in numbers, flags or even pointers
     * using the flag argument within ioctl.  */
    ioctl(fd, IO_IOCTL_SPI_SET_PORT_PCRS, 0);
    ioctl(fd, IO_IOCTL_SPI_SET_BAUD, SPI_BAUDRATE_CLKDIV_256);
    ioctl(fd, IO_IOCTL_SPI_SET_SCLK_MODE, SPI_SCLK_MODE_0);
    ioctl(fd, IO_IOCTL_SPI_SET_FMSZ, 8);
    ioctl(fd, IO_IOCTL_SPI_SET_OPTS, SPI_OPTS_MASTER);
    ioctl(fd, IO_IOCTL_SPI_SET_CS, SPI_CS_0);
    ioctl(fd, IO_IOCTL_SPI_SET_CS_INACT_STATE, SPI_CS_0_INACT_HIGH);

    /* WRITE to spi2. Not much to say here, just give a buffer
     * pointer and a length. Should check to see how many bytes
     * were actually written, but I forgot to do that here :D */
    write(fd, scrCmdReset, sizeof(scrCmdReset));
    delay();/* Device needs an unknown amount of time to reset .. */
    write(fd, scrCmdDispMode40, sizeof(scrCmdDispMode40));
    delay();
    write(fd, scrCmdDispEnBklghtOn, sizeof(scrCmdDispEnBklghtOn));
    delay();
    write(fd, scrCmdClear, sizeof(scrCmdClear));
    delay();
    ioctl(fd, IO_IOCTL_SPI_FLUSH_RX_FIFO, 0);
    delay();

    /* Use IOCTL for a synchronus write/read since the standard
     * system calls are not meant to do this. Notice the pointer
     * to a structure given as the flag argument.
     * Note: This example expects the MISO & MOSI to be connected
     * together forming a loopback. */
    wr.out = (uint8_t *) shaun;
    wr.in  = (uint8_t *) str;
    wr.len = sizeof(shaun);
    ioctl(fd, IO_IOCTL_SPI_WRITE_READ, (int) &wr);
    str[sizeof(shaun)] = '\0';
    if ( strcmp(shaun,str) != 0 ) {
          strcpy(shaun, "ERROR");
    }

    write(fd, scrCmdReset, sizeof(scrCmdReset));
    delay();/* Device needs an unknown amount of time to reset .. */
    write(fd, scrCmdDispMode40, sizeof(scrCmdDispMode40));
    delay();
    write(fd, scrCmdDispEnBklghtOn, sizeof(scrCmdDispEnBklghtOn));
    delay();
    for (;;) {
        write(fd, scrCmdClear, sizeof(scrCmdClear));
        delay();
        delay();
        delay();
        write(fd, scrCmdGotoLine1, sizeof(scrCmdGotoLine1));
        write(fd, shaun, sizeof(shaun));
        write(fd, " ", 1);
        write(fd, shaun, sizeof(shaun));
        write(fd, " ", 1);
        write(fd, shaun, sizeof(shaun));
        write(fd, " ", 1);
        write(fd, shaun, sizeof(shaun));
        write(fd, " ", 1);
        write(fd, scrCmdGotoLine2, sizeof(scrCmdGotoLine2));
        write(fd, shaun, sizeof(shaun));
        write(fd, " ", 1);
        write(fd, shaun, sizeof(shaun));
        write(fd, " ", 1);
        write(fd, shaun, sizeof(shaun));
        write(fd, " ", 1);
        write(fd, shaun, sizeof(shaun));
        write(fd, " ", 1);
        delay();
        delay();
        delay();
        for( i = 0; i < 10;i++) {
            write(fd, scrCmdScrollL1, sizeof(scrCmdScrollL1));
            delay();
            delay();
        }
        for( i = 0; i < 10;i++) {
            write(fd, scrCmdScrollR1, sizeof(scrCmdScrollR1));
            delay();
            delay();
        }
    }

    /* Should never happen */
    close(fd);
    return 0;
}
