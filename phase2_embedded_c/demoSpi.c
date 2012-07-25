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

char shaun[] = { 'S','h','a','u','n' };

/* Working Commands */
char scrCmdClear[]      = {0x1B, '[','0','j' };
char scrCmdReset[]      = {0x1B, '[','0','*' };
char scrCmdDispMode16[] = {0x1B, '[','0','h' };
char scrCmdDispMode40[] = {0x1B, '[','1','h' };
char scrCmdGotoLine1[]  = {0x1B, '[','0',';','0','0','H' };
char scrCmdGotoLine2[]  = {0x1B, '[','1',';','0','0','H' };
char scrCmdScrollL1[]   = {0x1B, '[','0','1','@' };
char scrCmdScrollR1[]   = {0x1B, '[','0','1','A' };

/* Screwed up commands */
char scrCmdDispEnBklghtOn[]  = {0x1B, '[','3','e' };

#define SPI2_SCK_PIN    12
#define SPI2_SCK_PORT   PORTD
#define SPI2_SCK_MUX    PORT_MUX_ALT2

#define SPI2_SIN_PIN    14
#define SPI2_SIN_PORT   PORTD
#define SPI2_SIN_MUX    PORT_MUX_ALT2

#define SPI2_SOUT_PIN   13
#define SPI2_SOUT_PORT  PORTD
#define SPI2_SOUT_MUX   PORT_MUX_ALT2

#define SPI2_PCS0_PIN   11
#define SPI2_PCS0_PORT  PORTD
#define SPI2_PCS0_MUX   PORT_MUX_ALT2

#define SPI0_SCK_PIN    15
#define SPI0_SCK_PORT   PORTA
#define SPI0_SCK_MUX    PORT_MUX_ALT2

#define SPI0_SIN_PIN    17
#define SPI0_SIN_PORT   PORTA
#define SPI0_SIN_MUX    PORT_MUX_ALT2

#define SPI0_SOUT_PIN   16
#define SPI0_SOUT_PORT  PORTA
#define SPI0_SOUT_MUX   PORT_MUX_ALT2

#define SPI0_PCS0_PIN   14
#define SPI0_PCS0_PORT  PORTA
#define SPI0_PCS0_MUX   PORT_MUX_ALT2

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

    /* Configure the gpio's*/
    SIM_SCGC5 |= (SIM_PORTD_ENABLE|SIM_PORTA_ENABLE); /* Gotta satisfy the bastard */

    /* Configure Pins for SPI 0 */
    PORT_PCR(SPI2_SCK_PORT, SPI2_SCK_PIN) = SPI2_SCK_MUX;
    PORT_PCR(SPI2_SIN_PORT, SPI2_SIN_PIN) = SPI2_SIN_MUX;
    PORT_PCR(SPI2_SOUT_PORT, SPI2_SOUT_PIN) = SPI2_SOUT_MUX;
    PORT_PCR(SPI2_PCS0_PORT, SPI2_PCS0_PIN) = SPI2_PCS0_MUX;

    /* Configure Pins SPI 1 */
    PORT_PCR(SPI0_SCK_PORT, SPI0_SCK_PIN) = SPI0_SCK_MUX;
    PORT_PCR(SPI0_SIN_PORT, SPI0_SIN_PIN) = SPI0_SIN_MUX;
    PORT_PCR(SPI0_SOUT_PORT, SPI0_SOUT_PIN) = SPI0_SOUT_MUX;
    PORT_PCR(SPI0_PCS0_PORT, SPI0_PCS0_PIN) = SPI0_PCS0_MUX;

    /* Power the SPI Module */
    /* SIM_SCGC6 |= SIM_SCGC6_SPI0_ENABLE; */
    /* SIM_SCGC6 |= SIM_SCGC6_SPI1_ENABLE; */
    SIM_SCGC3 |= SIM_SCGC3_SPI2_ENABLE;

    fd = open("spi2", 0, 0);

    ioctl(fd, IO_IOCTL_SPI_SET_BAUD, SPI_BAUDRATE_CLKDIV_256);
    ioctl(fd, IO_IOCTL_SPI_SET_SCLK_MODE, SPI_SCLK_MODE_0);
    ioctl(fd, IO_IOCTL_SPI_SET_FMSZ, 8);
    ioctl(fd, IO_IOCTL_SPI_SET_OPTS, SPI_OPTS_MASTER);
    ioctl(fd, IO_IOCTL_SPI_SET_CS, SPI_CS_0);
    ioctl(fd, IO_IOCTL_SPI_SET_CS_INACT_STATE, SPI_CS_0_INACT_HIGH);

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
