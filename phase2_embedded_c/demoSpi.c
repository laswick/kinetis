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

#include "kinetis.h"
#include "hardware.h"
#include "globalDefs.h"

/* Example using normal control, much easier to understand */
spiIF_t spi2 = {
    .ctrlType              = SPI_CTRL_TYPE_NORMAL,
    .module                = SPI_MODULE_2,
    .ctrl.nml.sclkMode     = SPI_SCLK_MODE_0,
    .ctrl.nml.baudRate     = SPI_BAUDRATE_CLKDIV_256,
    .ctrl.nml.options      = SPI_OPTS_MASTER,
    .ctrl.nml.chipSelect   = SPI_CS_0,
    .ctrl.nml.csInactState = SPI_CS_0_INACT_HIGH,
    .ctrl.nml.frameSize    = 8,
};

/* Example using raw control */
spiIF_t spi0 = {
    .ctrlType        = SPI_CTRL_TYPE_RAW,
    .module          = SPI_MODULE_0,
    .ctrl.raw.mcr    = SPI_MCR_MSTR | SPI_MCR_PCSIS0,
    .ctrl.raw.ctar0  = SPI_CTAR_FMSZ | SPI_CTAR_CPOL | SPI_CTAR_CSSCLK
                     | SPI_CTAR_ASC  | SPI_CTAR_DT   | SPI_CTAR_BR3,
    .ctrl.raw.ctar1  = 0,
    .ctrl.raw.rser   = 0,
    .ctrl.raw.pushr  = SPI_PUSHR_PCS0,
};

uint16_t shaun[] = { 'S','h','a','u','n' };

uint16_t scrCmdClear[] = {0x1B, '[','0','j',0 };
uint16_t scrCmdReset[] = {0x1B, '[','0','*',0 };
uint16_t scrCmdGotoLine2[] = {0x1B, '[','1',';','0','H' };
uint16_t scrCmdBacklightOn[] = {0x1B, '[','3','e',0 };
uint16_t scrCmdWrap16[] = {0x1B, '[','0','h',0 };
uint16_t scrCmdScrollR1[] = {0x1B, '[','1','A',0 };

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
    /* Configure the gpio's*/
    SIM_SCGC5 |= (SIM_PORTD_ENABLE|SIM_PORTA_ENABLE); /* Gotta satisfy the bastard */

    /* Configure Pins for SPI 0 */
    PORT_PCR(SPI2_SCK_PORT, SPI2_SCK_PIN) = SPI2_SCK_MUX;
    PORT_PCR(SPI2_SIN_PORT, SPI2_SIN_PIN) = SPI2_SIN_MUX;
    PORT_PCR(SPI2_SOUT_PORT, SPI2_SOUT_PIN) = SPI2_SOUT_MUX;
    PORT_PCR(SPI2_PCS0_PORT, SPI2_PCS0_PIN) = SPI2_PCS0_MUX;

    /* Configure Pins SPI 2 */
    PORT_PCR(SPI0_SCK_PORT, SPI0_SCK_PIN) = SPI0_SCK_MUX;
    PORT_PCR(SPI0_SIN_PORT, SPI0_SIN_PIN) = SPI0_SIN_MUX;
    PORT_PCR(SPI0_SOUT_PORT, SPI0_SOUT_PIN) = SPI0_SOUT_MUX;
    PORT_PCR(SPI0_PCS0_PORT, SPI0_PCS0_PIN) = SPI0_PCS0_MUX;

    /* Open & configure the spi, then write to the screen */
    spiOpen(&spi2);
    spiOpen(&spi0);

    spiWrite(&spi2, scrCmdReset, sizeof(scrCmdReset)/2);
    spiWrite(&spi2, scrCmdClear, sizeof(scrCmdClear)/2);
/*    spiWrite(&spi, scrCmdBacklightOn, sizeof(scrCmdBacklightOn)/2); */
/*    spiWrite(&spi, scrCmdWrap16, sizeof(scrCmdWrap16)/2); */
    delay();
    spiWrite(&spi2, shaun, sizeof(shaun)/2);
    delay();
    spiWrite(&spi2, scrCmdGotoLine2, sizeof(scrCmdGotoLine2)/2);
    delay();
    spiWrite(&spi2, shaun, sizeof(shaun)/2);
    delay();

    for (;;) {
        spiWrite(&spi2, scrCmdScrollR1, sizeof(scrCmdScrollR1)/2);
        delay();
        delay();
        delay();
    }

    /* Should never happen */
    spiClose(&spi0);
    spiClose(&spi2);
    return 0;
}
