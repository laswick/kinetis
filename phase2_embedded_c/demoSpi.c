/*******************************************************************************
*
* demoSpi.c
*
* Shaun Weise
* 2012 06 28
*
*******************************************************************************/

#include "kinetis.h"
#include "hardware.h"
#include "globalDefs.h"

spiIF_t spi = {
    .ctrlType              = SPI_CTRL_TYPE_NORMAL,
    .module                = SPI_MODULE_2,
    .ctrl.nml.sclkMode     = SPI_SCLK_MODE_0,
    .ctrl.nml.baudRate     = SPI_BAUDRATE_CLKDIV_64,
    .ctrl.nml.options      = SPI_OPTS_MASTER,
    .ctrl.nml.chipSelect   = SPI_CS_0,
    .ctrl.nml.csInactState = SPI_CS_0_INACT_HIGH,
    .ctrl.nml.frameSize    = 8,
};
uint16_t shaun[] = { 'S','h','a','u','n' };

uint16_t scrCmdClear[] = {0x1B, '[','0','j' };
uint16_t scrCmdReset[] = {0x1B, '[','0','*' };

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

static void delay(void)
{
    volatile uint32_t time = 0x0003ffff;
    while (time)
        --time;
}

int main(void)
{
    /* Configure the gpio's*/
    SIM_SCGC5 |= SIM_PORTD_ENABLE; /* Gotta satisfy the bastard */
    PORT_PCR(SPI2_SCK_PORT, SPI2_SCK_PIN) = SPI2_SCK_MUX;
    PORT_PCR(SPI2_SIN_PORT, SPI2_SIN_PIN) = SPI2_SIN_MUX;
    PORT_PCR(SPI2_SOUT_PORT, SPI2_SOUT_PIN) = SPI2_SOUT_MUX;
    PORT_PCR(SPI2_PCS0_PORT, SPI2_PCS0_PIN) = SPI2_PCS0_MUX;

    /* Open & configure the spi, then write to the screen */
    spiOpen(&spi);
    spiWrite(&spi, scrCmdReset, sizeof(scrCmdReset)/2);
    spiWrite(&spi, scrCmdClear, sizeof(scrCmdClear)/2);

    for (;;) {
        spiWrite(&spi, shaun, sizeof(shaun)/2);
        delay();
        delay();
        delay();
        delay();
    }

    /* Should never happen */
    spiClose(&spi);
    return 0;
}
