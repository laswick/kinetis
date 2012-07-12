/*******************************************************************************
* spi.c
*
* Low level driver for the Kinetis SPI module.
*
* Shaun Weise
* 2012-06-25
*******************************************************************************/

#include "kinetis.h"
#include "hardware.h"
#include "globalDefs.h"

/*******************************************************************************/
void spiOpen(spiIF_t *spi)
/*******************************************************************************/
{
    uint32_t mcr = 0;
    uint32_t ctar0 = 0;
    uint32_t ctar1 = 0;
    uint32_t rser = 0;

    if (spi->ctrlType == SPI_CTRL_TYPE_NORMAL) {        /* SPI Normal Control*/

        /* Set Normal SPI defaults */
        ctar0 = SPI_CTAR_CSSCLK0 | SPI_CTAR_CSSCLK1  | SPI_CTAR_CSSCLK2
              | SPI_CTAR_ASC0    | SPI_CTAR_ASC1     | SPI_CTAR_ASC2
              | SPI_CTAR_DT0     | SPI_CTAR_DT1      | SPI_CTAR_DT2;
        ctar1 = SPI_CTAR_CSSCLK0 | SPI_CTAR_CSSCLK1  | SPI_CTAR_CSSCLK2
              | SPI_CTAR_ASC0    | SPI_CTAR_ASC1     | SPI_CTAR_ASC2
              | SPI_CTAR_DT0     | SPI_CTAR_DT1      | SPI_CTAR_DT2;

        /* SPI SCLK Mode */
        if( spi->ctrl.nml.sclkMode < NUMSPI_SCLK_MODES ) {
            ctar0 |= spi->ctrl.nml.sclkMode << 25;
        } else {
            assert(0);
        }

        /* SPI Baud Rate */
        if( spi->ctrl.nml.baudRate < NUM_SPI_BAUDRATES ) {
              ctar0 |= spi->ctrl.nml.baudRate;
        } else {
            assert(0);
        }

        /* SPI Misc Options */
        if (spi->ctrl.nml.options & SPI_OPTS_MASTER) {
          mcr |= SPI_MCR_MSTR;
        }
        if (spi->ctrl.nml.options & SPI_OPTS_LSB_FIRST) {
          ctar0 |= SPI_CTAR_LSBFE;
        }
        if (spi->ctrl.nml.options & SPI_OPTS_CONT_SCK_EN) {
          mcr |= SPI_MCR_CONT_SCKE;
        }
        if (spi->ctrl.nml.options & SPI_OPTS_TX_FIFO_DSBL) {
          mcr |= SPI_MCR_DIS_TXF;
        }
        if (spi->ctrl.nml.options & SPI_OPTS_RX_FIFO_DSBL) {
          mcr |= SPI_MCR_DIS_RXF;
        }
        if (spi->ctrl.nml.options & SPI_OPTS_RX_FIFO_OVR_EN) {
          mcr |= SPI_MCR_ROOE;
        }

        /* SPI Chip select inactive state*/
        if( spi->ctrl.nml.csInactState < 0x3F ) {
            mcr |= spi->ctrl.nml.csInactState << 16;
        } else {
            assert(0);
        }

        /* SPI Frame Size */
        if( spi->ctrl.nml.frameSize >= SPI_FMSZ_MIN
            && spi->ctrl.nml.frameSize <= SPI_FMSZ_MAX ) {
              ctar0 |= (spi->ctrl.nml.frameSize-1) << 27;
        } else {
            assert(0);
        }
    }
    else {                                              /* RAW SPI control */
        mcr   = spi->ctrl.raw.mcr;
        ctar0 = spi->ctrl.raw.ctar0;
        ctar1 = spi->ctrl.raw.ctar0;
        rser  = spi->ctrl.raw.rser;
    }

    /* Ready to write config, wake up the asshole
     * and tell him to respect our authority */
    if( spi->module < NUM_SPI_MODULES) {
        switch( spi->module ){
            case SPI_MODULE_0:
                SIM_SCGC6 |= SIM_SCGC6_SPI0_ENABLE;
                spi->modAddr = SPI0_BASE_ADDR;
            break;
            case SPI_MODULE_1:
                SIM_SCGC6 |= SIM_SCGC6_SPI1_ENABLE;
                spi->modAddr = SPI1_BASE_ADDR;
            break;
            case SPI_MODULE_2:
                SIM_SCGC3 |= SIM_SCGC3_SPI2_ENABLE;
                spi->modAddr = SPI2_BASE_ADDR;
            break;
            default:
                assert(0);
            break;
        }

        /* Write the Register values values */
        SPI_MCR(spi->modAddr)   = mcr;
        SPI_CTAR0(spi->modAddr) = ctar0;
        SPI_CTAR1(spi->modAddr) = ctar1;
        SPI_RSER(spi->modAddr)  = rser;
    } else {
        assert(0);
    }
}

/*******************************************************************************/
void spiClose(spiIF_t *spi)
/*******************************************************************************/
{
    if (spi->module < NUM_SPI_MODULES) {
        switch( spi->module ){
            case SPI_MODULE_0:
                SIM_SCGC6 &= ~(SIM_SCGC6_SPI0_ENABLE);
            break;
            case SPI_MODULE_1:
                SIM_SCGC6 &= ~(SIM_SCGC6_SPI1_ENABLE);
            break;
            case SPI_MODULE_2:
                SIM_SCGC3 &= ~(SIM_SCGC3_SPI2_ENABLE);
            break;
            default:
                assert(0);
            break;
        }
    }
}
/*******************************************************************************/
void spiWrite(spiIF_t *spi, void *data, unsigned len)
/*******************************************************************************/
{
    unsigned i;
    uint16_t *dataPtr = (uint16_t *) data;
    uint32_t pushr = 0;
    uint32_t pushrMask = 0;

    /* TODO Remove magic numbers */

    if (spi->ctrlType == SPI_CTRL_TYPE_NORMAL) {      /* Normal SPI control */

        /* SPI Chip select is continuously asserted */
        if (spi->ctrl.nml.options & SPI_OPTS_PCS_CONT) {
            pushrMask |= SPI_PUSHR_CONT;
        }
        /* SPI Chip select to assert*/
        if( spi->ctrl.nml.chipSelect < 0x3F ) {
              pushrMask |= spi->ctrl.nml.chipSelect << 16;
        } else {
            assert(0);
        }
    }
    else {                                               /* RAW SPI Control */
        pushrMask = spi->ctrl.raw.pushr;
    }

   for(i = 0; i < len; i++) {

        pushr = (pushrMask | (uint32_t)(*dataPtr++));

        /* RANT START:
         * I could not use the Transmit fifo full flag (TFFF)! because
         * it would appears that it is only set if the dma
         * controller is responsible for a write to the PUSHR.
         * IMO, this flag should be set ANYTIME the fifo is
         * full! The documentation contradicts itself regarding
         * this flag. So the code would of looked like:
         *      while( !(SPI_SR(spi->modAddr) & SPI_SR_TFFF) );
         * and I would have a couple hours of my life back.
         */

        /* Instead, i am forced to read the number of entrys
         * currently in the fifo and comparing it against the
         * max. Small difference yes, but other way is less
         * cycles and more intuitive. */
        while( ((SPI_SR(spi->modAddr) & SPI_SR_TXCTR)>>12) >=4 ) {
            /* Add NOP here to prevent optimization from removing
             * this dead loop, if we ever turn it on */
            asm volatile("nop");
        };
        SPI_PUSHR(spi->modAddr) = pushr;
    }
}

/*******************************************************************************/
void spiRead(spiIF_t *spi, void *data, unsigned len)
/*******************************************************************************/
{
    /* To be done */
}

/*******************************************************************************/
void spiWriteRead(spiIF_t *spi, void *dataOut, unsigned lenOut,
                                void *dataIn,  unsigned lenIn   )
/*******************************************************************************/
{
    /* To be done */
}
