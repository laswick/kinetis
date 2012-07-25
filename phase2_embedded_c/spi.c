/*******************************************************************************
* spi.c
*
* Low level driver for the Kinetis SPI module.
*
* Shaun Weise
* 2012-06-25
*******************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "kinetis.h"
#include "hardware.h"
#include "globalDefs.h"

typedef enum spiModule_e{
    SPI_MODULE_0,
    SPI_MODULE_1,
    SPI_MODULE_2,
    NUM_SPI_MODULES,
} spiModule_t;

typedef struct spi_s {
    spiMcr_t      mcr;
    spiCtar_t     ctar0;
    spiCtar_t     ctar1;
    spiRser_t     rser;
    spiPushr_t    pushr;
    unsigned      addr;
    int           fd;
} spi_t;

spi_t spiList[NUM_SPI_MODULES] = {
    [SPI_MODULE_0] = {
        .mcr = 0,
        .ctar0 = 0,
        .ctar1 = 0,
        .rser = 0,
        .pushr = 0,
        .addr = SPI0_BASE_ADDR,
        .fd = 0
    },
    [SPI_MODULE_1] = {
        .mcr = 0,
        .ctar0 = 0,
        .ctar1 = 0,
        .rser = 0,
        .pushr = 0,
        .addr = SPI1_BASE_ADDR,
        .fd = 0
    },
    [SPI_MODULE_2] = {
        .mcr = 0,
        .ctar0 = 0,
        .ctar1 = 0,
        .rser = 0,
        .pushr = 0,
        .addr = SPI2_BASE_ADDR,
        .fd = 0
    },
};

/*******************************************************************************/
static spi_t *spiModuleFromFd(int fd)
/*******************************************************************************/
{
    int i;
    for (i =0; i < NUM_SPI_MODULES; i++) {
        if (spiList[i].fd == fd) return &spiList[i];
    }
    return NULL;
}

/*******************************************************************************/
static int spiOpen(spiModule_t mod, int fd)
/*******************************************************************************/
{
    spi_t *spi = NULL;

    spiList[mod].fd = fd;
    spi = spiModuleFromFd(fd);

    if (spi != NULL) {
        /* Set Normal SPI defaults */
        spi->mcr   = SPI_MCR_MSTR;
        spi->ctar0 = SPI_CTAR_CSSCLK0 | SPI_CTAR_CSSCLK1  | SPI_CTAR_CSSCLK2
                   | SPI_CTAR_ASC0    | SPI_CTAR_ASC1     | SPI_CTAR_ASC2
                   | SPI_CTAR_DT0     | SPI_CTAR_DT1      | SPI_CTAR_DT2;
        spi->ctar1 = SPI_CTAR_CSSCLK0 | SPI_CTAR_CSSCLK1  | SPI_CTAR_CSSCLK2
                   | SPI_CTAR_ASC0    | SPI_CTAR_ASC1     | SPI_CTAR_ASC2
                   | SPI_CTAR_DT0     | SPI_CTAR_DT1      | SPI_CTAR_DT2;
        spi->rser  = 0;
        spi->pushr = 0;
    }
    else {
        return -1;
    }

    /* Write the Register values values */
    SPI_MCR  (spi->addr) = spi->mcr;
    SPI_CTAR0(spi->addr) = spi->ctar0;
    SPI_CTAR1(spi->addr) = spi->ctar1;
    SPI_RSER (spi->addr) = spi->rser;
    return fd;
}

#if 0
/*******************************************************************************/
void spiClose(spi_t *spi)
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
#endif

/*******************************************************************************/
static unsigned spiWrite(spi_t *spi, const void *data, unsigned len)
/*******************************************************************************/
{
    unsigned i;
    uint8_t *dataPtr = (uint8_t *) data;
    uint32_t pushr = 0;

   for(i = 0; i < len; i++) {

        pushr = (spi->pushr | (uint32_t)(*dataPtr++));

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
        while( ((SPI_SR(spi->addr) & SPI_SR_TXCTR)>>12) >=4 ) {
            /* Add NOP here to prevent optimization from removing
             * this dead loop, if we ever turn it on */
            asm volatile("nop");
        };
        SPI_PUSHR(spi->addr) = pushr;
    }
   return len;
}

#if 0
/*******************************************************************************/
static void spiRead(spi_t *spi, void *data, unsigned len)
/*******************************************************************************/
{
    /* To be done */
}

/*******************************************************************************/
static void spiWriteRead(spi_t *spi, void *dataOut, unsigned lenOut,
                                     void *dataIn,  unsigned lenIn)
/*******************************************************************************/
{
    /* To be done */
}
#endif


/*******************************************************************************/
int spi_open_r (void *reent, int fd, const char *file, int flags, int mode )
/*******************************************************************************/
{
    spiModule_t mod;

    if (strcmp(file, "spi0") == 0 ) {
        mod = SPI_MODULE_0;
    }
    else if (strcmp(file, "spi1") == 0) {
        mod = SPI_MODULE_1;
    }
    else if (strcmp(file, "spi2") == 0) {
        mod = SPI_MODULE_2;
    }
    else {
        ((struct _reent *)reent)->_errno = ENODEV;
        return -1;
    }
    spiOpen(mod,fd);
    return fd;
}

/*******************************************************************************/
int spi_ioctl(int fd, int cmd,  int flags)
/*******************************************************************************/
/* Todo: return errors if flags or cmd is bad */
{
    spi_t *spi = spiModuleFromFd(fd);

    switch (cmd) {
        case IO_IOCTL_SPI_SET_BAUD:
            if (flags < NUM_SPI_BAUDRATES) {
                spi->ctar0 &= ~(SPI_CTAR_BR);
                spi->ctar0 |= flags;
                SPI_CTAR0(spi->addr) = spi->ctar0;
            }
        break;
        case IO_IOCTL_SPI_SET_SCLK_MODE:
            if (flags < NUMSPI_SCLK_MODES) {
                spi->ctar0 &= ~(SPI_CTAR_CPHA | SPI_CTAR_CPOL);
                spi->ctar0 |= flags << 25;
                SPI_CTAR0(spi->addr) = spi->ctar0;
            }
        break;
        case IO_IOCTL_SPI_SET_FMSZ:
            if (flags >= SPI_FMSZ_MIN && flags <= SPI_FMSZ_MAX) {
                spi->ctar0 &= ~(SPI_CTAR_FMSZ);
                spi->ctar0 |= (flags-1) << 27;
                SPI_CTAR0(spi->addr) = spi->ctar0;
            }
        break;
        case IO_IOCTL_SPI_SET_OPTS:
            if (flags & SPI_OPTS_MASTER) {
                spi->mcr |= SPI_MCR_MSTR;
                SPI_MCR(spi->addr) = spi->mcr;
            }
            if (flags & SPI_OPTS_LSB_FIRST) {
                spi->ctar0 |= SPI_CTAR_LSBFE;
                SPI_CTAR0(spi->addr) = spi->ctar0;
            }
            if (flags & SPI_OPTS_CONT_SCK_EN) {
                spi->mcr |= SPI_MCR_CONT_SCKE;
                SPI_MCR(spi->addr) = spi->mcr;
            }
            if (flags & SPI_OPTS_TX_FIFO_DSBL) {
                spi->mcr |= SPI_MCR_DIS_TXF;
                SPI_MCR(spi->addr) = spi->mcr;
            }
            if (flags & SPI_OPTS_RX_FIFO_DSBL) {
                spi->mcr |= SPI_MCR_DIS_RXF;
                SPI_MCR(spi->addr) = spi->mcr;
            }
            if (flags & SPI_OPTS_RX_FIFO_OVR_EN) {
                spi->mcr |= SPI_MCR_ROOE;
                SPI_MCR(spi->addr) = spi->mcr;
            }
        break;
        case IO_IOCTL_SPI_SET_CS:
            if( flags < 0x3F ) {
                spi->pushr &= ~(0x3F << 16);
                spi->pushr |= flags << 16;
            }
        break;
        case IO_IOCTL_SPI_SET_CS_INACT_STATE:
            if( flags < 0x3F ) {
                spi->mcr &= ~(0x3F << 16);
                spi->mcr |= flags << 16;
                SPI_MCR(spi->addr) = spi->mcr;
            }
        break;
        default:
            assert(0);
        break;
    }
    return 1;
}

/*******************************************************************************/
int spi_close_r (void *reent, int fd )
/*******************************************************************************/
{
    spi_t *spi = spiModuleFromFd(fd);
    spi->fd = 0;
    return spi->fd;
}
/*******************************************************************************/
long spi_write_r (void *reent, int fd, const void *buf, int len )
/*******************************************************************************/
{
    return spiWrite( spiModuleFromFd(fd), buf, len);
}
/*******************************************************************************/
long spi_read_r (void *reent, int fd, void *buf, int len )
/*******************************************************************************/
{
    return -1;
}
