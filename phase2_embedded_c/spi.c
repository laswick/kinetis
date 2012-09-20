/*******************************************************************************
*
* spi.c
*
* Shaun Weise
*
* Low level driver for the Kinetis SPI module.
*
* Copyright (C) 2012 www.laswick.net
*
* This program is free software.  It comes without any warranty, to the extent
* permitted by applicable law.  You can redistribute it and/or modify it under
* the terms of the WTF Public License (WTFPL), Version 2, as published by
* Sam Hocevar.  See http://sam.zoy.org/wtfpl/COPYING for more details.
*
******************************************************************************/
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "kinetis.h"
#include "hardware.h"
#include "globalDefs.h"

/* TODO: Clean these up later */
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
    unsigned      sim;
    volatile uint32_t * simScgcPtr;
    unsigned            simScgcEnBit;
} spi_t;

spi_t spiList[NUM_SPI_MODULES] = {
    [SPI_MODULE_0] = {
        .mcr   = SPI_MCR_MSTR,
        .ctar0 = SPI_CTAR_CSSCLK0 | SPI_CTAR_CSSCLK1  | SPI_CTAR_CSSCLK2
               | SPI_CTAR_ASC0    | SPI_CTAR_ASC1     | SPI_CTAR_ASC2
               | SPI_CTAR_DT0     | SPI_CTAR_DT1      | SPI_CTAR_DT2,
        .ctar1 = SPI_CTAR_CSSCLK0 | SPI_CTAR_CSSCLK1  | SPI_CTAR_CSSCLK2
               | SPI_CTAR_ASC0    | SPI_CTAR_ASC1     | SPI_CTAR_ASC2
               | SPI_CTAR_DT0     | SPI_CTAR_DT1      | SPI_CTAR_DT2,
        .rser  = 0,
        .pushr = 0,
        .simScgcPtr = SIM_SCGC6_PTR,
        .simScgcEnBit = SIM_SCGC6_SPI0_ENABLE,
    },
    [SPI_MODULE_1] = {
        .mcr   = SPI_MCR_MSTR,
        .ctar0 = SPI_CTAR_CSSCLK0 | SPI_CTAR_CSSCLK1  | SPI_CTAR_CSSCLK2
               | SPI_CTAR_ASC0    | SPI_CTAR_ASC1     | SPI_CTAR_ASC2
               | SPI_CTAR_DT0     | SPI_CTAR_DT1      | SPI_CTAR_DT2,
        .ctar1 = SPI_CTAR_CSSCLK0 | SPI_CTAR_CSSCLK1  | SPI_CTAR_CSSCLK2
               | SPI_CTAR_ASC0    | SPI_CTAR_ASC1     | SPI_CTAR_ASC2
               | SPI_CTAR_DT0     | SPI_CTAR_DT1      | SPI_CTAR_DT2,
        .rser  = 0,
        .pushr = 0,
        .addr  = SPI1_BASE_ADDR,
        .simScgcPtr = SIM_SCGC6_PTR,
        .simScgcEnBit = SIM_SCGC6_SPI1_ENABLE,
    },
    [SPI_MODULE_2] = {
        .mcr   = SPI_MCR_MSTR,
        .ctar0 = SPI_CTAR_CSSCLK0 | SPI_CTAR_CSSCLK1  | SPI_CTAR_CSSCLK2
               | SPI_CTAR_ASC0    | SPI_CTAR_ASC1     | SPI_CTAR_ASC2
               | SPI_CTAR_DT0     | SPI_CTAR_DT1      | SPI_CTAR_DT2,
        .ctar1 = SPI_CTAR_CSSCLK0 | SPI_CTAR_CSSCLK1  | SPI_CTAR_CSSCLK2
               | SPI_CTAR_ASC0    | SPI_CTAR_ASC1     | SPI_CTAR_ASC2
               | SPI_CTAR_DT0     | SPI_CTAR_DT1      | SPI_CTAR_DT2,
        .rser  = 0,
        .pushr = 0,
        .addr  = SPI2_BASE_ADDR,
        .simScgcPtr = SIM_SCGC3_PTR,
        .simScgcEnBit = SIM_SCGC3_SPI2_ENABLE,
    },
};

/*******************************************************************************/
int spi_install(void)
/*******************************************************************************/
{
    int ret = TRUE;
    if( !deviceInstall(DEV_MAJ_SPI,spi_open_r,  spi_ioctl, spi_close_r,
                                                       spi_write_r, spi_read_r)){
        ret = FALSE;
    }
    if( !deviceRegister("spi0", DEV_MAJ_SPI, SPI_MODULE_0,  NULL) ) {
        ret =  FALSE;
    }
    if( !deviceRegister("spi1", DEV_MAJ_SPI, SPI_MODULE_1,  NULL) ) {
        ret =  FALSE;
    }
    if( !deviceRegister("spi2", DEV_MAJ_SPI, SPI_MODULE_2,  NULL) ) {
        ret =  FALSE;
    }

    return ret;
}

/*******************************************************************************/
static int spiOpen(devoptab_t *dot)
/*******************************************************************************/
{
    spi_t *spi;

    if (dot->priv) return FALSE; /* Device is already open */

    /* Create 'private' spi structure and point devoptab's
     * private pointer to it */
    spi = (spi_t *) malloc(sizeof(spi_t));
    if (!spi) return FALSE;
    else dot->priv = spi;

    /* Load init & default info into private spi structure */
    memcpy(spi, &spiList[dot->min], sizeof(spi_t));

    /* Enable the SIMSCGC for the spi module being used*/
    *spi->simScgcPtr |= spi->simScgcEnBit;

    /* Write configuration register values  */
    SPI_MCR  (spi->addr) = spi->mcr;
    SPI_CTAR0(spi->addr) = spi->ctar0;
    SPI_CTAR1(spi->addr) = spi->ctar1;
    SPI_RSER (spi->addr) = spi->rser;
    return TRUE;
}

/*******************************************************************************/
static unsigned spiWrite(devoptab_t *dot, const void *data, unsigned len)
/*******************************************************************************/
{
    unsigned i;
    uint8_t *dataPtr = (uint8_t *) data;
    uint32_t pushr = 0;
    spi_t *spi;

    if (!dot || !dot->priv) return FALSE;
    else spi = (spi_t *) dot->priv;

    for(i = 0; i < len; i++) {

        pushr = (spi->pushr | (uint32_t)(*dataPtr++));

        /* Write to the TX FIFO if there is room */

        /* WTF!
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
        }
        SPI_PUSHR(spi->addr) = pushr;
    }
    /* Wait for transfer to finish */
    while( ((SPI_SR(spi->addr) & SPI_SR_TXCTR)>>12) !=0 ){
            asm volatile("nop");
    }
    return len;
}

/*******************************************************************************/
static unsigned spiRead(devoptab_t *dot, void *data, unsigned len)
/*******************************************************************************/
{
    unsigned i;
    uint8_t *dataPtr = (uint8_t *) data;
    spi_t *spi;

    if (!dot || !dot->priv) return FALSE;
    else spi = (spi_t *) dot->priv;

    for(i = 0; i < len; i++) {

        /* Wait for the RXFIFO not to be empty */

        /* WTF!
         * The RXDF is NEVER cleared! Even when explicitly clearing it
         * like it says to in the documentation it remains 1 when the
         * FIFO is CLEARLY empty, RXCTR = 0, POPNXTPTR = 0 ! Again I
         * am forced to read the number of entrys in the fifo and wait
         * for it to become non-zero. */
        while ( ((SPI_SR(spi->addr) & SPI_SR_RXCTR)>>4) == 0) {
            asm volatile("nop");
        }

        /* Take a entry of the RX FIFO */
        *dataPtr++ = (uint8_t) SPI_POPR(spi->addr);
    }
    return len;
}

/*******************************************************************************/
static unsigned spiWriteRead(spi_t *spi, spiWriteRead_t *wr)
/*******************************************************************************/
{
    unsigned lenIn = 0;
    unsigned lenOut = 0;
    uint8_t *outPtr = wr->out;
    uint8_t *inPtr = wr->in;

    while( lenIn < wr->len) {

        /* Write to the TX FIFO if there is room */
        if( ((SPI_SR(spi->addr) & SPI_SR_TXCTR)>>12) < 4 ) {
            if (lenOut < wr->len) {
                SPI_PUSHR(spi->addr) = (spi->pushr | (uint32_t)(*outPtr++));
                lenOut++;
            }
        }

        /* Read from RX FIFO if there is anything*/
        if ( ((SPI_SR(spi->addr) & SPI_SR_RXCTR)>>4) != 0) {
            *inPtr++ = (uint8_t)SPI_POPR(spi->addr);
            lenIn++;
        }
    }
    return lenIn;
}

/*=============================================================================*/
/* POSIX FUNCTIONS                                                             */
/*=============================================================================*/

/*******************************************************************************/
/* spi_open_r                                                                  */
/*******************************************************************************/
/* Jobs of the 'open' syscall:
 *      Check device name
 *      Create a device 'state' structure, hook it to the devoptab private ptr
 *      Enable the SIM SCGC for the device
 *      Initialize the device with a default configuration
 ********************************************************************************/
int spi_open_r (void *reent, devoptab_t *dot, int mode, int flags )
{

    if (!dot || !dot->name) {
        /* errno ? */
        return FALSE;
    }

    /* Verify module instance */
    if( dot->min >= NUM_SPI_MODULES ) {
        ((struct _reent *)reent)->_errno = ENODEV;
        return FALSE;
    }

    /* Try to open if not already open */
    if (spiOpen(dot)) {
        return TRUE;
    } else {
        /* Device is already open, is this an issue or not? */
        ((struct _reent *)reent)->_errno = EPERM;
        return FALSE;
    }
}

/*******************************************************************************/
/* spi_ioctl_r                                                                 */
/*******************************************************************************/
/* Jobs of the 'ioctl' syscall:
 *      Implement any device specific commands.
 *          Commands are listed in hardware.h in the specific driver section
 *          Commands are NOT standardized however:
 *              See MQX's I/O drivers guide for commands that it supports
 *              These can provide a guide of which commands to implement
 *          Some common commands funtions:
 *              Set baud rate
 *              Set device registers to specific values
 *              Configure I/O pins
 *******************************************************************************/
int spi_ioctl(devoptab_t *dot, int cmd,  int flags)
/* TODO: return errors if flags or cmd is bad */
{
    spi_t *spi;

    if (!dot || !dot->priv) return FALSE;
    else spi = (spi_t *) dot->priv;

    switch (cmd) {
    case IO_IOCTL_SPI_SET_PORT_PCRS:
        switch (spi->addr) {
        case SPI0_BASE_ADDR:
            SIM_SCGC5 |= SIM_SCGC5_PORTA_ENABLE;
            PORT_PCR(SPI0_SCK_PORT, SPI0_SCK_PIN) = SPI0_SCK_MUX;
            PORT_PCR(SPI0_SIN_PORT, SPI0_SIN_PIN) = SPI0_SIN_MUX;
            PORT_PCR(SPI0_SOUT_PORT, SPI0_SOUT_PIN) = SPI0_SOUT_MUX;
            PORT_PCR(SPI0_PCS0_PORT, SPI0_PCS0_PIN) = SPI0_PCS0_MUX;
        break;
        case SPI1_BASE_ADDR:
#if 0
            PORT_PCR(SPI1_SCK_PORT, SPI1_SCK_PIN) = SPI1_SCK_MUX;
            PORT_PCR(SPI1_SIN_PORT, SPI1_SIN_PIN) = SPI1_SIN_MUX;
            PORT_PCR(SPI1_SOUT_PORT, SPI1_SOUT_PIN) = SPI1_SOUT_MUX;
            PORT_PCR(SPI1_PCS0_PORT, SPI1_PCS0_PIN) = SPI1_PCS0_MUX;
#endif
        break;
        case SPI2_BASE_ADDR:
            SIM_SCGC5 |= SIM_SCGC5_PORTD_ENABLE;
            PORT_PCR(SPI2_SCK_PORT, SPI2_SCK_PIN) = SPI2_SCK_MUX;
            PORT_PCR(SPI2_SIN_PORT, SPI2_SIN_PIN) = SPI2_SIN_MUX;
            PORT_PCR(SPI2_SOUT_PORT, SPI2_SOUT_PIN) = SPI2_SOUT_MUX;
            PORT_PCR(SPI2_PCS0_PORT, SPI2_PCS0_PIN) = SPI2_PCS0_MUX;
        break;
        default:
            assert(0);
            return FALSE;
        break;
        }
    break;
    case IO_IOCTL_SPI_SET_BAUD:
        if (flags < NUM_SPI_BAUDRATES) {
            spi->ctar0 &= ~(SPI_CTAR_BR);
            spi->ctar0 |= flags;
            SPI_CTAR0(spi->addr) = spi->ctar0;
        }
    break;
    case IO_IOCTL_SPI_SET_SCLK_MODE:
        if (flags < MAX_SPI_SCLK_MODES) {
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
    case IO_IOCTL_SPI_FLUSH_RX_FIFO:
        spi->mcr = SPI_MCR_CLR_RXF | SPI_MCR(spi->addr);
        SPI_MCR(spi->addr) = spi->mcr;
        spi->mcr &= ~(SPI_MCR_CLR_RXF);
        /* WTF!
         * Writing a 1 to CLR_RXF in the MCR does not
         * clear the RX Counter in the SR! Documentation Fail!
         * This means i have to flush the fifo manually by
         * poping the RX FIFO till empty. */
        while ( ((SPI_SR(spi->addr) & SPI_SR_RXCTR)>>4) != 0) {
            SPI_POPR(spi->addr); /* Ridiculus */
        }
    break;
    case IO_IOCTL_SPI_WRITE_READ:
        if (!flags) {
            assert(0);
            return FALSE;
        }
        return spiWriteRead(spi,(spiWriteRead_t *)flags);
    break;
    default:
        assert(0);
        return FALSE;
    break;
    }
    return TRUE;
}

/*******************************************************************************/
/* spi_close_r                                                                 */
/*******************************************************************************/
/* Jobs of the 'close' syscall:
 *      Disable the SIM SCGC for the device
 *      Free the device 'state' structure, unhook it to the devoptab private ptr
 *******************************************************************************/
int spi_close_r (void *reent, devoptab_t *dot )
{
    spi_t *spi = dot->priv;

    if (spi) {
        /* Disable the SIMSCGC for the spi module being used*/
        *spi->simScgcPtr &= ~(spi->simScgcEnBit);
        /* Unhook the private spi structure and free it */
        dot->priv = NULL;
        free(spi);
        return TRUE;
    }
    else {
        return FALSE;
    }
}

/*******************************************************************************/
/* spi_write_r                                                                 */
/*******************************************************************************/
/* Jobs of the 'write' syscall:
 *      Write data to the device.
 *      Return the number of bytes written
 *******************************************************************************/
long spi_write_r (void *reent, devoptab_t *dot, const void *buf, int len )
{
    /* You could just put your write function here, but I want switch between
     * polled & interupt functions here at a later point.*/
    return spiWrite(dot, buf, len);
}

/*******************************************************************************/
/* spi_read_r                                                                  */
/*******************************************************************************/
/* Jobs of the 'read' syscall:
 *      Read data from the device
 *      Return the number of bytes read
 *******************************************************************************/
long spi_read_r (void *reent, devoptab_t *dot, void *buf, int len )
{
    /* You could just put your read function here, but I want switch between
     * polled & interupt functions here at a later point.*/
    return spiRead(dot, buf, len);
}
