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

static unsigned spi0Isr(void);
static unsigned spi1Isr(void);
static unsigned spi2Isr(void);
static unsigned spi0DmaIsr(void);
static unsigned spi1DmaIsr(void);
static unsigned spi2DmaIsr(void);

static unsigned spiWritePolled(void *spiPtr, const void *data, unsigned len);
static unsigned spiReadPolled(void *spiPtr, void *data, unsigned len);
static unsigned spiWriteInterrupt(void *spiPtr, const void *data, unsigned len);
static unsigned spiReadInterrupt(void *spiPtr, void *data, unsigned len);
static unsigned spiReadDma(void *spiPtr, void *data, unsigned len);
static unsigned spiWriteDma(void *spiPtr, const void *data, unsigned len);

typedef enum spiModule_e {
    SPI_MODULE_0,
    SPI_MODULE_1,
    SPI_MODULE_2,
    NUM_SPI_MODULES,
} spiModule_t;

typedef enum spiPins_e {
    PIN_SIN,
    PIN_SOUT,
    PIN_SCK,
    PIN_PCS0,
    NUM_PINS,
} spiPins_t;

typedef struct spiPin_s {
    unsigned num;
    unsigned port;
    unsigned mux;
} spiPin_t;

typedef struct buffer_s {
    uint8_t * data;
    uint8_t * head;
    uint8_t * tail;
}buffer_t;

typedef struct spiIrq_s {
    unsigned vector;
    unsigned (*isr)(void);
} spiIrq_t;

typedef struct spiDma_s {
    unsigned txChan;
    unsigned rxChan;
    unsigned txMuxSrc;
    unsigned rxMuxSrc;
    unsigned vector;
    unsigned (*isr)(void);
} spiDma_t;

typedef struct spi_s {
    unsigned  addr;
    buffer_t  txBuff;
    buffer_t  rxBuff;
    spiPin_t  pin[NUM_PINS];
    spiIrq_t  irq;
    spiDma_t  dma;
    unsigned  pushr;
    unsigned  (*write)(void *spiPtr, const void *data, unsigned len);
    unsigned  (*read) (void *spiPtr, void *data,       unsigned len);
    volatile uint32_t * simBBPtr;
} spi_t;

/* Static Buffer Allocation */
#define BUFF_ADDR_BITS (8)                        /* Used for DMA Addressing */
#define BUFF_SIZE      (1<<(BUFF_ADDR_BITS))      /* Must be a power of 2    */
static uint8_t spiBuff[NUM_SPI_MODULES][BUFF_SIZE];

static spi_t spiCfg[NUM_SPI_MODULES] = {
    [SPI_MODULE_0] = {
        .addr     = SPI0_ADDR,
        .read     = spiReadPolled,
        .write    = spiWritePolled,
        .txBuff   = {
            .data = &spiBuff[SPI_MODULE_0][0],
            .head = &spiBuff[SPI_MODULE_0][0],
            .tail = &spiBuff[SPI_MODULE_0][0],},
        .rxBuff   = {
            .data = &spiBuff[SPI_MODULE_0][0],
            .head = &spiBuff[SPI_MODULE_0][0],
            .tail = &spiBuff[SPI_MODULE_0][0],},
        .irq  = {
            .vector   = ISR_SPI0,
            .isr      = spi0Isr, },
        .dma  = {
            .txChan   = DMA_CHAN0,
            .rxChan   = DMA_CHAN1,
            .txMuxSrc = DMAMUX_SOURCE_SPI0TX,
            .rxMuxSrc = DMAMUX_SOURCE_SPI0RX,
            .vector   = ISR_DMA0,
            .isr      = spi0DmaIsr, },
        .pin  = {
            [PIN_SIN]  = {
                .num   = 17,
                .port  = PORTA,
                .mux   = PORT_MUX_ALT2 },
            [PIN_SOUT] = {
                .num   = 16,
                .port  = PORTA,
                .mux   = PORT_MUX_ALT2 },
            [PIN_SCK]  = {
                .num   = 15,
                .port  = PORTA,
                .mux   = PORT_MUX_ALT2 },
            [PIN_PCS0] = {
                .num   = 14,
                .port  = PORTA,
                .mux   = PORT_MUX_ALT2 }},
        .simBBPtr      = SIM_SCGC6_SPI0_BB_PTR,
    },
    [SPI_MODULE_1] = {
        .addr = SPI1_ADDR,
        .read     = spiReadPolled,
        .write    = spiWritePolled,
        .txBuff   = {
            .data = &spiBuff[SPI_MODULE_1][0],
            .head = &spiBuff[SPI_MODULE_1][0],
            .tail = &spiBuff[SPI_MODULE_1][0],},
        .rxBuff   = {
            .data = &spiBuff[SPI_MODULE_1][0],
            .head = &spiBuff[SPI_MODULE_1][0],
            .tail = &spiBuff[SPI_MODULE_1][0],},
        .irq      = {
            .vector   = ISR_SPI1,
            .isr      = spi1Isr , },
        .dma  = {
            .txChan   = DMA_CHAN2,
            .rxChan   = DMA_CHAN3,
            .txMuxSrc = DMAMUX_SOURCE_SPI1TX,
            .rxMuxSrc = DMAMUX_SOURCE_SPI1RX,
            .vector   = ISR_DMA2,
            .isr      = spi1DmaIsr, },
        .pin  = {
            [PIN_SIN] = {
                .num  = 3,
                .port = PORTE,
                .mux   = PORT_MUX_ALT2 },
            [PIN_SOUT] = {
                .num  = 1,
                .port = PORTE,
                .mux   = PORT_MUX_ALT2 },
            [PIN_SCK] = {
                .num  = 2,
                .port = PORTE,
                .mux   = PORT_MUX_ALT2 },
            [PIN_PCS0] = {
                .num  = 4,
                .port = PORTE,
                .mux   = PORT_MUX_ALT2 }},
        .simBBPtr = SIM_SCGC6_SPI1_BB_PTR,
    },
    [SPI_MODULE_2] = {
        .addr = SPI2_ADDR,
        .read     = spiReadPolled,
        .write    = spiWritePolled,
        .txBuff   = {
            .data = &spiBuff[SPI_MODULE_2][0],
            .head = &spiBuff[SPI_MODULE_2][0],
            .tail = &spiBuff[SPI_MODULE_2][0],},
        .rxBuff   = {
            .data = &spiBuff[SPI_MODULE_2][0],
            .head = &spiBuff[SPI_MODULE_2][0],
            .tail = &spiBuff[SPI_MODULE_2][0],},
        .irq  = {
            .vector   = ISR_SPI2,
            .isr      = spi2Isr ,},
        .dma  = {
            .txChan   = DMA_CHAN4,
            .rxChan   = DMA_CHAN5,
            .txMuxSrc = DMAMUX_SOURCE_SPI2TX,
            .rxMuxSrc = DMAMUX_SOURCE_SPI2RX,
            .vector   = ISR_DMA4,
            .isr      = spi2DmaIsr, },
        .pin  = {
            [PIN_SIN] = {
                .num  = 14,
                .port = PORTD,
                .mux   = PORT_MUX_ALT2 },
            [PIN_SOUT] = {
                .num  = 13,
                .port = PORTD,
                .mux   = PORT_MUX_ALT2 },
            [PIN_SCK] = {
                .num  = 12,
                .port = PORTD,
                .mux   = PORT_MUX_ALT2 },
            [PIN_PCS0] = {
                .num  = 11,
                .port = PORTD,
                .mux   = PORT_MUX_ALT2 }},
        .simBBPtr = SIM_SCGC3_SPI2_BB_PTR,
    }
};

/*******************************************************************************/
static int spiOpen(devoptab_t *dot)
/*******************************************************************************/
{
    if (dot->priv) return FALSE; /* Device is already open */

    /* Pass pointer to module context to the private pointer */
    spi_t *spi = &spiCfg[dot->min];
    dot->priv = spi;

    /* Power up the Module */
    *spi->simBBPtr = 1;

    /* Install & configure interrupts */
    volatile uint32_t * TFFF_RE  = SPI_RSER_BB_PTR(spi->addr,SPI_RSER_TFFF_RE_BIT);
    volatile uint32_t * RFDF_RE  = SPI_RSER_BB_PTR(spi->addr,SPI_RSER_RFDF_RE_BIT);
    volatile uint32_t * TFFF_DIRS = SPI_RSER_BB_PTR(spi->addr,SPI_RSER_TFFF_DIRS_BIT);
    volatile uint32_t * RFDF_DIRS = SPI_RSER_BB_PTR(spi->addr,SPI_RSER_RFDF_DIRS_BIT);
    *TFFF_RE = 0;   /* Disable TxFifoFillFlag  interrupt request */
    *RFDF_RE = 0;   /* Disable RxFifoDrainFlag interrupt request */
    *TFFF_DIRS = 0; /* TFFF triggers interrupt request, not DMA request */
    *RFDF_DIRS = 0; /* RFDF triggers interrupt request, not DMA request */
    hwInstallISRHandler(spi->irq.vector, spi->irq.isr);
    hwInstallISRHandler(spi->dma.vector, spi->dma.isr);

    /* Write initial SPI configuration */
    SPI_MCR  (spi->addr) = SPI_MCR_MSTR_MSK     | SPI_MCR_FRZ_MSK;
    SPI_CTAR0(spi->addr) = SPI_CTAR_CSSCLK0_MSK | SPI_CTAR_CSSCLK1_MSK
                         | SPI_CTAR_CSSCLK2_MSK
                         | SPI_CTAR_ASC0_MSK    | SPI_CTAR_ASC1_MSK
                         | SPI_CTAR_ASC2_MSK
                         | SPI_CTAR_DT0_MSK     | SPI_CTAR_DT1_MSK
                         | SPI_CTAR_DT2_MSK ;
    SPI_CTAR1(spi->addr) = SPI_CTAR_CSSCLK0_MSK | SPI_CTAR_CSSCLK1_MSK
                         | SPI_CTAR_CSSCLK2_MSK
                         | SPI_CTAR_ASC0_MSK    | SPI_CTAR_ASC1_MSK
                         | SPI_CTAR_ASC2_MSK
                         | SPI_CTAR_DT0_MSK     | SPI_CTAR_DT1_MSK
                         | SPI_CTAR_DT2_MSK ;
    return TRUE;
}

/*******************************************************************************/
__inline__ static void buffEmpty(buffer_t *buff)
/*******************************************************************************/
{
    buff->head = buff->data;
    buff->tail = buff->data;
}

/*******************************************************************************/
__inline__ static void buffWrite(buffer_t * buff, uint8_t value)
/*******************************************************************************/
{
    *buff->head++ = value;
    if(buff->head >= (buff->data + BUFF_SIZE)) {
        buff->head = buff->data;
    }
}

/*******************************************************************************/
__inline__ static uint8_t buffRead(buffer_t * buff)
/*******************************************************************************/
{
    if (buff->head != buff->tail) {
        return *buff->tail++;
    }
    return 0;
}

/*******************************************************************************/
static unsigned spiWritePolled(void *spiPtr, const void *data, unsigned len)
/*******************************************************************************/
{
    spi_t * spi = (spi_t *) spiPtr;
    uint8_t *dataPtr = (uint8_t *) data;
    volatile uint32_t * TFFF  = SPI_SR_BB_PTR(spi->addr,SPI_SR_TFFF_BIT);
    volatile uint32_t * PUSHR = SPI_PUSHR_PTR(spi->addr);
    volatile uint32_t * SR = SPI_SR_PTR(spi->addr);
    uint32_t pushr = 0;
    unsigned i;

    for(i = 0; i < len; i++) {

        //if (i=len-1) set end of queue flag */

        pushr = (spi->pushr | (uint32_t)(*dataPtr++));

        while (*TFFF == 0) { }   /* Wait for Flag */
        *PUSHR = pushr;          /* Write Data */
        *TFFF  = 1;              /* Clear Flag */

        /* Wait for transfer to finish */
    }
    /* TODO: Poll on End of Queue flag? */
    while( (( *SR & SPI_SR_TXCTR_MSK)>>12) !=0 ){
            asm volatile("nop");
    }
    /* Clear End of Queu Flag */
    return len;
}

/*******************************************************************************/
static unsigned spiReadPolled(void *spiPtr, void *data, unsigned len)
/*******************************************************************************/
{
    spi_t * spi = (spi_t *) spiPtr;
    volatile uint32_t * POPR  = SPI_POPR_PTR(spi->addr);
    uint8_t *dataPtr = (uint8_t *) data;
    unsigned i;

    for(i = 0; i < len; i++) {

        /* Wait for the RXFIFO not to be empty */

        /* WTF!
         * The RXDF is NEVER cleared! Even when explicitly clearing it
         * like it says to in the documentation it remains 1 when the
         * FIFO is CLEARLY empty, RXCTR = 0, POPNXTPTR = 0 ! Again I
         * am forced to read the number of entrys in the fifo and wait
         * for it to become non-zero. */
        while ( ((SPI_SR(spi->addr) & SPI_SR_RXCTR_MSK)>>4) == 0) { }

        /* Take a entry of the RX FIFO */
        *dataPtr++ = (uint8_t) *POPR;
    }
    return len;
}

/*******************************************************************************/
static unsigned spiWriteReadPolled(spi_t *spi, spiWriteRead_t *wr)
/*******************************************************************************/
{
    volatile uint32_t * SR    = SPI_SR_PTR(spi->addr);
    volatile uint32_t * POPR  = SPI_POPR_PTR(spi->addr);
    volatile uint32_t * PUSHR = SPI_PUSHR_PTR(spi->addr);
    uint8_t *inPtr = wr->in;
    uint8_t *outPtr = wr->out;
    unsigned lenIn = 0;
    unsigned lenOut = 0;

    while( lenIn < wr->len) {

        /* Write to the TX FIFO if there is room */
        if( (( *SR & SPI_SR_TXCTR_MSK)>>12) < 4 ) {
            if (lenOut < wr->len) {
                *PUSHR = (spi->pushr | (uint32_t)(*outPtr++));
                lenOut++;
            }
        }

        /* Read from RX FIFO if there is anything*/
        if ( (( *SR & SPI_SR_RXCTR_MSK)>>4) != 0) {
            *inPtr++ = (uint8_t) *POPR;
            lenIn++;
        }
    }
    return lenIn;
}


/*******************************************************************************/
static unsigned spiReadInterrupt(void *spiPtr, void *data, unsigned len)
/*******************************************************************************/
{
    spi_t * spi = (spi_t *) spiPtr;
    buffer_t *buff = &spi->rxBuff;
    uint8_t *dataPtr = (uint8_t *) data;
    unsigned i;

    for (i = 0; i < len && buff->head != buff->tail ; i++) {
        *dataPtr++ = (uint8_t) buffRead(buff);
    }
    return i;
}

/*******************************************************************************/
static unsigned spiWriteInterrupt(void *spiPtr, const void *data, unsigned len)
/*******************************************************************************/
{
    spi_t * spi = (spi_t *) spiPtr;
    volatile uint32_t * TFFF_RE = SPI_RSER_BB_PTR(spi->addr,SPI_RSER_TFFF_RE_BIT);
    buffer_t *buff = &spi->txBuff;
    uint8_t *dataPtr = (uint8_t *) data;
    unsigned i;

    for (i = 0; i < len && buff->head != (buff->tail-1); i++) {
        buffWrite(buff, (uint8_t) *dataPtr++);
    }
    *TFFF_RE = 1;   /* There is data to write, enable interrupt */
    return i;
}

/*******************************************************************************/
static unsigned spiRxFifoDrainHandler(spi_t *spi)
/*******************************************************************************/
{
    volatile uint32_t * SR   = SPI_SR_PTR(spi->addr);
    volatile uint32_t * POPR = SPI_POPR_PTR(spi->addr);
    buffer_t *buff = &spi->rxBuff;
    unsigned len = ((*SR & SPI_SR_RXCTR_MSK)>>SPI_SR_RXCTR0_BIT);
    unsigned i;

    for(i = 0; i < len ; i++) {
        buffWrite(buff, (uint8_t) *POPR);
    }
    return i;
}

/*******************************************************************************/
static unsigned spiTxFifoFillHandler(spi_t *spi)
/*******************************************************************************/
{
    volatile uint32_t * SR   = SPI_SR_PTR(spi->addr);
    volatile uint32_t * PUSHR = SPI_PUSHR_PTR(spi->addr);
    volatile uint32_t * TFFF_RE = SPI_RSER_BB_PTR(spi->addr,SPI_RSER_TFFF_RE_BIT);
    buffer_t *buff = &spi->txBuff;
    unsigned len = (SPI_FIFO_SIZE-((*SR & SPI_SR_TXCTR_MSK)>>SPI_SR_TXCTR0_BIT));
    unsigned i;

    for(i = 0; i < len && (buff->head != buff->tail); i++) {
        *PUSHR = (uint32_t)(spi->pushr | (uint32_t) buffRead(buff));
    }
    if (buff->head == buff->tail) {
        *TFFF_RE = 0; /* No more data to write, turn off interrupt */
    }
    return i;
}

/*******************************************************************************/
static unsigned spiReadDma(void *spiPtr, void *data, unsigned len)
/*******************************************************************************/
{
    spi_t * spi = (spi_t *) spiPtr;
    unsigned chan = spi->dma.rxChan;
    volatile uint32_t * ACTIVE = DMA_CSR_BB_PTR(chan,DMA_CSR_ACTIVE_BIT);
    buffer_t *buff = &spi->rxBuff;
    uint8_t *dataPtr = (uint8_t *) data;
    unsigned i;

    DMA_CERQ = (uint8_t)chan;       /* Disable dma requests for the channel */

 //   while (!(*ACTIVE)) { }           /* Wait until DMA channel is not active */

    /* Read Data */
    buff->head = (uint8_t *) DMA_DADDR(chan);
    for (i = 0; i < len && buff->head != buff->tail ; i++) {
        *dataPtr++ = (uint8_t) buffRead(buff);
    }
    DMA_SERQ = (uint8_t)chan;                  /* Enable DMA channel again */
    return i;
}

/*******************************************************************************/
static unsigned spiWriteDma(void *spiPtr, const void *data, unsigned len)
/*******************************************************************************/
{
    spi_t * spi = (spi_t *) spiPtr;
    unsigned chan = spi->dma.txChan;
    volatile uint32_t * DONE   = DMA_CSR_BB_PTR(chan,DMA_CSR_DONE_BIT);
    volatile uint32_t * ACTIVE = DMA_CSR_BB_PTR(chan,DMA_CSR_ACTIVE_BIT);
    volatile uint32_t * INTMAJ = DMA_CSR_BB_PTR(chan,DMA_CSR_INTMAJ_BIT);
    volatile uint32_t * TFFF_RE = SPI_RSER_BB_PTR(spi->addr,SPI_RSER_TFFF_RE_BIT);
    buffer_t *buff = &spi->txBuff;
    uint8_t *dataPtr = (uint8_t *) data;
    unsigned i;

    DMA_CERQ = (uint8_t)chan;        /* Disable dma requests for the channel */

//    while (!(*ACTIVE)) { }           /* Wait until DMA channel is not active */

    buff->tail = (uint8_t *) DMA_SADDR(chan);
    for( i = 0; i < len && buff->head != (buff->tail-1); i++) {
        buffWrite(buff,*dataPtr++);
    }

    /* DONE? */
    DMA_BITER(chan) = (uint16_t) abs(DMA_CITER(chan)-DMA_BITER(chan)) + i;
    DMA_CITER(chan) = DMA_BITER(chan); /* CITER must equal BITER when starting */

    *INTMAJ = 1;        /* Request an interrupt when a major loop completes */
    DMA_SERQ = (uint8_t)chan;                   /* Enable DMA channel again */
    *TFFF_RE = 1;                /* Disable Tx Fifo Fill  Interrupt Request */
    return i;
}

/*******************************************************************************/
static void spiDmaRxConf(spi_t *spi)
/*******************************************************************************/
{
    unsigned chan = spi->dma.rxChan;
    unsigned muxSrc = spi->dma.rxMuxSrc;
    uint32_t buffAddr = (uint32_t) spi->rxBuff.data;
    uint32_t sourceAddr = (uint32_t) SPI_POPR_ADDR(spi->addr);

    /* Power both the DMA & DMA MUX */
    SIM_SCGC6_DMAMUX_BB = 1;
    SIM_SCGC7_DMA_BB    = 1;

    /* Disable MUX While Configuring DMA Channel */
    DMAMUX_CHCFG_BB(chan, DMAMUX_ENBL_BIT) = 0;
    DMAMUX_CHCFG_BB(chan, DMAMUX_TRIG_BIT) = 0;

    /* Source */
    DMA_SADDR(chan)    = sourceAddr;
    DMA_SOFF(chan)     = 0;
    DMA_SLAST(chan)    = 0;

    /* Destination */
    DMA_DADDR(chan)    = buffAddr;
    DMA_DOFF(chan)     = 1;     /* Add 1 to dest addr every req */
    DMA_DLASTSGA(chan) = 0;

    /* Transfer Options */
    DMA_BITER(chan)    = 1; /* Current Loop Cnt, Must be at least 1 */
    DMA_CITER(chan)    = 1; /* Beginning Loop Cnt, Must start equal to BITER */
    DMA_NBYTES(chan)   = 1; /* Btyes to tx, Must be at least 1 */
    DMA_ATTR(chan)     = 0x0040; /* 8Bit src to 8bit dest, 8mod dest bits */
                                 /* TODO: Use proper masking for ATTR */

    /* Configure the dma mux's source & enbale dma requests on the channel */
    DMAMUX_CHCFG(chan) = (uint8_t) (muxSrc | DMAMUX_ENBL_MSK);
    DMA_SERQ = (uint8_t)chan;
}

/*******************************************************************************/
static void spiDmaTxConf(spi_t *spi)
/*******************************************************************************/
{
    unsigned chan = spi->dma.txChan;
    unsigned muxSrc = spi->dma.txMuxSrc;
    uint32_t buffAddr = (uint32_t) spi->txBuff.data;
    uint32_t destAddr = (uint32_t) SPI_PUSHR_ADDR(spi->addr);

    /* Power both the DMA & DMA MUX */
    SIM_SCGC6_DMAMUX_BB = 1;
    SIM_SCGC7_DMA_BB    = 1;

    /* Disable MUX While Configuring DMA Channel */
    DMAMUX_CHCFG_BB(chan,DMAMUX_ENBL_BIT) = 0;
    DMAMUX_CHCFG_BB(chan,DMAMUX_TRIG_BIT) = 0;

    /* Source */
    DMA_SADDR(chan)    = buffAddr;
    DMA_SOFF(chan)     = 1;     /* Add 1 to our source addr every req */
    DMA_SLAST(chan)    = 0;

    /* Destination */
    DMA_DADDR(chan)    = destAddr;
    DMA_DOFF(chan)     = 0;
    DMA_DLASTSGA(chan) = 0;

    /* Transfer Options */
    DMA_BITER(chan)    = 1; /* Current Loop Cnt, Must be at least 1 */
    DMA_CITER(chan)    = 1; /* Beginning Loop Cnt, Must start equal to BITER */
    DMA_NBYTES(chan)   = 1; /* Btyes to tx, Must be at least 1 */
    DMA_ATTR(chan)     = 0x4000; /* 8Bit src to 8bit dest, 8mod src bits */
                                 /* TODO: Use proper masking for ATTR */

    /* Configure the dma mux's source & disable dma requests on the channel */
    DMAMUX_CHCFG(chan) = (uint8_t) (muxSrc | DMAMUX_ENBL_MSK);
    DMA_CERQ = (uint8_t)chan;
}

/*******************************************************************************/
static unsigned spiDmaIsr(spi_t *spi)
/*******************************************************************************/
{
    unsigned chan = spi->dma.txChan;
    volatile uint32_t * TFFF  = SPI_SR_BB_PTR(spi->addr,SPI_SR_TFFF_BIT);
    volatile uint32_t * DMA_INT_TX = DMA_INT_BB_PTR(chan);
    volatile uint32_t * TFFF_RE = SPI_RSER_BB_PTR(spi->addr,SPI_RSER_TFFF_RE_BIT);

    *TFFF_RE           = 0;                 /* Disable Fifo Fill DMA requests */
    DMA_CERQ           = (uint8_t)chan;       /* Disable DMA requests on chan */
    DMA_BITER(chan)    = 1; /* Current Loop Cnt, Must be at least 1 */
    DMA_CITER(chan)    = 1; /* Beginning Loop Cnt, Must start equal to BITER */
    DMA_NBYTES(chan)   = 1; /* Btyes to tx, Must be at least 1 */
    *DMA_INT_TX        = 1; /* Clear Flag */
    *TFFF              = 1; /* Clear Flag */
    return;
}

/*******************************************************************************/
static unsigned spiIsr(spi_t *spi) {
/*******************************************************************************/

    volatile uint32_t * TFFF  = SPI_SR_BB_PTR(spi->addr,SPI_SR_TFFF_BIT);
    volatile uint32_t * RFDF  = SPI_SR_BB_PTR(spi->addr,SPI_SR_RFDF_BIT);

    /* TODO: Fix up, use else if */
    if (*TFFF) {
        spiTxFifoFillHandler(spi);
        *TFFF = 1; /* Clear Flag */
    }
    if (*RFDF) {
        spiRxFifoDrainHandler(spi);
        *RFDF = 1; /* Clear Flag */
    }
    return TRUE;
}


/*******************************************************************************/
static unsigned spi0Isr(void) {
    return spiIsr(&spiCfg[SPI_MODULE_0]);
}
static unsigned spi1Isr(void) {
    return spiIsr(&spiCfg[SPI_MODULE_1]);
}
static unsigned spi2Isr(void) {
    return spiIsr(&spiCfg[SPI_MODULE_2]);
}
static unsigned spi0DmaIsr(void) {
    return spiDmaIsr(&spiCfg[SPI_MODULE_0]);
}
static unsigned spi1DmaIsr(void) {
    return spiDmaIsr(&spiCfg[SPI_MODULE_1]);
}
static unsigned spi2DmaIsr(void) {
    return spiDmaIsr(&spiCfg[SPI_MODULE_2]);
}
/*******************************************************************************/

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
static int spi_open_r (void *reent, devoptab_t *dot, int mode, int flags )
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
static int spi_ioctl(devoptab_t *dot, int cmd,  int flags)
/* TODO: return errors if flags or cmd is bad */
{
    if (!dot || !dot->priv) return FALSE;

    spi_t *spi = (spi_t *) dot->priv;
    unsigned reg = 0;
    unsigned i;

    switch (cmd) {
    case IO_IOCTL_SPI_SET_PORT_PCRS: {

        /* TODO: Fix this ... */
        switch (spi->addr) {
        case SPI0_ADDR:
            SIM_SCGC5 |= SIM_SCGC5_PORTA_ENABLE;
        break;
        case SPI1_ADDR:
            SIM_SCGC5 |= SIM_SCGC5_PORTE_ENABLE;
        break;
        case SPI2_ADDR:
            SIM_SCGC5 |= SIM_SCGC5_PORTD_ENABLE;
        break;
        default:
            assert(0);
            return FALSE;
        break;
        }

        spiPin_t *pin = spi->pin;
        for (i = 0; i < NUM_PINS; i++) {
            PORT_PCR(pin[i].port, pin[i].num) = pin[i].mux;
        }
      }
    break;
    case IO_IOCTL_SPI_SET_BAUD: {
        if (flags < NUM_SPI_BAUDRATES) {
            volatile uint32_t * CTAR0 = SPI_CTAR0_PTR(spi->addr);
            reg  = *CTAR0 & ~(SPI_CTAR_BR_MSK);
            reg |= flags;
            *CTAR0 = reg;
        }
      }
    break;
    case IO_IOCTL_SPI_SET_SCLK_MODE: {
        if (flags < MAX_SPI_SCLK_MODES) {
            volatile uint32_t * CTAR0 = SPI_CTAR0_PTR(spi->addr);
            reg  = *CTAR0 & ~(SPI_CTAR_CPHA_MSK | SPI_CTAR_CPOL_MSK);
            reg |= flags << 25;
            *CTAR0 = reg;
        }
      }
    break;
    case IO_IOCTL_SPI_SET_FMSZ: {
        if (flags >= SPI_FMSZ_MIN && flags <= SPI_FMSZ_MAX) {
            volatile uint32_t * CTAR0 = SPI_CTAR0_PTR(spi->addr);
            reg  = *CTAR0 & ~(SPI_CTAR_FMSZ_MSK);
            reg |= (flags-1) << 27;
            *CTAR0 = reg;
        }
      }
    break;
    case IO_IOCTL_SPI_SET_OPTS: {

        if (flags & SPI_OPTS_MASTER)
            SPI_MCR_BB(spi->addr,SPI_MCR_MSTR_BIT) = 1;

        if (flags & SPI_OPTS_LSB_FIRST)
            SPI_CTAR0_BB(spi->addr,SPI_CTAR_LSBFE_BIT) = 1;

        if (flags & SPI_OPTS_CONT_SCK_EN)
            SPI_MCR_BB(spi->addr,SPI_MCR_CONT_SCKE_BIT) = 1;

        if (flags & SPI_OPTS_TX_FIFO_DSBL)
            SPI_MCR_BB(spi->addr,SPI_MCR_DIS_TXF_BIT) = 1;

        if (flags & SPI_OPTS_RX_FIFO_DSBL)
            SPI_MCR_BB(spi->addr,SPI_MCR_DIS_RXF_BIT) = 1;

        if (flags & SPI_OPTS_RX_FIFO_OVR_EN)
            SPI_MCR_BB(spi->addr,SPI_MCR_ROOE_BIT) = 1;
      }
    break;
    case IO_IOCTL_SPI_SET_CS: {
        if( flags < 0x3F ) {
            spi->pushr &= ~(0x3F << 16);
            spi->pushr |= flags << 16;
        }
      }
    break;
    case IO_IOCTL_SPI_SET_CS_INACT_STATE: {
        if( flags < 0x3F ) {
            volatile uint32_t * MCR = SPI_MCR_PTR(spi->addr);
            reg  = *MCR & ~(0x3F << 16);
            reg |= flags << 16;
            *MCR = reg;
        }
      }
    break;
    case IO_IOCTL_SPI_SET_METHOD: {
        volatile uint32_t * TFFF_RE = SPI_RSER_BB_PTR(spi->addr,SPI_RSER_TFFF_RE_BIT);
        volatile uint32_t * RFDF_RE = SPI_RSER_BB_PTR(spi->addr,SPI_RSER_RFDF_RE_BIT);
        volatile uint32_t * TFFF_DIRS = SPI_RSER_BB_PTR(spi->addr,SPI_RSER_TFFF_DIRS_BIT);
        volatile uint32_t * RFDF_DIRS = SPI_RSER_BB_PTR(spi->addr,SPI_RSER_RFDF_DIRS_BIT);
        switch (flags) {
        default:
        case SPI_METHOD_POLLED:
            hwInterruptsDisable();
            buffEmpty(&spi->txBuff);
            buffEmpty(&spi->rxBuff);
            spi->read = spiReadPolled;
            spi->write = spiWritePolled;
            *TFFF_RE   = 0; /* Disable Tx Fifo Fill  Interrupt Request */
            *RFDF_RE   = 0; /* Disable Rx Fifo Drain Interrupt Request */
            hwInterruptsEnable();
        break;
        case SPI_METHOD_INTERRUPT:
            hwInterruptsDisable();
            buffEmpty(&spi->txBuff);
            buffEmpty(&spi->rxBuff);
            spi->read = spiReadInterrupt;
            spi->write = spiWriteInterrupt;
            *TFFF_DIRS = 0; /* TFFF selects an interrupt not dma */
            *RFDF_DIRS = 0; /* RFDF selects an interrupt not dma */
            *TFFF_RE   = 0; /* Disable Tx Fifo Fill  Interrupt Request */
            *RFDF_RE   = 1; /* Enable Rx Fifo Drain Interrupt Request */
            hwInterruptsEnable();
        break;
        case SPI_METHOD_DMA:
            hwInterruptsDisable();
            buffEmpty(&spi->txBuff);
            buffEmpty(&spi->rxBuff);
            spi->read = spiReadDma;
            spi->write = spiWriteDma;
            spiDmaTxConf(spi);
            spiDmaRxConf(spi);
            *TFFF_DIRS = 1; /* TFFF causes dma request */
            *RFDF_DIRS = 1; /* RFDF causes dma request */
            *TFFF_RE   = 0; /* Disable Tx Fifo Fill  Interrupt Request */
            *RFDF_RE   = 1; /* Enable Rx Fifo Drain Interrupt Request */
            hwInterruptsEnable();
        break;
        }
      }
    break;
    case IO_IOCTL_SPI_FLUSH_RX_FIFO: {
        volatile uint32_t * SR = SPI_SR_PTR(spi->addr);
        volatile uint32_t * POPR = SPI_POPR_PTR(spi->addr);
        volatile uint32_t * CLR_RXF = SPI_MCR_BB_PTR(spi->addr,SPI_MCR_CLR_RXF_BIT);

        /* Flush the RX FIFO, but pop manually. Cant get this to work normally */
        *CLR_RXF = 1;
        while ( ((*SR & SPI_SR_RXCTR_MSK)>>4) != 0) {
            *POPR; /* Ridiculus */
        }
      }
    break;
    case IO_IOCTL_SPI_FLUSH_TX_FIFO: {
        volatile uint32_t * SR = SPI_SR_PTR(spi->addr);
        volatile uint32_t * CLR_TXF = SPI_MCR_BB_PTR(spi->addr,SPI_MCR_CLR_TXF_BIT);

        *CLR_TXF = 1;
        while ( ((*SR & SPI_SR_TXCTR_MSK)>>4) != 0) { }
      }
    break;
    case IO_IOCTL_SPI_FLUSH_RX_BUFF: {
        hwInterruptsDisable();
        buffEmpty(&spi->rxBuff);
        hwInterruptsEnable();
      }
    break;
    case IO_IOCTL_SPI_FLUSH_TX_BUFF: {
        hwInterruptsDisable();
        buffEmpty(&spi->rxBuff);
        hwInterruptsEnable();
      }
    break;
    case IO_IOCTL_SPI_WRITE_READ: {
        if (!flags)
            return FALSE;
        return spiWriteReadPolled(spi,(spiWriteRead_t *)flags);
      }
    break;
    default: {
        return FALSE;
      }
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
static int spi_close_r (void *reent, devoptab_t *dot )
{
    spi_t *spi = dot->priv;

    if (spi) {
        /* Disable the SIMSCGC for the spi module being used*/
        *spi->simBBPtr = 0;
        /* Unhook the private spi structure and free it */
        dot->priv = NULL;
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
static long spi_write_r (void *reent, devoptab_t *dot, const void *buf, int len )
{
    if (!dot || !dot->priv) return FALSE;
    spi_t * spi = (spi_t *) dot->priv;
    return spi->write(spi, buf, len);
}

/*******************************************************************************/
/* spi_read_r                                                                  */
/*******************************************************************************/
/* Jobs of the 'read' syscall:
 *      Read data from the device
 *      Return the number of bytes read
 *******************************************************************************/
static long spi_read_r (void *reent, devoptab_t *dot, void *buf, int len )
{
    if (!dot || !dot->priv) return FALSE;
    spi_t * spi = (spi_t *) dot->priv;
    return spi->read(spi, buf, len);
}

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

