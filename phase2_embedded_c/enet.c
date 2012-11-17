/*******************************************************************************
*
* enet.c
*
* Steve Holford
*
* Low level driver for the Kinetis ENET (Ethernet) module.
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
#include "globalDefs.h"
#include "kinetis.h"
#include "hardware.h"
#include "eth_frame.h"


static unsigned enetWritePolled(void *enetPtr, const void *data, unsigned len);
static unsigned enetReadPolled(void *enetPtr, void *data, unsigned len);
static unsigned enetWriteInterrupt(void *enetPtr, const void *data, unsigned len);
static unsigned enetReadInterrupt(void *enetPtr, void *data, unsigned len);

static unsigned enet0Mac1588Isr(void); 
static unsigned enet0MacTxIsr(void);
static unsigned enet0MacRxIsr(void);
static unsigned enet0MacErrorIsr(void);
    
/*****************************************************************************/
/*
 * Enums and structure to hold per-channel ethernet configuration
 *
 * Note: You need a real legal MAC address if you are doing anything
 * other than testing.
 *
 */

/*****************************************************************************/
/* Physical Kinetis defines
 */
typedef enum enetPins_e {
    PIN_ENET_MDC,
    PIN_ENET_MDIO,
    PIN_ENET_RXER,
    PIN_ENET_RXDV,
    PIN_ENET_RXD0,
    PIN_ENET_RXD1,
    PIN_ENET_TXEN,
    PIN_ENET_TXD0,
    PIN_ENET_TXD1,
    NUM_PINS,
} enetPins_t;

typedef struct enetPin_s {
    unsigned num;
    unsigned port;
    unsigned mux;
} enetPin_t;

typedef struct enetIrq_s {
    unsigned vector;
    unsigned (*isr)(void);
} enetIrq_t;

typedef enum enetIrqHandler_e {
    HND_ENET_MAC_1588_TIMER,
    HND_ENET_MAC_TX,
    HND_ENET_MAC_RX,
    HND_ENET_MAC_ERROR,
    NUM_HANDLERS
} enetIrqHandler_t;

/*****************************************************************************/
/* 
 * Buffers and buffer descriptors
 */

#define ENET_RX_BUFFER_SIZE 256     /* Must be a multiple of 16 */
#define ENET_TX_BUFFER_SIZE 256

#define ENET_NUM_RXBDS 16           /* defines number of BD,s and buffers */
#define ENET_NUM_TXBDS 16

typedef uint8_t enet_rxbuf_t [ENET_RX_BUFFER_SIZE];
typedef uint8_t enet_txbuf_t [ENET_TX_BUFFER_SIZE];

/* Buffer descriptors  MUST BE 16-byte boundary aligned */
static enet_descr_t rxbds[ENET_NUM_RXBDS] __attribute__ ((aligned (16)));
static enet_descr_t txbds[ENET_NUM_TXBDS] __attribute__ ((aligned (16)));

/* Actual buffers MUST BE 16-byte boundary aligned */
static enet_rxbuf_t rxbuffers[ENET_NUM_RXBDS] __attribute__ ((aligned (16)));
static enet_txbuf_t txbuffers[ENET_NUM_TXBDS] __attribute__ ((aligned (16)));

/* Next BD indicies for BD ring */ 
static int next_rxbd;
static int next_txbd;

/*****************************************************************************/
/* 
 * Array of ethernet module control structures, 1 per channel
 * Everything is controlled and accessed through this array
 *
 */

typedef enum enetModule_e {
    ENET_MODULE_0,
    NUM_ENET_MODULES,
} enetModule_t;

typedef struct enet_s {
    enet_map_ptr addr;
    enet_cfg_t config;
    enet_status_t status;
    int phy_auto_req_data;
    enet_descr_t last_rx_packet;
    enetPin_t  pin[NUM_PINS];
    enetIrq_t  irq[NUM_HANDLERS];
    unsigned  (*write)(void *enetPtr, const void *data, unsigned len);
    unsigned  (*read) (void *enetPtr, void *data,       unsigned len);
    volatile uint32_t * simBBPtr;
} enet_t;

static enet_t enetModules[NUM_ENET_MODULES] = {
    [ENET_MODULE_0] = {
        .addr     = ((enet_map_ptr) ENET_ADDR),
        .config   = {
            .phy_addr = 1,
            .autoneg = ENET_AUTONEG_ON,
            .speed = ENET_100BASET,
            .duplex = ENET_DUPLEX_FULL,
            .loopback = ENET_LOOPBACK_OFF,
            .prom = ENET_PROM_OFF,
            .mac_addr = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
        .status = {
            .on_off = ENET_OFF,
            .link = ENET_LINK_DOWN,
            .speed = ENET_100BASET,
            .duplex = ENET_DUPLEX_FULL },
        .phy_auto_req_data = 0,
        .last_rx_packet = {
            .status = 0,
            .length = 0,
            .buf_addr = NULL },
        .irq  = {
            [HND_ENET_MAC_1588_TIMER] = {
                .vector   = ISR_ENET_MAC_1588_TIMER,
                .isr      = enet0Mac1588Isr, 
                },
            [HND_ENET_MAC_TX] = {
                .vector   = ISR_ENET_MAC_TX,
                .isr      = enet0MacTxIsr, 
                },
            [HND_ENET_MAC_RX] = {
                .vector   = ISR_ENET_MAC_RX,
                .isr      = enet0MacRxIsr, 
                },
            [HND_ENET_MAC_ERROR] = {
                .vector   = ISR_ENET_MAC_ERROR,
                .isr      = enet0MacErrorIsr, 
                } },
        .pin  = {
            [PIN_ENET_MDC]  = {
                .num   = 1,
                .port  = PORTB,
                .mux   = PORT_MUX_ALT4 },
            [PIN_ENET_MDIO] = {
                .num   = 0,
                .port  = PORTB,
                .mux   = PORT_MUX_ALT4 },
            [PIN_ENET_RXER]  = {
                .num   = 5,
                .port  = PORTA,
                .mux   = PORT_MUX_ALT4 },
            [PIN_ENET_RXDV] = {
                .num   = 14,
                .port  = PORTA,
                .mux   = PORT_MUX_ALT4 },
            [PIN_ENET_RXD0] = {
                .num   = 13,
                .port  = PORTA,
                .mux   = PORT_MUX_ALT4 },
            [PIN_ENET_RXD1] = {
                .num   = 12,
                .port  = PORTA,
                .mux   = PORT_MUX_ALT4 },
            [PIN_ENET_TXEN] = {
                .num   = 15,
                .port  = PORTA,
                .mux   = PORT_MUX_ALT4 },
            [PIN_ENET_TXD0] = {
                .num   = 16,
                .port  = PORTA,
                .mux   = PORT_MUX_ALT4 },
            [PIN_ENET_TXD1] = {
                .num   = 17,
                .port  = PORTA,
                .mux   = PORT_MUX_ALT4 } },
        .read     = enetReadPolled,
        .write    = enetWritePolled,
        .simBBPtr      = SIM_SCGC2_ENET_BB_PTR
    }
};

/********************************************************************/
int eth_hwaddr_match (uint8_t* a1, uint8_t* a2)
{   
    int i;

    for (i = 0; i < ETH_ADDR_LEN; i++) {
        if (a1[i] != a2[i]) return FALSE;
    }
    return TRUE;
}

int eth_hwaddr_zero (uint8_t* a1)
{   
    int i;

    for (i = 0; i < ETH_ADDR_LEN; i++) {
        if (a1[i] != 0x00) return FALSE;
    }
    return TRUE;
}

int eth_hwaddr_eff (uint8_t* a1)
{   
    int i;

    for (i = 0; i < ETH_ADDR_LEN; i++) {
        if (a1[i] != 0xFF) return FALSE;
    }
    return TRUE;
}

void eth_hwaddr_copy (uint8_t* dest, uint8_t* src)
{   
    int i;

    for (i = 0; i < ETH_ADDR_LEN; i++) {
        dest[i] = src[i];
    }
}

/********************************************************************/
static void phyInit(enet_t *enet)
{
    enet_map_ptr fec = enet->addr;
    uint32_t clockHz = clockGetFreq(CLOCK_SYSTEM);
    uint8_t mii_speed = (uint8_t)((clockHz/5000000) - 1);

    fec->mscr = ENET_MSCR_MII_SPEED(mii_speed);
}

/********************************************************************/
static int phyWrite(enet_t *enet, int reg_addr, int data)
{
    enet_map_ptr fec = enet->addr;
    int timeout;

    /* Clear MII interrupt bit */
    fec->eir = ENET_EIR_MII_MASK;

    /* Request phy write */
    fec->mmfr = ENET_MMFR_ST(0x01) | ENET_MMFR_OP(0x01)
        | ENET_MMFR_PA(enet->config.phy_addr) 
        | ENET_MMFR_RA(reg_addr) | ENET_MMFR_TA(0x02)
        | ENET_MMFR_DATA(data);

    /* Poll for MII interrupt */
    for (timeout = 0; timeout < MII_TIMEOUT; timeout++) {
        if (fec->eir & ENET_EIR_MII_MASK) {
            fec->eir = ENET_EIR_MII_MASK;
            return TRUE;
        }
    }

    return FALSE;
}

/********************************************************************/
static int phyRead(enet_t *enet, int reg_addr, int *data)
{
    enet_map_ptr fec = enet->addr;
    int timeout;

    /* Clear MII interrupt bit */
    fec->eir = ENET_EIR_MII_MASK;

    /* Request phy read */
    fec->mmfr = ENET_MMFR_ST(0x01) | ENET_MMFR_OP(0x2)
        | ENET_MMFR_PA(enet->config.phy_addr)
        | ENET_MMFR_RA(reg_addr) | ENET_MMFR_TA(0x02);

    /* Poll for the MII interrupt */
    for (timeout = 0; timeout < MII_TIMEOUT; timeout++) {
        if (fec->eir & ENET_EIR_MII_MASK) {
            fec->eir = ENET_EIR_MII_MASK;
            *data = fec->mmfr & 0x0000FFFF;
            return TRUE;
        }
    }
    
    return FALSE;
}

/********************************************************************/
static int phyReset(enet_t *enet)
{
    int timeout; 
    int phy_data;

    /* Reset the PHY */
    if (!phyWrite(enet, PHY_BMCR, PHY_BMCR_RESET)) 
        return FALSE;
    for (timeout = 0; timeout < MII_LINK_TIMEOUT; ++timeout) {
        if ((phyRead(enet, PHY_BMCR, &phy_data)) && !(phy_data & PHY_BMCR_RESET))
            return TRUE;
    }
    return FALSE;
}

/********************************************************************/
static int phyAutoneg(enet_t *enet)
{
    int timeout;
    int phy_data;
    enet_cfg_t *config = &(enet->config);
    enet_status_t *status = &(enet->status);

    status->link = ENET_LINK_DOWN;
    status->speed = config->speed;
    status->duplex = config->duplex;
    config->loopback = ENET_LOOPBACK_OFF;
    phyReset(enet);

    /* Create Auto-Negotiation Advertisement Data */
    if (config->speed == ENET_10BASET) {
        phy_data = (config->duplex == ENET_DUPLEX_FULL) 
            ? PHY_ANAR_10BT_FDX | PHY_ANAR_10BT 
            : PHY_ANAR_10BT;
    } else {
        phy_data = (config->duplex == ENET_DUPLEX_FULL)  
            ? PHY_ANAR_100BTX_FDX   | 
              PHY_ANAR_100BTX       | 
              PHY_ANAR_10BT_FDX     | 
              PHY_ANAR_10BT
            : PHY_ANAR_10BT_FDX     | 
              PHY_ANAR_10BT;
    }
    
    enet->phy_auto_req_data = phy_data; /* Save for later */
    /* Set the autonegotiate advertisement register */
    if (!phyWrite(enet, PHY_ANAR, phy_data))
        return FALSE;
        
    /* Enable Auto-Negotiation */
    if (!phyWrite(enet, PHY_BMCR, PHY_BMCR_AN_ENABLE | PHY_BMCR_AN_RESTART))
        return FALSE;

    /* Wait for auto-negotiation to complete */
    for (timeout = 0; timeout < MII_LINK_TIMEOUT; ++timeout) {
        if (!phyRead(enet, PHY_BMSR, &phy_data))
            return FALSE;
        if (phy_data & PHY_BMSR_AN_COMPLETE)
            break;
    }
    /* Read the BMSR one last time */
    if (!phyRead(enet, PHY_BMSR, &phy_data))
        return FALSE;
    if (timeout == MII_LINK_TIMEOUT || !(phy_data & PHY_BMSR_LINK))
        return FALSE;

    status->link = ENET_LINK_UP;
    return TRUE;
}

/********************************************************************/
static int phyManual(enet_t *enet)
{
    int timeout; 
    int phy_data = 0;
    enet_cfg_t *config = &(enet->config);
    enet_status_t *status = &(enet->status);

    status->link = ENET_LINK_DOWN;
    status->speed = config->speed;
    status->duplex = config->duplex;
    enet->phy_auto_req_data = 0;
    phyReset(enet);
  
    if (config->loopback == ENET_LOOPBACK_INT)
        phy_data |= PHY_BMCR_LOOP;
    if (config->duplex == ENET_DUPLEX_FULL)
        phy_data |= PHY_BMCR_FDX;
    if (config->speed == ENET_100BASET)
        phy_data |= PHY_BMCR_SPEED;

    if (!phyWrite(enet, PHY_BMCR, phy_data))
        return FALSE;
    
    /* Wait for link */
    for (timeout = 0; timeout < MII_LINK_TIMEOUT; ++timeout) {
        if (!phyRead(enet, PHY_BMSR, &phy_data))
            return FALSE;
        if (phy_data & PHY_BMSR_LINK)
            break;
    }
    if (timeout == MII_LINK_TIMEOUT || !(phy_data & PHY_BMSR_LINK))
        return FALSE;

    status->link = ENET_LINK_UP;
    return TRUE;
}

/********************************************************************/
static int phyGetAutoResults(enet_t *enet)
{
    int timeout;
    int phy_data = 0;
    enet_status_t *status = &(enet->status);

    for (timeout = 0; timeout < MII_TIMEOUT; ++timeout) {
        if (!phyRead(enet, PHY_ANLPAR, &phy_data))
            return FALSE;
        else
            break;
    }
    if (timeout == MII_TIMEOUT)
        return FALSE;
    
    phy_data &= enet->phy_auto_req_data;
    if (phy_data & PHY_ANLPAR_100BT4     ||
        phy_data & PHY_ANLPAR_100BTX_FDX ||
        phy_data & PHY_ANLPAR_100BTX)
        status->speed = ENET_100BASET;
    else
        status->speed = ENET_10BASET;

    if (phy_data & PHY_ANLPAR_100BTX_FDX ||
        phy_data & PHY_ANLPAR_10BTX_FDX)
        status->duplex = ENET_DUPLEX_FULL;
    else
        status->duplex = ENET_DUPLEX_HALF;

    return TRUE;
}

/********************************************************************/
static int phyGetManualResults(enet_t *enet)
{
    int timeout;
    int phy_data = 0;
    enet_status_t *status = &(enet->status);

    for (timeout = 0; timeout < MII_TIMEOUT; ++timeout) {
        if (!phyRead(enet, PHY_PHYCTRL2, &phy_data))
            return FALSE;
        else
            break;
    }
    if (timeout == MII_TIMEOUT)
        return FALSE;

    phy_data = (phy_data & PHY_PHYCTRL2_OP_MOD_MASK)>>PHY_PHYCTRL2_OP_MOD_SHIFT;
    
    if (phy_data == PHY_PHYCTRL2_MODE_OP_MOD_10MBPS_HD     ||
        phy_data == PHY_PHYCTRL2_MODE_OP_MOD_10MBPS_FD)
        status->speed = ENET_10BASET;
    else
        status->speed = ENET_100BASET;
    
    if (phy_data == PHY_PHYCTRL2_MODE_OP_MOD_10MBPS_HD     ||
        phy_data == PHY_PHYCTRL2_MODE_OP_MOD_100MBPS_HD)
        status->duplex = ENET_DUPLEX_HALF;
    else
        status->duplex = ENET_DUPLEX_FULL;

    return TRUE;
}

/********************************************************************/
static int phySetRemoteLoopback(enet_t *enet)
{
    int timeout;
    int phy_data = 0;
    enet_cfg_t *config = &(enet->config);

    for (timeout = 0; timeout < MII_TIMEOUT; ++timeout) {      
        if (!phyRead(enet, PHY_PHYCTRL1, &phy_data))
            return FALSE;
        else
            break;
    }
    if (timeout == MII_TIMEOUT)
        return FALSE;

    if(config->loopback == ENET_LOOPBACK_EXT)
        phy_data |= PHY_PHYCTRL1_REMOTE_LOOP;
    else      
        phy_data &= ~PHY_PHYCTRL1_REMOTE_LOOP;
    
    if (!phyWrite(enet, PHY_PHYCTRL1, phy_data))
        return FALSE;

    return TRUE;
}

/*******************************************************************************/
static void enetBDRingInit()
{
    int i;

    next_rxbd = 0;
    next_txbd = 0;

    for (i = 0; i < ENET_NUM_RXBDS; i++) {
        rxbds[i].status = ENET_BD_RX_E;
        rxbds[i].length = 0;
        rxbds[i].buf_addr = (uint8_t *)BSWAP32((uint32_t)rxbuffers[i]);
    }
    for (i = 0; i < ENET_NUM_TXBDS; i++) {
        txbds[i].status = 0x0000;
        txbds[i].length = 0;        
        txbds[i].buf_addr = (uint8_t *)BSWAP32((uint32_t)txbuffers[i]);
    }

    /* Last buffer descriptor in each ring needs the wrap bit on */
    rxbds[ENET_NUM_RXBDS - 1].status |= ENET_BD_RX_W;
    rxbds[ENET_NUM_TXBDS - 1].status |= ENET_BD_TX_W;
}

/********************************************************************/
static void enetBDInit(enet_t *enet)
{
    enet_map_ptr fec = enet->addr;
    fec->mrbr = (uint16_t)ENET_RX_BUFFER_SIZE;  
    fec->rdsr = (uint32_t)rxbds;
    fec->tdsr = (uint32_t)txbds;
}
/********************************************************************/
static void enetBDStartRX(enet_t *enet)
{
    enet_map_ptr fec = enet->addr;

    fec->rdar= ENET_RDAR_RDAR_MASK;
    while( !fec->rdar ) { /* if this gets stuck there is a DMA engine problem */
    }
}

/*******************************************************************************/
/*
 * From Freescale's example code
 *
 * Generate the hash table settings for the given address
 *
 * Parameters:
 *  addr    48-bit (6 byte) Address to generate the hash for
 *
 * Return Value:
 *  The 6 most significant bits of the 32-bit CRC result
 */

static uint8_t enetHashAddress(const uint8_t* addr)
{
    uint32_t crc;
    uint8_t byte;
    int i, j;

    crc = 0xFFFFFFFF;
    for(i=0; i<6; ++i) {
        byte = addr[i];
        for(j=0; j<8; ++j) {
            if((byte & 0x01)^(crc & 0x01)) {
                crc >>= 1;
                crc = crc ^ 0xEDB88320;
            }
            else
                crc >>= 1;
            byte >>= 1;
        }
    }
    return (uint8_t)(crc >> 26);
}

/*******************************************************************************/
static void enetSetAddress (enet_map_ptr fec, const uint8_t *mac_addr)
{
    uint8_t crc;

    /* Write the MAC address into the FEC */
    fec->palr = (uint32_t)((mac_addr[0]<<24) | (mac_addr[1]<<16) 
                         | (mac_addr[2]<<8)  |  mac_addr[3]);
    fec->paur = (uint32_t)((mac_addr[4]<<24) | (mac_addr[5]<<16));

    /*
     * Calculate and set the hash for given Physical Address
     * in the  Individual Address Hash registers
     */
    crc = enetHashAddress(mac_addr);
    if(crc >= 32)
        fec->iaur |= (uint32_t)(1 << (crc - 32));
    else
        fec->ialr |= (uint32_t)(1 << crc);
}

/*******************************************************************************/
static void enetReset (enet_t *enet)
{
    enet_map_ptr fec = enet->addr;
    int i;

    fec->ecr = ENET_ECR_RESET_MASK;

    /* Wait at least 8 clock cycles */
    for (i=0; i<10; ++i)
        asm( "NOP" );
}

/*******************************************************************************/
static void enetInit (enet_t *enet)
{
    enet_map_ptr fec = enet->addr;
    enet_cfg_t *config = &(enet->config);
    enet_status_t *status = &(enet->status);

    enetBDRingInit(enet);
    enetReset(enet);
    /* Clear address hash registers */
    fec->ialr = 0;
    fec->iaur = 0;
    fec->galr = 0;
    fec->gaur = 0;

    enetSetAddress(fec, config->mac_addr);

    /* Mask interrupts */
    fec->eimr = 0;

    /* Clear old interrupt events */
    fec->eir = 0xFFFFFFFF;
    
    /* Init Receive Control Register */
    fec->rcr = ENET_RCR_MAX_FL(ETH_MAX_FRM)
        | ENET_RCR_MII_MODE_MASK 
        | ENET_RCR_CRCFWD_MASK
        | ENET_RCR_RMII_MODE_MASK;
      
    phyInit(enet);
    if (config->autoneg == ENET_AUTONEG_ON) {
        phyAutoneg(enet);
        phyGetAutoResults(enet);
    } else {
        phyManual(enet);
        phyGetManualResults(enet);
        phySetRemoteLoopback(enet);
    }

    if( status->speed == ENET_10BASET )
        fec->rcr |= ENET_RCR_RMII_10T_MASK;
    
    fec->tcr = 0;    
    
    /* Set the duplex */
    if (status->duplex == ENET_DUPLEX_HALF) {      
        fec->rcr |= ENET_RCR_DRT_MASK;
        fec->tcr &= (uint32_t)~ENET_TCR_FDEN_MASK;
    } else {
        fec->rcr &= ~ENET_RCR_DRT_MASK;
        fec->tcr |= ENET_TCR_FDEN_MASK;
    }
    if (config->prom == ENET_PROM_ON)
        fec->rcr |= ENET_RCR_PROM_MASK; 
    
    /* If you use enhanced BD's you need to set IEEE1588 mode here */
    fec->ecr = 0;
    
    /* transmit auto calc and insert IP and Protocol checksums - leave 0 */
    fec->tfwr = ENET_TFWR_STRFWD_MASK; /* must be enabled to use auto csums */
    fec->tacc = (ENET_TACC_IPCHK_MASK | ENET_TACC_PROCHK_MASK);
    /* rx,  toss packest with IP, Protocol or Line problems */
    fec->rsfl = 0;  /* enable rx store and forward */
    fec->racc = (ENET_RACC_IPDIS_MASK | ENET_RACC_PRODIS_MASK 
        | ENET_RACC_LINEDIS_MASK);

    if(config->loopback == ENET_LOOPBACK_INT)
        fec->rcr |= ENET_RCR_LOOP_MASK;

    enetBDInit(enet);
}

/*******************************************************************************/
static void enetSetState (enet_t *enet, int new_state)
{
    enet_map_ptr fec = enet->addr;
    enet_status_t *status = &(enet->status);
    enet_cfg_t *config = &(enet->config);

    if (eth_hwaddr_zero(config->mac_addr) || eth_hwaddr_eff(config->mac_addr)) {
        return;
    }

    if (new_state == ENET_ON) {
        if (status->on_off == ENET_OFF) {
            enetInit(enet);
            if (status->link == ENET_LINK_UP) {
                fec->ecr |= ENET_ECR_ETHEREN_MASK;
                enetBDStartRX(enet);
                status->on_off = ENET_ON;
            }
        }
    } else {
        if (status->on_off == ENET_ON) {
            status->on_off = ENET_OFF;
            fec->ecr &= ~ENET_ECR_ETHEREN_MASK;
        }
    }
}

/*******************************************************************************/
static int enetWaitFrameRX(enet_t *enet, int timeout)
{
    int i;
    enet_map_ptr fec = enet->addr;
        
    for (i=0; i < timeout; i++) {
        if (fec->eir & ENET_EIR_RXF_MASK) {
            fec->eir = ENET_EIR_RXF_MASK;
            return TRUE;        
        }
    }
    return FALSE;
}


/*******************************************************************************/
static int enetReadPacket(enet_t *enet, uint8_t* buffer, unsigned maxlen )
{
    int last_buffer;
    int cur_rxbd;
    int accumulated_len;
    int read_len;
    enet_descr_t* rx_packet = &(enet->last_rx_packet);

    last_buffer = 0;
    rx_packet->length = 0;
    accumulated_len = 0;
    read_len = 0;
    cur_rxbd = next_rxbd;

    if (maxlen < ETH_MIN_FRM) return -1;
    if(rxbds[cur_rxbd].status & ENET_BD_RX_E) return -1;    


    while(!last_buffer) {
        rx_packet->buf_addr = (uint8_t *)BSWAP32((uint32_t)rxbds[cur_rxbd].buf_addr);
        rx_packet->status = rxbds[cur_rxbd].status;
        rx_packet->length = BSWAP16(rxbds[cur_rxbd].length);
        last_buffer = (rx_packet->status & ENET_BD_RX_L);

        if(last_buffer) {
            read_len = (rx_packet->length - accumulated_len); /* On last BD len is total */
        } else {
            read_len = rx_packet->length; /* On intermediate BD's len is BUFFER_SIZE */
        }

        /* accumation of all reads cannot blow past passed maxlen, so clamp read_len */
        read_len = ((accumulated_len + read_len) <= maxlen) ? read_len : (maxlen - accumulated_len);
        if (read_len > 0) { /* Got data to copy, move it so buffer can be re-used */
            memcpy((void *)buffer, (void *) rx_packet->buf_addr, read_len);
            buffer += read_len;
            accumulated_len += read_len;
        }

        /* Mark rxbd as empty so uDMA can re-use it */
        if(rx_packet->status & ENET_BD_RX_W) {
            rxbds[cur_rxbd].status = (ENET_BD_RX_W | ENET_BD_RX_E);
            cur_rxbd = 0;
        } else {
            rxbds[cur_rxbd].status = ENET_BD_RX_E;
            cur_rxbd++;
        }
    }

    enetBDStartRX(enet);
    next_rxbd = cur_rxbd;
    return accumulated_len;
}

/********************************************************************/
static int enetWritePacket(enet_t *enet, uint8_t* buffer, unsigned len)
{
    int num_txbds;
    int cur_txbd;
    int i;
    uint16_t buf_len;
    unsigned written_len = 0;
    enet_map_ptr fec = enet->addr;

    cur_txbd = next_txbd;
    num_txbds = (len/ENET_TX_BUFFER_SIZE);
    if((num_txbds * ENET_TX_BUFFER_SIZE) < len) {
        num_txbds = num_txbds + 1;
    }
    
    for (i = 0; i < num_txbds; i++) {

        /* Block while buffer is still in use, make sure xmit is on */
        while (txbds[cur_txbd].status & ENET_BD_TX_R) {
            if (!(fec->tdar & ENET_TDAR_TDAR_MASK))  
                fec->tdar = ENET_TDAR_TDAR_MASK;
        }
        
        txbds[cur_txbd].status = ENET_BD_TX_TC;
        if(i == num_txbds - 1) {
            buf_len = (uint16_t) len;
            txbds[cur_txbd].status |= ENET_BD_TX_L;      
        } else {
            buf_len = (uint16_t) ENET_TX_BUFFER_SIZE;
            len -= ENET_TX_BUFFER_SIZE;
        }
        txbds[cur_txbd].length = BSWAP16(buf_len);
        
        memcpy((void *)BSWAP32((uint32_t)txbds[cur_txbd].buf_addr), 
            (void *)((uint32_t) buffer), buf_len); 

        buffer += buf_len;
        written_len += buf_len;

        /* This buffer is ready to go, check for wrap condition */
        if(cur_txbd == (ENET_NUM_TXBDS - 1))
        {
            txbds[cur_txbd].status |= (ENET_BD_TX_W | ENET_BD_TX_R);
            cur_txbd = 0;
        } else {
            txbds[cur_txbd].status |= ENET_BD_TX_R;
            cur_txbd++;
        }
    }
    
    next_txbd = cur_txbd;
    fec->tdar = ENET_TDAR_TDAR_MASK; /* Tell FEC to let 'r rip */
    return written_len;
}

/*******************************************************************************/
/* Driver support functions below here
 *
 */

/*******************************************************************************/
static int enetOpen(devoptab_t *dot)
{
    int i;

    if (dot->priv) return FALSE; /* Device is already open */

    enet_t *enet = &enetModules[dot->min];
    enet_status_t *status = &(enet->status);
    dot->priv = enet;

    /* Enable the module in the SIM */
    *enet->simBBPtr = 1;
    SIM_SCGC7 |= SIM_SCGC7_MPU_MSK;
    MPU_CESR = 0;    /* MPU off */

    /* Throw the Pin Muxes to connect the module to external I/O */
    if (dot->min == 0) {
        SIM_SCGC5 |= SIM_SCGC5_PORTA_ENABLE;
        SIM_SCGC5 |= SIM_SCGC5_PORTB_ENABLE;
    }
    enetPin_t *pin = enet->pin;
    for (i = 0; i < NUM_PINS; i++) {
        PORT_PCR(pin[i].port, pin[i].num) = pin[i].mux;
    }
    enetReset(enet);
    status->on_off = ENET_OFF;
    return TRUE;
}

/*******************************************************************************/
static int enetClose(devoptab_t *dot)
{
    enet_t *enet = (enet_t *) dot->priv;
    enet_status_t *status = &(enet->status);

    if (status->link == ENET_LINK_UP) {
        enetSetState(enet, ENET_OFF);
    }
    *enet->simBBPtr = 0;
    dot->priv = NULL;
    return TRUE;
}

/*******************************************************************************/
static unsigned enetWritePolled(void *enetPtr, const void *data, unsigned len)
{
    enet_t * enet = (enet_t *) enetPtr;
    uint8_t *dataPtr = (uint8_t *) data;

    return enetWritePacket(enet, dataPtr, len);
}

/*******************************************************************************/
static unsigned enetReadPolled(void *enetPtr, void *data, unsigned len)
{
    enet_t * enet = (enet_t *) enetPtr;
    uint8_t *dataPtr = (uint8_t *) data;

    return enetReadPacket(enet, dataPtr, len );
}

/*******************************************************************************/
static unsigned enetReadInterrupt(void *enetPtr, void *data, unsigned len)
{
    enet_t * enet = (enet_t *) enetPtr;
    return 0;
}

/*******************************************************************************/
static unsigned enetWriteInterrupt(void *enetPtr, const void *data, unsigned len)
{
    enet_t * enet = (enet_t *) enetPtr;
    return 0;
}

/*******************************************************************************/
static unsigned enetMac1588Isr(enet_t *enet) 
{

    return TRUE;
}


/*******************************************************************************/
static unsigned enetMacTxIsr(enet_t *enet) 
{

    return TRUE;
}


/*******************************************************************************/
static unsigned enetMacRxIsr(enet_t *enet) 
{

    return TRUE;
}


/*******************************************************************************/
static unsigned enetMacErrorIsr(enet_t *enet) 
{

    return TRUE;
}


/*******************************************************************************/

static unsigned enet0Mac1588Isr(void) 
{
    return enetMac1588Isr(&enetModules[ENET_MODULE_0]);
}

static unsigned enet0MacTxIsr(void) 
{
    return enetMacTxIsr(&enetModules[ENET_MODULE_0]);
}

static unsigned enet0MacRxIsr(void) 
{
    return enetMacRxIsr(&enetModules[ENET_MODULE_0]);
}

static unsigned enet0MacErrorIsr(void) 
{
    return enetMacErrorIsr(&enetModules[ENET_MODULE_0]);
}

/*******************************************************************************/

/*=============================================================================*/
/* POSIX FUNCTIONS                                                             */
/*=============================================================================*/

/*******************************************************************************/
/* enet_open_r                                                                  */
/*******************************************************************************/
/* Jobs of the 'open' syscall:
 *      Check device name
 *      Create a device 'state' structure, hook it to the devoptab private ptr
 *      Enable the SIM SCGC for the device
 *      Initialize the device with a default configuration
 ********************************************************************************/
static int enet_open_r (void *reent, devoptab_t *dot, int mode, int flags )
{

    if (!dot || !dot->name) {
        /* errno ? */
        return FALSE;
    }

    /* Verify module instance */
    if( dot->min >= NUM_ENET_MODULES ) {
        ((struct _reent *)reent)->_errno = ENODEV;
        return FALSE;
    }

    /* Try to open if not already open */
    if (enetOpen(dot)) {
        return TRUE;
    } else {
        /* Device is already open, is this an issue or not? */
        ((struct _reent *)reent)->_errno = EPERM;
        return FALSE;
    }
}

/*******************************************************************************/
/* enet_ioctl_r                                                                 */
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
static int enet_ioctl(devoptab_t *dot, int cmd,  int flags)
{
    if (!dot || !dot->priv) return FALSE;

    enet_t *enet = (enet_t *) dot->priv;
    enet_status_t *status = &(enet->status);
    enet_cfg_t *config = &(enet->config);

    switch (cmd) {
    case IO_IOCTL_ENET_SET_MAC_ADDRESS:
        if (flags == 0) return FALSE;
        eth_hwaddr_copy(config->mac_addr, (uint8_t *)flags);
        break;
    case IO_IOCTL_ENET_SET_ENET_STATE:
        enetSetState(enet, flags);
        break;
    case IO_IOCTL_ENET_GET_ENET_STATE:
        if (flags == 0) return FALSE;
        *((int *)flags) = status->on_off;
        break;
    case IO_IOCTL_ENET_GET_DETAILED_ERROR:
        if (flags == 0) return FALSE;
        break;
    case IO_IOCTL_ENET_SET_AUTONEGOTIATE:
        if (flags == ENET_AUTONEG_OFF)
            config->autoneg = ENET_AUTONEG_OFF;
        else
            config->autoneg = ENET_AUTONEG_ON;
        break;
    case IO_IOCTL_ENET_SET_SPEED:
        if (flags == ENET_10BASET)
            config->speed = ENET_10BASET;
        else
            config->speed = ENET_100BASET;
        break;
    case IO_IOCTL_ENET_SET_DUPLEX:
        if (flags == ENET_DUPLEX_HALF)
            config->duplex = ENET_DUPLEX_HALF;
        else
            config->duplex = ENET_DUPLEX_FULL;
        break;
    case IO_IOCTL_ENET_GET_PHY_CONFIG:
        if (flags == 0) return FALSE;
        *((enet_cfg_t *)flags) = *config;
        break;
    case IO_IOCTL_ENET_GET_PHY_STATUS:
        if (flags == 0) return FALSE;
        *((enet_status_t *)flags) = *status;
        break;
    case IO_IOCTL_ENET_GET_PHY_REG:
        if (flags == 0) return FALSE;
        if ((status->on_off) == ENET_OFF) return FALSE;
        return (phyRead(enet, *(int *)flags, (int *)flags));
        break;
    case IO_IOCTL_ENET_GET_LAST_RXBD:
        if (flags == 0) return FALSE;
        *((enet_descr_t *)flags) = enet->last_rx_packet;
        break;
    default: 
        return FALSE;
    }
    return TRUE;
}

/*******************************************************************************/
/* enet_close_r                                                                 */
/*******************************************************************************/
/* Jobs of the 'close' syscall:
 *      Disable the SIM SCGC for the device
 *      Free the device 'state' structure, unhook it to the devoptab private ptr
 *******************************************************************************/
static int enet_close_r (void *reent, devoptab_t *dot )
{
    if (!dot || !dot->priv) return FALSE;

    return enetClose(dot);
}

/*******************************************************************************/
/* enet_write_r                                                                 */
/*******************************************************************************/
/* Jobs of the 'write' syscall:
 *      Write data to the device.
 *      Return the number of bytes written
 *******************************************************************************/
static long enet_write_r (void *reent, devoptab_t *dot, const void *buf, int len )
{
    if (!dot || !dot->priv) return FALSE;
    enet_t * enet = (enet_t *) dot->priv;
    return enet->write(enet, buf, len);
}

/*******************************************************************************/
/* enet_read_r                                                                  */
/*******************************************************************************/
/* Jobs of the 'read' syscall:
 *      Read data from the device
 *      Return the number of bytes read
 *******************************************************************************/
static long enet_read_r (void *reent, devoptab_t *dot, void *buf, int len )
{
    if (!dot || !dot->priv) return FALSE;
    enet_t * enet = (enet_t *) dot->priv;
    return enet->read(enet, buf, len);
}

/*******************************************************************************/
int enet_install(void)
/*******************************************************************************/
{
    int ret = TRUE;
    if( !deviceInstall(DEV_MAJ_ENET,enet_open_r,  enet_ioctl, enet_close_r,
                                                       enet_write_r, enet_read_r)){
        ret = FALSE;
    }
    if( !deviceRegister("eth0", DEV_MAJ_ENET, ENET_MODULE_0,  NULL) ) {
        ret =  FALSE;
    }
    return ret;
}

