/*******************************************************************************
*
* uart.c
*
* jimmyMac!
*
* Low level driver for the Kinetis UART module.
*
* API: uartInit(), uartWrite(), uartRead(),
*
*  Driver assumes 8N1, no hardware control.   Let me know if you need something
*  else.
*
* Copyright (C) 2012 www.laswick.net
*
* This program is free software.  It comes without any warranty, to the extent
* permitted by applicable law.  You can redistribute it and/or modify it under
* the terms of the WTF Public License (WTFPL), Version 2, as published by
* Sam Hocevar.  See http://sam.zoy.org/wtfpl/COPYING for more details.
*
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "kinetis.h"

#include "hardware.h"
#include "globalDefs.h"


enum {
    UART_MODULE_3,
    NUM_UART_MODULES,
};

typedef struct {
    int32_t              minor;
    volatile uartPort_t *reg;
    unsigned             port;
    unsigned             sim;
    volatile uint32_t   *simScgcPtr;
    unsigned             simScgcEnBit;
    unsigned             simScgc5PortEn;
    uint32_t             txPin;
    uint32_t             rxPin;
    uint32_t             txPortCtrlBits;
    uint32_t             rxPortCtrlBits;
} uart_t;

static uart_t uartList[NUM_UART_MODULES] = {
    [UART_MODULE_3] = {
        .minor          = UART_MODULE_3,
        .reg            = UART3_REG_PTR,
        .port           = UART3_PORT,
        .simScgcPtr     = SIM_SCGC4_PTR,
        .simScgcEnBit   = SIM_SCGC4_UART3_ENABLE,
        .simScgc5PortEn = UART3_PORT_ENABLE,
        .txPin          = UART3_TX_PIN,
        .rxPin          = UART3_RX_PIN,
        .txPortCtrlBits = UART3_TX_MUX,
        .rxPortCtrlBits = UART3_RX_MUX,
    },
};


typedef struct {
    int32_t      clockHz;
    int32_t      baud;
} uartDev_t;

static uartDev_t uartDev[NUM_UART_MODULES] = {
    [UART_MODULE_3] = {
        .clockHz    = BUS_CLOCK_HZ,
        .baud       = 115200,
    },
};

static void setBaud(uart_t *uart)
{
    uint16_t sbr;
    uint16_t baudFineAdjust;

    int32_t minor = uart->minor;
    int32_t clockHz = uartDev[minor].clockHz;
    int32_t baud    = uartDev[minor].baud;


    uart->reg->c2 &= ~(UART_C2_RX_ENABLE | UART_C2_TX_ENABLE);

    sbr = (uint16_t)(clockHz/(baud * 16));
    uart->reg->bdh = (sbr & UART_BDH_SBR_MASK) >> UART_BDH_SBR_SHIFT;
    uart->reg->bdl =  sbr & UART_BDL_SBR_MASK;

    /* fine adjust to sbr is in 1/32 increments, calculated as
     * (uartClockHz * 32) / (baud * 16) - (sbr * 32) */
    baudFineAdjust = 2 * clockHz / baud  - sbr * 32;
    uart->reg->c4 = baudFineAdjust & UART_C4_BRFA_MASK;


    uart->reg->cfifo |= UART_CFIFO_RXFLUSH;
    uart->reg->pfifo |= UART_PFIFO_RXFE;

    uart->reg->c2 |= UART_C2_RX_ENABLE | UART_C2_TX_ENABLE;

    return;
}

static int uartOpen(devoptab_t *dot)
{
    uart_t *uart;

    if (dot->priv) return TRUE; /* Device is already open */

    uart = &uartList[dot->min];
    dot->priv = uart;

    /*
     * Config the SIM Clock Gate
     */

    SIM_SCGC5 |= uart->simScgc5PortEn;

    /*
     * Config the Port Controller
     */
    PORT_PCR(uart->port, uart->txPin) = uart->txPortCtrlBits;
    PORT_PCR(uart->port, uart->rxPin) = uart->rxPortCtrlBits;


    /*
     * Config the SIM Uart Enable
     */
    *uart->simScgcPtr |= uart->simScgcEnBit;


    /*
     * Write configuration register values
     */
    setBaud(uart);
    uart->reg->c2 &= ~(UART_C2_RX_ENABLE | UART_C2_TX_ENABLE);
    uart->reg->cfifo |= UART_CFIFO_RXFLUSH;
    uart->reg->pfifo |= UART_PFIFO_RXFE;
    uart->reg->c2 |= UART_C2_RX_ENABLE | UART_C2_TX_ENABLE;

    return TRUE;
}


/*******************************************************************************
*
* uartWrite
*
* This routine transmits the bytes in a variable length buffer out the
* requested UART port.
*
* RETURNS: Number bytes written
*
*******************************************************************************/
int32_t uartWrite(devoptab_t *dot, const void *data, unsigned len)
{
    int32_t i;
    uint8_t *dataPtr = (uint8_t *) data;
    uart_t *uart;

    if (!dot || !dot->priv) return FALSE;
    else uart = (uart_t *) dot->priv;

    for (i = 0; i < len; i++) {
        /* Wait for space in the FIFO */
        while(!(uart->reg->s1 & UART_S1_TX_DATA_LOW));
        uart->reg->d = *dataPtr++;
    }

    return dataPtr - (uint8_t *)data;
}

/*******************************************************************************
*
* uartRead
*
* This routine receives the requested number of bytes from a uart port
*
* RETURNS: Number bytes received
*
*******************************************************************************/
int32_t uartRead(devoptab_t *dot, const void *data, unsigned len)
{
    int32_t i;
    uint8_t *dataPtr = (uint8_t *) data;
    uart_t *uart;

    if (!dot || !dot->priv) return FALSE;
    else uart = (uart_t *) dot->priv;

    for (i = 0; i < len; i++) {
        int readyRetry = 1000;

        while (!(uart->reg->s1 & UART_S1_RX_DATA_FULL) && --readyRetry);

        if (readyRetry) {
            *dataPtr++ = uart->reg->d;
        }
        else {
            break;
        }
    }
    return dataPtr - (uint8_t *)data;
}


/*=============================================================================*/
/* POSIX FUNCTIONS                                                             */
/*=============================================================================*/

/*******************************************************************************/
/* uart_open_r                                                                  */
/*******************************************************************************/
/* Jobs of the 'open' syscall:
 *      Check device name
 *      Create a device 'state' structure, hook it to the devoptab private ptr
 *      Enable the SIM SCGC for the device
 *      Initialize the device with a default configuration
 ********************************************************************************/
static int uart_open_r (void *reent, devoptab_t *dot, int mode, int flags )
{
    if (!dot || !dot->name) {
        /* errno ? */
        return FALSE;
    }

    /* Test the module instance */
    if ( dot->min >= NUM_UART_MODULES ) {
        /* Device does not exist */
        ((struct _reent *)reent)->_errno = ENODEV;
        return FALSE;
    }

    /* Try to open */
    if (uartOpen(dot)) {
        return TRUE;
    } else {
        /* Could not allocate memory */
        return FALSE;
    }
}

/*******************************************************************************/
/* uart_ioctl                                                                  */
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
static int uart_ioctl(devoptab_t *dot, int cmd,  int flags)
{
    uart_t *uart;


    if (!dot || !dot->priv) return FALSE;
    else uart = (uart_t *) dot->priv;

    switch (cmd) {
    case IO_IOCTL_UART_BAUD_SET:
        uartDev[uart->minor].baud = flags;
        setBaud(uart);
        break;
    default:
        return FALSE;
    }

    return TRUE;
}

/*******************************************************************************/
/* uart_close_r                                                                 */
/*******************************************************************************/
/* Jobs of the 'close' syscall:
 *      Disable the SIM SCGC for the device
 *      Free the device 'state' structure, unhook it to the devoptab private ptr
 *******************************************************************************/
static int uart_close_r (void *reent, devoptab_t *dot )
{
    uart_t *uart = dot->priv;

    if (uart) {
        /* Disable the SIMSCGC for the uart module being used*/
        *uart->simScgcPtr &= ~uart->simScgcEnBit;
        dot->priv = NULL;
        return TRUE;
    }
    else {
        return FALSE;
    }
}

/*******************************************************************************/
/* uart_write_r                                                                 */
/*******************************************************************************/
/* Jobs of the 'write' syscall:
 *      Write data to the device.
 *      Return the number of bytes written
 *******************************************************************************/
static long uart_write_r (void *reent, devoptab_t *dot, const void *buf,int len)
{
    /* You could just put your write function here, but I want switch between
     * polled & interupt functions here at a later point.*/
    return uartWrite(dot, buf, len);
}

/*******************************************************************************/
/* uart_read_r                                                                  */
/*******************************************************************************/
/* Jobs of the 'read' syscall:
 *      Read data from the device
 *      Return the number of bytes read
 *******************************************************************************/
static long uart_read_r (void *reent, devoptab_t *dot, void *buf, int len )
{
    /* You could just put your read function here, but I want switch between
     * polled & interupt functions here at a later point.*/
    return uartRead(dot, buf, len);
}


int uart_install(void)
{
    int ret = TRUE;

    if( !deviceInstall(DEV_MAJ_UART,uart_open_r, uart_ioctl, uart_close_r,
                                                 uart_write_r, uart_read_r) ){
        ret = FALSE;
    }
    if( !deviceRegister("uart3", DEV_MAJ_UART, UART_MODULE_3,  NULL) ) {
        ret =  FALSE;
    }

    return ret;
}

