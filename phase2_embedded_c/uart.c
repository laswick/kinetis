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
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "kinetis.h"

#include "hardware.h"
#include "globalDefs.h"



typedef struct {
    int32_t             major;
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

typedef enum uartModule_e{
    UART_MODULE_0,
    UART_MODULE_1,
    UART_MODULE_2,
    UART_MODULE_3,
    UART_MODULE_4,
    UART_MODULE_5,
    NUM_UART_MODULES,
} uartModule_t;



static uart_t uartList[NUM_UART_MODULES] = {
    [UART_MODULE_0] = {
        .major          = UART_MODULE_0,
        .reg            = UART0_REG_PTR,
        .simScgcPtr     = SIM_SCGC4_PTR,
        .simScgcEnBit   = SIM_SCGC4_UART0_ENABLE,
#if 0 /* RFI */
        .port           = UART0_PORT
        .simScgc5PortEn = UART0_PORT_ENABLE,
        .txPin          = UART0_TX_PIN,
        .rxPin          = UART0_RX_PIN,
        .txPortCtrlBits = UART0_TX_MUX,
        .rxPortCtrlBits = UART0_RX_MUX,
#endif
    },
    [UART_MODULE_1] = {
        .major          = UART_MODULE_1,
        .reg            = UART1_REG_PTR,
        .simScgcPtr     = SIM_SCGC4_PTR,
        .simScgcEnBit   = SIM_SCGC4_UART1_ENABLE,
#if 0 /* RFI */
        .port           = UART1_PORT
        .simScgc5PortEn = UART1_PORT_ENABLE,
        .txPin          = UART1_TX_PIN,
        .rxPin          = UART1_RX_PIN,
        .txPortCtrlBits = UART1_TX_MUX,
        .rxPortCtrlBits = UART1_RX_MUX,
#endif
    },
    [UART_MODULE_2] = {
        .major          = UART_MODULE_2,
        .reg            = UART2_REG_PTR,
        .simScgcPtr     = SIM_SCGC4_PTR,
        .simScgcEnBit   = SIM_SCGC4_UART2_ENABLE,
#if 0 /* RFI */
        .port           = UART2_PORT
        .simScgc5PortEn = UART2_PORT_ENABLE,
        .txPin          = UART2_TX_PIN,
        .rxPin          = UART2_RX_PIN,
        .txPortCtrlBits = UART2_TX_MUX,
        .rxPortCtrlBits = UART2_RX_MUX,
#endif
    },
    [UART_MODULE_3] = {
        .major          = UART_MODULE_3,
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
    [UART_MODULE_4] = {
        .major          = UART_MODULE_4,
        .reg            = UART4_REG_PTR,
        .port           = UART4_PORT,
        .simScgcPtr     = SIM_SCGC1_PTR,
        .simScgcEnBit   = SIM_SCGC1_UART4_ENABLE,
        .simScgc5PortEn = UART4_PORT_ENABLE,
        .txPin          = UART4_TX_PIN,
        .rxPin          = UART4_RX_PIN,
        .txPortCtrlBits = UART4_TX_MUX,
        .rxPortCtrlBits = UART4_RX_MUX,
    },
    [UART_MODULE_5] = {
        .major          = UART_MODULE_5,
        .reg            = UART5_REG_PTR,
        .port           = UART5_PORT,
        .simScgcPtr     = SIM_SCGC1_PTR,
        .simScgcEnBit   = SIM_SCGC1_UART5_ENABLE,
        .simScgc5PortEn = UART5_PORT_ENABLE,
        .txPin          = UART5_TX_PIN,
        .rxPin          = UART5_RX_PIN,
        .txPortCtrlBits = UART5_TX_MUX,
        .rxPortCtrlBits = UART5_RX_MUX,
    },
};


#define UART_BUFFER_SIZE 256
#define UART_BUFFER_WRAP (UART_BUFFER_SIZE - 1)

typedef struct {
    volatile uint8_t buffer[UART_BUFFER_SIZE];
    volatile uint8_t tailIdx;
    volatile uint8_t headIdx;
    volatile uint8_t length;
} uartBuffer_t;

typedef struct {
    int32_t      clockHz;
    int32_t      baud;
    int32_t      (*callBack)(uint8_t const *buf, int len);
    char         terminator;
    uartBuffer_t uartRxBuffer;
} uartDev_t;

static uartDev_t uartDev[NUM_UART_MODULES] = {
    [UART_MODULE_0] = {
        .clockHz    = SYSTEM_CLOCK_HZ,
        .baud       = 9600,
    },
    [UART_MODULE_1] = {
        .clockHz    = SYSTEM_CLOCK_HZ,
        .baud       = 9600,
    },
    [UART_MODULE_2] = {
        .clockHz    = BUS_CLOCK_HZ,
        .baud       = 9600,
    },
    [UART_MODULE_3] = {
        .clockHz    = BUS_CLOCK_HZ,
        .baud       = 9600,
    },
    [UART_MODULE_4] = {
        .clockHz    = BUS_CLOCK_HZ,
        .baud       = 9600,
    },
    [UART_MODULE_5] = {
        .clockHz    = BUS_CLOCK_HZ,
        .baud       = 9600,
    },
};

int uart_install(void)
{
    int ret = TRUE;

    /* Std  In, Out, Err use uart3 */
    if (!deviceInstall("uart3", uart_open_r, uart_ioctl, uart_close_r,
                                uart_write_r, uart_read_r, NULL))
        ret = FALSE;
    if (!deviceInstall("uart3", uart_open_r, uart_ioctl, uart_close_r,
                                uart_write_r, uart_read_r, NULL))
        ret = FALSE;
    if (!deviceInstall("uart3", uart_open_r, uart_ioctl, uart_close_r,
                                uart_write_r, uart_read_r, NULL))
        ret = FALSE;


    if (!deviceInstall("uart0", uart_open_r, uart_ioctl, uart_close_r,
                                uart_write_r, uart_read_r, NULL))
        ret = FALSE;

    if (!deviceInstall("uart1", uart_open_r, uart_ioctl, uart_close_r,
                                uart_write_r, uart_read_r, NULL))
        ret = FALSE;

    if (!deviceInstall("uart2", uart_open_r, uart_ioctl, uart_close_r,
                                uart_write_r, uart_read_r, NULL))
        ret = FALSE;

    if (!deviceInstall("uart3", uart_open_r, uart_ioctl, uart_close_r,
                                uart_write_r, uart_read_r, NULL))
        ret = FALSE;

    if (!deviceInstall("uart4", uart_open_r, uart_ioctl, uart_close_r,
                                uart_write_r, uart_read_r, NULL))
        ret = FALSE;

    if (!deviceInstall("uart5", uart_open_r, uart_ioctl, uart_close_r,
                                uart_write_r, uart_read_r, NULL))
        ret = FALSE;

    return ret;
}


static void rxMsgNotify(uart_t *uart, uartBuffer_t *bufferPtr)
{
    uint8_t buf[bufferPtr->length];
    int i;
    for (i = 0; i < bufferPtr->length; i++) {
        buf[i] = bufferPtr->buffer[bufferPtr->headIdx];
        bufferPtr->headIdx = (bufferPtr->headIdx + 1)
            & UART_BUFFER_WRAP;
    }
    uartDev[uart->major].callBack(buf, bufferPtr->length);
    bufferPtr->length = 0;
    return;
}
/******************************************************************************
* isrrUartX _status_sources(void)
*
* Status ISR definition.
*
******************************************************************************/
static void isrHandler(int major)
{
    uart_t *uart = &uartList[major];
    char d;
    uartBuffer_t *bufferPtr = &uartDev[major].uartRxBuffer;

    if (uart->reg->s1 & UART_S1_RX_DATA_FULL) {
        while (uart->reg->s1 & UART_S1_RX_DATA_FULL) {
            d = uart->reg->d;
            if (uartDev[major].callBack && uartDev[major].terminator == d) {
                rxMsgNotify(uart, bufferPtr);
            }
            else {
                bufferPtr->buffer[bufferPtr->tailIdx] = d;
                bufferPtr->tailIdx = (bufferPtr->tailIdx + 1)
                    & UART_BUFFER_WRAP;
                bufferPtr->length++;
                if (uartDev[major].callBack && !uartDev[major].terminator) {
                    /* Send every char back to caller when no
                     * terminator specified
                     */
                    rxMsgNotify(uart, bufferPtr);
                }
            }
        }
    }
    return;
}

static void isrUart0(void)
{
    isrHandler(UART_MODULE_0);
    return;
}

static void isrUart1(void)
{
    isrHandler(UART_MODULE_1);
    return;
}

static void isrUart2(void)
{
    isrHandler(UART_MODULE_2);
    return;
}

static void isrUart3(void)
{
    isrHandler(UART_MODULE_3);
    return;
}

static void isrUart4(void)
{
    isrHandler(UART_MODULE_4);
    return;
}

static void isrUart5(void)
{
    isrHandler(UART_MODULE_5);
    return;
}


#if 0
/******************************************************************************
* _isr_uart3_error_sources
*
* Error ISR definition.
*
******************************************************************************/
extern void _isr_uart3_error_sources(void)

{

   return;
}

#endif
static void setBaud(uart_t *uart)
{
    uint16_t sbr;
    uint16_t baudFineAdjust;

    int32_t major = uart->major;
    int32_t clockHz = uartDev[major].clockHz;
    int32_t baud    = uartDev[major].baud;


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
static int uartOpen(uartModule_t mod, devoptab_t *dot)
{
    uart_t *uart;
    void   *isrPtr;

    if (dot->priv) return FALSE; /* Device is already open */

    /* Create 'private' uart structure and point devoptab's
     * private pointer to it */
    uart = (uart_t *) malloc(sizeof(uart_t));
    if (!uart) return FALSE;
    else dot->priv = uart;



    /* Load init & default info into private spi structure */
    memcpy(uart, &uartList[mod], sizeof(uart_t));

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

    switch (uart->major) {
        case UART_MODULE_0:
            isrPtr = isrUart0;
            break;
        case UART_MODULE_1:
            isrPtr = isrUart1;
            break;
        case UART_MODULE_2:
            isrPtr = isrUart2;
            break;
        case UART_MODULE_3:
            isrPtr = isrUart3;
            break;
        case UART_MODULE_4:
            isrPtr = isrUart4;
            break;
        case UART_MODULE_5:
            isrPtr = isrUart5;
            break;
        default:
            assert(0);
            return FALSE;
    }

    hwInstallISRHandler(ISR_UART0_STATUS_SOURCES + 2 * uart->major,
            isrPtr);

    uart->reg->c2 |= UART_C2_RX_ENABLE | UART_C2_TX_ENABLE
        | UART_C2_RX_FULL_INT_ENABLE;


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


return 0;
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
int uart_open_r (void *reent, devoptab_t *dot, int mode, int flags )
{
    uartModule_t mod;

    if (!dot || !dot->name) {
        /* errno ? */
        return FALSE;
    }

    /* Determine the module instance */
    if (strcmp(DEVOPTAB_UART0_STR, dot->name) == 0 ) {
        mod = UART_MODULE_0;
    }
    else if (strcmp(DEVOPTAB_UART1_STR, dot->name) == 0) {
        mod = UART_MODULE_1;
    }
    else if (strcmp(DEVOPTAB_UART2_STR, dot->name) == 0) {
        mod = UART_MODULE_2;
    }
    else if (strcmp(DEVOPTAB_UART3_STR, dot->name) == 0) {
        mod = UART_MODULE_3;
    }
    else if (strcmp(DEVOPTAB_UART4_STR, dot->name) == 0) {
        mod = UART_MODULE_4;
    }
    else if (strcmp(DEVOPTAB_UART5_STR, dot->name) == 0) {
        mod = UART_MODULE_5;
    }
    else {
        /* Device does not exist */
        ((struct _reent *)reent)->_errno = ENODEV;
        return FALSE;
    }

    /* Try to open if not already open */
    if (uartOpen(mod,dot)) {
        return TRUE;
    } else {
        /* Device is already open, is this an issue or not? */
        ((struct _reent *)reent)->_errno = EPERM;
        return FALSE;
    }
}

/*******************************************************************************/
/* uart_ioctl_r                                                                 */
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
int uart_ioctl(devoptab_t *dot, int cmd,  int flags)
/* TODO: return errors if flags or cmd is bad */
{
    uart_t *uart;
    uartBuffer_t *bufferPtr;


    if (!dot || !dot->priv) return FALSE;
    else uart = (uart_t *) dot->priv;

    bufferPtr = &uartDev[uart->major].uartRxBuffer;

    switch (cmd) {
    case IO_IOCTL_UART_CALL_BACK_SET:
        if (flags) {
            uartDev[uart->major].callBack = (void *) flags;
        }
        break;
    case IO_IOCTL_UART_TERMINATOR_SET:
        uartDev[uart->major].terminator = (char) flags;
        break;
    case IO_IOCTL_UART_FLUSH_RX_FIFO:
        uart->reg->c2 &= ~UART_C2_RX_FULL_INT_ENABLE;
        bufferPtr->tailIdx = 0;
        bufferPtr->length  = 0;
        uart->reg->c2 |= UART_C2_RX_FULL_INT_ENABLE;
        break;
    case IO_IOCTL_UART_LOOPBACK_ENABLE:
        if (flags == TRUE) {
            uart->reg->c1 |= UART_C1_LOOP_BACK;
        }
        else {
            uart->reg->c1 &= ~UART_C1_LOOP_BACK;
        }
        break;
    case IO_IOCTL_UART_BAUD_SET:
                            /*
                             * I'm not checking standard baud rates.
                             * Just don't be an a$$hole... :)
                             */
        uartDev[uart->major].baud = flags;
        setBaud(uart);
        break;
    default:
        assert(0);
        return FALSE;
        break;
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
int uart_close_r (void *reent, devoptab_t *dot )
{
    uart_t *uart = dot->priv;

    if (uart) {
        /* Disable the SIMSCGC for the uart module being used*/
        *uart->simScgcPtr &= ~uart->simScgcEnBit;
        /* Unhook the private uart structure and free it */
        dot->priv = NULL;
        free(uart);
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
long uart_write_r (void *reent, devoptab_t *dot, const void *buf, int len )
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
long uart_read_r (void *reent, devoptab_t *dot, void *buf, int len )
{
    /* You could just put your read function here, but I want switch between
     * polled & interupt functions here at a later point.*/
    return uartRead(dot, buf, len);
}


