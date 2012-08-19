/*******************************************************************************
*
* uartLoader.c
*
* Minimal Low level POSIX driver for the Kinetis UART module.
*
* jimmyMac!
* June 26 2012
*
*******************************************************************************/
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "kinetis.h"
#include "hardware.h"
#include "globalDefs.h"

typedef struct {
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
    int32_t              clockHz;
    int32_t              baud;
} uart_t;

typedef enum uartModule_e{
    UART_MODULE_3,
    NUM_UART_MODULES,
} uartModule_t;


static uart_t uartList[NUM_UART_MODULES] = {
    [UART_MODULE_3] = {
        .reg            = UART3_REG_PTR,
        .port           = UART3_PORT,
        .simScgcPtr     = SIM_SCGC4_PTR,
        .simScgcEnBit   = SIM_UART3_ENABLE,
        .simScgc5PortEn = UART3_PORT_ENABLE,
        .txPin          = UART3_TX_PIN,
        .rxPin          = UART3_RX_PIN,
        .txPortCtrlBits = UART3_TX_MUX,
        .rxPortCtrlBits = UART3_RX_MUX,
        .clockHz        = BUS_CLOCK_HZ,
        .baud           = 115200,
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

    return ret;
}

static int uartOpen(uartModule_t mod, devoptab_t *dot)
{
    uart_t *uart;

    if (dot->priv || mod < NUM_UART_MODULES)
        return FALSE; /* Device is already open */

    /* Create 'private' uart structure and point devoptab's
     * private pointer to it */
    uart = &uartList[mod];
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
    {
        uint16_t sbr;
        uint16_t baudFineAdjust;


        uart->reg->c2 &= ~(UART_C2_RX_ENABLE | UART_C2_TX_ENABLE);
        uart->reg->c1 = 0; /* Cleared for default 8n1 behaviour */

        sbr = (uint16_t)(uart->clockHz/(uart->baud * 16));
        uart->reg->bdh = (sbr & UART_BDH_SBR_MASK) >> UART_BDH_SBR_SHIFT;
        uart->reg->bdl =  sbr & UART_BDL_SBR_MASK;

        /* fine adjust to sbr is in 1/32 increments, calculated as
         * (uartClockHz * 32) / (baud * 16) - (sbr * 32) */
        baudFineAdjust = 2 * uart->clockHz / uart->baud  - sbr * 32;
        uart->reg->c4 = baudFineAdjust & UART_C4_BRFA_MASK;

        /* Setup RX FIFO */
        uart->reg->pfifo  = UART_PFIFO_RXFIFOSIZE_16;
        uart->reg->cfifo |= UART_CFIFO_RXFLUSH;
        uart->reg->pfifo |= UART_PFIFO_RXFE;
        uart->reg->rwfifo = 1; /* FIFO is 16 datawords. Trigger buffer full
                                 flag when at least one byte is in the FIFO */

        uart->reg->c2 |= UART_C2_RX_ENABLE | UART_C2_TX_ENABLE;
    }

    return TRUE;
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
        return FALSE;
    }

    /* Determine the module instance */
    if (strcmp(DEVOPTAB_UART3_STR, dot->name) == 0) {
        mod = UART_MODULE_3;
    }
    else {
        /* Device does not exist */
        ((struct _reent *)reent)->_errno = ENODEV;
        return FALSE;
    }

    /* Try to open if not already open */
    if (uartOpen(mod,dot)) {
        return TRUE;
    }
    else {
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
{
    /* Not Required */
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
    dot->priv = NULL;
    return TRUE;
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
    uint8_t *dataPtr = (uint8_t *) buf;
    uart_t *uart;
    int i;

    if (!dot || !dot->priv)
        return FALSE;
    else
        uart = (uart_t *) dot->priv;

    for (i = 0; i < len; i++) {
        /* Wait for space in the FIFO */
        while(!(uart->reg->s1 & UART_S1_TX_DATA_LOW))
            ;

        uart->reg->d = *dataPtr++;
    }

    return dataPtr - (uint8_t *)buf;
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
    uint8_t *dataPtr = (uint8_t *) buf;
    uart_t *uart;
    int i;

    if (!dot || !dot->priv)
        return FALSE;
    else
        uart = (uart_t *) dot->priv;

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

    return dataPtr - (uint8_t *)buf;
}
