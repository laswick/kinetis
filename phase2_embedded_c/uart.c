/*******************************************************************************
*
* uart.c
*
* Low level driver for the Kinetis UART module.
*
* API: uartInit(), uartWrite(), uartRead(),
*
*  Driver assumes 8N1, no hardware control.   Let me know if you need something
*  else.
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
    UART_MODULE_0,
    UART_MODULE_1,
    UART_MODULE_2,
    UART_MODULE_3,
    UART_MODULE_4,
    UART_MODULE_5,
    NUM_UART_MODULES,
} uartModule_t;


uart_t uartList[NUM_UART_MODULES] = {
    [UART_MODULE_0] = {
        .reg            = UART0_REG_PTR,
        .simScgcPtr     = SIM_SCGC4_PTR,
        .simScgcEnBit   = SIM_UART0_ENABLE,
#if 0 /* RFI */
        .port           = UART0_PORT
        .simScgc5PortEn = UART0_PORT_ENABLE,
        .txPin          = UART0_TX_PIN,
        .rxPin          = UART0_RX_PIN,
        .txPortCtrlBits = UART0_TX_MUX,
        .rxPortCtrlBits = UART0_RX_MUX,
#endif
        .clockHz    = SYSTEM_CLOCK_HZ,
        .baud       = 115200,

    },
    [UART_MODULE_1] = {
        .reg            = UART1_REG_PTR,
        .simScgcPtr     = SIM_SCGC4_PTR,
        .simScgcEnBit   = SIM_UART1_ENABLE,
#if 0 /* RFI */
        .port           = UART1_PORT
        .simScgc5PortEn = UART1_PORT_ENABLE,
        .txPin          = UART1_TX_PIN,
        .rxPin          = UART1_RX_PIN,
        .txPortCtrlBits = UART1_TX_MUX,
        .rxPortCtrlBits = UART1_RX_MUX,
#endif
        .clockHz    = SYSTEM_CLOCK_HZ,
        .baud       = 115200,
    },
    [UART_MODULE_2] = {
        .reg            = UART2_REG_PTR,
        .simScgcPtr     = SIM_SCGC4_PTR,
        .simScgcEnBit   = SIM_UART2_ENABLE,
#if 0 /* RFI */
        .port           = UART2_PORT
        .simScgc5PortEn = UART2_PORT_ENABLE,
        .txPin          = UART2_TX_PIN,
        .rxPin          = UART2_RX_PIN,
        .txPortCtrlBits = UART2_TX_MUX,
        .rxPortCtrlBits = UART2_RX_MUX,
#endif
        .clockHz    = BUS_CLOCK_HZ,
        .baud       = 115200,
    },
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
        .clockHz    = BUS_CLOCK_HZ,
        .baud       = 115200,
    },
    [UART_MODULE_4] = {
        .reg            = UART4_REG_PTR,
        .port           = UART4_PORT,
        .simScgcPtr     = SIM_SCGC1_PTR,
        .simScgcEnBit   = SIM_UART4_ENABLE,
        .simScgc5PortEn = UART4_PORT_ENABLE,
        .txPin          = UART4_TX_PIN,
        .rxPin          = UART4_RX_PIN,
        .txPortCtrlBits = UART4_TX_MUX,
        .rxPortCtrlBits = UART4_RX_MUX,
        .clockHz    = BUS_CLOCK_HZ,
        .baud       = 115200,
    },
    [UART_MODULE_5] = {
        .reg            = UART5_REG_PTR,
        .port           = UART5_PORT,
        .simScgcPtr     = SIM_SCGC1_PTR,
        .simScgcEnBit   = SIM_UART5_ENABLE,
        .simScgc5PortEn = UART5_PORT_ENABLE,
        .txPin          = UART5_TX_PIN,
        .rxPin          = UART5_RX_PIN,
        .txPortCtrlBits = UART5_TX_MUX,
        .rxPortCtrlBits = UART5_RX_MUX,
        .clockHz    = BUS_CLOCK_HZ,
        .baud       = 115200,
    },
};

static int uartOpen(uartModule_t mod, devoptab_t *dot)
{
    uart_t *uart;

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


#if 0
/* WIP */
#define  _isr_uart3_status_sources isr_uart3_status_sources
/******************************************************************************
* isr_uart3_status_sources(void)
*
* Status ISR definition.
*
******************************************************************************/
extern void isr_uart3_status_sources(void)

{
    static volatile int32_t count;

    count++;
   #define VECTORNUM                     (*(volatile uint8_t*)(0xE000ED04))

   return;
}


/******************************************************************************
* _isr_uart3_error_sources
*
* Error ISR definition.
*
******************************************************************************/
extern void _isr_uart3_error_sources(void)

{
   #define VECTORNUM                     (*(volatile uint8_t*)(0xE000ED04))

   return;
}

#endif
