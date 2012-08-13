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
#include <string.h>

#include "kinetis.h"

#include "hardware.h"
#include "globalDefs.h"

static uartIF_t *uartOwner[MAX_UARTS];

static uartIF_t stdOutUartIF;

/*******************************************************************************
*
* _write
*
* This routine retargets STDOUT to the UART configured by setStdout().
*
* RETURNS: Number of bytes written.
*
*******************************************************************************/
int _write(int fd, char *ptr, int len)
{
    assert(stdOutUartIF.uart);           /* Ensure a UART has been designated */

    int i;
    for (i = 0; i < len; i++)
        uartWrite(&stdOutUartIF, (uint8_t *) ptr++, 1);
    return len;
}

/*******************************************************************************
*
* setStdout
*
* This routine sets the argument uart to be used as STDOUT.
*
* RETURNS: Nothing.
*
*******************************************************************************/
void setStdout(uartIF_t *uartIF)
{
    memcpy(&stdOutUartIF, uartIF, sizeof(uartIF_t));
}

/*******************************************************************************
*
* uartGet
*
* This helper function determines the appropriate uart module base address
* based on the argument "uart".
*
* RETURNS: The corresponding UART_BASE_ADDR.
*
*******************************************************************************/
static volatile uartPort_t *uartPortGet(uint32_t uart)
{
    uint32_t addr;

    switch (uart) {
    case UART0: addr = UART0_BASE_ADDR; break;
    case UART1: addr = UART1_BASE_ADDR; break;
    case UART2: addr = UART2_BASE_ADDR; break;
    case UART3: addr = UART3_BASE_ADDR; break;
    case UART4: addr = UART4_BASE_ADDR; break;
    case UART5: addr = UART5_BASE_ADDR; break;
    default:
        assert(0);
        return 0;
    }

    return ((volatile uartPort_t *) addr);
}

/*******************************************************************************
*
* uartGetClock
*
* This helper function maps the correct clock for each uart
*
* RETURNS: UART clock in Hz
*
*******************************************************************************/
static int32_t uartGetClock(uint32_t uart, int32_t systemClockHz,
                                                             int32_t busClockHz)
{
    int32_t uartClockHz;

    switch (uart) {
    case UART0:
    case UART1:
        uartClockHz = systemClockHz;
        break;
    case UART2:
    case UART3:
    case UART4:
    case UART5:
        uartClockHz = busClockHz;
        break;
    default:
        assert(0);
        return 0;
    }

    return uartClockHz;

}

/*******************************************************************************
*
* uartReserve
*
* This helper function controls ownership of a uart
*
* RETURNS: !ERROR on successful reservation
*
*******************************************************************************/
static int32_t uartReserve(uartIF_t *uartIF)
{
    int32_t  retVal = ERROR;
    int32_t uartIdx;

    switch (uartIF->uart) {
    case UART0:
        uartIdx = 0;
        break;
    case UART1:
        uartIdx = 1;
        break;
    case UART2:
        uartIdx = 2;
        break;
    case UART3:
        uartIdx = 3;
        break;
    case UART4:
        uartIdx = 4;
        break;
    case UART5:
        uartIdx = 5;
        break;
    default:
        assert(0);
        return 0;
    }

    if (!uartOwner[uartIdx]) {
        uartOwner[uartIdx] = uartIF;
        retVal = !ERROR;
    }
    return retVal;

}

/*******************************************************************************
*
* uartFree
*
* This routine releases ownership of a uart.
*
* RETURNS: Nothing
*
*******************************************************************************/
void uartFree(uartIF_t *uartIF)
{
    int32_t uartIdx;

    switch (uartIF->uart) {
    case UART0:
        uartIdx = 0;
        break;
    case UART1:
        uartIdx = 1;
        break;
    case UART2:
        uartIdx = 2;
        break;
    case UART3:
        uartIdx = 3;
        break;
    case UART4:
        uartIdx = 4;
        break;
    case UART5:
        uartIdx = 5;
        break;
    default:
        assert(0);
        return;
    }
    uartOwner[uartIdx] = 0;
}

/*******************************************************************************
*
* uartInit
*
* This routine configures the provided uart to 8N1 with requested baud rate.
*
* RETURNS: ERROR
*
*******************************************************************************/
int32_t uartInit(uartIF_t *uartIF)
{
    int32_t retVal = ERROR;
    int32_t uartClockHz = uartGetClock(uartIF->uart, uartIF->systemClockHz,
                                                     uartIF->busClockHz);

    /*
     * Configure the SIM Clock Gate and Port Controller
     */
    uint32_t port;
    uint32_t txPin, rxPin;
    uint32_t txPortCtrlBits, rxPortCtrlBits;

    if (uartReserve(uartIF) == ERROR) {
        assert(0);
        return ERROR;
    }

    switch (uartIF->uart) {
    case UART3:
        port  = UART3_PORT;
        txPin = UART3_TX_PIN;
        rxPin = UART3_RX_PIN;
        txPortCtrlBits = PORT_MUX_ALT3; /* UART is ALT3 on these pins */
        rxPortCtrlBits = PORT_MUX_ALT3;
        break;
    case UART4:
        port  = UART4_PORT;
        txPin = UART4_TX_PIN;
        rxPin = UART4_RX_PIN;
        txPortCtrlBits = PORT_MUX_ALT3; /* UART is ALT3 on these pins */
        rxPortCtrlBits = PORT_MUX_ALT3;
        break;
    case UART5:
        port  = UART5_PORT;
        txPin = UART5_TX_PIN;
        rxPin = UART5_RX_PIN;
        txPortCtrlBits = PORT_MUX_ALT3; /* UART is ALT3 on these pins */
        rxPortCtrlBits = PORT_MUX_ALT3;
        break;
    case UART0:
    case UART1:
    case UART2:
    /* Please define uart pin connection in hardware.h.
     * Add defines near the current UART4_PORT) */
    default:
        assert(0);
        return ERROR;
    }

                                                            /* SIM Clock Gate */
    switch (port) {
    case PORTA: SIM_SCGC5 |= SIM_PORTA_ENABLE; break;
    case PORTB: SIM_SCGC5 |= SIM_PORTB_ENABLE; break;
    case PORTC: SIM_SCGC5 |= SIM_PORTC_ENABLE; break;
    case PORTD: SIM_SCGC5 |= SIM_PORTD_ENABLE; break;
    case PORTE: SIM_SCGC5 |= SIM_PORTE_ENABLE; break;
    default:
        assert(0);
        return ERROR;
    }
                                                           /* Port Controller */
    PORT_PCR(port, txPin) = txPortCtrlBits;
    PORT_PCR(port, rxPin) = rxPortCtrlBits;

    /*
     * Config the SIM Uart Enable
     */

    switch (uartIF->uart) {
    case UART0: SIM_SCGC4 |= SIM_UART0_ENABLE; break;
    case UART1: SIM_SCGC4 |= SIM_UART1_ENABLE; break;
    case UART2: SIM_SCGC4 |= SIM_UART2_ENABLE; break;
    case UART3: SIM_SCGC4 |= SIM_UART3_ENABLE; break;
    case UART4: SIM_SCGC1 |= SIM_UART4_ENABLE; break;
    case UART5: SIM_SCGC1 |= SIM_UART5_ENABLE; break;
    default:
        assert(0);
    }

    /*
     * Enabel UART Pins
     * TODO: J-Mac why the same code twice?
     */

    switch (uartIF->uart) {
    case UART0: SIM_SCGC4 |= SIM_UART0_ENABLE; break;
    case UART1: SIM_SCGC4 |= SIM_UART1_ENABLE; break;
    case UART2: SIM_SCGC4 |= SIM_UART2_ENABLE; break;
    case UART3: SIM_SCGC4 |= SIM_UART3_ENABLE; break;
    case UART4: SIM_SCGC1 |= SIM_UART4_ENABLE; break;
    case UART5: SIM_SCGC1 |= SIM_UART5_ENABLE; break;
    default:
        assert(0);
    }

    volatile uartPort_t *uartPort = uartPortGet(uartIF->uart);

    uint16_t sbr;
    uint16_t baudFineAdjust;

    if (uartPort) {

        uartPort->c2 &= ~(UART_C2_RX_ENABLE | UART_C2_TX_ENABLE);
        uartPort->c1 = 0; /* Cleared for default 8n1 behaviour */

        sbr = (uint16_t)(uartClockHz/(uartIF->baud * 16));
        uartPort->bdh = (sbr & UART_BDH_SBR_MASK) >> UART_BDH_SBR_SHIFT;
        uartPort->bdl =  sbr & UART_BDL_SBR_MASK;

        /* fine adjust to sbr is in 1/32 increments, calculated as
         * (uartClockHz * 32) / (baud * 16) - (sbr * 32) */
        baudFineAdjust = 2 * uartClockHz / uartIF->baud  - sbr * 32;
        uartPort->c4 = baudFineAdjust & UART_C4_BRFA_MASK;

        /* Setup RX FIFO */
        uartPort->pfifo  = UART_PFIFO_RXFIFOSIZE_16;
        uartPort->cfifo |= UART_CFIFO_RXFLUSH;
        uartPort->pfifo |= UART_PFIFO_RXFE;
        uartPort->rwfifo = 1; /* FIFO is 16 datawords. Trigger buffer full
                                 flag when at least one byte is in the FIFO */

        uartPort->c2 |= UART_C2_RX_ENABLE | UART_C2_TX_ENABLE;

        retVal = !ERROR;
    }
    return retVal;
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
int32_t uartWrite(uartIF_t *uartIF, uint8_t *buffer, int32_t len)
{
    volatile uartPort_t *uartPort = uartPortGet(uartIF->uart);
    int32_t i;
    uint8_t *dataPtr = buffer;

    if (uartPort) {

        for (i = 0; i < len; i++) {
            int readyRetry = 1000;
            /* Wait for space in the FIFO */
            while(!(uartPort->s1 & UART_S1_TX_DATA_LOW) && --readyRetry);

            if (readyRetry) {
                /* Send the character */
                uartPort->d = *dataPtr++;
            }
            else {
                assert(0);
                break;
            }
        }
    }

    return dataPtr - buffer;
}

/*******************************************************************************
*
* uartPrint
*
* Writes a null terminated string to the uart.
*
* RETURNS: Number bytes written
*
*******************************************************************************/
int32_t uartPrint(uartIF_t *uartIF, char *string)
{
    return uartWrite(uartIF, (uint8_t *) string, strlen(string));
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
int32_t uartRead(uartIF_t *uartIF, uint8_t *buffer, int32_t len)
{
    volatile uartPort_t *uartPort = uartPortGet(uartIF->uart);
    int i;
    uint8_t *dataPtr = buffer;

    if (uartPort) {
        for (i = 0; i < len; i++) {
            int readyRetry = 1000;

            while (!(uartPort->s1 & UART_S1_RX_DATA_FULL) && --readyRetry)
                ;

            if (readyRetry)
                *dataPtr++ = uartPort->d;
            else
                break;
        }
    }

    return dataPtr - buffer;
}
