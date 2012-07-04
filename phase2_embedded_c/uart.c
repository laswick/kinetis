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
#include "kinetis.h"

#include "hardware.h"
#include "globalDefs.h"

#define MAX_UARTS 5
static uartIF_t *uartOwner[MAX_UARTS];

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
int32_t uartGetClock(uint32_t uart, int32_t systemClockHz, int32_t busClockHz)
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
int32_t uartReserve(uartIF_t *uartIF)
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
* This helper function releases ownership of a uart
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
    default:
        assert(0);
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
    default:
        assert(0);
    }

    /*
     * Enabel UART Pins
     */

    switch (uartIF->uart) {
    case UART0: SIM_SCGC4 |= SIM_UART0_ENABLE; break;
    case UART1: SIM_SCGC4 |= SIM_UART1_ENABLE; break;
    case UART2: SIM_SCGC4 |= SIM_UART2_ENABLE; break;
    case UART3: SIM_SCGC4 |= SIM_UART3_ENABLE; break;
    case UART4: SIM_SCGC1 |= SIM_UART4_ENABLE; break;
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


        uartPort->c2 |= UART_C2_RX_ENABLE | UART_C2_TX_ENABLE;

        retVal = !ERROR;
    }
    return retVal;
}


/*******************************************************************************
*
* uartWrite
*
*
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
            int readyRetry = 100;
            /* Wait for space in the FIFO */
            while(!(uartPort->s1 & UART_S1_TX_DATA_LOW) && readyRetry--);

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

#if 0

/* MODULE CONFIG **************************************************************/


/* UART CONFIG ****************************************************************/

static uartIF_t uartConfig;

static uartIF_t uartConfigDflt = {
    .baudRate = 9600,
    .mode = UART_MODE_CMD_RESP,
    .respWaitTime = 100,
};

static const dbtd_t uartConfigDBTD = {
    DBTID_UART_CONFIG,
    1,
    "UART1 Config",
    sizeof(uartIF_t),
    DBT_TYPE_FLASH,
};

static const dbfd_t uartConfigDBFD[] = {
    { DBF_TYPE_UINT32, 1, offsetof(uartIF_t, baudRate),
        "Baud Rate" },
    { DBF_TYPE_LIST8, 1, offsetof(uartIF_t, mode),
        "Mode", DBFL { UART_MODE_STRINGS } },
    { DBF_TYPE_UINT16, 1, offsetof(uartIF_t, respWaitTime),
        "Resp Wait Time [msec]" },
    { DBF_TYPE_HEX16, 1, offsetof(uartIF_t, destAddr),
        "Dest Addr"},
    { DBF_TYPE_BOOLEAN8, 1, offsetof(uartIF_t, loopback),
        "Internal Loopback" },
    { 0 },
};

static void uartConfigNotify(int type, void *argPtr, int record)
{
    if (type == NOTIFY_SET) {
        MAP_UARTConfigSetExpClk(UART1_BASE, MAP_SysCtlClockGet(),
                            uartConfig.baudRate,
            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

        HWREGBITW(UART1_BASE + UART_O_CTL, 7) = (uartConfig.loopback)? 1 : 0;
    }
}

/* UART STATUS ****************************************************************/

typedef struct {
    unsigned txMsgs;
    unsigned rxMsgs;
} uartStatus_t;

static uartStatus_t uartStatus;

static const dbtd_t uartStatusDBTD = {
    DBTID_UART_STATUS,
    1,
    "UART1 Status",
    sizeof(uartStatus_t),
    DBT_TYPE_RAM,
};

static const dbfd_t uartStatusDBFD[] = {
    { DBF_TYPE_UINT32, 1, offsetof(uartStatus_t, txMsgs), "Tx Msgs" },
    { DBF_TYPE_UINT32, 1, offsetof(uartStatus_t, rxMsgs), "Rx Msgs" },
    { 0 },
};

/* UART WRITE *****************************************************************/

#define UART_BUFFER_SIZE 256

typedef struct {
    uint8_t buffer[UART_BUFFER_SIZE];
    uint8_t inIndex;
    uint8_t outIndex;
    uint8_t length;
} uartWrite_t;

static uartWrite_t uartWrite;

static const dbtd_t uartWriteDBTD = {
    DBTID_UART_WRITE,
    1,
    "UART1 Write",
    sizeof(uartWrite_t),
    DBT_TYPE_RAM,
};

static const dbfd_t uartWriteDBFD[] = {
    { DBF_TYPE_UINT8, 1, offsetof(uartWrite_t, length), "Length" },
    { DBF_TYPE_HEX8, 16, offsetof(uartWrite_t, buffer[16 * 0]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartWrite_t, buffer[16 * 1]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartWrite_t, buffer[16 * 2]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartWrite_t, buffer[16 * 3]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartWrite_t, buffer[16 * 4]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartWrite_t, buffer[16 * 5]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartWrite_t, buffer[16 * 6]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartWrite_t, buffer[16 * 7]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartWrite_t, buffer[16 * 8]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartWrite_t, buffer[16 * 9]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartWrite_t, buffer[16 * 10]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartWrite_t, buffer[16 * 11]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartWrite_t, buffer[16 * 12]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartWrite_t, buffer[16 * 13]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartWrite_t, buffer[16 * 14]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartWrite_t, buffer[16 * 15]), "Data" },
    { DBF_TYPE_UINT8, 1, offsetof(uartWrite_t, inIndex), "In Index" },
    { DBF_TYPE_UINT8, 1, offsetof(uartWrite_t, outIndex), "Out Index" },
    { 0 },
};

static void uartWriteNotify(int type, void *argPtr, int record)
{
    if (type == NOTIFY_SET) {
        uart1Write((uint8_t *) uartWrite.buffer, uartWrite.length);
    }
}

/* UART READ ******************************************************************/

    /*
     * This table is a raw view into the rx ring buffer.
     */

#define UART_BUFFER_MASK (UART_BUFFER_SIZE - 1)

typedef struct {
    volatile uint8_t buffer[UART_BUFFER_SIZE];
    volatile uint8_t inIndex;
    volatile uint8_t outIndex;
    volatile uint8_t length;
} uartRead_t;

static uartRead_t uartRead;

static const dbtd_t uartReadDBTD = {
    DBTID_UART_READ,
    1,
    "UART1 Read",
    sizeof(uartRead_t),
    DBT_TYPE_RAM,
};

static const dbfd_t uartReadDBFD[] = {
    { DBF_TYPE_UINT8, 1, offsetof(uartRead_t, length), "Length" },
    { DBF_TYPE_HEX8, 16, offsetof(uartRead_t, buffer[16 * 0]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartRead_t, buffer[16 * 1]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartRead_t, buffer[16 * 2]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartRead_t, buffer[16 * 3]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartRead_t, buffer[16 * 4]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartRead_t, buffer[16 * 5]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartRead_t, buffer[16 * 6]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartRead_t, buffer[16 * 7]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartRead_t, buffer[16 * 8]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartRead_t, buffer[16 * 9]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartRead_t, buffer[16 * 10]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartRead_t, buffer[16 * 11]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartRead_t, buffer[16 * 12]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartRead_t, buffer[16 * 13]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartRead_t, buffer[16 * 14]), "Data" },
    { DBF_TYPE_HEX8, 16, offsetof(uartRead_t, buffer[16 * 15]), "Data" },
    { DBF_TYPE_UINT8, 1, offsetof(uartRead_t, inIndex), "In Index" },
    { DBF_TYPE_UINT8, 1, offsetof(uartRead_t, outIndex), "Out Index" },
    { 0 },
};

/*******************************************************************************
*
* uart1Write
*
* This routine writes bytes to the tx fifo, waiting on an event when full.
*
* RETURNS: Number of bytes written.
*
*******************************************************************************/
TX_EVENT_FLAGS_GROUP eventFlagsUart1;

int uart1Write(uint8_t *buffer, unsigned length)
{
    int len  = length;
    int size = ((uint8_t) (uartWrite.inIndex - uartWrite.outIndex) + len);

    if (!size || size >= UART_BUFFER_SIZE)
        return ERROR;

    if (uartConfig.mode == UART_MODE_CMD_RESP) {
        uartRead.inIndex  = 0;
        uartRead.outIndex = 0;
    }

    while (len--) {
        uartWrite.buffer[uartWrite.inIndex] = *buffer++;
        uartWrite.inIndex = (uartWrite.inIndex + 1) & (UART_BUFFER_MASK);
    }

    MAP_IntDisable(INT_UART1);
    while (MAP_UARTSpaceAvail(UART1_BASE) && (uartWrite.inIndex
                                                       != uartWrite.outIndex)) {
        MAP_UARTCharPut(UART1_BASE, uartWrite.buffer[uartWrite.outIndex]);
        uartWrite.outIndex = (uartWrite.outIndex + 1) & (UART_BUFFER_MASK);
    }
    MAP_IntEnable(INT_UART1);

    return length;
}

/*******************************************************************************
*
* uart1Read
*
* This routine reads bytes from the rx ring buffer, and waits on an event
* when empty.
*
* RETURNS: Byte from ring buffer.
*
*******************************************************************************/
#define UART_RX_TIMEOUT 0x55550000

#define EVENT_RX_FIFO_LEVEL BIT_0
#define EVENT_RX_TIMEOUT    BIT_1
/*      EVENT_RESPONSE_REQUESTED                      * defined in hardware.h */

static uint32_t uart1Read(void)
{
    ULONG events = 0;

    while (uartRead.inIndex == uartRead.outIndex) {
        int status;
        status = tx_event_flags_get(&eventFlagsUart1,
                                         EVENT_RX_FIFO_LEVEL | EVENT_RX_TIMEOUT,
                                                      TX_OR_CLEAR, &events, 10);
        assert((status == TX_SUCCESS) || (status == TX_NO_EVENTS));

        if (status == TX_NO_EVENTS)
            return UART_RX_TIMEOUT;
    }

    uint32_t data = uartRead.buffer[uartRead.outIndex];
    uartRead.outIndex = (uartRead.outIndex + 1) & UART_BUFFER_MASK;

    return data;
}

/*******************************************************************************
* UART1 INTERRUPT HANDLER
*******************************************************************************/
void isr_uart1(void)
{
    unsigned flags = MAP_UARTIntStatus(UART1_BASE, TRUE);
    int status;

    if (flags & UART_INT_TX) {
        MAP_UARTIntClear(UART1_BASE, UART_INT_TX);

        while (MAP_UARTSpaceAvail(UART1_BASE) && (uartWrite.inIndex
                                                       != uartWrite.outIndex)) {
            MAP_UARTCharPut(UART1_BASE, uartWrite.buffer[uartWrite.outIndex]);
            uartWrite.outIndex = (uartWrite.outIndex + 1) & (UART_BUFFER_MASK);
        }
    }

    if (flags & (UART_INT_RX | UART_INT_RT)) {
        if (uartRead.inIndex == uartRead.outIndex) {
            if (flags & UART_INT_RX) {
                status = tx_event_flags_set(&eventFlagsUart1,
                                                    EVENT_RX_FIFO_LEVEL, TX_OR);
                assert(status == TX_SUCCESS);
            }

            if (flags & UART_INT_RT) {
                status = tx_event_flags_set(&eventFlagsUart1, EVENT_RX_TIMEOUT,
                                                                         TX_OR);
                assert(status == TX_SUCCESS);
            }

            uartRead.length = 0;
        }

        while (MAP_UARTCharsAvail(UART1_BASE)) {
            uartRead.buffer[uartRead.inIndex] = MAP_UARTCharGet(UART1_BASE);
            uartRead.inIndex = (uartRead.inIndex + 1) & UART_BUFFER_MASK;
            ++uartRead.length;
        }
    }
} REGISTER_INTERRUPT(isr_uart1);

/*******************************************************************************
*
* UART Task
*
* Depending on the "mode" of the UART when it's initialized, one of the two
* task below is started.
*
* See the comment at the top of this file, and uart1Init() below.
*
* RETURNS: Nothing.
*
*******************************************************************************/
static TX_THREAD uartTid;
static unsigned uartTaskStack[511] __attribute__((aligned(8)));

static void uartCmdRespTask(ULONG arg)
{
    for (;;) {
        int status;
        ULONG events;

        status = tx_event_flags_get(&eventFlagsUart1, EVENT_RESPONSE_REQUESTED,
                                         TX_OR_CLEAR, &events, TX_WAIT_FOREVER);
        assert(status == TX_SUCCESS);

        tx_thread_sleep(uartConfig.respWaitTime);
        int len = uartRead.inIndex - uartRead.outIndex;
        wscpMsg_t *msgPtr = moduleMsgBufferReserve('0');
        int i;
        for (i = 0; i < len; i++) {
            msgPtr->data[i] = uartRead.buffer[uartRead.outIndex];
            uartRead.outIndex = (uartRead.outIndex + 1) & UART_BUFFER_MASK;
        }

        moduleSendResp(module, uartConfig.destAddr, CAN_CMD_UART1_READ, msgPtr,
                                                                           len);
        ++uartStatus.rxMsgs;
    }
}

static void uartAsyncTask(ULONG arg)
{
    uint32_t data;
    uint8_t buffer[8];
    uint8_t bufferCnt = 0;
    bool rxTimeout = FALSE;

    for (;;) {
        if ((data = uart1Read()) == UART_RX_TIMEOUT)
            rxTimeout = TRUE;
        else
            buffer[bufferCnt++] = data;

        if ((bufferCnt == 6) || (rxTimeout && bufferCnt)) {
            wscpMsg_t *msgPtr = moduleMsgBufferReserve('0');
            memcpy(msgPtr->data, buffer, bufferCnt);
            moduleSendMsg(module, uartConfig.destAddr, CAN_CMD_UART1_READ,
                                                             msgPtr, bufferCnt);
            ++uartStatus.rxMsgs;
            bufferCnt = 0;
            rxTimeout = FALSE;
        }
    }
}

/*******************************************************************************
*
* uart1Init
*
* This routine initializes UART1 and launches one of two tasks depending
* on the "mode" of the UART (uartConfig.mode).
*
* It's assumed that this routine will be called by an application layer module,
* and only called once.
*
* RETURNS: Nothing.
*
*******************************************************************************/
void uart1Init(uartIF_t *uartIF)
{
    assert(uartIF);
    assert(uartIF->mode < UART_MODE_LAST);

    memcpy(&uartConfig, uartIF, sizeof(uartIF_t));
    uartConfigNotify(NOTIFY_SET, 0, -1);

    static bool firstTime = TRUE;

    if (firstTime) {
        void (*uartTask)(ULONG) = (uartConfig.mode == UART_MODE_CMD_RESP)?
                                                uartCmdRespTask : uartAsyncTask;
        int status;
        status = tx_thread_create(&uartTid, "UART1", uartTask, 0,
                                  uartTaskStack, sizeof(uartTaskStack),
                                  TASK_PRI_UART_TX, TASK_PRI_UART_TX,
                                  TX_NO_TIME_SLICE, TX_AUTO_START);
        assert(status == TX_SUCCESS);
    }

    firstTime = FALSE;
}

/*******************************************************************************
* uart1Startup
*******************************************************************************/
static int uart1Startup(unsigned moduleNum)
{
    module = moduleNum;

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    MAP_GPIOPinConfigure(GPIO_PD2_U1RX | GPIO_PD3_U1TX);
    MAP_GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    MAP_UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX1_8, UART_FIFO_RX4_8);
    MAP_UARTFIFOEnable(UART1_BASE);
    MAP_UARTEnable(UART1_BASE);
    MAP_UARTIntEnable(UART1_BASE, UART_INT_TX | UART_INT_RX | UART_INT_RT);
    MAP_IntEnable(INT_UART1);

    dbRegister(moduleNum, &uartConfigDBTD, uartConfigDBFD,
                       0, &uartConfig, 1, &uartConfigDflt, uartConfigNotify, 0);

    uartConfigNotify(NOTIFY_SET, 0, -1);

    dbRegister(moduleNum, &uartStatusDBTD, uartStatusDBFD,
                                                    0, &uartStatus, 1, 0, 0, 0);

    dbRegister(moduleNum, &uartWriteDBTD, uartWriteDBFD,
                                       0, &uartWrite, 1, 0, uartWriteNotify, 0);

    dbRegister(moduleNum, &uartReadDBTD, uartReadDBFD, 0, &uartRead, 1, 0, 0,0);

    int status;
    status = tx_event_flags_create(&eventFlagsUart1, "UART1 Events");
    assert(status == TX_SUCCESS);

    return 0;
}
#endif
