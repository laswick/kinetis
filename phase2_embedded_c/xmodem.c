/*****************************************************************************
 * XMODEM File Transfer Module
 *
 * This module is a receive only implemenation of the xmodem protocol
 * defined by Chuck Forsberg. www.textfiles.com/programming/ymodem.txt
 *
 ****************************************************************************/
#include "globalDefs.h"
#include "kinetis.h"
#include "hardware.h"

#include "xmodem.h"

#define SOH 0x01
#define STX 0x02
#define EOT 0x04
#define ACK 0x06
#define NAK 0x15
#define CAN 0x18
#define CRC 0x43

typedef enum {
    STATE_UNINITIALIZED,
    STATE_WAITING,
    STATE_RECEIVING,
} xmodemState_t;

typedef struct {
    uartIF_t uart;
    xmodemState_t state;
    uint32_t numRetries;
    uint32_t sequence;
    uint8_t buffer[1024];
} xmodem_t;

static xmodem_t xmodem;

static void flushBuffer(void)
{
    int i;
    for (i = 0; i < 1024; i++)
        xmodem.buffer[i] = 0;
}

/******************************************************************************
 * initCRC
 *
 * Calculates XMODEM CRC lookup table to save ROM space
 * See document "Calculating the XMODEM CRC" by J. LeVan
 * http://www.std.com/obi/Standards/FileTransfer/XMODEM-CRC.NOTE.1
 *
 *****************************************************************************/
static uint16_t crctab[256];
static void initCRC(void)
{
    uint16_t count;
    for (count = 0; count < 256; count++) {
        uint16_t crc = (count << 8) ^ 0;
        int i;
        for (i = 0; i < 8; i++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
        crctab[count] = crc;
    }
}

/******************************************************************************
 * calcCRC
 *
 * Calculates XMODEM CRC using a pre-generated lookup table
 *
 * RETURNS: CRC value
 *****************************************************************************/
static int16_t calcCRC(uint8_t *dataPtr, int numBytes)
{
    int16_t crc = 0;
    while (numBytes--)
        crc = (crc << 8) ^ crctab[(crc >> 8) ^ *dataPtr++];

    return crc;
}

/******************************************************************************
 * xmodemInit
 *
 * Initializes XMODEM module. Requires pointer to initialized uart
 *
 * RETURNS: OK/ERROR
 *****************************************************************************/
int32_t xmodemInit(xmodemCfg_t *cfg)
{
    int32_t returnVal;

    if (cfg->uartPtr) {
        xmodem.uart = *cfg->uartPtr;
        xmodem.numRetries = cfg->numRetries;
        xmodem.state = STATE_WAITING;
        initCRC();
        returnVal = !ERROR;
    }
    else {
        returnVal = ERROR;
    }

    return returnVal;
}

/******************************************************************************
 * xmodemAbort
 *
 * Abort transmission if in process
 *
 * RETURNS: OK/ERROR
 *****************************************************************************/
int32_t xmodemAbort(void)
{
    uint8_t cmd = CAN;

    if (xmodem.state == STATE_UNINITIALIZED)
        return ERROR;

    if (xmodem.state != STATE_WAITING) {
        xmodem.state  = STATE_WAITING;
        uartWrite(&xmodem.uart, &cmd, 1);
        uartWrite(&xmodem.uart, &cmd, 1);
    }

    return OK;
}

/******************************************************************************
 * xmodemRecv
 *
 * Receives a CRC xmodem packet (1024 or 128 bytes).
 *
 * RETURNS: ERROR or numBytes of received. If 0 then transmission is complete
 *****************************************************************************/
int32_t xmodemRecv(uint8_t *outBuffer, uint32_t numBytes)
{
    uartIF_t *uartPtr = &xmodem.uart;
    int32_t returnVal = ERROR;
    uint8_t cmd = 0;
    int32_t retry;
    bool32_t done;

    assert(numBytes > 0);

    if (numBytes > 1024)
        numBytes = 1024;

    switch (xmodem.state) {
    case STATE_WAITING:
        /* Attempt to initiate transfer. CRC mode only */
        xmodem.sequence = 1;
        cmd = CRC;
        break;
    case STATE_RECEIVING:
        /* Request next packet from sender */
        xmodem.sequence = (xmodem.sequence + 1) % 0x100;
        cmd = ACK;
        break;
    case STATE_UNINITIALIZED:
        return ERROR;
    }

    flushBuffer();

//  uartPtr->timeout = 1000;
    uartWrite(uartPtr, &cmd, 1);

    done  = FALSE;
    retry = xmodem.numRetries;
    while (retry-- && !done) {
        uint32_t packetSize;

        if (uartRead(uartPtr, xmodem.buffer, 1) <= 0)
            continue;

        switch (xmodem.buffer[0]) {
        case SOH:
            packetSize = 128;
            break;
        case STX:
            packetSize = 1024;
            break;
        case EOT:
            packetSize = 0;
            returnVal  = 0;
            break;
        case CAN:
        default:
            packetSize = 0;
            returnVal  = ERROR;
            break;
        }

        if (packetSize) {
            uint8_t seq1, seq2;

            uartRead(uartPtr, &seq1, 1);
            uartRead(uartPtr, &seq2, 1);

            /* Verify sequence number is what's expected */
            if (seq1 == xmodem.sequence && (0xff - seq2) == xmodem.sequence) {
                uint16_t crc;
                uartRead(uartPtr, xmodem.buffer, packetSize);
                uartRead(uartPtr, (uint8_t *)&crc, 2);
                if (calcCRC(xmodem.buffer, packetSize) != crc) {
                    int i;
                    /* Copy data to output buffer */
                    if (numBytes > packetSize)
                        numBytes = packetSize;

                    for (i = 0; i < numBytes; i++) {
                        outBuffer[i] = xmodem.buffer[i];
                    }
                    returnVal = numBytes;
                    done = TRUE;
                }
            }
        }
        else {
            done = TRUE;
        }

        /* Request message resend */
        if (retry && !done) {
            flushBuffer();
            cmd = NAK;
            uartWrite(uartPtr, &cmd, 1);
        }
    }

    if (returnVal == 0) {
        /* Send ACK to end transfer */
        cmd = ACK;
//      uartPtr->timeout = 1000;
        uartWrite(uartPtr, &cmd, 1);
        xmodem.state = STATE_WAITING;
    }
    else if (returnVal > 0) {
        xmodem.state = STATE_RECEIVING;
    }
    else { /* Error */
        if (xmodem.state != STATE_WAITING) {
            /* Cancel transfer */
            xmodemAbort();
            xmodem.state = STATE_WAITING;
        }
    }

    return returnVal;
}
