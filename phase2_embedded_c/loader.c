#include <stdio.h>
#include <string.h>

#include "kinetis.h"
#include "hardware.h"
#include "globalDefs.h"
#include "util.h"
#include "xmodem.h"

static flashConfig_t flashConfig;

/*****************************************************************************
 * SREC Parser Functions
 ****************************************************************************/
typedef struct {
    bool32_t eof;
    uint32_t address;
    uint32_t numBytes;
    uint8_t  data[255];
} srecData_t;

static uint8_t atoh(uint8_t in) {
    if (in <= '9')
        return (in & 0xf);
    else if (in <= 'F')
        return ((in - 'A') + 0xa);
    else
        return in;
}

/*****************************************************************************
 * srecASCIIToHex
 *      Converts an input array from ASCII Hex to actual hex
 *
 *  RETURNS: len of the modified input array
 ****************************************************************************/
static int srecASCIIToHex(uint8_t *dataPtr, int len)
{
    int hexLen = 0;
    int i;

    assert((len % 2) == 0);

    for (i = 0; i < len; i += 2) {
        if (dataPtr[i] == 'S') {
            dataPtr[hexLen++] = 'S';
            dataPtr[hexLen++] = atoh(dataPtr[i + 1]);
        }
        else if ((dataPtr[i] >= '0' && dataPtr[i] <= '9')
              || (dataPtr[i] >= 'A' && dataPtr[i] <= 'F')) {
            uint8_t hex;
            hex  = atoh(dataPtr[i]) << 4;
            hex |= atoh(dataPtr[i + 1]);
            dataPtr[hexLen++] = hex;
        }
    }
    return hexLen;
}

/*****************************************************************************
 * srecValid
 *      The input array is parsed and checked to make sure the record is
 *  formatted correctly and the checksum matches
 *
 *  RETURNS: TRUE/FALSE
 ****************************************************************************/
static int srecValid(uint8_t *dataPtr, int len)
{
    int valid = FALSE;

    if (dataPtr[0] == 'S' && len > 6) {
        int sum = 0;
        int i;

        for (i = 2; i < len - 1; i++)
            sum += dataPtr[i];
        sum = ~sum & 0xff;

        if (sum == dataPtr[len - 1]) {
            switch (dataPtr[1]) {
            case 0:
            case 1:
            case 2:
            case 3:
            case 5:
            case 7:
            case 8:
            case 9:
                valid = TRUE;
                break;
            }
        }
    }

    return valid;
}

/*****************************************************************************
 * srecParse
 *      Each byte in the input buffer is parsed until a complete SREC record
 *  is present in the functions internal static buffer. This allows records
 *  spread over mutple consecutive buffers to be parsed correctly.
 *
 *  RETURNS: Number of bytes processed from input array. Parsed SREC record
 *  if available. If there wasn't sufficient data to complete the record the
 *  numBytes field in the output data structure is 0. EOF field indicates
 *  if the SREC file is complete
 *
 ****************************************************************************/
static int srecParse(uint8_t *data, int len, srecData_t *srecOut)
{
    static uint8_t buffer[255 + 3]; /* Max data bytes + header bytes */
    static int pos;

    bool32_t fullRecord = FALSE;
    uint8_t *dataPtr = data;
    int recordLen = 0;
    int returnVal;

    assert(len > 0);
    assert(srecOut);

    while (len-- && !fullRecord) {
        if (recordLen == 0 && pos > 2) { /* Record length known at this point */
            recordLen  = 3 + buffer[2];
            assert(recordLen >= 6);
        }
        if (recordLen && pos >= recordLen) {
            fullRecord = TRUE;
        }
        else {
            if (pos == 0)
                assert(*dataPtr == 'S');

            assert(pos < 255);
            buffer[pos++] = *dataPtr++;
        }
    }

    srecOut->eof = FALSE;
    if (fullRecord) {
        pos = 0; /* Reset position counter */
        if (srecValid(buffer, recordLen)) {
            switch(buffer[1]) {
            case 0:
            case 5:
                srecOut->address  = 0;
                srecOut->numBytes = 0;
                break;
            case 1:
                srecOut->address   = buffer[3] << 8;
                srecOut->address  |= buffer[4];
                srecOut->numBytes  = recordLen - 6;
                memcpy(srecOut->data, &buffer[5], srecOut->numBytes);
                break;
            case 2:
                srecOut->address   = buffer[3] << 16;
                srecOut->address  |= buffer[4] << 8;
                srecOut->address  |= buffer[5];
                srecOut->numBytes  = recordLen - 7;
                memcpy(srecOut->data, &buffer[6], srecOut->numBytes);
                break;
            case 3:
                srecOut->address   = buffer[3] << 24;
                srecOut->address  |= buffer[4] << 16;
                srecOut->address  |= buffer[5] <<  8;
                srecOut->address  |= buffer[6];
                srecOut->numBytes  = recordLen - 8;
                memcpy(srecOut->data, &buffer[7], srecOut->numBytes);
                break;
            case 7:
            case 8:
            case 9:
                srecOut->eof = TRUE;
                srecOut->address  = 0;
                srecOut->numBytes = 0;
                break;
            }
            returnVal = dataPtr - data;
        }
        else {
            returnVal = ERROR;
        }
    }
    else {
        srecOut->address  = 0;
        srecOut->numBytes = 0;
        returnVal = dataPtr - data;
    }

    return returnVal;
}

/****************************************************************************
 * errorLED
 *      Indicate an unrecoverable error has occured using the orange led
 ***************************************************************************/
static void errorLED(void)
{
    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioClear (N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
    gpioSet   (N_LED_BLUE_PORT,   N_LED_BLUE_PIN);
    gpioSet   (N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
    gpioSet   (N_LED_GREEN_PORT,  N_LED_GREEN_PIN);
    while(1)
        ;
}

/****************************************************************************
 * jumpToApp
 *      Jumps to the application code.
 ***************************************************************************/
static void jumpToApp(void)
{
    gpioSet(N_LED_GREEN_PORT, N_LED_GREEN_PIN);
    /* NVIC offset register has to be 64bit word aligned as per the ARM TRM
     * in order to support a maximum of 128 vectors. Rather then waste flash
     * space implementing the application code address to start at this
     * boundary use the flash IRQ's from this loader. It is up to
     * the applications startup code to remap its vector map into
     * SRAM and update VTOR */
    NVIC_VTOR_SET(0);
    asm volatile (
        "ldr r1, =_app_code_start\n\t"  /* TODO: pass in start address */
        "add r1,r1,#4\n\t"
        "ldr r0,[r1]\n\t"
        "bx r0"
    );
    /* Should be running in application land now.
     * Getting here is bad news bears */
    assert(FALSE);
    errorLED();
}


/*****************************************************************************
 * UART Loader Menu
 *
 *      Erases flash and receives a new SREC image to parse and program
 *  over UART3. Supports XMODEM interface only
 ****************************************************************************/
static uartIF_t uartConfig = {
    .uart             = UART3,
    .systemClockHz    = 20480000,
    .busClockHz       = 20480000,
    .baud             = 115200,
    .responseWaitTime = 250,
};
static xmodemCfg_t xmodemConfig = {
    .uartPtr    = &uartConfig,
    .numRetries = 50,
};

static void uartLoader(uint32_t appCodeAddr, uint32_t appCodeSize)
{
    static uint8_t rxBuffer[1024];
    static srecData_t srecData;

    if (flashErase(appCodeAddr, appCodeSize) == ERROR) {
        uartPrint(&uartConfig, "Flash Failed to erase\r\n");
        errorLED();
    }
    uartPrint(&uartConfig, "Flash Erased. Waiting for XMODEM Transfer\r\n");
    gpioSet(N_LED_GREEN_PORT,  N_LED_GREEN_PIN);
    gpioSet(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);

    while (1) {
        int len = xmodemRecv(rxBuffer, sizeof(rxBuffer));

        if (len == 0) {
            uartPrint(&uartConfig, "\r\nProgramming succesfull!\r\n");
            break;
        }
        else if (len > 0) {
            int offset = 0;
            gpioClear(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);

            len = srecASCIIToHex(rxBuffer, len);
            while (len) {
                int value = srecParse(&rxBuffer[offset], len, &srecData);
                if (value == ERROR) {
                    xmodemAbort();
                    uartPrint(&uartConfig, "\r\nSREC FILE ERROR!\r\n");
                    uartPrint(&uartConfig, "Waiting for re-transfer...\r\n");
                    break;
                }
                else {
                    if (srecData.numBytes) {
                        flashWrite(srecData.address, (uint32_t *)srecData.data,
                                                         srecData.numBytes / 4);
                    }
                    offset += value;
                    len -= value;
                }
            }
        }
        else {
            delay();
        }

        gpioToggle(N_LED_BLUE_PORT, N_LED_BLUE_PIN);
    }

    gpioSet  (N_LED_BLUE_PORT,   N_LED_BLUE_PIN);
    gpioSet  (N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
    gpioClear(N_LED_GREEN_PORT,  N_LED_GREEN_PIN);
}

/*****************************************************************************
 * Loader Menu
 ****************************************************************************/
extern uint32_t _app_code_start;
int main(void)
{
    uint32_t appCodeAddr = (uint32_t)&_app_code_start;
    uint32_t appCodeSize = 0x20000; /* 128K by default */
    bool32_t codePresent = (*(volatile uint32_t *)appCodeAddr) != 0xffffffff;
    uint8_t menuSel = 0;

    gpioConfig(N_LED_BLUE_PORT,   N_LED_BLUE_PIN,   GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_GREEN_PORT,  N_LED_GREEN_PIN,  GPIO_OUTPUT | GPIO_HIGH);
    gpioConfig(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN, GPIO_OUTPUT | GPIO_HIGH);
    gpioConfig(N_SWITCH_1_PORT,   N_SWITCH_1_PIN,   GPIO_INPUT);

    /* Jump to application code if its present and SWITCH_1 isn't pressed */
    if (codePresent && gpioRead(N_SWITCH_1_PORT, N_SWITCH_1_PIN) != 0) {
        jumpToApp();
        assert(FALSE);
        errorLED();
    }

    /* Initialize peripherals */
    if (uartInit(&uartConfig) == ERROR || xmodemInit(&xmodemConfig) == ERROR
                                       || flashInit(&flashConfig) == ERROR) {
        errorLED();
    }

    /* Loader Menu
     * TODO: Add options to display and modify code address + size
     *       Requires varg printf */
    while (1) {
        codePresent = (*(volatile uint32_t *)appCodeAddr) != 0xffffffff;

        uartPrint(&uartConfig, "\r\nLoader Menu\r\n");
        uartPrint(&uartConfig, "1. Load new SREC image from UART\r\n");
        if (codePresent) {
            uartPrint(&uartConfig, "2. Jump to application\r\n");
        }

        while (uartRead(&uartConfig, &menuSel, 1) < 1)
            ;
        uartWrite(&uartConfig, &menuSel, 1);
        uartPrint(&uartConfig, "\r\n");

        switch (menuSel) {
        case '1':
            uartLoader(appCodeAddr, appCodeSize);
            break;
        case '2':
            if (codePresent) {
                uartPrint(&uartConfig, "Jumping to application. Bye Bye\r\n");
                jumpToApp();
                assert(FALSE);
                errorLED();
            }
            /* No Break */
        default:
            uartPrint(&uartConfig, "Invalid Selection\r\n");
            break;
        }
    }

    return 0;
}
