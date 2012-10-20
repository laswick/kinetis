/*******************************************************************************
*
* loader.c
*
* Paul Quevedo
*
* UART bootloader for the Kinetis K60 chip:
* Module resides in the first 16Kbytes of flash memory. On powerup the
* loader checks to see if application code exists at the start of the next
* memory location (0x4000). If it doesn't or the user is pressing switch 1
* the loader menu will be displayed. An active serial connection over UART3
* is required to interface with the loader (baud is 115200).
* Any terminal program that supports the XMODEM file transfer protocol
* (hyperTerm, minicom etc..) can be used. Loader is expecting images to be
* in SREC format. To build an image the loader can be sent use
* linkerscript_app.ld. This just offsets the start of flash to after the boot
* space.
*
* Read the note under NVIC_VTOR_SET regarding the application codes vectors
*
* UART Application Loader:
* For systems with two flash blocks that support the flash swap feature.
* Application code resides entirely in one flash block. Call loader() to
* erase the other flash block and program it with new application code via
* uart XMODEM transfer. Loader will then logically remap the other flash
* block to address 0x0 effective after power cycling.
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
#include <unistd.h>
#include <string.h>
#include <fcntl.h>

#include "kinetis.h"
#include "hardware.h"
#include "globalDefs.h"
#include "util.h"
#include "xmodem.h"

#define CODE_INFO_SIGNATURE 0x12344321
typedef struct {
    uint32_t signature;
    uint32_t codeAddr;
    uint32_t codeSize;
    uint32_t codeLen;
    uint32_t codeChecksum;
} codeInfo_t;

#if 0 /* Redefine malloc? libc version is huge and not necessary for loader */
void *_malloc(size_t size)
{
    extern uint8_t _heap_start;
    extern uint8_t _heap_end;

    static uint8_t *ptr = &_heap_start;
    uint8_t *allocPtr;

    if ((ptr + size) > (uint8_t *)&_heap_end) {
        allocPtr = 0;
    }
    else {
        allocPtr = ptr;
        ptr += size;
    }

    return (void *)allocPtr;
}

void free(void *ptr)
{
    return;
}
#endif

/*****************************************************************************
 * print
 *      Lightweight print function to stdout
 ****************************************************************************/
static void print(char *ptr)
{
    static const char *newLine = "\r\n";
    write(STDOUT_FILENO, ptr, strlen(ptr));
    write(STDOUT_FILENO, newLine, strlen(newLine));
}

/****************************************************************************
 * errorLED
 *      Indicate an unrecoverable error has occured using the orange led
 ***************************************************************************/
static void errorLED(void)
{
    gpioClear (N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
    gpioSet   (N_LED_BLUE_PORT,   N_LED_BLUE_PIN);
    gpioSet   (N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
    gpioSet   (N_LED_GREEN_PORT,  N_LED_GREEN_PIN);
    while(1)
        ;
}

/*****************************************************************************
 * gpioInit
 *      Initializes GPIOs
 ****************************************************************************/
static void gpioInit(void)
{
    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_BLUE_PORT,   N_LED_BLUE_PIN,   GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_GREEN_PORT,  N_LED_GREEN_PIN,  GPIO_OUTPUT | GPIO_HIGH);
    gpioConfig(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN, GPIO_OUTPUT | GPIO_HIGH);
    gpioConfig(N_SWITCH_1_PORT,   N_SWITCH_1_PIN,   GPIO_INPUT);
}

/*****************************************************************************
 * periphInit
 *      Retarget std streams to 115200 baud uart.
 *      Initialize multimedia drivers
 ****************************************************************************/
static void periphInit(int retarget)
{
    flashConfig_t flashConfig;
    xmodemCfg_t   xmodemConfig = {
        .numRetries = 50,
    };

    if (retarget) {
        int fd;

        uart_install();
        fd = fdevopen(stdin,  "uart3", 0, 0);
        assert(fd == STDIN_FILENO);

        fd = fdevopen(stdout, "uart3", 0, 0);
        assert(fd == STDOUT_FILENO);

        fd = fdevopen(stderr, "uart3", 0, 0);
        assert(fd == STDERR_FILENO);

        ioctl(fd, IO_IOCTL_UART_BAUD_SET, 115200);
    }

    xmodemConfig.uartFd = STDIN_FILENO;

    if (xmodemInit(&xmodemConfig) == ERROR
      || flashInit(&flashConfig)  == ERROR) {
        errorLED();
    }
}

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

/*****************************************************************************
 *
 * validateCode
 *
 * This routine checks if an application image is present in flash.
 * It uses the data stored in the codeInfo structure. It also performs
 * a checksum on the application code to verify data integrity
 ****************************************************************************/
static bool32_t validateCode(const volatile codeInfo_t *codeInfoPtr,
                                                            uint32_t codeOffset)
{
    volatile uint8_t *codePtr;
    uint32_t checksum = 0;
    int i;

    codePtr = (volatile uint8_t *)(codeInfoPtr->codeAddr + codeOffset);

    if (codeInfoPtr->signature != CODE_INFO_SIGNATURE)
        return FALSE;

    if (codeInfoPtr->codeChecksum == 0xffffffff
          && codeInfoPtr->codeLen == 0xffffffff) {
        return FALSE;
    }

    for (i = 0; i < codeInfoPtr->codeLen; i++)
        checksum += *codePtr++;

    return (checksum == codeInfoPtr->codeChecksum);
}

/*****************************************************************************
 * UART Transfer
 *
 *      Erases flash and receives a new SREC image to parse and program
 *  over UART3. Supports CRC XMODEM 128Byte/1K interface only
 ****************************************************************************/
static int uartTransfer(codeInfo_t *codeInfoPtr, uint32_t codeOffset)
{
    static uint8_t rxBuffer[1024];
    static srecData_t srecData;

    if (codeInfoPtr) {
        codeInfoPtr->signature    = CODE_INFO_SIGNATURE;
        codeInfoPtr->codeAddr     = -1;
        codeInfoPtr->codeLen      =  0;
        codeInfoPtr->codeChecksum =  0;
    }

    print("Waiting for XMODEM Transfer");
    gpioSet(N_LED_GREEN_PORT,  N_LED_GREEN_PIN);
    gpioSet(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);

    while (1) {
        int len = xmodemRecv(rxBuffer, sizeof(rxBuffer));

        if (len == 0) {
            print("Programming succesfull!");
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
                    print("SREC FILE ERROR!");
                    print("Waiting for re-transfer");
                    break;
                }
                else {
                    if (srecData.numBytes) {
                        if (codeInfoPtr) {
                            int i;
                            /* Calculate checkum for the image being loaded */
                            codeInfoPtr->codeLen += srecData.numBytes;
                            for (i = 0; i < srecData.numBytes; i++)
                                codeInfoPtr->codeChecksum += srecData.data[i];

                            /* Record the address of where the image is loaded */
                            if (codeInfoPtr->codeAddr == -1)
                                codeInfoPtr->codeAddr  = srecData.address;
                        }

                        flashWrite(srecData.address + codeOffset,
                                                    (uint32_t *)srecData.data,
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

    return OK;
}

#if defined(BOOTLOADER)
/*****************************************************************************
 * BOOT Loader
 * writeCodeInfo
 *
 * This structure is retained at the end of the flash memory allocated for
 * boot space (defined in linkerscript_boot.ld). It retains the address of
 * where the application image was loaded to and a checksum of the code used
 * to verify image integrity on powerup
 ****************************************************************************/
static const volatile
__attribute__ ((section(".nvStorage"))) codeInfo_t codeInfo = {
    .signature    = CODE_INFO_SIGNATURE,
    .codeAddr     = 0x8000,     /* 32kB Bootloader  */
    .codeSize     = 0x40000,    /* 256kB code space */
    .codeLen      = 0xffffffff,
    .codeChecksum = 0xffffffff,
};

static void writeCodeInfo(codeInfo_t *codeInfoPtr)
{
    extern uint32_t _nvStorage_start;
    flashErase((uint32_t)&_nvStorage_start, FTFL_FLASH_SECTOR_SIZE);
    flashWrite((uint32_t)&_nvStorage_start, (uint32_t *)codeInfoPtr,
                                            sizeof(codeInfo_t) / 4);
}

/****************************************************************************
 * BOOT Loader
 * jumpToApp
 *      Jumps to the application code.
 ***************************************************************************/
static void jumpToApp(void)
{
    uint32_t stackAddr = *(uint32_t *)(codeInfo.codeAddr);
    uint32_t codeAddr  = *(uint32_t *)(codeInfo.codeAddr + 0x4);

    gpioSet(N_LED_GREEN_PORT, N_LED_GREEN_PIN);

    /* NVIC offset register has to be 64bit aligned as per the ARM TRM
     * in order to support a maximum of 128 vectors. */
    if ((codeInfo.codeAddr & 0x7)) {
        assert(FALSE);
        errorLED();
        while(1)
            ;
    }

    /* Remap vectors */
    NVIC_VTOR_SET(codeInfo.codeAddr);
    /* Update Stack Pointer */
    asm volatile  ("mov sp, %[addr]" : : [addr] "r" (stackAddr));
    /* Jump to application code */
    asm volatile  ("bx %[addr]" : : [addr] "r" (codeAddr));
    /* Should be running in application land now.
     * Getting here is bad news bears */
    assert(FALSE);
    errorLED();
}

/*****************************************************************************
 * BOOT Loader Menu
 ****************************************************************************/
extern uint32_t _app_code_start;
int main(void)
{
    static codeInfo_t newCodeInfo;
    bool32_t codePresent = validateCode(&codeInfo, 0);
    uint8_t menuSel = 0;

    gpioInit();

    /* Jump to application code if its present and SWITCH_1 isn't pressed */
    if (codePresent && gpioRead(N_SWITCH_1_PORT, N_SWITCH_1_PIN) != 0) {
        jumpToApp();
        assert(FALSE);
        errorLED();
    }

    periphInit(TRUE);

    /* Loader Menu
     * TODO: Add options to display codeInfo. Requires varg print */
    while (1) {
        codePresent = validateCode(&codeInfo, 0);

        print("Loader Menu");
        print("1. Erase Flash");
        print("2. Load new SREC image from UART");
        if (codePresent) {
            print("3. Jump to application");
        }

        while (read(STDIN_FILENO, &menuSel, 1) < 1)
            ;
        write(STDOUT_FILENO, &menuSel, 1);
        print(" ");

        switch (menuSel) {
        case '1':
            if (flashErase(codeInfo.codeAddr, codeInfo.codeSize) == ERROR){
                print("Flash Failed to erase");
                errorLED();
            }
            else {
                print("Done");
            }
            break;
        case '2':
            if ((*(uint32_t *)codeInfo.codeAddr) != 0xffffffff) {
                print("Flash not erased\r\b");
                break;
            }
            uartTransfer(&newCodeInfo, 0);
            writeCodeInfo(&newCodeInfo);
            break;
        case '3':
            if (codePresent) {
                print("Jumping to app. Bye Bye");
                jumpToApp();
                assert(FALSE);
                errorLED();
            }
            /* No Break */
        default:
            print("Invalid Selection");
            break;
        }
    }

    return 0;
}

#else
/*****************************************************************************
 * APPLICATION Loader Menu
 * Loader FSM as defined by Freescale App Note AN4533 Rev 0, 06/2012
 ****************************************************************************/
int loader(void)
{
    enum {
        STATE_INIT,
        STATE_SWAP_INIT,
        STATE_CODE_ERASED,
        STATE_CODE_LOADED,
        STATE_DONE,
    };
    static codeInfo_t codeInfo = {
        .signature = CODE_INFO_SIGNATURE,
        .codeLen      = 0xffffffff,
        .codeChecksum = 0xffffffff,
    };
    uint32_t state = STATE_INIT;
    uint32_t menuSel = 0;

    gpioInit();
    periphInit(FALSE);

    /* Loader Menu */
    while (state != STATE_DONE) {
        print("Loader Menu");
        print("1. Return to Calling Code");
        switch (state) {
        case STATE_INIT:
        case STATE_SWAP_INIT:
            print("2. Erase Flash");
            break;
        case STATE_CODE_ERASED:
            print("2. Load new SREC image from UART");
            break;
        case STATE_CODE_LOADED:
            print("2. Remap Code");
            break;
        default:
            print("?????");
            break;
        }

        while (read(STDIN_FILENO, &menuSel, 1) < 1)
            ;
        write(STDOUT_FILENO, &menuSel, 1);
        print(" ");

        switch (menuSel) {
        case '1':
            state = STATE_DONE;
            break;
        case '2':
            switch (state) {
            case STATE_INIT:
                if (flashSwapInit() == ERROR) {
                    print("Failed to Init Flash Swap");
                    break;
                }
                state = STATE_SWAP_INIT;
                /* No Break */
            case STATE_SWAP_INIT:
                if (flashEraseBlock(FLASH_BLOCK_1) == ERROR) {
                    print("Failed to Erase Flash Block");
                    break;
                }
                state = STATE_CODE_ERASED;
                break;
            case STATE_CODE_ERASED:
                uartTransfer(&codeInfo,     FTFL_FLASH_BLOCK_SIZE);
                if (validateCode(&codeInfo, FTFL_FLASH_BLOCK_SIZE) == FALSE) {
                    print("Programming Checksum Error");
                    break;
                }
                state = STATE_CODE_LOADED;
                break;
            case STATE_CODE_LOADED:
                if (flashSwap() == ERROR) {
                    print("Failed to remap flash");
                    break;
                }
                print("Flash remapped. Safe to Power Cycle");
                state = STATE_DONE;
                break;
            }
            break;
        default:
            print("Invalid Selection");
            break;
        }

        delay();
        delay();
        delay();
    }

    /* Return to calling function */
    return 0;
}
#endif
