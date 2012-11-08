/*******************************************************************************
*
* demoSdhc.c
*
* Geoff Chapman 
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
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <ctype.h>
#include "kinetis.h"
#include "hardware.h"
#include "globalDefs.h"
#include "util.h"

static unsigned int buffer[128];   /* 128 words = 512 bytes */

/*******************************************************************************
*
*******************************************************************************/
void clearBuffer(unsigned int buffer[], int n)
{
    int i;

    for (i = 0; i < n; i++) {
        buffer[i] = 0;
    }
}

void reverse(char s[])
{
    int c, i, j;

    for (i = 0, j = strlen(s)-1; i < j; i++, j--) {
        c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
}

void itoa(int n, char s[])
{
    int i, sign;

    if ((sign = n) < 0)
        n = -n;
    i = 0;
    do {
        s[i++] = n % 10 + '0';
    } while ((n /= 10) > 0);
    if (sign < 0)
        s[i++] = '-';
    s[i] = '\0';
    reverse(s);
}

char *mfgStr[5]     = {"Unknown", "Unknown", "Toshiba", "Sandisk", "Unknown"};

char *monthStr[12]  = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug",
                       "Sep", "Oct", "Nov", "Dec" };

/*******************************************************************************
*
*******************************************************************************/
int main(void)
{
    char cYear[5];
    cidReg_t cid;

    int i,j;
    char  menuSel   = 0;
    char  blkSel    = 0;
    char  rdBlkNum  = 0;
    char  wrBlkNum  = 0;
    char  eraseBlk  = 0;
    
    unsigned char *cptr;

    int   lgBlkNum  = 0;

    uart_install();
    sdhc_install();

    int fd0 = fdevopen(stdin,  "uart3", 0, 0);
    int fd1 = fdevopen(stdout, "uart3", 0, 0);
    int fd2 = fdevopen(stderr, "uart3", 0, 0);

    assert(fd0 != -1);
    assert(fd1 != -1);
    assert(fd2 != -1);

    int fdUart = open("uart3", 0, 0);

    int fdSdhc = open("sdhc0", 0, 0);

    ioctl(fdUart, IO_IOCTL_UART_BAUD_SET, 115200);

    ioctl(fdSdhc, IO_IOCTL_SDHC_GET_CID, (int) &cid);

    itoa(2000 + (cid.mfgDate[1] >> 4), cYear);

    printf("\nSD Card Data\n\r");
    printf("Manufacturer     : %s   \n\r", mfgStr[cid.mfgId]);
    printf("Manufacture Date : %s %s\n\r", monthStr[(cid.mfgDate[1] >> 0) & 0xf],
                                           cYear);
    printf("\n\r");

    while (1) {
        printf("Select: \n\r");
        printf("1. Set Read Block Number 1000000\n\r");
        printf("2. Set Read Block Number 1000001\n\r");
        printf("3. Set Read Block Number 1000002\n\r");
        printf("4. Set Read Block Number 1000009\n\r");
        printf("5. Read Block number %d  \n\r", lgBlkNum);
        printf("\n\r");
        printf("6. Set Write block Number       \n\r");
        printf("7. Set Read block Number        \n\r");
        printf("8. Write block number %d \n\r", wrBlkNum);
        printf("9. Read block number  %d \n\r", rdBlkNum);
        printf("a. Erase block           \n\r");
        printf("\n\r");

        while (read(fdUart, &menuSel, 1) < 1)
            ;
        write(fdUart, &menuSel, 1);
        printf("\n\r");

        switch(menuSel) {
        case '1':
            lgBlkNum = 1000000;
            break;
        case '2':
            lgBlkNum = 1000001;
            break;
        case '3':
            lgBlkNum = 1000002;
            break;
        case '4':
            lgBlkNum = 1000009;
            break;
        case '5':
            ioctl(fdSdhc, IO_IOCTL_SDHC_SET_READ_BLOCK, lgBlkNum);

            clearBuffer(buffer, 128);

            read(fdSdhc, buffer, 512);

            for (i = 0; i < 128; i += 4) {
                printf("%3d " , i);
                printf("%08x ", buffer[i + 0]);
                printf("%08x ", buffer[i + 1]);
                printf("%08x ", buffer[i + 2]);
                printf("%08x ", buffer[i + 3]);
                printf("\r\n");
            }
            break;
        case '6':
            while (read(fdUart, &blkSel, 1) < 1)
                ;
            wrBlkNum = blkSel - '0';
            printf("%d\n\r", wrBlkNum);
            break;
        case '7':
            while (read(fdUart, &blkSel, 1) < 1)
                ;
            rdBlkNum = blkSel - '0';
            printf("%d\n\r", rdBlkNum);
            break;
        case '8':
            clearBuffer(buffer, 128);
            ioctl(fdSdhc, IO_IOCTL_SDHC_SET_WRITE_BLOCK, wrBlkNum);
            while (read(fdUart, buffer, 1) < 1)
                ;
            printf("%x\n\r", buffer[0]);
            write(fdSdhc, buffer, 512);
            break;
        case '9':
            clearBuffer(buffer, 128);
            ioctl(fdSdhc, IO_IOCTL_SDHC_SET_READ_BLOCK, rdBlkNum);

            read(fdSdhc, buffer, 512);

            for (i = 0; i < 128; i += 4) {
                printf("%3d " , i);
                printf("%08x ", buffer[i + 0]);
                printf("%08x ", buffer[i + 1]);
                printf("%08x ", buffer[i + 2]);
                printf("%08x ", buffer[i + 3]);
                printf("  ");
                cptr = (unsigned char *) &buffer[i];
		for (j = 0; j < 16; j++) {
                    unsigned char c = *(cptr + j);
		    if (isalnum(c)) {
			printf("%c", c); 
		    }
                    else { 
			printf("."); 
                    }
		}
                printf("\r\n");
            }
            break;
        case 'a':
            while (read(fdUart, &blkSel, 1) < 1)
                ;
            eraseBlk = blkSel - '0';
            printf("%d\n\r", eraseBlk);
            ioctl(fdSdhc, IO_IOCTL_SDHC_ERASE_BLOCK, eraseBlk);
            break;
        default:
            printf("Error\n\r");
            break;
        }
    }
    
    return 0;
}
