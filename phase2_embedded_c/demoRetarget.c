/*******************************************************************************
*
* demoRetarget.c
*
* Simple demonstration of the configuration and use of the standard I/O
* streams.
*
* XXX   NOTE: If you're getting unexpected results on your terminal, ensure
*             your terminal is configured correctly.  i.e. ensure it
*             interprets CR as CR LF (and/or vise versa) when receiving data,
*             and ensure you know how to send a full CR LF as well, i.e. I'm
*             using putty, and to satisfy fgets() you need to use <ctrl><j>,
*             not <enter>, as <enter> only sends LF.
*
* Rob Laswick
* Sept 17 2012
*
* Copyright (C) 2012 www.laswick.net
*
* This program is free software.  It comes without any warranty, to the extent
* permitted by applicable law.  You can redistribute it and/or modify it under
* the terms of the WTF Public License (WTFPL), Version 2, as published by
* Sam Hocevar.  See http://sam.zoy.org/wtfpl/COPYING for more details.
* Canada Day 2012
*
*******************************************************************************/
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include "kinetis.h"

#include "hardware.h"
#include "globalDefs.h"
#include "util.h"

int main(void)
{
    /*
     * Install/Register the UART Driver.
     *
     * I would prefer a common driver installer, i.e. hwDriverInstall(), etc.
     */

    uart_install();

    /*
     * Register the standard I/O streams with a particular deivce driver.
     */

    int fd0 = fdevopen(stdin,  "uart3", 0, 0);
    int fd1 = fdevopen(stdout, "uart3", 0, 0);
    int fd2 = fdevopen(stderr, "uart3", 0, 0);

    assert(fd0 != -1);
    assert(fd1 != -1);
    assert(fd2 != -1);

    /*
     * Open a "raw" device.
     */

    int fd = open("uart3", 0, 0);

    assert(fd != -1);

    /*
     * Configure the driver.
     */

    ioctl(fd0, IO_IOCTL_UART_BAUD_SET, 115200);
    ioctl(fd0, IO_IOCTL_UART_ECHO, TRUE);

    /*
     * Clear Screen.
     */

    int i;
    for (i = 0; i < 500; i++)
        puts("");

    /*
     * Banner.
     */

    puts  ("Retargeting Demo for the Kinetis Project");
    puts  ("Rob Laswick");
    printf("File: %s\n", __FILE__);
    printf("Build Date: %s\n\n", __DATE__);

    /*
     * fprintf test.
     */

    fprintf(stdout, "Printing to the STDOUT\n");
    fprintf(stderr, "Printing to the STDERR\n\n");

    /*
     * More printf.
     */

    printf("********************************************\n\n");
    printf("test 1\n");
    printf("test %d\n", 2);
    printf("test %.2f\n", 3.0);
    printf("test %.3f\n", 4.1);
    printf("%s %.4f\n\n", "test", 5.5);

    /*
     * sprintf test.
     */

    char str[200];
    sprintf(str, "%s %.2f\n", "This string was built by sprintf!", 6.6);
    puts(str);

    /*
     * scanf test.
     */

    printf("Enter your first name: ");
    scanf (" %s", str);
    printf("\nYour name is %s!\n\n", str);

    printf("Enter your last name and age: ");
    int age;
    char str2[20];
    char tmp;
    scanf ("%s %d%c", str2, &age, &tmp);
    strcat(str, " ");
    strcat(str, str2);
    printf("\nYour name is %s, and your are %d years old!\n\n", str, age);

    /*
     * fputs/fgets test.
     */

    fputs("Where are you?\n> ", stderr);
    fgets(str2, sizeof(str2), stdin);

    /*
     * fgets seems to keep CR LF in the returned buffer for
     * some reason, so strip them out.
     */

    for (i = 0; i < sizeof(str2); i++) {
        if ((str2[i] == '\r') || (str2[i] == '\n'))
            str2[i] = 0;
    }

    sprintf(str, "\nYou are at %s!\n\n", str2);
    fputs(str, stdout);

    /*
     * Stream sharing.
     *
     * Contrary to Dr.Egon Spengler, crossing the streams is just fine.
     */

    sprintf(str, "You can still \"write\" to a device that's also\n" \
                 "currently being shared with the standard I/O streams ;)\n\n");
    write(fd, str, strlen(str));

    /*
     * iprintf test.
     */

    iprintf("note: iprintf is lighter weight than printf: %d\n\n", 77);

    puts("End of demo :)\n\n");

    /*
     * Book keeping.
     *
     * Note: (by implementation) You can't actually close a device that's
     *       currently associated with the standard I/O streams.
     */

    close(fd0);
    close(fd1);
    close(fd2);

    close(fd);

    return 0;
}

