/*******************************************************************************
*
* demoRetarget.c
*
* Simple demonstration of the configuration and use of the standard I/O
* streams.
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

    /*
     * Clear Screen.
     */

    int i;
    for (i = 0; i < 500; i++)
        puts("");

    /*
     * Banner.
     */

    puts("Retargeting Demo for the Kinetis Project");
    puts("Rob Laswick");
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
    printf("test %.2f\n", 4.1);
    printf("%s %.2f\n\n", "test", 5.5);

    /*
     * sprintf test.
     */

    char str[200];
    sprintf(str, "%s %.2f\n", "This string was build by sprintf!", 6.6);
    puts(str);

    /*
     * scanf test.
     *
     * FIXME  This broken at the moment.  The uart read routine returns
     *        immediately for some reason.
     */

    printf("Enter your name: ");
    scanf("%s", str);
    printf("\nYour name is %s!\n\n", str);

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

    iprintf("iprintf is lighter weight than printf: %d\n\n", 77);

    puts("...end of demo...\n\n");

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

