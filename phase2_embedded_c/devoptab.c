/*******************************************************************************
*
* devoptab.c
*
* Shaun Weise
*
* POSIX Interface
*
* TODO: Shaun, Must update this mini-tutorial
*
* This was based on an excellent article by bill gatliff. It can be found at:
*   http://neptune.billgatliff.com/newlib.html2
*
* Small changes were made to his implementation, but the fundatmentals are the
* same.
*
* Usefull Docs regarding MQX POSIX drivers:
*   www.freescale.com/files/32bit/doc/user_guide/MQXIOUG.pdf
*
* Our project team decided to implement POSIX model interfaces for MOST drivers
* and devices. There are some instances where a POSIX model does not make a lot
* of sense. Ex:
*    ioctl( fd, IO_IOCTRL_PORTA_SET_BIT, BIT_5 ); //A lot of work for a GPIO
*
* In general, any I/O device driver should follow the POSIX model for its
* interface going forward. Exceptions will be discussed & decided by the
* project team.
*
* Copyright (C) 2012 www.laswick.net
*
* This program is free software.  It comes without any warranty, to the extent
* permitted by applicable law.  You can redistribute it and/or modify it under
* the terms of the WTF Public License (WTFPL), Version 2, as published by
* Sam Hocevar.  See http://sam.zoy.org/wtfpl/COPYING for more details.
*
********************************************************************************

    Steps to add your open/read/write/etc

    1) In your driver, create the following sytem call stubs:

        int  xxx_open_r(void *reent, devoptab_t *dot, int mode, int flags )
        int  xxx_close_r(void *reent, devoptab_t *dot )
        int  xxx_ioctl(devoptab_t *dot, int cmd,  int flags)
        long xxx_write_r(void *reent, devoptab_t *dot, const void *buf,int len)
        long xxx_read_r(void *reent, devoptab_t *dot, void *buf, int len )

        Just have them just return 1 initially, you'll be testing to make sure
        everything compiles and links first.

        Also, make sure to put the prototypes in hardware.h to they can be
        linked.

    2) In devoptab.c (this file) create devoptab_t entries for your device(s).

        The devoptab_t defines:
            The name of your device
            Function pointers to your system call stubs
            Initialization flags
            (See hardware.h for the struct definition)

        See DEVOPTAB Entry's section in this file.

        An example entry:
        devoptab_t devoptab_xxx   = { "xxx", xxx_open_r, xxx_ioctl, xxx_close_r,
                                             xxx_write_r, xxx_read_r, NULL    };

    3) In devoptab.c, place you devoptab_t entry into the devoptab. By doing
        this you are 'assigning' it a file descriptor, or an entry index the
        table.

        See the DEVOPTAB section in this file.


    4) In your test application, now test if the POSIX sytem calls call
        your system call stubs in your driver.

        Ex.

            fd = open("xxx", 0, 0);

            if (fd != -1) {
                //Found your devoptab_t entry !

                //Test your driver ioctl stub
                ioctl(fd, IO_IOCTL_XXX_XXX, SOME_MAGIC_NUM);

                //Test your driver write stub
                write(fd, &data, sizeof(data));

                //Test your driver read stub
                read(fd, &data, sizeof(data));

                //Test your driver close stub
                close(fd);
            }

    5) Add this file to your makefile along with your driver and test app.
        Load and run you test app putting breakpoints in your system call
        stubs. If you hit the breakpoints you are golden.

    6) Now that the POSIX system call layer is working, you can now go about
        your business of making your driver do something usefull.

        You should define your IO_IOCTL_XXX_XXX commands and any flags you use
        in hardware.h so that they can be linked.

        See spi.c for an example of how a driver implements the POSIX model and
        see demoSpi.c for an example of how to use the spi POSIX driver.

*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "kinetis.h"

#include "globalDefs.h"
#include "hardware.h"

/* Error Stuff, WIP */
/* TODO: This doesnt link. If theres a trick to this please add comment */
#if 0
#define errno (*__errno())
extern int *__errno ( void );
//static struct _reent impure_data = { 0, 0, "", 0, "C" };
//struct _reent * _impure_ptr = &impure_data;
int * __errno () {
    return &_impure_ptr->_errno;
}
#endif

/*
 * File Descriptor Table
 */

enum {
    FD_STDIN,
    FD_STDOUT,
    FD_STDERR,
    FD_START,

    MAX_FD = 20,
};

static int fdTable[MAX_FD] = {
  FD_STDIN, FD_STDOUT, FD_STDERR,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
};

/******************************************************************************/
/* DEVOPTAB Section */
/******************************************************************************/
#define MAX_POSIX_DEVICES       10
#define MAX_DEVICE_NAME_SIZE    10
static devoptab_t devoptab[MAX_POSIX_DEVICES];
static devlist_t devlist[MAX_POSIX_DEVICES];
static int devoptab_size = 0;

/******************************************************************************/
int deviceInstall (
    uint32_t maj,
    int  (*open_r )( void *reent, struct devoptab_s *dot, int mode, int flags ),
    int  (*ioctl  )(              struct devoptab_s *dot, int cmd,  int flags ),
    int  (*close_r)( void *reent, struct devoptab_s *dot ),
    long (*write_r)( void *reent, struct devoptab_s *dot, const void *buf,
                                                                      int len ),
    long (*read_r )( void *reent, struct devoptab_s *dot,       void *buf,
                                                                      int len ))
/******************************************************************************/
{
    if (maj >= MAX_POSIX_DEVICES)
        return FALSE;

    devlist[maj].open_r  = open_r;
    devlist[maj].ioctl   = ioctl;
    devlist[maj].close_r = close_r;
    devlist[maj].write_r = write_r;
    devlist[maj].read_r  = read_r;

    return TRUE;
}

/******************************************************************************/
int deviceRegister (const char *name, uint32_t maj, uint32_t min, void *priv)
/******************************************************************************/
{
    if (devoptab_size+1 >= MAX_POSIX_DEVICES) {
        return FALSE;
    }
    devoptab[devoptab_size].name    = name;
    devoptab[devoptab_size].maj     = maj;
    devoptab[devoptab_size].min     = min;
    devoptab[devoptab_size].priv    = priv;
    devoptab_size++;
    return TRUE;
}

#if defined(PRO_TOOLS)
#define USE_REENT 0
static struct _reent reentDummy;
static struct _reent *ptr = &reentDummy;
#else
#define USE_REENT 1
#endif

/******************************************************************************/
#if USE_REENT
int _open_r ( struct _reent *ptr, const char *file, int flags, int mode )
#else
int _open   ( const char *file, int flags, int mode )
#endif
/******************************************************************************/
{
    int fd = -1;
    int devop = -1;
    int i;

    /* search for the first free FD */
    for (i = FD_START; i < MAX_FD; i++) {
        if (fdTable[i] == -1) {
            fd = i;
            break;
        }
    }
    /* search for "file" in dotab_list[].name */
    for (i = 0; i < devoptab_size; i++) {
        if( strcmp( devoptab[i].name, file ) == 0 ) {
            devop = i;
            break;
        }
    }

    /* if we found the requested file/device and the FD table is not full,
     *     then invoke the device's open_r() method */
    if (devop == -1) {
        /* it doesn't exist in the devoptab list! */
        ptr->_errno = ENODEV;
    }
    else if (fd == -1) {
        /* too many files open, no room in fdTable */
        ptr->_errno = ENFILE;
    }
    else {
        if (devlist[devoptab[devop].maj].open_r
                                         (ptr, &devoptab[devop], mode, flags)) {
            fdTable[fd] = devop;
        }
        else {
            /* open failed, do not store device in FD table */
            fd = -1;
        }
    }

    return fd;
}

/*******************************************************************************
*
* fdevopen
*
* Extremely similar to _open_r, but specific to the standard I/O streams.
*
* RETURNS: fd.
*
*******************************************************************************/
int fdevopen(FILE *stream, const char *file, int flags, int mode)
{
    int fd = (stream == stdin)? 0:(stream == stdout)? 1:(stream == stderr)?2:-1;

    if (fd == -1) {
        assert(0);
        return -1;
    }

    int i;
    int devop = -1;

    for (i = 0; i < devoptab_size; i++) {
        if (!strcmp(devoptab[i].name, file)) {
            devop = i;
            break;
        }
    }

    if (devop == -1) {
        assert(0);
        return -1;
    }

    struct _reent tmp;
    if (devlist[devoptab[devop].maj].open_r(&tmp, &devoptab[devop],mode,flags))
        fdTable[fd] = devop;
    else {
        assert(0);
        fd = -1;
    }

    return fd;
}

/******************************************************************************/
static bool is_fd_valid(struct _reent *ptr, int fd )
/******************************************************************************/
{
    if (fd < 0 || fd >= MAX_FD) {
        ptr->_errno = EBADF;
        return (FALSE);
    }
    else if (fdTable[fd] == -1) {
        ptr->_errno = ENOENT;
        return (FALSE);
    }
    else {
        return (TRUE);
    }
}
/******************************************************************************/
#if USE_REENT
int _close_r ( struct _reent *ptr, int fd )
#else
int _close   ( int fd )
#endif
/******************************************************************************/
{
    int retVal = -1;
    if (fd > FD_STDERR) {       /* Don't allow the stdio streams to be closed */
        if (is_fd_valid(ptr, fd)) {
            int devop = fdTable[fd];
            retVal = devlist[devoptab[devop].maj].close_r(ptr,&devoptab[devop]);
            if (!retVal) {
                fdTable[fd] = -1;
            }
        }
    }
    return retVal;
}

/******************************************************************************/
int ioctl ( int fd, int cmd, int flags )
/******************************************************************************/
{
    int retVal = -1;
    struct _reent tmp;
    if (is_fd_valid(&tmp, fd)) {
        int devop = fdTable[fd];
        retVal = devlist[devoptab[devop].maj].ioctl(&devoptab[devop],cmd,flags);
    }
    return retVal;

}

/******************************************************************************/
#if USE_REENT
long _write_r ( struct _reent *ptr, int fd, const void *buf, size_t cnt )
#else
long _write   ( int fd, const void *buf, size_t cnt )
#endif
/******************************************************************************/
{
    long retVal = -1;
    if (is_fd_valid(ptr, fd)) {
        int devop = fdTable[fd];
        retVal = devlist[devoptab[devop].maj].write_r
                                              (ptr, &devoptab[devop], buf, cnt);
    }
    return retVal;
}

/******************************************************************************/
#if USE_REENT
long _read_r ( struct _reent *ptr, int fd, void *buf, size_t cnt )
#else
long _read   ( int fd, void *buf, size_t cnt )
#endif
/******************************************************************************/
{
    long retVal = -1;
    if (is_fd_valid(ptr, fd)) {
        int devop = fdTable[fd];
        retVal = devlist[devoptab[devop].maj].read_r
                                           (ptr, &devoptab[devop], buf, cnt);
    }
    return retVal;
}
