/*******************************************************************************
*
* devoptab.c
*
* POSIX Interface
* http://neptune.billgatliff.com/newlib.html
*
*******************************************************************************/
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "kinetis.h"

#include "globalDefs.h"
#include "hardware.h"


/* Error Stuff, WIP */
#define errno (*__errno())
extern int *__errno ( void );
//static struct _reent impure_data = { 0, 0, "", 0, "C" };
//struct _reent * _impure_ptr = &impure_data;
int * __errno () {
    return &_impure_ptr->_errno;
}

/* DEVOPTAB Entry's */
devoptab_t devoptab_spi0   = { "spi0", spi_open_r,  spi_ioctl, spi_close_r,
                                             spi_write_r, spi_read_r, NULL  };
devoptab_t devoptab_spi1   = { "spi1", spi_open_r,  spi_ioctl, spi_close_r,
                                             spi_write_r, spi_read_r, NULL  };
devoptab_t devoptab_spi2   = { "spi2", spi_open_r,  spi_ioctl, spi_close_r,
                                             spi_write_r, spi_read_r, NULL  };

devoptab_t devoptab_uart1 = { "uart1", 0, 0, 0, 0, 0 };

/* DEVOPTAB */
devoptab_t *devoptab_list[] = {
    &devoptab_uart1, /* standard input */
    &devoptab_uart1, /* standard output */
    &devoptab_uart1, /* standard error */
    &devoptab_spi0,
    &devoptab_spi1,
    &devoptab_spi2,
    0                /* terminates the list */
};

/*******************************************************************************/
int _open_r (struct _reent *ptr, const char *file, int flags, int mode )
/*******************************************************************************/
{
    int which_devoptab = 0;
    int fd = -1;

    /* search for "file" in dotab_list[].name */
    do {
        if( strcmp( devoptab_list[which_devoptab]->name, file ) == 0 ) {
            fd = which_devoptab;
            break;
        }
    } while( devoptab_list[which_devoptab++] );


    /* if we found the requested file/device,
     *     then invoke the device's open_r() method */

    if( fd != -1 ) {
        devoptab_list[fd]->open_r( ptr, devoptab_list[fd], mode, flags );
    } else {
        /* it doesn't exist in the devoptab list! */
        ptr->_errno = ENODEV;
    }

    return fd;
}

/*******************************************************************************/
int _close_r ( struct _reent *ptr, int fd )
/*******************************************************************************/
{
    return devoptab_list[fd]->close_r(ptr, devoptab_list[fd]);
}

/*******************************************************************************/
int ioctl (int fd, int cmd, int flags)
/*******************************************************************************/
{
    return devoptab_list[fd]->ioctl(devoptab_list[fd], cmd, flags);
}

/*******************************************************************************/
long _write_r (struct _reent *ptr, int fd, const void *buf, size_t cnt )
/*******************************************************************************/
{
    return devoptab_list[fd]->write_r(ptr, devoptab_list[fd], buf, cnt);
}

/*******************************************************************************/
long _read_r (struct _reent *ptr, int fd, void *buf, size_t cnt )
/*******************************************************************************/
{
    return devoptab_list[fd]->read_r(ptr, devoptab_list[fd], buf, cnt);
}
