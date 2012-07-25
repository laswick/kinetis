/*******************************************************************************
*
* hardware.c
*
*******************************************************************************/
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "kinetis.h"

#include "globalDefs.h"
#include "hardware.h"

/*******************************************************************************
*
* assert_
*
* This routine has no impact on the running software, however it's a good idea
* to set a breakpoint on assert_.
*
* When this breakpoint hits, gdb will automatically display the function
* arguments (files and line).
*
* RETURNS: Nothing
*
*******************************************************************************/
void assert_(const char *file, const int line) { }

/* POSIX Interface ************************************************************/

/* Error Stuff */
#define errno (*__errno())
extern int *__errno ( void );
//static struct _reent impure_data = { 0, 0, "", 0, "C" };
//struct _reent * _impure_ptr = &impure_data;
int * __errno () {
    return &_impure_ptr->_errno;
}

typedef struct {
    const char *name;
    int  (*open_r  )( void *reent, int fd, const char *file, int flags, int mode );
    int  (*ioctl)   (              int fd, int cmd,   int flags );
    int  (*close_r )( void *reent, int fd );
    long (*write_r )( void *reent, int fd, const void *buf, int len );
    long (*read_r  )( void *reent, int fd,       void *buf, int len );
} devoptab_t;

const devoptab_t devoptab_spi0   = { "spi0", spi_open_r,  spi_ioctl, spi_close_r,
                                             spi_write_r, spi_read_r };
const devoptab_t devoptab_spi1   = { "spi1", spi_open_r,  spi_ioctl, spi_close_r,
                                             spi_write_r, spi_read_r };
const devoptab_t devoptab_spi2   = { "spi2", spi_open_r,  spi_ioctl, spi_close_r,
                                             spi_write_r, spi_read_r };
const devoptab_t devoptab_uart1 = { "uart1", 0, 0, 0, 0 };

const devoptab_t *devoptab_list[] = {
    &devoptab_uart1, /* standard input */
    &devoptab_uart1, /* standard output */
    &devoptab_uart1, /* standard error */
    &devoptab_spi0,
    &devoptab_spi1,
    &devoptab_spi2,
    0                /* terminates the list */
};

int _open_r (struct _reent *ptr, const char *file, int flags, int mode )
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
        fd = devoptab_list[fd]->open_r( ptr, fd, file, flags, mode );
    } else {
        /* it doesn't exist in the devoptab list! */
        ptr->_errno = ENODEV;
    }

    return fd;
}

int _close_r ( struct _reent *ptr, int fd )
{
    return devoptab_list[fd]->close_r( ptr, fd );
}

int ioctl (int fd, int cmd, int flags)
{
    return devoptab_list[fd]->ioctl(fd, cmd, flags);
}

long _write_r (struct _reent *ptr, int fd, const void *buf, size_t cnt )
{
    return devoptab_list[fd]->write_r( ptr, fd, buf, cnt );
}

long _read_r (struct _reent *ptr, int fd, void *buf, size_t cnt )
{
    return devoptab_list[fd]->read_r( ptr, fd, buf, cnt );
}

