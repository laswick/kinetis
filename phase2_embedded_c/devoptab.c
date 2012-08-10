/*******************************************************************************
* devoptab.c
********************************************************************************
*
* POSIX Interface
*
* This was based on an article by bill gatliff. It can be found:
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
*******************************************************************************

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
        devoptab_t devoptab_xxx   = { "xxx", xxx_open_r,  xxx_ioctl, xxx_close_r,
                                             xxx_write_r, xxx_read_r, NULL     };

    3) In devoptab.c, place you devoptab_t entry into the devoptab_List. By doing
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

********************************************************************************/
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

/*******************************************************************************/
/* DEVOPTAB Entry's Section */
/*******************************************************************************/
devoptab_t devoptab_spi0   = { "spi0", spi_open_r,  spi_ioctl, spi_close_r,
                                             spi_write_r, spi_read_r, NULL  };
devoptab_t devoptab_spi1   = { "spi1", spi_open_r,  spi_ioctl, spi_close_r,
                                             spi_write_r, spi_read_r, NULL  };
devoptab_t devoptab_spi2   = { "spi2", spi_open_r,  spi_ioctl, spi_close_r,
                                             spi_write_r, spi_read_r, NULL  };
devoptab_t devoptab_crc    = { "crc",  crc_open_r,  crc_ioctl, crc_close_r,
                                             crc_write_r, crc_read_r, NULL  };
devoptab_t devoptab_uart1  = { "uart1", 0, 0, 0, 0, 0 };

/*******************************************************************************/
/* DEVOPTAB Section */
/*******************************************************************************/
devoptab_t *devoptab_list[] = {
    &devoptab_uart1, /* standard input */
    &devoptab_uart1, /* standard output */
    &devoptab_uart1, /* standard error */
    &devoptab_spi0,
    &devoptab_spi1,
    &devoptab_spi2,
    &devoptab_crc,
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
