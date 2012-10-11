/*******************************************************************************
*
* syscalls.c
*
* Rob Laswick
* Sept 28 2012
*
* Platform dependent implementation of the required Newlib stubs.
*
* Key References:
*   soureware.org/newlib
*   neptune.billgatliff.com/newlib.html
*   balau82.wordpress.com/2010/12/16/using-newlib-in-arm-bare-metal-programs
*
* Copyright (C) 2012 www.laswick.net
*
* This program is free software.  It comes without any warranty, to the extent
* permitted by applicable law.  You can redistribute it and/or modify it under
* the terms of the WTF Public License (WTFPL), Version 2, as published by
* Sam Hocevar.  See http://sam.zoy.org/wtfpl/COPYING for more details.
*
*******************************************************************************/
#include <sys/stat.h>

/*******************************************************************************
* sbrk (set the "program break").
*
* The "program break" is a memory address, and is c library's concept of the
* "end of the heap", or more specifically "the end of a process's data
* region".
*
* Although we define a heap in our linker script, the c library's memory
* allocator (malloc) initially has no idea where our heap is or how big it is.
*
* Each time malloc needs "more" memory that it already controls/or is aware of,
* sbrk is called to effectively increase malloc's understanding/concept of
* available memory.
*
* Eventually malloc could potentially allocat the entire amount of
* heap space defined in the linker script.
*
*******************************************************************************/
void *_sbrk_r(void *reent, int size)
{
    extern char _heap_start;  /* _heap_start/end defined in the linker script */
    extern char _heap_end;
    static char *brk = &_heap_start;
    char *prevBrk;

    prevBrk = brk;
    if ((brk + size) > (char *) &_heap_end)
        return (void *) -1;             /* No more heap for malloc to consume */
    brk += size;
    return (void *) prevBrk;
}

/*******************************************************************************
*
* fstat
*
* Although this routine is required, it's of little use when the target
* environment lacks an underlying filesystem.  So simply let the caller
* know that argument fd is a character device.
*
*******************************************************************************/
int _fstat_r(void *reent, int fd, struct stat *st)
{
    /* Let the caller know this is a character device */
    st->st_mode = S_IFCHR;
    return 0;
}

/*******************************************************************************
*
* isatty
*
* This routine determines whether or not the argument fd is a terminal.
*
* For now, just return TRUE.
*
*******************************************************************************/
int _isatty_r(void *reent, int fd)
{
    return 1;
}

/*******************************************************************************
*
* lseek
*
* When the target environment supports an underlying filesystem, this routine
* would set a position in a file.
*
*******************************************************************************/
int _lseek_r(void *reent, int fd, int ptr, int dir)
{
    return 0;
}

#if 0

    /*
     * These required routines are implemented in our devoptab.c.
     */

int _write_r(void *reent, int fd, const void *buf, int len)
{
    return len;
}

int _close_r(int fd)
{
    return -1;
}

int _read_r(int fd, char *ptr, int len)
{
    return len;
}
#endif
