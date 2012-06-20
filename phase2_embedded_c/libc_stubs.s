/*
 * libc_stubs.s
 *
 * Weak defines to work around the undefined references in libc.a.
 *
 * Rob Laswick
 * June 19 2012
 */

    .syntax unified
    .thumb

    .text

    .global _exit
    .weak   _exit
    .thumb_func
    .align 2
_exit:

    .global __aeabi_uldivmod
    .weak   __aeabi_uldivmod
    .thumb_func
    .align 2
__aeabi_uldivmod:

    .global _sbrk_r
    .weak   _sbrk_r
    .thumb_func
    .align 2
_sbrk_r:

    .global _kill
    .weak   _kill
    .thumb_func
    .align 2
_kill:

    .global _getpid
    .weak   _getpid
    .thumb_func
    .align 2
_getpid:

    .global _write
    .weak   _write
    .thumb_func
    .align 2
_write:

    .global _close
    .weak   _close
    .thumb_func
    .align 2
_close:

    .global _fstat
    .weak   _fstat
    .thumb_func
    .align 2
_fstat:

    .global _isatty
    .weak   _isatty
    .thumb_func
    .align 2
_isatty:

    .global _lseek
    .weak   _lseek
    .thumb_func
    .align 2
_lseek:

    .global _read
    .weak   _read
    .thumb_func
    .align 2
_read:

forever:
    b forever


