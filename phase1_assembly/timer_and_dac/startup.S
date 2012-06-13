#define __ASM_SRC__
#include "arm_cm4.h"
#include "k60n512.h"

    .syntax unified
    .thumb
    .section .flash_config
flash_config_field:
    .long   0x01234567      /* Backdoor key                 */
    .long   0x89abcdef
    .long   0xffffffff      /* Program flash protection      */
    .byte   0xfe            /* Flash security byte           */
    .byte   0xff            /* Flash nonvolailte option byte */
    .byte   0xff            /* EEPROM protection byte        */
    .byte   0xff            /* Data flash protection byte    */

#define WDOG_UNLOCK_KEY_1  0xC520
#define WDOG_UNLOCK_KEY_2  0xD928

    .text
    .globl __start_s
    .thumb_func
__start_s:
    ldr  r1,=WDOG_UNLOCK
    /* Write the first unlock word */
    ldr  r0,=WDOG_UNLOCK_KEY_1
    strh r0,[r1]
    /* Write the second unlock word within 20 bus clock cycles */
    ldr  r0,=WDOG_UNLOCK_KEY_2
    strh r0,[r1]
    ldr  r1,=WDOG_STCTRLH
    ldr  r0,=0x01D2
    strh r0,[r1]

    /* Set Stack Pointer */
    ldr r1,=_bootstack_top
    mov sp,r1

    ldr r0,=main
    bx  r0
