/*
 * application.s
 *
 * Example user/application code for the Freescale Kinetis K60 / ARM Cortex-M4.
 *
 * Chris Atkinson
 * June 7 2012
 */

    .syntax unified
    .thumb

    .text
    .align 2
    .global asm_main
    .thumb_func

asm_main:                                        /* Called by our start code */

    /* Your code goes here */
    .set FTFL_FSTAT,    0x40020000      /* Status Register */

    .set FTFL_FCCOB0,   0x40020007      /* Command Register  0 */
    .set FTFL_FCCOB1,   0x40020006      /* Command Register  1 */
    .set FTFL_FCCOB2,   0x40020005      /* Command Register  2 */
    .set FTFL_FCCOB3,   0x40020004      /* Command Register  3 */
    .set FTFL_FCCOB4,   0x4002000B      /* Command Register  4 */
    .set FTFL_FCCOB5,   0x4002000A      /* Command Register  5 */
    .set FTFL_FCCOB6,   0x40020009      /* Command Register  6 */
    .set FTFL_FCCOB7,   0x40020008      /* Command Register  7 */
    .set FTFL_FCCOB8,   0x4002000F      /* Command Register  8 */
    .set FTFL_FCCOB9,   0x4002000E      /* Command Register  9 */
    .set FTFL_FCCOBA,   0x4002000D      /* Command Register 10 */
    .set FTFL_FCCOBB,   0x4002000C      /* Command Register 11 */

    .set STARFISH,      0x00074000      /* Flash address we will use. */


    /*************** ERASE FLASH SECTOR *************************************/
    /* Take a look at fstat. */
    ldr     r0, = FTFL_FSTAT
    ldrb    r1, [r0]

    /* Take a look at what is in our flash address. */
    ldr     r0, = STARFISH
    ldr     r1, [r0]

    /* Erase Flash Sector Command. */
    ldr     r0, = FTFL_FCCOB0
    ldr     r1, = 0x09
    strb    r1, [r0]

    /* Address */
    ldr     r0, = FTFL_FCCOB1
    ldr     r1, = 0x07
    strb    r1, [r0]

    ldr     r0, = FTFL_FCCOB2
    ldr     r1, = 0x40
    strb    r1, [r0]

    ldr     r0, = FTFL_FCCOB3
    ldr     r1, = 0x00
    strb    r1, [r0]

    /* Here goes nothing !!! */
    ldr     r0, = FTFL_FSTAT
    ldrb    r1, [r0]
    orr.w   r1, r1, #0x80
    strb    r1, [r0]

    ldr     r0, = FTFL_FSTAT
    ldrb    r1, [r0]

    /* Take a look at what is in our flash address. */
    ldr     r0, = STARFISH
    ldr     r1, [r0]

    /* Take another look at what is in our flash address. */
    ldr     r0, = STARFISH
    ldr     r1, [r0]


    /************* PROGRAM FLASH ********************************/
    /* Take a look at fstat. */
    ldr     r0, = FTFL_FSTAT
    ldrb    r1, [r0]

    /* Take a look at what is in our flash address. */
    ldr     r0, = STARFISH
    ldr     r1, [r0]

    /* Program longword Command. */
    ldr     r0, = FTFL_FCCOB0
    ldr     r1, = 0x06
    strb    r1, [r0]

    /* Address */
    ldr     r0, = FTFL_FCCOB1
    ldr     r1, = 0x07
    strb    r1, [r0]

    ldr     r0, = FTFL_FCCOB2
    ldr     r1, = 0x40
    strb    r1, [r0]

    ldr     r0, = FTFL_FCCOB3
    ldr     r1, = 0x00
    strb    r1, [r0]

    /* Data to write. */
    ldr     r0, = FTFL_FCCOB4
    ldr     r1, = 0xA2
    strb    r1, [r0]

    ldr     r0, = FTFL_FCCOB5
    ldr     r1, = 0xA2
    strb    r1, [r0]

    ldr     r0, = FTFL_FCCOB6
    ldr     r1, = 0xA2
    strb    r1, [r0]

    ldr     r0, = FTFL_FCCOB7
    ldr     r1, = 0xA2
    strb    r1, [r0]

    /* Here goes nothing !!! */
    ldr     r0, = FTFL_FSTAT
    ldrb    r1, [r0]
    orr.w   r1, r1, #0x80
    strb    r1, [r0]

    ldr     r0, = FTFL_FSTAT
    ldrb    r1, [r0]

    /* Take a look at what is in our flash address. */
    ldr     r0, = STARFISH
    ldr     r1, [r0]

    /* Take another look at what is in our flash address. */
    ldr     r0, = STARFISH
    ldr     r1, [r0]


    bx lr                 /* Return to the start code for epilogue processing */

    .end
