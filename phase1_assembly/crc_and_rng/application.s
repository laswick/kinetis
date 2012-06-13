/*
 * application.s
 *
 * Example user/application code for the Freescale Kinetis K60 / ARM Cortex-M4.
 *
 * Simple CRC/RNG Demonstration:
 *  -Randomly generates a number when the button is pressed
 *  -These random numbers are added to the CRC
 *  -The bottom 4 bits of the CRC will be reflected via the LEDs
 *
 * Andrew Krajewski
 * June 2 2012
 */

    .syntax unified
    .thumb

    .text
    .align 2
    .global asm_main
    .thumb_func

asm_main:                                         /* Called by our start code */

    push { lr }

    /**** Chapter 31 from K60 referance manual ****/

    /* CRC Module Defines */
    .set CRC_BASE_ADDR, 0x40032000
    .set CRC_CRC,       CRC_BASE_ADDR + 0x0
    .set CRC_GPOLY,     CRC_BASE_ADDR + 0x4
    .set CRC_CTRL,      CRC_BASE_ADDR + 0x8

    .set CRC_CTRL_TOT_BITS,  0xc0000000 /* Types of Transpose Write */
    .set CRC_CTRL_TOTR_BITS, 0x30000000 /* Types of Transpose Read  */
    .set CRC_CTRL_FXOR,      (1 << 26)  /* Complement Read of CRC data reg */
    .set CRC_CTRL_WAS,       (1 << 25)  /* Write CRC data reg as sees */
    .set CRC_CTRL_TCRC,      (1 << 24)  /* Width of CRC - 0 => 16, 1 => 32 */

    /* TOT Enum */
    .set TOT_NO,             0
    .set TOT_BITS,           1
    .set TOT_BITS_AND_BYTES, 2
    .set TOT_BYTES,          3

    /* CRC SIM Config */
    ldr r0, =0x4004803c
    ldr r1, =0x00040000
    str r1, [r0]

    /* RNG (B) SIM Config */
    ldr r0, =0x40048030
    ldr r1, =0x00000001
    str r1, [r0]

    /* 31.3.1 CRC Initialization */
crc_init:
    /* Select 16-bit CRC */
    ldr r0, =0 /* 16-bit CRC */
    ldr r1, =CRC_CTRL
    str r0, [r1]

    /* Program Polynomial */
    ldr r0, =0x66 /* Couldn't find description of what the resulting poly is */
    ldr r1, =CRC_GPOLY
    str r0, [r1]

    /* Program transpose and complement options
     * Note: All other params must be set before programming the seed value */
    ldr r0, =(CRC_CTRL_TOT_BITS | CRC_CTRL_FXOR)
    ldr r1, =CRC_CTRL
    str r0, [r1]

    /* Program 'WAS' a.k.a. seed */
    ldr r0, =(CRC_CTRL_TOT_BITS | CRC_CTRL_FXOR | CRC_CTRL_WAS)
    ldr r1, =CRC_CTRL
    str r0, [r1]

    ldr r0, =0x13       /* My seed ... err ... */
    ldr r1, =CRC_CRC
    str r0, [r1]

    /* Clear 'WAS' so we can begin CRC'ing */
    ldr r0, =(CRC_CTRL_TOT_BITS | CRC_CTRL_FXOR)
    ldr r1, =CRC_CTRL
    str r0, [r1]

    /**** Chapter 33 from K60 reference manual ****/

    /* Random Number Generator Block (RNGB) Defines */
    .set RNG_BASE_ADDR, 0x400A0000
    .set RNG_VER,       RNG_BASE_ADDR + 0x0     /* Version ID */
    .set RNG_CMD,       RNG_BASE_ADDR + 0x4     /* Command */
    .set RNG_CR,        RNG_BASE_ADDR + 0x8     /* Control */
    .set RNG_SR,        RNG_BASE_ADDR + 0xC     /* Status */
    .set RNG_ESR,       RNG_BASE_ADDR + 0x10    /* Error Status */
    .set RNG_OUT,       RNG_BASE_ADDR + 0x14    /* Output */

    .set RNG_CMD_SOFT_RESET,        (1 << 6)
    .set RNG_CMD_CLEAR_ERROR,       (1 << 5)
    .set RNG_CMD_CLEAR_INTERRUPT,   (1 << 4)
    .set RNG_CMD_GENERATE_SEED,     (1 << 1)
    .set RNG_CMD_SELF_TEST,         (1 << 0)    /* Cough cough */

    .set RNG_CR_MASK_ERROR,     (1 << 6)
    .set RNG_CR_MASK_DONE,      (1 << 5)
    .set RNG_CR_AUTO_RESEED,    (1 << 4)
    .set RNG_CR_FUFMOD,         (0x3)       /* FIFO underflow response mode */

    /* FUFMOD Enum */
    .set FUFMOD_ZEROES,                     0x0
    .set FUFMOD_ZEROES_THE_SECOND_COMING,   0x1     /* Don't ask ... */
    .set FUFMOD_GEN_BUS_TRANS_ERROR,        0x2
    .set FUFMOD_GEN_INT_RET_ZEROES,         0x3

    /* Status Register Masks */
    .set RNG_SR_STATPF,     0xff000000  /* Results of 8 statistical tests */
    .set RNG_SR_ST_PF,      0x00e00000  /* Results TRNG, PRNG, RESEED self test*/
    .set RNG_SR_ERR,        (1 << 16)   /* Error in RNGB read RNG_ESR for deets*/
    .set RNG_SR_FIFO_SIZE,  0x0000f000
    .set RNG_SR_FIFO_LVL,   0x00000f00
    .set RNG_SR_NSDN,       (1 << 6)    /* New Seed Done */
    .set RNG_SR_SDN,        (1 << 5)    /* Seed Done */
    .set RNG_SR_STDN,       (1 << 4)    /* Self Test Done */
    .set RNG_SR_RS,         (1 << 3)    /* Reseed Needed */
    .set RNG_SR_SLP,        (1 << 2)    /* Zzzzzzzzzzzzz */
    .set RNG_SR_BUSY,       (1 << 1)    /* Essentially all of the above !done's */

    /* Error Status Register Masks */
    .set RNG_ESR_FUFE,      (1 << 4)    /* FIFO underflow error */
    .set RNG_ESR_SATE,      (1 << 3)    /* Statistical test error */
    .set RNG_ESR_STE,       (1 << 2)    /* Self test error */
    .set RNG_ESR_OSCE,      (1 << 1)    /* Oscillator error */
    .set RNG_ESR_LFE,       (1 << 0)    /* Linear feedback shift reg error */

rng_self_test:
    /* 33.2.1 Self Test Mode: Makes sure hardware is working
     * Self Test takes 29,000 cycles to complete
     * Interrupt may be generated upon self test completion if desired */
    ldr r0, =(RNG_CMD_SELF_TEST)
    ldr r1, =RNG_CMD
    str r0, [r1]

    /* Poll RNG_SR[STDN] until cleared */
    ldr r0, =RNG_SR
    ldr r2, =RNG_SR_STDN
self_poll:
    ldr r1, [r0]        /* Read status register */
    ands r1, r2
    bne rng_seed_generation
    b self_poll

rng_seed_generation:
    /* 33.2.2 Seed Generation Mode
     * Add entropy generated in the TRNG (true random number generator) to
     * 256-bit XKEY register.
     * PRNG (Pseudo random number generator) alg executes 20000 times sampling
     * the entropy from the TRNG to create an initial seed.
     * Simple statistical tests are run in parallel on the output.
     * When seed generation is complete, the TRNG reports pass/fail via RNG_ESR.
     * If new seed passes the tests, RNG_SR[SDN] is set signalling that we are
     * now ready to generate secure pseudo-random data. */
    ldr r0, =(RNG_CMD_GENERATE_SEED)
    ldr r1, =RNG_CMD
    str r0, [r1]

    /* Poll RNG_SR[BUSY] or RNG_SR[SDN] until cleared */
    ldr r0, =RNG_SR
/*  ldr r2, =RNG_SR_BUSY*/
    ldr r2, =RNG_SR_SDN
busy_poll:
    ldr r1, [r0]        /* Read status register */
    ands r1, r2
    bne rng_mode
    b busy_poll

rng_mode:
    /* 33.2.3 Random Number Generation Mode
     * Automatically entered on the heals of seed generation mode
     * When 5 word output FIFO is empty, a new 160-bit random number is generated
     * When output FIFO contains data, the RNGB auto enters sleep mode.
     * After 2^20 random data is generated, reseeding is required. */

led_setup:
    /* Tower Schematic:
     *   LED0: PTA11 (active low) (orange)
     *   LED1: PTA28 (active low) (yellow)
     *   LED2: PTA29 (active low) (green)
     *   LED3: PTA10 (active low) (blue)
     *   SW0 : PTE26 (active low) (needs internal pull up)
     *   SW1 : PTA19 (active low) (needs internal pull up) */

    /* PORT Controller Defines */

    .set PORTA_CTRL_BASE, 0x40049000
    .set PIN_11_OFFSET, (4 * 11)
    .set PIN_28_OFFSET, (4 * 28)
    .set PIN_29_OFFSET, (4 * 29)
    .set PIN_10_OFFSET, (4 * 10)
    .set PIN_19_OFFSET, (4 * 19)

    .set LED_ORANGE_CTRL_ADDR, PORTA_CTRL_BASE + PIN_11_OFFSET
    .set LED_YELLOW_CTRL_ADDR, PORTA_CTRL_BASE + PIN_28_OFFSET
    .set LED_GREEN_CTRL_ADDR,  PORTA_CTRL_BASE + PIN_29_OFFSET
    .set LED_BLUE_CTRL_ADDR,   PORTA_CTRL_BASE + PIN_10_OFFSET

    .set SW_1_CTRL_ADDR,       PORTA_CTRL_BASE + PIN_19_OFFSET

    .set PORTE_CTRL_BASE, 0x4004d000
    .set PIN_26_OFFSET, (4 * 26)

    .set SW_0_CTRL_ADDR,       PORTE_CTRL_BASE + PIN_26_OFFSET

    .set GPIO_ENABLE, (0x001 << 8)
    .set PULL_UP_ENABLE, (1 << 1)
    .set PULL_UP_SELECT, (1 << 0)
    .set PORT_CTRL_FLAGS, (GPIO_ENABLE | PULL_UP_ENABLE | PULL_UP_SELECT)

    /* GPIO Controller Defines */

    .set PORTA_BASE_ADDR, 0x400ff000
    .set PORTA_OUTPUT_REG,          PORTA_BASE_ADDR + 0x0
    .set PORTA_SET_REG,             PORTA_BASE_ADDR + 0x4
    .set PORTA_CLR_REG,             PORTA_BASE_ADDR + 0x8
    .set PORTA_TGL_REG,             PORTA_BASE_ADDR + 0xc
    .set PORTA_INPUT_REG,           PORTA_BASE_ADDR + 0x10
    .set PORTA_DATA_DIRECTION_ADDR, PORTA_BASE_ADDR + 0x14

    .set LED_ORANGE, (1 << 11)
    .set LED_YELLOW, (1 << 28)
    .set LED_GREEN,  (1 << 29)
    .set LED_BLUE,   (1 << 10)
    .set LEDS_MASK,  (LED_ORANGE | LED_YELLOW | LED_GREEN | LED_BLUE)

    .set SW_1_MASK, (1 << 19)

    .set PORTE_BASE_ADDR, 0x400ff100
    .set PORTE_INPUT_REG,           PORTE_BASE_ADDR + 0x10

    .set SW_0_MASK, (1 << 26)

    /* SIM Defines */

    .set SIM_SCGC5, 0x40048038
    .set PORTA_ENABLE, (1 << 9)
    .set PORTE_ENABLE, (1 << 13)
    .set SIM_SCGC5_FLAGS, (PORTA_ENABLE | PORTE_ENABLE)

    /* Enable PORT (Clock Gate) */

    ldr r0, =SIM_SCGC5_FLAGS
    ldr r1, =SIM_SCGC5
    str r0, [r1]

    /* Configure PORT Controller */

    ldr r0, =PORT_CTRL_FLAGS
    ldr r1, =LED_ORANGE_CTRL_ADDR
    ldr r2, =LED_YELLOW_CTRL_ADDR
    ldr r3, =LED_GREEN_CTRL_ADDR
    ldr r4, =LED_BLUE_CTRL_ADDR
    ldr r5, =SW_1_CTRL_ADDR
    ldr r6, =SW_0_CTRL_ADDR
    str r0, [r1]
    str r0, [r2]
    str r0, [r3]
    str r0, [r4]
    str r0, [r5]
    str r0, [r6]

    /* Configure GPIOs
     * Note: 0 = input, 1 = ouput (all pins default hi z inputs). */

    ldr r0, =LEDS_MASK                                      /* LEDs = Outputs */
    ldr r1, =PORTA_DATA_DIRECTION_ADDR
    str r0, [r1]

    /* Preset initial delay time */
    ldr r9, =0xcffff

    /* Preset all lights on */
    ldr r5, =0xf

main_loop:

test_dec_switch:                            /* Generate random num if pressed */

    ldr r0, =PORTA_INPUT_REG
    ldr r1, [r0]
    ldr r2, =SW_1_MASK
    ands r1, r2
    bne light_up_leds

    /* Poll RNG_SR[FIFO_LVL] to ensure random values are present in FIFO */
    ldr r0, =RNG_SR
poll_fifo:
    ldr r1, [r0]
    ldr r2, =RNG_SR_FIFO_LVL
    ands r1, r2
    bne poll_fifo

    /* Read new RNG output */
    ldr r0, =RNG_OUT
    ldr r1, [r0]  /* 32-bits */
/* ldr r1, =0x5 */

    /* Add random number to CRC. Top 16 ignored */
    ldr r0, =CRC_CRC
    str r1, [r0]

    /* Read current CRC value */
    ldr r5, [r0]

light_up_leds:

    ldr r0, =LEDS_MASK
    ldr r1, =PORTA_SET_REG          /* Turn all LEDs off (LEDs are NEG logic) */
    str r0, [r1]
    bl delay

    ldr r0, =PORTA_CLR_REG
    ldr r1, =LED_ORANGE
    ldr r2, =LED_YELLOW
    ldr r3, =LED_GREEN
    ldr r4, =LED_BLUE
light_up_orange:
    ands r5, #1
    bne light_up_yellow
    str r1, [r0]
light_up_yellow:
    ands r5, #2
    bne light_up_green
    str r2, [r0]
light_up_green:
    ands r5, #4
    bne light_up_blue
    str r3, [r0]
light_up_blue:
    ands r5, #8
    bne main_loop
    str r4, [r0]

    b main_loop

    /* If main_loop didn't loop forever we'd want to
     * pull the stacked link register into the program counter so we
     * can return to the start code for epilogue processing (if applicable).  */

    pop { pc }

    .global delay
    .align 2
    .thumb_func

delay:

    push { r0 }
    mov r0, r9

delay_loop:

    subs r0, #1
    bne delay_loop
    pop { r0 }
    bx lr

    .end
