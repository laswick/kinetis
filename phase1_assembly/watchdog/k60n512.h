#ifndef __K60N512_H
#define __K60N512_H

#define BIT_0  0x00000001
#define BIT_1  0x00000002
#define BIT_2  0x00000004
#define BIT_3  0x00000008
#define BIT_4  0x00000010
#define BIT_5  0x00000020
#define BIT_6  0x00000040
#define BIT_7  0x00000080
#define BIT_8  0x00000100
#define BIT_9  0x00000200
#define BIT_10 0x00000400
#define BIT_11 0x00000800
#define BIT_12 0x00001000
#define BIT_13 0x00002000
#define BIT_14 0x00004000
#define BIT_15 0x00008000
#define BIT_16 0x00010000
#define BIT_17 0x00020000
#define BIT_18 0x00040000
#define BIT_19 0x00080000
#define BIT_20 0x00100000
#define BIT_21 0x00200000
#define BIT_22 0x00400000
#define BIT_23 0x00800000
#define BIT_24 0x01000000
#define BIT_25 0x02000000
#define BIT_26 0x04000000
#define BIT_27 0x08000000
#define BIT_28 0x10000000
#define BIT_29 0x20000000
#define BIT_30 0x40000000
#define BIT_31 0x80000000

/* Port Contorl and GPIO Modules (s11,s54) */
#define PORTA_CTRL_BASE 0x40049000
#define PORTA_ISFR      0x400490A0

#define PORTA_OUTPUT_REG          0x400FF000
#define PORTA_SET_REG             0x400FF004
#define PORTA_CLR_REG             0x400FF008
#define PORTA_TGL_REG             0x400FF00C
#define PORTA_DATA_DIRECTION_ADDR 0x400FF014

/* System Interface Module (SIM) (s12) */
#define SIM_SCGC1       0x40048028
#define SIM_SCGC2       0x4004802C
#define SIM_SCGC3       0x40048030
#define SIM_SCGC4       0x40048034
#define SIM_SCGC5       0x40048038
#define SIM_SCGC6       0x4004803C
#define SIM_SCGC7       0x40048040

/* Watchdog Timer (WDOG) Module (s23) */
#define WDOG_STCTRLH    0x40052000
#define WDOG_STCTRLL    0x40052002
#define WDOG_TOVALH     0x40052004
#define WDOG_TOVALL     0x40052006
#define WDOG_WINH       0x40052008
#define WDOG_WINL       0x4005200A
#define WDOG_REFRESH    0x4005200C
#define WDOG_UNLOCK     0x4005200E
#define WDOG_TRMOUTH    0x40052010
#define WDOG_TMROUTL    0x40052012
#define WDOG_RSTCNT     0x40052014
#define WDOG_PRESC      0x40052016

/* Multipurpose Clock Generator (MCG) Module (s24) */
#define MCG_C1     0x40064000
#define MCG_C2     0x40064001
#define MCG_C3     0x40064002
#define MCG_C4     0x40064003
#define MCG_C5     0x40064004
#define MCG_C6     0x40064005
#define MCG_S      0x40064006
#endif
