/****************************************************************************/
/*                          NVIC REGISTERS                                  */
/****************************************************************************/
#define NVIC_ICTR   0xE000E004

#define NVIC_ISER0  0xE000E100
#define NVIC_ISER1  0xE000E104
#define NVIC_ISER2  0xE000E108
#define NVIC_ISER3  0xE000E10C
#define NVIC_ISER4  0xE000E110
#define NVIC_ISER5  0xE000E114
#define NVIC_ISER6  0xE000E118
#define NVIC_ISER7  0xE000E11C

#define NVIC_ICER0  0xE000E180
#define NVIC_ICER1  0xE000E184
#define NVIC_ICER2  0xE000E188
#define NVIC_ICER3  0xE000E18C
#define NVIC_ICER4  0xE000E190
#define NVIC_ICER5  0xE000E194
#define NVIC_ICER6  0xE000E198
#define NVIC_ICER7  0xE000E19C

#define NVIC_ISPR0  0xE000E200
#define NVIC_ISPR1  0xE000E204
#define NVIC_ISPR2  0xE000E208
#define NVIC_ISPR3  0xE000E20C
#define NVIC_ISPR4  0xE000E210
#define NVIC_ISPR5  0xE000E214
#define NVIC_ISPR6  0xE000E218
#define NVIC_ISPR7  0xE000E21C

#define NVIC_ICPR0  0xE000E280
#define NVIC_ICPR1  0xE000E284
#define NVIC_ICPR2  0xE000E288
#define NVIC_ICPR3  0xE000E28C
#define NVIC_ICPR4  0xE000E290
#define NVIC_ICPR5  0xE000E294
#define NVIC_ICPR6  0xE000E298
#define NVIC_ICPR7  0xE000E29C

#define NVIC_IABR0  0xE000E300
#define NVIC_IABR1  0xE000E304
#define NVIC_IABR2  0xE000E308
#define NVIC_IABR3  0xE000E30C
#define NVIC_IABR4  0xE000E310
#define NVIC_IABR5  0xE000E314
#define NVIC_IABR6  0xE000E318
#define NVIC_IABR7  0xE000E31C

#define NVIC_PR51   0xE000E400 + 51
#define NVIC_PR87   0xE000E400 + 87
/****************************************************************************/
/*                       SYSTEM CONTROL REGISTERS                           */
/****************************************************************************/
#define SCR_VTOR    0xE000ED08
