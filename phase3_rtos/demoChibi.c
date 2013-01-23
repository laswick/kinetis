#include "ch.h"
#include "kinetis.h"
#include "hardware.h"

void assert_(const char *file, const int line) { }

static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {
    while (TRUE) {
        chThdSleepMilliseconds(500);
        gpioToggle(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN);
    }

    return 0;
}

static void clocksInit(void)
{
    /* -------- 100 MHz (external clock) -----------
     * Configure the Multipurpose Clock Generator output to use the external
     * clock locked with a PLL at the maximum frequency of 100MHZ
     *
     * For PLL, the dividers must be set first.
     *
     * System:  100 MHz
     * Bus:      50 MHz
     * Flexbus:  50 MHz
     * Flash:    25 MHz
     */
    clockSetDividers(DIVIDE_BY_1, DIVIDE_BY_2, DIVIDE_BY_4, DIVIDE_BY_4);
    clockConfigMcgOut(MCG_PLL_EXTERNAL_100MHZ);
}

static void systickInit(void)
{
    uint32_t freq = clockGetFreq(CLOCK_CORE);

    NVIC_SYSTICK_RELOAD = freq / CH_FREQUENCY - 1;
    NVIC_SYSTICK_VALUE = 0;
    NVIC_SYSTICK_CONTROL = NVIC_SYSTICK_CONTROL_ENABLE
                         | NVIC_SYSTICK_CONTROL_TICKINT
                         | NVIC_SYSTICK_CONTROL_CLKSOURCE;
}

int main(void)
{
    clocksInit();
    systickInit();
    chSysInit();

    gpioConfig(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN, GPIO_OUTPUT | GPIO_LOW);
    gpioConfig(N_LED_YELLOW_PORT, N_LED_YELLOW_PIN, GPIO_OUTPUT | GPIO_LOW);

    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

    hwInterruptsEnable();

    while (TRUE) {
        chThdSleepMilliseconds(123);
        gpioToggle(N_LED_ORANGE_PORT, N_LED_ORANGE_PIN);
    }

    return 0;
}
