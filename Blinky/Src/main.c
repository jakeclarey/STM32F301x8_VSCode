/**************************************************************************************************
 * File: main.c
 * Author: Jacob Clarey
 * Date: 11/10/2024
 * Description: This is an example blink project for an STM32F301x8 MCU with an LED on pin B5.
 *************************************************************************************************/

#include "blink.h"
#include <stm32f301x8.h>

int main(void)
{
    systickInit();
    initLD2();

    while (1)
    {
        systickDelayMS(BLINK_DELAY);
        toggleLD2();
    } // while(1)
} // main(void)

/* EOF */
