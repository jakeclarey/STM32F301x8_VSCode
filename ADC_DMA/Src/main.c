/******************************************************************************
 * File: main.c
 * Author: Jacob Clarey
 * Date: 11/5/2024
 * Description: This code is for testing ADC with DMA and then transmitting the
 * ADC values over I2C. Meant for use with the STM32F301K8T6 Current Receiver
 * Module. Notes: PA9->SCL, PA10->SDA. Slave Address->0x32 (defined in i2c.h
 * file as STM32_SADDR).
 *****************************************************************************/

#include <stm32f301x8.h>

#include "adc.h"
#include "i2c.h"

uint16_t adcData[4]; // channels 1-4 on custom board in order of 0-3 of array
uint8_t *adcTxData;

int main(void)
{
    __disable_irq();

    I2C_Slave_Init(I2C_BUS, GPIOA, 9, 10); // slave mode device on I2C_BUS (defined in i2c.h).

    ADC_Init();
    ADC_Enable();

    DMA_Init();
    DMA_Config((uint32_t)&ADC1->DR, (uint32_t)adcData, 4); // ADC data register to memory via DMA

    ADC_Start();

    __enable_irq();
    while (1)
    {
        I2C_Slave_Transmit(I2C_BUS, (uint8_t *)adcTxData); // will transmit raw ADC codes upon request.
    } // while(1)
} // main(void)

/* EOF */