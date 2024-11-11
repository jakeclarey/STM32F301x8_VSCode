/**************************************************************************************************
 * File: adc.h
 * Author: Jacob Clarey
 * Date: 11/4/2024
 * Description: This header is for the STM32F301K8T6 Current Receiver Module.
 *************************************************************************************************/

#ifndef __ADC_H__
#define __ADC_H__

#include <stm32f301x8.h>

#define ADC_IN4_GPIO  GPIOA
#define ADC_IN4_PIN   3
#define ADC_IN5_GPIO  GPIOA
#define ADC_IN5_PIN   4
#define ADC_IN10_GPIO GPIOA
#define ADC_IN10_PIN  6
#define ADC_IN11_GPIO GPIOB
#define ADC_IN11_PIN  0

extern uint8_t *adcTxData;
extern uint16_t adcData[5];

void ADC_Init(void);
void ADC_Enable(void);
void ADC_Start(void);
void DMA_Init(void);
void DMA_Config(uint32_t srcAdd, uint32_t destAdd, uint16_t size);

extern void DMA1_Channel1_IRQHandler(void);

#endif // __ADC_H__

/* EOF */