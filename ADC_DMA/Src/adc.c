/**************************************************************************************************
 * File: adc.c
 * Author: Jacob Clarey
 * Date: 11/4/2024
 * Description: This source is for the STM32F301K8T6 Current Receiver Module.
 *************************************************************************************************/

#include "adc.h"

void ADC_Init(void) {
  RCC->AHBENR |= RCC_AHBENR_ADC1EN;
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

  ADC1_COMMON->CCR |= ADC_CCR_CKMODE_0;

  ADC1->CFGR &= ~ADC_CFGR_RES;  // 12-bit resolution

  ADC1->CFGR = ADC_CFGR_CONT;
  ADC1->CFGR &= ~ADC_CFGR_ALIGN;

  ADC1->SMPR1 &=
      ~(ADC_SMPR1_SMP4 | ADC_SMPR1_SMP5);  // sample at 3 cycles per channel
  ADC1->SMPR2 &= ~(ADC_SMPR2_SMP10 | ADC_SMPR2_SMP11);

  ADC1->SQR1 |= (4 << ADC_SQR1_L_Pos);  // L = 4 for 5 conversion channels

  ADC_IN4_GPIO->MODER |= (3U << (2 * ADC_IN4_PIN));
  ADC_IN5_GPIO->MODER |= (3U << (2 * ADC_IN5_PIN));
  ADC_IN10_GPIO->MODER |= (3U << (2 * ADC_IN10_PIN));
  ADC_IN11_GPIO->MODER |= (3U << (2 * ADC_IN11_PIN));

  ADC1->SMPR2 |= ADC_SMPR2_SMP16;    // maximum sample cycles for temp sensor
  ADC1_COMMON->CCR |= ADC_CCR_TSEN;  // enable temp sensor

  ADC1->CFGR |= ADC_CFGR_DMAEN;   // enable DMA
  ADC1->CFGR |= ADC_CFGR_DMACFG;  // circular mode

  ADC1->SQR1 |= (4U << ADC_SQR1_SQ1_Pos);  // set channel sequencing
  ADC1->SQR1 |= (5U << ADC_SQR1_SQ2_Pos);
  ADC1->SQR1 |= (10U << ADC_SQR1_SQ3_Pos);
  ADC1->SQR1 |= (11U << ADC_SQR1_SQ4_Pos);
  ADC1->SQR2 |= (16U << ADC_SQR2_SQ5_Pos);
}

void ADC_Enable(void) {
  ADC1->CR |= ADC_CR_ADEN;

  while (!(ADC1->ISR & ADC_ISR_ADRDY)) {
  }
}

void ADC_Start(void) {
  ADC1->ISR = 0;
  ADC1->CR |= ADC_CR_ADSTART;
}

void DMA_Init(void) {
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;

  DMA1_Channel1->CCR &= ~DMA_CCR_DIR;
  DMA1_Channel1->CCR |=
      DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0;
}

void DMA_Config(uint32_t srcAdd, uint32_t destAdd, uint16_t size) {
  DMA1_Channel1->CNDTR = size;
  DMA1_Channel1->CPAR  = srcAdd;
  DMA1_Channel1->CMAR  = destAdd;

  DMA1_Channel1->CCR |= DMA_CCR_EN;
}

/* EOF */