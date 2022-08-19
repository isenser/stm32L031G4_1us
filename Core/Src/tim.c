/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */
extern const uint8_t DMA_BUFFER_SIZE;
extern uint16_t dma_gpio_buffer[];
extern uint16_t dma_test[];
/* USER CODE END 0 */

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* TIM2 DMA Init */

//  /* TIM2_UP Init */
//  LL_DMA_SetPeriphRequest        (DMA1, LL_DMA_CHANNEL_2, LL_DMA_REQUEST_8);
//  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
//	//LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_MEMORY);
//  LL_DMA_SetChannelPriorityLevel (DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_MEDIUM);
//  LL_DMA_SetMode                 (DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_CIRCULAR);
//  LL_DMA_SetPeriphIncMode        (DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
//  LL_DMA_SetMemoryIncMode        (DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
//  LL_DMA_SetPeriphSize           (DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_HALFWORD);
//  LL_DMA_SetMemorySize           (DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_HALFWORD);

//  /* USER CODE BEGIN TIM2_Init 1 */
//  LL_DMA_SetDataLength           (DMA1, LL_DMA_CHANNEL_2, DMA_BUFFER_SIZE);
//	//LL_DMA_ConfigAddresses         (DMA1, LL_DMA_CHANNEL_2,(uint32_t)&dma_gpio_buffer,(uint32_t)&GPIOC->ODR,LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
//	LL_DMA_ConfigAddresses         (DMA1, LL_DMA_CHANNEL_2,(uint32_t)&dma_gpio_buffer,(uint32_t)&dma_test,LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
//  
////	/* Enable the selected DMAy Channelx */
////	LL_DMA_ClearFlag_TC2(DMA1);
//  DMA1_Channel2->CCR |= DMA_CCR_EN;	
////	LL_DMA_EnableIT_TC(DMA1,LL_DMA_CHANNEL_2);

 /* TIM21 interrupt Init */
  NVIC_SetPriority(TIM2_IRQn, 0);
  NVIC_EnableIRQ(TIM2_IRQn);
	
  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler     = 0;//31;
  TIM_InitStruct.CounterMode   = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload    = 0x01;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
	
  LL_TIM_EnableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
	LL_TIM_SetOnePulseMode(TIM2,LL_TIM_ONEPULSEMODE_SINGLE);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}
/* TIM21 init function */
void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM21);

  /* TIM21 interrupt Init */
  NVIC_SetPriority(TIM21_IRQn, 0);
  NVIC_EnableIRQ(TIM21_IRQn);

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  TIM_InitStruct.Prescaler = 0x20;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 0x100;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM21, &TIM_InitStruct);
	
  LL_TIM_EnableARRPreload(TIM21);
  LL_TIM_SetClockSource(TIM21, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM21, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM21);
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */

}
/* TIM22 init function */
void MX_TIM22_Init(void)
{

  /* USER CODE BEGIN TIM22_Init 0 */

  /* USER CODE END TIM22_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM22);

  /* TIM22 interrupt Init */
  NVIC_SetPriority(TIM22_IRQn, 2);
  NVIC_EnableIRQ(TIM22_IRQn);

  /* USER CODE BEGIN TIM22_Init 1 */

  /* USER CODE END TIM22_Init 1 */
  TIM_InitStruct.Prescaler   = 31;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload  = 1000;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM22, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM22);
  LL_TIM_SetClockSource(TIM22, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM22, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM22);
  /* USER CODE BEGIN TIM22_Init 2 */

  /* USER CODE END TIM22_Init 2 */

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
