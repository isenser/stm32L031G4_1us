/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_it.h"

#include <string.h>
#include "dev.h"


/* USER CODE BEGIN EV */
extern uint16_t led_timer;
extern uint16_t sec1_timer;
extern uint8_t  adc_timer;
//SPI
extern uint8_t spi_led_timer;
extern uint8_t spi_rx_buffer[];
extern uint8_t spi_tx_buffer[];
extern uint8_t rx_cmd_buffer[];
extern uint8_t spi_rx_count;
		
extern struct struct_impulse_now impulse_now;
extern uint8_t btn_tim_update;

extern uint8_t sleep;

extern uint8_t spi_data_read;

extern uint8_t spi_tx_dma_buff[];
extern uint8_t spi_rx_dma_buff[];
 	
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable Interrupt.
  */
void NMI_Handler(void)
{

}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  while (1)
  {
 
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{

}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{

}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel 2 and channel 3 interrupts.
  */
void DMA1_Channel2_3_IRQHandler(void)
{
	//SPI_RX_Complete
  if (LL_DMA_IsActiveFlag_TC2(DMA1)) {
			LL_DMA_ClearFlag_TC2(DMA1);
		
			memcpy(&rx_cmd_buffer,&spi_rx_dma_buff,SPI_RX_TX_SIZE);
			spi_data_read=1;
		
#if defined (DEBUG_MODE)
		  Led_blink;
#endif
			LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_2); //SPI_RX
			LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, SPI_RX_TX_SIZE);
			LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_2); //SPI_RX
	}//SPI_RX_Complete
	
	//SPI_TX_Complete
	if (LL_DMA_IsActiveFlag_TC3(DMA1)) {
			LL_DMA_ClearFlag_TC3(DMA1);
		
			//clear  spi_tx_dma_buff 
			memset(spi_tx_dma_buff, 0, SPI_RX_TX_SIZE);
			LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_3); //SPI_TX
			LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, SPI_RX_TX_SIZE);
			LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_3); //SPI_TX
		
			//SPI_ANSWER_LOW
			LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_8);
		
//#if defined (DEBUG_MODE)
//		  Led_blink;
//#endif
	}//SPI_TX_Complete
	
}//void DMA1_Channel2_3_IRQHandler(void)

//uint16_t tim_1us_interval=10;
extern volatile uint8_t  channel_counter;
extern uint16_t hv_current_adc;
extern uint16_t adc_current[];

extern float hv_current;
extern float Current_Filter(uint16_t newVal);

void TIM2_IRQHandler(void) //0.7-120us
{
//	uint16_t i=0;
//	uint32_t a_c=0;
	
	GPIOC->ODR=0x0000; //ENB0; ENA0;
	
	ADC1->CR|=ADC_CR_ADSTP; //ADC_STOP
	
	LL_TIM_ClearFlag_UPDATE(TIM2); // TIM2_Clear

	TIM2->ARR=impulse_now.tim_1us_interval; //0.7-120us

	if (++channel_counter==OBP_NUM) { //OBR_NUM 65
		 channel_counter=1;
	}
	set_channel(channel_counter);	
	
//	//CURRENT
//	a_c=0;
//	for (i=0;i<16;i++) {
//		a_c+=adc_current[i];
//	}
//	hv_current_adc=(uint16_t)(a_c>>4);//16
	
	//memset(&adc_current,0,16);
	
}//void TIM2_IRQHandler(void)

/**
  * @brief This function handles TIM21 global interrupt.
  */
void TIM21_IRQHandler(void) //312.5us t_pause
{
	  uint16_t ch=0;
		LL_TIM_ClearFlag_UPDATE(TIM21);
	
	  if (impulse_now.device_status==0) { //STOP
			LL_TIM_DisableCounter(TIM21); //312,5us
		}
	
	  if (impulse_now.device_status==1) { //RUN
			//CHANGE_CHANNELS
			if ((channel_counter>0)&&(channel_counter<49)) 
			{
				 ch=set_channel_output(channel_counter); //ENA ENB
				 //ch=0xC000; //test
			}
			
			//START_TIM2 //0.5-120us
			TIM2->CR1 |= TIM_CR1_CEN;
			GPIOC->ODR=ch;
			
			//START_DMA_ADC
			if (ch!=0) {
				ADC1->CR|=ADC_CR_ADSTART; //ADC_START
			}
		}//if (impulse_now.device_status==1) { //RUN
		
}//void TIM21_IRQHandler(void) //312us

/**
  * @brief This function handles TIM22 global interrupt.
  */
void TIM22_IRQHandler(void) //1ms
{ 
	static uint8_t  time100=0;
	static uint8_t  time1sec=0;
   //1ms
   if (LL_TIM_IsActiveFlag_UPDATE(TIM22)==SET) {
		  LL_TIM_ClearFlag_UPDATE(TIM22);
		 
			if (led_timer!=0) {led_timer--;}
			
			if (adc_timer!=0) {adc_timer--;}
			
			if (spi_led_timer!=0)  {
				spi_led_timer--;
				if (spi_led_timer==0)  {
					Led_Off;
				}
      }
			
		//100ms
			if (time100!=0) {
				time100--;
			} else { //tim100==0
				time100=100;//100ms
        dev_100ms();
				
				time1sec++;
				if (time1sec==10) {
					time1sec=0;
					dev_1sec();
				}
	   }//time100
		
//		//1sec
//			if (time1sec!=0) {
//				time1sec--;
//			} else {
//				time1sec=1000;
//				dev_1sec();
//			}//time1sec
			
			
	  }//if (LL_TIM_IsActiveFlag_UPDATE(TIM22)==SET) {
}//void TIM22_IRQHandler(void) //1ms

/**
  * @brief This function handles SPI1 global interrupt.
  */
//void SPI1_IRQHandler(void)
//{
//	//READ from Nrf52
//	if (LL_SPI_IsActiveFlag_RXNE(SPI1)==SET) {
//		spi_rx_buffer[spi_rx_count++]=SPI1->DR;
//	}//if (LL_SPI_IsActiveFlag_RXNE(SPI1)==SET) {
//}


void EXTI4_15_IRQHandler(void) {
	
//	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_4)) {
//		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
//		if (LL_GPIO_IsInputPinSet(GPIOA,LL_GPIO_PIN_4)==0) { //SPI_NSS
//			//START_SPI  ---|___
//			spi_rx_count=0;
//			//memset(spi_rx_buffer, 0, SPI_RX_TX_SIZE);
//			Led_blink;
//		} else {
//			//STOP_SPI ___|----
//			memcpy(&rx_cmd_buffer,&spi_rx_buffer,SPI_RX_TX_SIZE);
//			LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_3); //SPI_TX
//		  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, SPI_RX_TX_SIZE);
//		  LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_3); //SPI_TX
//			
//			//SPI_ANSWER_LOW
//			LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_8);
//			
//			spi_data_read=1;
//		}
//	}//if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_4)) {
	
	//SLEEP_PIN
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_9)) 
	{
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_9);
		if (LL_GPIO_IsInputPinSet(GPIOA,LL_GPIO_PIN_9)==0) {
			 sleep=0;
		} else {
			 sleep=1; 
		}
	}//if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_9)) 
	
	
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
