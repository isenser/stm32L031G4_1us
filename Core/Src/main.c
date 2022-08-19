/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdlib.h>
#include <math.h> 

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "iwdg.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

#include "eeprom.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dev.h"
/* USER CODE END Includes */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SPI_SlaveSend(void);
void SPI_Parce(void);
static inline void Stop_Mode_Enter(void);
static inline void System_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t led_timer=0;
uint16_t sec1_timer=0;
uint8_t  adc_timer=0;

#define SPI_LED_TIME 30
uint8_t spi_led_timer=0;

//SPI_RX_BUFFER
//SPI_RX_TX_SIZE in dev.h
uint8_t spi_rx_buffer[SPI_RX_TX_SIZE]={0,};
uint8_t spi_rx_count=0;
uint8_t rx_cmd_buffer[SPI_RX_TX_SIZE]={0,};
uint8_t spi_data_read=0;

//SPI_TX_SENDER
uint8_t spi_tx_buffer[SPI_RX_TX_SIZE];
t_spi_send_buffer spi_send_buffer[SPI_RX_TX_SIZE];

uint8_t spi_tx_write=0;
uint8_t spi_tx_count=0;
uint8_t spi_tx_dma_buff[SPI_RX_TX_SIZE];

uint8_t spi_rx_dma_buff[SPI_RX_TX_SIZE];

/*--------- ADC -------------*/
//ADC_DMA
uint16_t adc_current[64]={0,};

uint16_t hight_voltage     = 0;
uint16_t hv_current_adc    = 0;
uint8_t  hv_current_status = 0;
float    hv_current=0;
uint16_t current_min=0;

//SLEEP_MODE
uint8_t sleep=0;

//EXTERN 
extern uint8_t btn_left_click;
extern uint8_t btn_right_click;

//static uint8_t dc_dc=0;

//extern uint8_t device_status;
extern struct  struct_impulse_prog  prog1;
extern struct  struct_impulse_now   impulse_now;
/* USER CODE END 0 */



float k = 0.1;  // коэффициент фильтрации, 0.0-1.0
// бегущее среднее
float Current_Filter(uint16_t newVal) {
  static float filVal = 0;
  //init
  if (filVal==0) {
    filVal=newVal;
    return filVal;
  }
  //filter
  filVal += (newVal - filVal) * k;
  return filVal;
}

//// бегущее среднее с адаптивным коэффициентом
//float expRunningAverageAdaptive(uint16_t newVal) {
//  static float filVal = 0;
//  float k;
//  // резкость фильтра зависит от модуля разности значений
//  if (abs(newVal - filVal) > 15) 
//		k = 0.9;
//  else 
//		k = 0.03;
//  
//  filVal += (newVal - filVal) * k;
//  return filVal;
//}

static uint16_t hv_c=0;

int main(void)
{
	uint16_t i=0;
	uint32_t a_c=0;
	
	System_Init();
	
	#if defined( DEBUG_MODE )
		SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STOP);   //Enable the Debug Module during STOP mode
		SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_SLEEP);  //Enable the Debug Module during SLEEP mode
		SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STANDBY);//Enable the Debug Module during STANDBY mode
	#else
		CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STOP);   //Disable the Debug Module during STOP mode
		CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_SLEEP);  //Disable the Debug Module during SLEEP mode
		CLEAR_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STANDBY);//Disable the Debug Module during STANDBY mode
	#endif
	

  dev_init();
	led_timer=100;
	
	impulse_now.device_status=4;//SLEEP
	sleep=1;
	Led_Off;
  /* Infinite loop */
  while (1)
  {
		//LL_IWDG_ReloadCounter(IWDG);
		
		if ((sleep) || (LL_GPIO_IsInputPinSet(GPIOA,LL_GPIO_PIN_9))) {
#if !defined( DEBUG_MODE )
			Stop_Mode_Enter();
			System_Init();
			sleep=0;
#endif
		}
		
		//19V
		if (LL_GPIO_IsInputPinSet(GPIOA,LL_GPIO_PIN_2)==1) {
			//Led_On;
			hight_voltage=1;
		} else {
			//Led_Off;
			hight_voltage=0;
		}
		
#if !defined (DEBUG_MODE)
		if (hv_current_status>=50) {
			Led_On;
		} else {
			Led_Off;
		}
#endif
		
		if (adc_timer==0) {
			adc_timer=100;
			if (hight_voltage) {	
				//CURRENT
				a_c=0;
				for (i=0;i<64;i++) {
					a_c+=adc_current[i];
				}
				a_c=(uint16_t)(a_c>>6);// /64
				
			  hv_current=Current_Filter(a_c);
				
				//y = 1,0764x - 14,739
				
				
				if (impulse_now.impulse_now_us<=100) {
					
					if (impulse_now.impulse_now_us<10) {
						hv_current_status=0;
					} else {
						current_min=1.0764*impulse_now.impulse_now_us - 14.739;
						hv_c = (hv_current*100/(current_min*4));
						hv_current_status = (uint8_t)hv_c;
						if (hv_current_status>100) {hv_current_status=100;}
					}
				} else {
					current_min = 110;
					hv_c = (hv_current*100/(current_min*3));
					hv_current_status = (uint8_t)hv_c;
					if (hv_current_status>100) {hv_current_status=100;}
				}

			} else {//if (hight_voltage) {	
			  hv_current_status=0;
			}
		}//if (adc_timer==0) {
		
		if (led_timer==0) {
			static uint8_t send_info=0;
			led_timer=1000;
			
//			//выполнять между импульсами внутри каналов
//			hv_current=Current_Filter(hv_current_adc);
			
			if (++send_info==1) {
				send_info=0;
			  Get_Device_Status(hight_voltage,(uint8_t)hv_current,hv_current_status);
			}
		}//if (led_timer==0)
		
		if (spi_data_read) {
			SPI_Parce();
			spi_data_read=0;
		}
		
		if (spi_tx_write!=spi_tx_count) {
		  if ((LL_DMA_IsActiveFlag_TC2(DMA1)==0) && (LL_DMA_IsActiveFlag_TC3(DMA1)==0)) {
			  memcpy(&spi_tx_dma_buff,&spi_send_buffer[spi_tx_write],SPI_RX_TX_SIZE);
				//SPI_ANSWER_HIGHT
			  LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_8);
				if (spi_tx_write<SPI_RX_TX_SIZE) {spi_tx_write++;} else {spi_tx_write=0;}
		 }//if ((LL_DMA_IsActiveFlag_TC2(DMA1)==0) and (LL_DMA_IsActiveFlag_TC3(DMA1)==0)) {
	 }//if (spi_tx_write!=spi_tx_count) {
		

  }//  while (1)
  /* USER CODE END 3 */
}//main

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_1)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_LSI_Enable();

   /* Wait till LSI is ready */
  while(LL_RCC_LSI_IsReady() != 1)
  {

  }
  LL_PWR_EnableBkUpAccess();
  if(LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSI)
  {
    LL_RCC_ForceBackupDomainReset();
    LL_RCC_ReleaseBackupDomainReset();
    LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
  }
  LL_RCC_EnableRTC();
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_4, LL_RCC_PLL_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }

  LL_Init1msTick(32000000);

  LL_SetSystemCoreClock(32000000);
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

static inline void Stop_Mode_Enter(void) {
	  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
		
		//LL_SYSTICK_DisableIT();
		//LL_LPM_EnableSleepOnExit();
	
	  impulse_now.device_status=4;//SLEEP
		
		/* выключили прерывания; пробуждению по ним это не помешает */
		//uint16_t state = irq_disable();
		NVIC_DisableIRQ(DMA1_Channel1_IRQn);
		NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);
		NVIC_DisableIRQ(TIM2_IRQn);
		NVIC_DisableIRQ(TIM21_IRQn);
		NVIC_DisableIRQ(TIM22_IRQn);
		NVIC_DisableIRQ(SPI1_IRQn);
		//NVIC_DisableIRQ(EXTI4_15_IRQn);
		
		LL_TIM_DisableIT_UPDATE(TIM2);
		LL_TIM_DisableCounter(TIM2);
		
		LL_TIM_DisableIT_UPDATE(TIM21);
		LL_TIM_DisableCounter(TIM21);
		
		LL_TIM_DisableIT_UPDATE(TIM22);
		LL_TIM_DisableCounter(TIM22);
		
		LL_SPI_DisableIT_RXNE(SPI1);
		LL_SPI_Disable(SPI1);
		
		Led_Off;
		
		DC19_OFF;
		DCDC_OFF;
	
	  CURRENT_CONTROL_OFF;
		
		//PB1_Out_Low
		GPIO_InitStruct.Pin  = LL_GPIO_PIN_1;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		
		//SWDIO
		GPIO_InitStruct.Pin  = LL_GPIO_PIN_13|LL_GPIO_PIN_14;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	  MUX_OFF;
		
		//SPI_ANSWER_LOW
		LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_8);
		
		//Other_pin_Out_Low
		LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_0);
		
		//STOP_MODE
		/* PDDS, LPSDSR or LPDS bits + SLEEPDEEP bit + WFI, 
		Return from ISR or WFE */
		/* Prepare to enter stop mode */
		/* флаг Wakeup должн быть очищен, иначе есть шанс проснуться немедленно */    
    PWR->CR |= PWR_CR_CWUF;      // clear the WUF flag after 2 clock cycles
		/* флаг PDDS определяет выбор между Stop и Standby, его надо сбросить */
    PWR->CR &= ~( PWR_CR_PDDS ); // Enter stop mode when the CPU enters deepsleep /*!< Power Down Deepsleep */
    // V_REFINT startup time ignored | V_REFINT off in LP mode | regulator in LP mode
		/* источник опорного напряжения Vref выключить автоматически */
    PWR->CR |= PWR_CR_FWU | PWR_CR_ULP | PWR_CR_LPSDSR;
    RCC->CFGR |= RCC_CFGR_STOPWUCK; // HSI16 oscillator is wake-up from stop clock
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; // low-power mode = stop mode
		
		
//		/* флаг PDDS определяет выбор между Stop и Standby, его надо сбросить */
//		PWR->CR &= ~(PWR_CR_PDDS);   
//		/* флаг Wakeup должн быть очищен, иначе есть шанс проснуться немедленно */    
//		PWR->CR |= PWR_CR_CWUF;
//		/* стабилизатор питания в low-power режим, у нас в Stop потребления-то почти не будет */
//		PWR->CR |= PWR_CR_LPSDSR;
//		/* источник опорного напряжения Vref выключить автоматически */
//		PWR->CR |= PWR_CR_ULP;
//		/* с точки зрения ядра Cortex-M, что Stop, что Standby - это режим Deep Sleep */
//		/* поэтому надо в ядре включить Deep Sleep */
//		SCB->SCR |=  (SCB_SCR_SLEEPDEEP_Msk);
		
		/* Select the regulator state in Stop mode: Set PDDS and LPSDSR bit according to PWR_Regulator value */
    //MODIFY_REG(PWR->CR, (PWR_CR_PDDS | PWR_CR_LPSDSR), Regulator);
		
		/* завершили незавершённые операция сохранения данных */
		//Data Synchronization Barrier
		//Acts as a special kind of Data Memory Barrier.
		__DSB();
		/* заснули */
		__WFI();
		
		//Проснулись
		//Инициализация всего
		
		/* Reset SLEEPDEEP bit of Cortex System Control Register */
	  //CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));
		
}//static inline void Stop_Mode_Enter(void) {

static inline void System_Init(void) 
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
		
	/* переинициализация*/
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();   //0,3us DMA_FAIL
  MX_TIM21_Init();  //312,5us IBP=5 ms
	
  MX_TIM22_Init();  //1ms
  MX_ADC_Init();
  MX_SPI1_Init();
  //MX_IWDG_Init();
  //MX_RTC_Init();

//ADC
  //START_TO_ZERO
	ADC1->CR|=ADC_CR_ADSTP; ///*!< ADC stop of conversion command */
	DMA1_Channel1->CCR |= DMA_CCR_EN; /* (7) */
	//ADC1->CR|=ADC_CR_ADSTART; //ADC_START LL_ADC_REG_CONV_CONTINUOUS
	
	CURRENT_CONTROL_ON;
	
//RTC
//  //EXTI_LINE_20 RTC_WakeUp
//	LL_EXTI_InitTypeDef EXTI_InitStructure;
//	
//	EXTI_InitStructure.Line_0_31   = LL_EXTI_LINE_20;
//  EXTI_InitStructure.LineCommand = ENABLE;
//  EXTI_InitStructure.Mode        = LL_EXTI_MODE_IT;
//  EXTI_InitStructure.Trigger     = LL_EXTI_TRIGGER_RISING;
//  LL_EXTI_Init(&EXTI_InitStructure);
	
	//LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_20);
	
//  LL_RTC_DisableWriteProtection(RTC);
//	LL_RTC_WAKEUP_SetAutoReload(RTC,0X05);
//	LL_RTC_EnableIT_WUT(RTC);
//	LL_RTC_WAKEUP_Enable(RTC);
	
//1ms timer
  LL_TIM_EnableARRPreload(TIM22);
	LL_TIM_EnableIT_UPDATE(TIM22);
	LL_TIM_EnableCounter(TIM22);
	
//TIM2 0,3us 
	LL_TIM_EnableARRPreload(TIM2);
	LL_TIM_ClearFlag_UPDATE(TIM2);
	LL_TIM_EnableIT_UPDATE(TIM2);
	//LL_TIM_EnableCounter(TIM2);
	
//TIM21 1-120ms IBP
	LL_TIM_EnableARRPreload(TIM21);
	LL_TIM_ClearFlag_UPDATE(TIM21);
	LL_TIM_EnableIT_UPDATE(TIM21);
	//LL_TIM_EnableCounter(TIM21);

//SPI1
//	LL_SPI_EnableIT_RXNE(SPI1);//Enable Rx buffer not empty interrupt
//	LL_SPI_Enable(SPI1);
	
//SPI_NSS_INTERRUPT
  //LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
	//LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	 
//SPI NSS (P0.10) PA4 EXTI_LINE_4 
	LL_EXTI_InitTypeDef EXTI_InitStructure;
//	
//	EXTI_InitStructure.Line_0_31   = LL_EXTI_LINE_4;
//  EXTI_InitStructure.LineCommand = ENABLE;
//  EXTI_InitStructure.Mode        = LL_EXTI_MODE_IT;
//  EXTI_InitStructure.Trigger     = LL_EXTI_TRIGGER_RISING_FALLING;
//  LL_EXTI_Init(&EXTI_InitStructure);
	
//SLEEP_SLEEP (P0.15) PA9 EXTI_LINE_9
  GPIO_InitStruct.Pin     = LL_GPIO_PIN_9;
	GPIO_InitStruct.Mode    = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed   = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pull    = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  EXTI_InitStructure.Line_0_31   = LL_EXTI_LINE_9;
  EXTI_InitStructure.LineCommand = ENABLE;
  EXTI_InitStructure.Mode        = LL_EXTI_MODE_IT;
  EXTI_InitStructure.Trigger     = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStructure);
	
	//LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_4);
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_9);
	
	NVIC_SetPriority(EXTI4_15_IRQn,2);
  NVIC_EnableIRQ(EXTI4_15_IRQn);
		
} //void System_Init

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */



void SPI_Parce(void) {
	//uint8_t len=0;
	uint8_t cmd=0;
//	uint8_t  data_num=0;
//	uint32_t data=0;

	//len=rx_cmd_buffer[0];
	cmd=rx_cmd_buffer[1];
	
	//crc=rx_cmd_buffer[len];
	
	//0x01-0x10 //SYSTEM_CMD
	//0x11-0x20 //HARDWARE (BTN_LED)
	//0x30-0x40 //FROM_BLE
	
	switch (cmd) {
//0x01-0x10 //SYSTEM_CMD
		
		case 0x01:
			  sleep=0;
		    break;
		
		case 0x02:
			  sleep=1;
		    break;
		
		case 0x03:
			  //PING_ANSWER
	 	    spi_tx_buffer[0]=0xff;
			  spi_tx_buffer[1]=0x02;//len
			  spi_tx_buffer[2]=0x03;//cmd
		    spi_tx_buffer[3]=rx_cmd_buffer[3]++;//0x56
		    Add_To_Spi_Buffer();
		    break;
		
		case 0x05: //EEPROM READ
			
//			data_num=rx_cmd_buffer[4];
//			//data=EEPROM_Read_DataNum(data_num);
//		  
//		  spi_tx_buffer[0]=0xff;
//			spi_tx_buffer[1]=0x06;//len
//			spi_tx_buffer[2]=0x05;//cmd
//			spi_tx_buffer[3]=(uint8_t)(data);     //Low
//			spi_tx_buffer[4]=(uint8_t)(data<<8);  //m_l
//			spi_tx_buffer[5]=(uint8_t)(data<<16); //m_H
//			spi_tx_buffer[6]=(uint8_t)(data<<24); //High
//			if (!spi_slave_send) {spi_slave_send=true;} //сделать очередь на передачу
		break;
		   
		case 0x06:
			//EEPROM WRITE
//			data_num=rx_cmd_buffer[4];
//		  data=(rx_cmd_buffer[8]<<24)+(rx_cmd_buffer[7]<<16)+(rx_cmd_buffer[6]<<8)+rx_cmd_buffer[5];
//			EEPROM_Write_DataNum(data_num,data);
			
		break;
		
//0x11-0x20 //HARDWARE (BTN_LED)
		case 0x11: //BTN_CLICK
			if (rx_cmd_buffer[2]==0x01) {
        btn_left_click=rx_cmd_buffer[3];
				if (impulse_now.device_status==1) {
				  dev_btn_click();
				}
			}
			if (rx_cmd_buffer[2]==0x02) {
        btn_right_click=rx_cmd_buffer[3];
				if (impulse_now.device_status==1) {
				  dev_btn_click();
				}
			}
		break;
			
//0x30-0x40 //FROM_BLE
		//SET PROGRAM REQUEST
		case 0x31: 
			//if (len==17) {}
		  prog1.prog_id          = (rx_cmd_buffer[3]<<8)  + rx_cmd_buffer[2];    //номер шаблона
			prog1.imp_voltage      =  rx_cmd_buffer[4];    //19V
			prog1.imp_t_pause      = (rx_cmd_buffer[6]<<8)  + rx_cmd_buffer[5];    //ms время между импульсами одного канала
			prog1.imp_t_min        = (rx_cmd_buffer[8]<<8)  + rx_cmd_buffer[7];    //us
			prog1.imp_t_max        = (rx_cmd_buffer[10]<<8) + rx_cmd_buffer[9];   //us
			prog1.prog_time        = (rx_cmd_buffer[12]<<8) + rx_cmd_buffer[11];  //sec
			prog1.prog_num         =  rx_cmd_buffer[13];   //число шагов
			prog1.prog_time_num    = (rx_cmd_buffer[15]<<8) + rx_cmd_buffer[14];; //sec, време перехода к конечному уровню
			prog1.prog_button_mode =  rx_cmd_buffer[16]; //по кнопкам	
		  
			dev_init();
			impulse_now.device_status=0;//STOP
		
			//ANSWER
//			spi_tx_buffer[0]=0xff;
//			spi_tx_buffer[1]=0x02;//len
//			spi_tx_buffer[2]=0x31;//cmd
//			spi_tx_buffer[3]=len;
//			Add_To_Spi_Buffer();
			break;
		
		//GET PROGRAM DEVICE REQUEST
		case 0x32: 
			
			break;
		
		case 0x33: //START DEVICE REQUEST
			if (impulse_now.prog_time!=0) {
				DCDC_ON;
				impulse_now.device_status=1;//RUN
				//START_TIMER
				LL_TIM_EnableCounter(TIM21); //312,5us
			}
			break;
		
		case 0x34: //STOP DEVICE REQUEST
			DCDC_OFF;
			impulse_now.device_status=0;//STOP
			break;
		
		case 0x35: //PAUSE DEVICE REQUEST
			impulse_now.device_status=2;//PAUSE
			break;

		
		
		//CHANGE PROGRAM LEVEL REQUEST
		case 0x41:
			break;
		
		//START/CONTINUE PROGRAM REQUEST
		case 0x43:
			
			break;
		
		
	}//switch
	
}//void SPI_Parce(void) 


//**********END OF FILE****/

