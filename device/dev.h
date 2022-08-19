/* Includes ------------------------------------------------------------------*/
//#ifndef __DEV_H
//#define __DEV_H

#include <stdint.h>
#include <string.h>
#include <math.h>

//#define DEBUG_MODE   1

#include "stm32l0xx.h"

#include "stm32l0xx_ll_tim.h"
#include "stm32l0xx_ll_gpio.h"

//extern const uint8_t SPI_RX_TX_SIZE;
#define SPI_RX_TX_SIZE 32
typedef struct {
	uint8_t send_buf[SPI_RX_TX_SIZE];
} t_spi_send_buffer;

//STM8L #if DEVICE == 1 //312,5us*32=10ms \ ~3us - max_perion 60us

struct struct_impulse_prog {
  uint16_t prog_id;            //номер шаблона
  uint16_t count_limit;        //
  uint16_t imp_voltage;        //19V
  uint16_t imp_t_pause;        //ms время между импульсами одного канала
  uint16_t imp_t_min;          //*10=us
  uint16_t imp_t_max;          //*10=us
  uint16_t prog_time;          //sec
  uint16_t prog_num;           //число шагов
  uint16_t prog_time_num;      //sec, време перехода к конечному уровню
  uint16_t prog_button_mode;   //по кнопкам
  } ;

typedef enum 
{
  STOP  = 0, 
  RUN   = 1,
	PUSE  = 2,
	SLEEP = 3
} DevStatus;	
//STOP=0//RUN=1//PAUSE=2///SLEEP=3

struct struct_impulse_now {
 uint16_t  tim_1us_interval;        //tim_1us TIM2_ARR 
 uint16_t  tim_1us_interval_min;    //tim_1us TIM2_ARR 
 uint16_t  tim_1us_interval_max;    //tim_1us TIM2_ARR 
 uint16_t  tim_1us_interval_to;     //плавное увеличение по кнопке на 100ms
 float     tim_1us_delta_btn;       //плавное увеличение по кнопке на 100ms
 float     tim_1us_delta_time;      //увеличение\уменьшение на каждую секунду
 float     tim_1us_interval_static; //float_to_int
 float     tim_1us_delta_num;       //увеличение\уменьшение на каждый шаг
	
 uint16_t  impulse_now_us;          //текущее значение импульса, us

 uint16_t  tim_5ms_interval;        //tim21_ARR

 uint16_t  prog_time;
 uint16_t  prog_time_num;
	
 uint8_t   prog_num_now;
 uint8_t   prog_num_last;
 uint8_t   prog_num_max;
 uint8_t   btn_mode;
 uint8_t   device_status;  //STOP=0//RUN=1//PAUSE=2///SLEEP=3
} ;

#define OBP_NUM 65

//A
#define ENA0 LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_15); 
#define ENA1 LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_15); 
	
//B
#define ENB0 LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_14); 
#define ENB1 LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_14); 

#define A00 LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_3); 
#define A01 LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_3);

#define A10 LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_7); 
#define A11 LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_7);

#define A20 LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_6); 
#define A21 LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_6);

#define MUX_OFF ENA0;ENB0;A00;A10;A20

//19V
#define DCDC_OFF LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);
#define DCDC_ON  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);

#define DC19_OFF LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_15);
#define DC19_ON  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_15); 

#define CURRENT_CONTROL_OFF LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_10);
#define CURRENT_CONTROL_ON  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_10); 

#define Led_blink  spi_led_timer=30;LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_0);
#define Led_Off    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_0);
#define Led_On     LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_0);
	

void dev_init(void);
void dev_100ms(void);
void dev_1sec(void);
void dev_btn_click(void);


__STATIC_INLINE void set_channel(uint8_t channel);
__STATIC_INLINE uint16_t set_channel_output(uint8_t channel);

__STATIC_INLINE void set_channel(uint8_t channel) {
     //CHANGE_CHANNELS
     switch (channel) {
       case 1:  case 17: case 33: A00;A10;A21; break;
       case 2:  case 18: case 34: A00;A11;A21; break;
       case 3:  case 19: case 35: A01;A11;A20; break;
       case 4:  case 20: case 36: A01;A10;A20; break;
       case 5:  case 21: case 37: A00;A10;A21; break;
       case 6:  case 22: case 38: A00;A11;A21; break;
       case 7:  case 23: case 39: A01;A11;A20; break;
       case 8:  case 24: case 40: A01;A10;A20; break;
       case 9:  case 25: case 41: A01;A10;A21; break;
       case 10: case 26: case 42: A01;A11;A21; break;
       case 11: case 27: case 43: A00;A11;A20; break;
       case 12: case 28: case 44: A00;A10;A20; break;
       case 13: case 29: case 45: A01;A10;A21; break;
       case 14: case 30: case 46: A01;A11;A21; break;
       case 15: case 31: case 47: A00;A11;A20; break;
       case 16: case 32: case 48: A00;A10;A20; break;
     }
}

////ENA1_ENB0
//GPIOC->ODR=0x8000;
////ENA0_ENB1
//GPIOC->ODR=0x4000;
//#define ENA1_ENB0 GPIOC->ODR=0x8000;
//#define ENA0_ENB1 GPIOC->ODR=0x4000;

#define ENA1_ENB0 0x8000
#define ENA0_ENB1 0x4000
#define ENA1_ENB1 0xC000

__STATIC_INLINE uint16_t set_channel_output(uint8_t channel) {
     //CHANGE_CHANNELS
     switch (channel) {
       case 1:  case 17: case 33: return ENA1_ENB0;
       case 2:  case 18: case 34: return ENA1_ENB0;
       case 3:  case 19: case 35: return ENA1_ENB0;
       case 4:  case 20: case 36: return ENA1_ENB0;
       case 5:  case 21: case 37: return ENA0_ENB1;
       case 6:  case 22: case 38: return ENA0_ENB1;
       case 7:  case 23: case 39: return ENA0_ENB1;
       case 8:  case 24: case 40: return ENA0_ENB1;
       case 9:  case 25: case 41: return ENA1_ENB0;
       case 10: case 26: case 42: return ENA1_ENB0;
       case 11: case 27: case 43: return ENA1_ENB0;
       case 12: case 28: case 44: return ENA1_ENB0;
       case 13: case 29: case 45: return ENA0_ENB1;
       case 14: case 30: case 46: return ENA0_ENB1;
       case 15: case 31: case 47: return ENA0_ENB1;
       case 16: case 32: case 48: return ENA0_ENB1;
			 default: return 0;
     }
}

static uint16_t us_to_tim_arr(uint16_t time_us) {
  uint16_t tim_ns=time_us*100;
	uint16_t ret_tim_arr=0;
	if (tim_ns>1312) {
			ret_tim_arr=(tim_ns-621.98)/31.233;
	} else {
			ret_tim_arr=(tim_ns-386.81)/48.166;
	}
  return (uint16_t)ret_tim_arr;
}

static uint16_t tim_arr_to_us(uint16_t tim_arr) {
   float ret=0;
   if (tim_arr>20) {
        ret = 31.233*tim_arr + 621.98;
   } else {
        ret = 48.166*tim_arr + 386.81;
	 }
   return (uint16_t)(ret/100);
}

static uint16_t us_to_tim_5ms(uint16_t prog_time) {
  float t_us=0;
  float ret=0;
  t_us=prog_time*100;
  ret=(t_us-15.132)/17.491;
  return (uint16_t)ret;
}

void Send_History(uint8_t prog_num_now,uint8_t change_type);
void Get_Device_Status(uint8_t hight_voltage,uint8_t hv_current_adc,uint8_t hv_current_status);
void Add_To_Spi_Buffer(void);


