#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "dev.h"

//------------------- MED -------------------------------
volatile uint8_t  channel_counter=1;

uint8_t btn_left_click=0;
uint8_t btn_right_click=0;
uint8_t btn_tim_update=0;

//------------------ PROGRAMM --------------------------//
struct  struct_impulse_prog prog1;
//------------------ TIME1 0.7-120us ------------------//
struct struct_impulse_now impulse_now;

//----- EXTERN ------------//
extern uint8_t sleep;
//extern bool spi_slave_send;

//extern const uint8_t SPI_RX_TX_SIZE;
extern uint8_t spi_tx_count;
extern uint8_t spi_tx_buffer[];

extern t_spi_send_buffer spi_send_buffer[];

//---------------- TIME1 0.3-120us ---- END --------------//

void dev_init(void) {
 
//   //LOAD_PROGRAMM     //

//  prog1.prog_id       = 1;     //номер шаблона
//  prog1.imp_voltage   = 19;    //19V
//	                             //312,5us IBP=5 ms T паузы, 1 — 120 млс +-1
//  prog1.imp_t_pause   = 5;     //ms время между импульсами одного канала 
//  prog1.imp_t_min     = 1*10;  //us +-1
//  prog1.imp_t_max     = 60*10; //us +-1
//  prog1.prog_time     = 1200;  //sec
//  prog1.prog_num      = 20;    //число шагов
//  prog1.prog_time_num = 1200;  //sec, време перехода к конечному уровню
//  prog1.prog_button_mode = 1;  //по кнопкам
//	Режим работы программы:
//1 – ручное изменение уровня запрещено
//2 – ручное изменение уровня разрешено


  impulse_now.tim_1us_interval         = us_to_tim_arr(prog1.imp_t_min);
  impulse_now.tim_1us_interval_min     = impulse_now.tim_1us_interval;
  impulse_now.tim_1us_interval_static  = impulse_now.tim_1us_interval;
  impulse_now.tim_1us_interval_max     = us_to_tim_arr(prog1.imp_t_max); 
  impulse_now.tim_5ms_interval         = us_to_tim_5ms(prog1.imp_t_pause);
  impulse_now.prog_time                = prog1.prog_time;
  impulse_now.prog_num_now             = 1;
  impulse_now.prog_num_max             = prog1.prog_num;
  impulse_now.prog_time_num            = prog1.prog_time_num;
  impulse_now.btn_mode                 = prog1.prog_button_mode;
	
	if (prog1.imp_voltage==0x16) {
		DC19_OFF; //???
	} else {
		DC19_ON; //???
	}
   
  if (impulse_now.prog_num_max>1) {
      impulse_now.tim_1us_delta_num   = (float) (impulse_now.tim_1us_interval_max-impulse_now.tim_1us_interval_min)/(impulse_now.prog_num_max-1); //increment in 1step
      impulse_now.tim_1us_delta_time  = (float) (impulse_now.tim_1us_interval_max-impulse_now.tim_1us_interval_min)/impulse_now.prog_time_num;//increment in 1sec
      impulse_now.tim_1us_delta_btn   = impulse_now.tim_1us_delta_time*5  ; //*10/2увеличение каждые 100ms в течении 2сек
  } else {
			//NUM=1
			impulse_now.tim_1us_delta_num  = 0;
			impulse_now.tim_1us_delta_time = 0;
			impulse_now.tim_1us_delta_btn  = 0;
			prog1.prog_button_mode         = 0;//отключить управление по кнопкам
  }
	
	LL_TIM_SetAutoReload(TIM2,impulse_now.tim_1us_interval);  //1us
	LL_TIM_SetAutoReload(TIM21,impulse_now.tim_5ms_interval); //312,5us IBP=5 ms T паузы, 1 — 120 млс +-0,1
	
//	//START_TIMER
//	LL_TIM_EnableCounter(TIM21); //312,5us //312,5us IBP=5 ms T паузы, 1 — 120 млс +-0,1
  
}//void dev_init()

void dev_100ms(void) {
	if (btn_tim_update) {
	 if (impulse_now.tim_1us_interval_static<impulse_now.tim_1us_interval_to) {
		 impulse_now.tim_1us_interval_static+=impulse_now.tim_1us_delta_btn;
		 impulse_now.tim_1us_interval = (uint16_t) impulse_now.tim_1us_interval_static;
		 
		 if (impulse_now.tim_1us_interval<impulse_now.tim_1us_interval_min) 
			{
				impulse_now.tim_1us_interval=impulse_now.tim_1us_interval_min;
			}
			
		 if (impulse_now.tim_1us_interval_static>impulse_now.tim_1us_interval_to) {
			 btn_tim_update=0;
		 }
	 } else {
		 impulse_now.tim_1us_interval_static-=impulse_now.tim_1us_delta_btn;
		 impulse_now.tim_1us_interval = (uint16_t) impulse_now.tim_1us_interval_static;
		 
		 if (impulse_now.tim_1us_interval>impulse_now.tim_1us_interval_max) 
			{
				impulse_now.tim_1us_interval=impulse_now.tim_1us_interval_max;
			}
			
		 if (impulse_now.tim_1us_interval_static<impulse_now.tim_1us_interval_to) {
			btn_tim_update=0;
		 }
	 }
 }//if (btn_tim_update) {
}//void dev_100ms(void)

uint16_t prog_count_timer=0;

void dev_1sec(void) {
	  float prog_num;

	  if (impulse_now.device_status==1) {
				//увеличение импульса каждую секунду, если кнопка не нажата
				if ((impulse_now.tim_1us_interval<impulse_now.tim_1us_interval_max)&&(btn_tim_update==0)) {
					 impulse_now.tim_1us_interval_static += impulse_now.tim_1us_delta_time;
					 impulse_now.tim_1us_interval = (uint16_t) impulse_now.tim_1us_interval_static;
					 //пересчет номера шага
					 prog_num  = 1+(float) (impulse_now.tim_1us_interval-impulse_now.tim_1us_interval_min)/impulse_now.tim_1us_delta_num;
					 impulse_now.prog_num_now = (uint8_t)rint(prog_num);
					 if (impulse_now.prog_num_now!=impulse_now.prog_num_last) {
						 Send_History(impulse_now.prog_num_now,0x01);
						 impulse_now.prog_num_last=impulse_now.prog_num_now;
					 }

					 //ram_history[ram_history_write].datetime_offset=datetime_offset;
					 //if (++ram_history_write==RAM_HISTORY_SIZE) {ram_history_write=0;}
				}

				if (impulse_now.prog_time_num!=0) {impulse_now.prog_time_num--;}

				//1sec  ----- END -------------
				if (impulse_now.prog_time!=0) {
					 impulse_now.prog_time--;
					 if (impulse_now.prog_time==0) {
						 //devise_stop
						 DCDC_OFF;
						 impulse_now.device_status=0;//STOP
						 //send_to_rf52
						 spi_tx_buffer[0]=0xff;
						 spi_tx_buffer[1]=0x03; //len
						 spi_tx_buffer[2]=0x01; //cmd //Время вышло, импульсы кончились
						 spi_tx_buffer[3]=0x55; //просто так
						 Add_To_Spi_Buffer();
					 }
				}//if (impulse_now.prog_time!=0) {
			
		}//if (impulse_now.device_status==1) {4
		
		
	
}//void dev_1s(void)

void dev_btn_click(void) {
	
	if (btn_left_click==1) {
		btn_left_click=0;
		if (impulse_now.prog_num_now>1) {
			//IMPULSE_MINUS
			impulse_now.prog_num_now--;
			impulse_now.prog_num_last=impulse_now.prog_num_now;
			
			impulse_now.tim_1us_interval_to = impulse_now.tim_1us_interval_min+impulse_now.tim_1us_delta_num*(impulse_now.prog_num_now-1);
			if (impulse_now.tim_1us_interval_to<impulse_now.tim_1us_interval_min) 
			{
				impulse_now.tim_1us_interval_to=impulse_now.tim_1us_interval_min;
			}
			btn_tim_update=1;
			//impulse_now.tim_1us_interval = impulse_now.tim_1us_interval_min+impulse_now.tim_1us_delta_num*(impulse_now.prog_num_now-1);
			//impulse_now.tim_1us_interval_static=impulse_now.tim_1us_interval;
			Send_History(impulse_now.prog_num_now,0x02);
		}
	}//if (btn_left_click==1) {
	
	if (btn_right_click==1) {
		  btn_right_click=0;
		//IMPULSE_PLUS
			if (impulse_now.prog_num_now<impulse_now.prog_num_max) {
				 impulse_now.prog_num_now++;
				 impulse_now.prog_num_last=impulse_now.prog_num_now;
				
				 impulse_now.tim_1us_interval_to = impulse_now.tim_1us_interval_min+impulse_now.tim_1us_delta_num*(impulse_now.prog_num_now-1);
				 if (impulse_now.tim_1us_interval_to>impulse_now.tim_1us_interval_max) 
				 {
					impulse_now.tim_1us_interval_to=impulse_now.tim_1us_interval_max;
				 }
				 btn_tim_update=1;  
				 //impulse_now.tim_1us_interval = impulse_now.tim_1us_interval_min+impulse_now.tim_1us_delta_num*(impulse_now.prog_num_now-1);
				 //impulse_now.tim_1us_interval_static=impulse_now.tim_1us_interval;
				Send_History(impulse_now.prog_num_now,0x02);
			}
	}//if (btn_right_click==1) {	
	
}//void dev_btn_click(void)

//prog_num_now - номер уровня, на который произошел переход
//change_type - 1 - auto; 2 - btn_click
void Send_History(uint8_t prog_num_now,uint8_t change_type) {
	spi_tx_buffer[0]=0xff;
	spi_tx_buffer[1]=0x04;//len
	spi_tx_buffer[2]=0x02;//cmd
	spi_tx_buffer[3]=prog_num_now;
	spi_tx_buffer[4]=change_type;
	Add_To_Spi_Buffer();
}//void Send_History(uint8_t prog_num_now,uint8_t change_type) {

//void Send_Device_Off(void) {
//	
//	
//}

void Get_Device_Status(uint8_t hight_voltage,uint8_t hv_current_adc,uint8_t hv_current_status) {
	//DEVICE_STATUS
	spi_tx_buffer[0]= 0xff;
	spi_tx_buffer[1]= 12;   //len=spi_rx_buf[1];
	spi_tx_buffer[2]= 0x0a; //cmd=spi_rx_buf[2];
	spi_tx_buffer[3]= impulse_now.device_status; //Device status STOP\RUN\PAUSE
	
	impulse_now.impulse_now_us = tim_arr_to_us(impulse_now.tim_1us_interval);

	spi_tx_buffer[4]=  (uint8_t)(impulse_now.impulse_now_us);    //Текущее T импульса 
	spi_tx_buffer[5]=  (uint8_t)(impulse_now.impulse_now_us>>8); //Текущее T импульса 
	
	spi_tx_buffer[6]=  (uint8_t)(impulse_now.prog_time);         //Текущее время выполнения программы, с
	spi_tx_buffer[7]=  (uint8_t)(impulse_now.prog_time>>8);      //Текущее время выполнения программы, с
	
	spi_tx_buffer[8]=  (uint8_t)(impulse_now.prog_num_now);      //Текущий уровень
	
	spi_tx_buffer[9] = (uint8_t)(impulse_now.prog_time_num);    //Время  ДО перехода к конечному уровню работы, с
	spi_tx_buffer[10]= (uint8_t)(impulse_now.prog_time_num>>8); //Время  ДО перехода к конечному уровню работы, с
	
	if (hight_voltage) {
	  spi_tx_buffer[11]= (uint8_t)(prog1.imp_voltage);      // 190/10 = Высокое напряжение,В
	} else {
		spi_tx_buffer[11]=0;
	}
	
	spi_tx_buffer[12]= (uint8_t)(hv_current_adc);     // мА = Потребление HV, мА
	spi_tx_buffer[13]= (uint8_t)(hv_current_status);  // 0-100% состояние прибора\на языке или нет
	
	if (impulse_now.device_status!=4) { //!=SLEEP	
		Add_To_Spi_Buffer();
	}
}//void Get_Device_Status(void) {

void Add_To_Spi_Buffer(void) {
	memcpy(&spi_send_buffer[spi_tx_count],&spi_tx_buffer,SPI_RX_TX_SIZE);
	if (spi_tx_count<SPI_RX_TX_SIZE) {spi_tx_count++;} else {spi_tx_count=0;}
}








