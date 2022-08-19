 
 /*===============================================================================
                     ##### DATA EEPROM Programming functions ##### 
 =============================================================================== 
 
    [..] Any operation of erase or program should follow these steps:
    (#) Call the @ref HAL_FLASHEx_DATAEEPROM_Unlock() function to enable the data EEPROM access
        and Flash program erase control register access.
    (#) Call the desired function to erase or program data.
    (#) Call the @ref HAL_FLASHEx_DATAEEPROM_Lock() to disable the data EEPROM access
        and Flash program erase control register access(recommended
        to protect the DATA_EEPROM against possible unwanted operation).
*/ 

#include "stm32l0xx_hal_flash.h"
#include "eeprom.h"

//Data EEPROM 0x08080000 - 0x080803FF 1024 bytes Data EEPROM
#define START_ADDRESS 0x08080000
#define END_ADDRESS   0x080803FC
#define MIN_NUM 1
#define MAX_NUM 255

//         1          2          3          4
//0x08080000 0x08080004 0x08080008 0x0808000C
//         5          5          7          8
//0x08080010 0x08080014 0x08080018 0x0808001C
//       253        254        255        256
//0x080803F0 0x080803F4 0x080803F8 0x080803FC



uint32_t EEPROM_Read_DataNum (uint16_t data_num)  {
	uint32_t address;
	if ((data_num<=255)&&(data_num>0)) {
		address=START_ADDRESS+0x04*(data_num-1);
		return EEPROM_Read(address);
	} else {
		return 0;
	}
}

uint8_t EEPROM_Write_DataNum (uint16_t data_num, uint32_t data) {
	uint32_t address;
	if ((data_num<=255)&&(data_num>0)) {
		address=START_ADDRESS+0x04*(data_num-1);
		return EEPROM_Write(address,data);
	} else {
		return 1;
	}
	
}


uint8_t EEPROM_WaitForLastOperatin(void) {
	uint8_t err=0;
	/* Wait for last operation to be completed */
	uint32_t timer=0xFFF;
  while((__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) && (timer>0)) 
  { 
    timer--;
  }
	if (timer==0) return 1;
	/* Check FLASH End of Operation flag  */
  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_EOP))
  {
    /* Clear FLASH End of Operation pending bit */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
  }
	return err;
}

uint32_t EEPROM_Read (uint32_t address)
{
  return (*(__IO uint32_t *)address);
}

uint8_t EEPROM_Write (uint32_t address,uint32_t data )
{
	uint8_t ret=0;
  /* Unlocking the Data memory and FLASH_PECR register access*/
	FLASH->PEKEYR = FLASH_PEKEY1;
	FLASH->PEKEYR = FLASH_PEKEY2;
	
	SET_BIT(FLASH->PECR, FLASH_PECR_FIX);
	//Если же он установлен, то перед записью байта, 
	//полу слова или слова автоматически выполняется фаза стирания, 
	//из чего следует, что время записи будет равно 2xTprog.
	
//	/* ERASE - Write 00000000h to valid address in the data memory */
//  *(__IO uint32_t *) address = 0x00000000U;
//	EEPROM_WaitForLastOperatin();
	/* Program word (32-bit) at a specified address.*/
   *(__IO uint32_t *)address = data; //*(__IO uint16_t *)address = (uint16_t) data;
	ret=EEPROM_WaitForLastOperatin();
	
	/* Set the PELOCK Bit to lock the data memory and FLASH_PECR register access */
  SET_BIT(FLASH->PECR, FLASH_PECR_PELOCK);
	
	return ret;
}

