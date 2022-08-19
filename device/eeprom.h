#include <stdint.h>
#include <string.h>
#include <math.h>

#include "stm32l0xx.h"

uint32_t EEPROM_Read_DataNum  (uint16_t data_num);
uint8_t  EEPROM_Write_DataNum (uint16_t data_num, uint32_t data);

uint8_t  EEPROM_WaitForLastOperatin(void) ;
uint32_t EEPROM_Read  (uint32_t address);
uint8_t  EEPROM_Write (uint32_t address,uint32_t data );

