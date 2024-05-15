/**
 *      ADS1256 SPI Driver Utility Functions
 *  
 *      Author:     Ethan Garnier
 *      Date:       May 13th, 2024
*/
#ifndef ADS1256_UTIL_H
#define ADS1256_UTIL_H

#include "stdint.h"
#include "stm32f7xx_hal.h" /* Needed for structure defs */

uint32_t DWT_Delay_Init(void);
HAL_StatusTypeDef SPI_Transmit_Byte(SPI_HandleTypeDef *spiHandle, uint8_t byte);
HAL_StatusTypeDef SPI_Transmit_Bytes(SPI_HandleTypeDef *spiHandle, uint8_t nBytes, uint8_t *txBuff);
HAL_StatusTypeDef SPI_Receive_Byte(SPI_HandleTypeDef *spiHandle, uint8_t *rxBuff);
HAL_StatusTypeDef SPI_Receive_Bytes(SPI_HandleTypeDef *spiHandle, uint8_t nBytes, uint8_t *rxBuff);

/**
 *  void DWT_Delay_us(volatile uint32_t au32_microseconds)
 *  
 *  Wait au32_microseconds. Count is enabled by the Data Watchpoint
 *  Trigger, therefore the above DWT_Delay_Init(void) must be called
 *  successfully before using this function. This code has been taken 
 *  from the following article by Khaled Magdy:
 *  https://deepbluembedded.com/stm32-delay-microsecond-millisecond-utility-dwt-delay-timer-delay/
*/
static inline void DWT_Delay_us(volatile uint32_t au32_microseconds)
{
  uint32_t au32_initial_ticks = DWT->CYCCNT;
  uint32_t au32_ticks = (HAL_RCC_GetHCLKFreq() / 1000000);
  au32_microseconds *= au32_ticks;
  while ((DWT->CYCCNT - au32_initial_ticks) < au32_microseconds-au32_ticks);
}
#endif