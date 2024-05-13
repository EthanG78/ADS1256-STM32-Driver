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
static inline void DWT_Delay_us(volatile uint32_t au32_microseconds);
uint8_t SPI_Transmit(SPI_HandleTypeDef *spiHandle, uint8_t nBytes, uint8_t *txBuff);
uint8_t SPI_Receive(SPI_HandleTypeDef *spiHandle, uint8_t nBytes, uint8_t *rxBuff);

#endif