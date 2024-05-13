/**
 *      ADS1256 SPI Driver Utility Functions
 *  
 *      Author:     Ethan Garnier
 *      Date:       May 13th, 2024
*/
#include "util.h"

/**
 *  uint32_t DWT_Delay_Init(void)
 * 
 *  Initialization code for the Data Watchpoint Trigger. This is
 *  required for us to count clock cycles and achieve precise 
 *  timings and delays in driver code. This code has been taken 
 *  from the following article by Khaled Magdy:
 *  https://deepbluembedded.com/stm32-delay-microsecond-millisecond-utility-dwt-delay-timer-delay/
 * 
 *  Returns 0 on success, 1 indicating error.
*/
uint32_t DWT_Delay_Init(void)
{
    /* Disable TRC */
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
    /* Enable TRC */
    CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;

    /* Disable clock cycle counter */
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
    /* Enable  clock cycle counter */
    DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

    /* Reset the clock cycle counter value */
    DWT->CYCCNT = 0;

    /* 3 NO OPERATION instructions */
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");

    /* Check if clock cycle counter has started */
    if(DWT->CYCCNT)
    {
       return 0; /*clock cycle counter started*/
    }
    else
    {
      return 1; /*clock cycle counter not started*/
    }
}

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

/**
 *  SPI_Transmit(SPI_HandleTypeDef *spiHandle, uint8_t nBytes, uint8_t *txBuff)
 *  
 *  Transmit nBytes of data from array pointed to by txBuff over STM32 SPI interface specified by spiHandle. This function
 *  is optimized using the -Ofast compilation flag to make it as fast as possible. Configuration of the SPI handler
 *  is not done in this function, only sending of data to its data register. The array pointed to by txBuff must
 *  be greater than or equal to nBytes in size.
 *
 *  Returns the number of bytes successfully transmitted. 
*/
__attribute__((optimize("-Ofast"))) uint8_t SPI_Transmit(SPI_HandleTypeDef *spiHandle, uint8_t nBytes, uint8_t *txBuff)
{
    // Send n bytes of data to SPI Tx
    // buffer via the data register
    uint8_t bytesSent = 0;
    while (nBytes > 0)
    {
        // Check if Tx buffer is empty (bit 1 of status register is set)
        if ((spiHandle->Instance->SR & 0x0002) == 0x0002)
        {
            if (nBytes > 1)
            {
                // write on the data register in packing mode
                spiHandle->Instance->DR = *((uint16_t *)txBuff);
                txBuff += 2 * sizeof(uint8_t);
                nBytes -= 2;
                bytesSent += 2;
            }
            else
            {
                *((volatile uint8_t *)&spiHandle->Instance->DR) = txBuff;
                txBuff += sizeof(uint8_t);
                nBytes--;
                bytesSent++;
            }
        }
    }

    return bytesSent;
}

/**
 *  uint8_t SPI_Receive(SPI_HandleTypeDef *spiHandle, uint8_t nBytes, uint8_t *rxBuff)
 *  
 *  Receive nBytes of data the STM32 SPI interface specified by spiHandle and store in array pointed to by rxBuff. This function
 *  is optimized using the -Ofast compilation flag to make it as fast as possible. Configuration of the SPI handler
 *  is not done in this function, only reading of data from its data register. The array pointed to by rxBuff must
 *  be greater than or equal to nBytes in size.
 *
 *  Returns the number of bytes successfully received. 
*/
__attribute__((optimize("-Ofast"))) uint8_t SPI_Receive(SPI_HandleTypeDef *spiHandle, uint8_t nBytes, uint8_t *rxBuff)
{
    // Read n bytes of data from SPI Rx
    // buffer via the data register
    uint8_t bytesRead = 0;
    while (nBytes > 0)
    {
        // Check if Rx buffer is full (bit 0 of status register is set)
        if ((spiHandle->Instance->SR & 0x0001) == 0x0001)
        {
            (* (uint8_t *)rxBuff) = *(volatile uint8_t *)&spiHandle->Instance->DR;
            rxBuff += sizeof(uint8_t);
            nBytes--;
            bytesRead++;
        }
    }

    return bytesRead;
}