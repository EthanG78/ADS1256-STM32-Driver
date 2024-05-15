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

    // Magic fix for when debugging https://stackoverflow.com/a/37345912
    DWT->LAR = 0xC5ACCE55; 

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
 *  HAL_StatusTypeDef SPI_Disable(SPI_HandleTypeDef *spiHandle, uint32_t timeout)
 *  
 *  Disable the provided STM32 SPI handler based on the standard
 *  SPI master procedure outlined in Section 32.5.9 of Reference
 *  Manual 0385.
 * 
 *  TODO: Implement a timeout function that actually times out after
 *  waiting for buffers to queue and returns a HAL_ERROR or HAL_BUSY
 * 
 *  Returns a HAL_StatusTypeDef indicating success or failure.
*/
HAL_StatusTypeDef SPI_Disable(SPI_HandleTypeDef *spiHandle, uint32_t timeout)
{
    // 1. Wait for FIFO Tx buffer to finish transmitting
    // by waiting until FTLVL[1:0] is 0b00
    while ((spiHandle->Instance->SR & 0x1800) == 0x1800);

    // 2. Wait until BSY flag is 0
    while ((spiHandle->Instance->SR & 0x0080) == 0x0080);

    // 3. Disable SPI by clearing SPE bit (bit 6)
    spiHandle->Instance->CR1 &= ~0x0040;

    // 4. Flush FIFO Rx buffer until FRLVL[1:0] is 0b00 
    volatile uint8_t *tempRx = (volatile uint8_t *)&spiHandle->Instance->DR;
    volatile uint8_t tempReg = 0x00;
    while ((spiHandle->Instance->SR & 0x0600) == 0x0600)
    {
        tempReg = *tempRx;
        (void)tempReg;  /* Avoids GCC unused warning */
    }

    return HAL_OK;
}

/**
 *  uint8_t SPI_Transmit_Byte(SPI_HandleTypeDef *spiHandle, uint8_t byte)
 *  
 *  Transmit a single byte of data over STM32 SPI interface specified by spiHandle. This function
 *  is optimized using the -Ofast compilation flag to make it as fast as possible. 
 *
 *  Returns a HAL_StatusTypeDef indicating success
*/
__attribute__((optimize("-Ofast"))) HAL_StatusTypeDef SPI_Transmit_Byte(SPI_HandleTypeDef *spiHandle, uint8_t byte)
{
	__HAL_LOCK(spiHandle);

	// Enable SPI by setting SPE bit (bit 6)
    spiHandle->Instance->CR1 |= 0x0040;

    // Send a single byte of data to the SPI data register
    // when the Tx buffer is empty
    if ((spiHandle->Instance->SR & 0x0002) == 0x0002)
    {
        *((volatile uint8_t *)&spiHandle->Instance->DR) = byte;
    }

    SPI_Disable(spiHandle, 0);

    __HAL_UNLOCK(spiHandle);

    return HAL_OK;
}

/**
 *  SPI_Transmit_Bytes(SPI_HandleTypeDef *spiHandle, uint8_t nBytes, uint8_t *txBuff)
 *  
 *  Transmit nBytes of data from array pointed to by txBuff over STM32 SPI interface specified by spiHandle. This function
 *  is optimized using the -Ofast compilation flag to make it as fast as possible. The array pointed to by txBuff must
 *  be greater than or equal to nBytes in size.
 *
 *  Returns a HAL_StatusTypeDef indicating success
*/
__attribute__((optimize("-Ofast"))) HAL_StatusTypeDef SPI_Transmit_Bytes(SPI_HandleTypeDef *spiHandle, uint8_t nBytes, uint8_t *txBuff)
{
    __HAL_LOCK(spiHandle);

	// Enable SPI by setting SPE bit (bit 6)
	spiHandle->Instance->CR1 |= 0x0040;

    // Send n bytes of data to SPI Tx
    // buffer via the data register
    uint8_t bytesSent = 0;
    while (nBytes > 0)
    {
        // Check if Tx buffer is empty (bit 1 of status register is set)
        if ((spiHandle->Instance->SR & 0x0002) == 0x0002)
        {
        	*((volatile uint8_t *)&spiHandle->Instance->DR) = txBuff[bytesSent];
        	nBytes--;
        	bytesSent++;
        }
    }

    SPI_Disable(spiHandle, 0);

    __HAL_UNLOCK(spiHandle);

    return HAL_OK;
}

/**
 *  uint8_t SPI_Receive_Bytes(SPI_HandleTypeDef *spiHandle, uint8_t nBytes, uint8_t *rxBuff)
 *  
 *  Receive nBytes of data the STM32 SPI interface specified by spiHandle and store in array pointed to by rxBuff. This function
 *  is optimized using the -Ofast compilation flag to make it as fast as possible. The array pointed to by rxBuff must
 *  be greater than or equal to nBytes in size.
 *
 *  Returns HAL_StatusTypeDef indicating sucess.
*/
__attribute__((optimize("-Ofast"))) HAL_StatusTypeDef SPI_Receive_Bytes(SPI_HandleTypeDef *spiHandle, uint8_t nBytes, uint8_t *rxBuff)
{
    __HAL_LOCK(spiHandle);

	// Enable SPI by setting SPE bit (bit 6)
	spiHandle->Instance->CR1 |= 0x0040;

	// Set the RXNE event to fire when Rx buffer is 1/4 full (8 bits)
	spiHandle->Instance->CR2 |= 0x1000;

    // Read n bytes of data from SPI Rx
    // buffer via the data register
    uint8_t bytesRead = 0;
    uint8_t txNext = 1U;
    while (nBytes > 0)
    {
    	// Since we are operating in master mode, we must send a dummy
    	// transmission to generate a clock cycle to read in this data.
    	// Check if Tx buffer is empty (bit 1 of status register is set)
    	if (((spiHandle->Instance->SR & 0x0002) == 0x0002) && (txNext == 1U))
    	{
    		*((volatile uint8_t *)&spiHandle->Instance->DR) = 0x00;

            // Clear this flag so if the RXNE event has yet to fire,
            // we do not resend the dummy transmission
    		txNext = 0U;
    	}

        // Check if Rx buffer is full (bit 0 of status register is set)
        if ((spiHandle->Instance->SR & 0x0001) == 0x0001)
        {
            ((uint8_t *)rxBuff)[bytesRead] = *(volatile uint8_t *)&spiHandle->Instance->DR;
            nBytes--;
            bytesRead++;

            // Set this flag so we send another dummy transmission 
            // if there is another byte we want to receive
            txNext = 1U;
        }
    }

    SPI_Disable(spiHandle, 0);

    __HAL_UNLOCK(spiHandle);

    return HAL_OK;
}

/**
 *  uint8_t SPI_Receive_Byte(SPI_HandleTypeDef *spiHandle, uint8_t *rxBuff)
 *  
 *  Receive a single byte of data from the STM32 SPI interface specified by spiHandle and store in 
 *  array pointed to by rxBuff. This function is optimized using the -Ofast compilation flag to make 
 *  it as fast as possible. The array pointed to by rxBuff must greater than 8 bits large.
 *
 *  Returns HAL_StatusTypeDef indicating sucess.
*/
__attribute__((optimize("-Ofast"))) HAL_StatusTypeDef SPI_Receive_Byte(SPI_HandleTypeDef *spiHandle, uint8_t *rxBuff)
{
    __HAL_LOCK(spiHandle);

	// Enable SPI by setting SPE bit (bit 6)
	spiHandle->Instance->CR1 |= 0x0040;

	// Set the RXNE event to fire when Rx buffer is 1/4 full (8 bits)
	spiHandle->Instance->CR2 |= 0x1000;

    // Wait for the Tx buffer to be empty
    while ((spiHandle->Instance->SR & ~0x0002) != 0x0002);

    // Since we are operating in master mode, we must send a dummy
    // transmission to generate a clock cycle to read in this data.
    *((volatile uint8_t *)&spiHandle->Instance->DR) = 0x00;

    // Wait for the Rx buffer to be 1/4 full (have 8 bits)
    while ((spiHandle->Instance->SR & ~0x0001) != 0x0001);

    // Store the byte currently in the SPI data register
    // at the starting address of rxBuff
    *rxBuff = *(volatile uint8_t *)&spiHandle->Instance->DR;

    SPI_Disable(spiHandle, 0);

    __HAL_UNLOCK(spiHandle);

    return HAL_OK;
}