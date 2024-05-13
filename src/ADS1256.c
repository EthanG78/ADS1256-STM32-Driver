/**
 *      ADS1256 STM32F7 SPI Driver
 * 
 *      Author:     Ethan Garnier
 *      Date:       May 13th, 2024
*/
#include "ADS1256.h"
#include "util.h"

/**
 *  HAL_StatusTypeDef ADS1256_Init(ADS1256 *ads, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin, GPIO_TypeDef *rdyPort, uint16_t rdyPin)
 *  
 *  Initialize the ADS1256 struct with spiHandle and the ports/pins
 *  for the active low chip select and data ready GPIO lines. This function
 *  must be called before usage of the other functions in this driver. The ADS1256
 *  is initialized with the following settings:
 *      - Most significant bit first
 *      - Auto-calibration enabled
 *      - Analog input buffer disabled
 *      - Sensor detect current source disabled
 *      - Input gain of 1
 *      - Data rate of 30 000 SPS
 *      - Single ended analog input on channel 0
 *  Any of these settings may be overwritten using the ADS1256_Register_Write function.
 * 
 *  This function performs a full self-calibration before returning.
 *  
 *  Returns a HAL_StatusTypeDef.
*/
HAL_StatusTypeDef ADS1256_Init(ADS1256 *ads, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin, GPIO_TypeDef *rdyPort, uint16_t rdyPin)
{
    HAL_StatusTypeDef status;

    // Initialize the ADS1256 struct
    ads->spiHandle = spiHandle;
    ads->csPort = csPort;
    ads->csPin = csPin;
    ads->rdyPort = rdyPort;
    ads->rdyPin = rdyPin;

    // Initialize the Data Watchpoint Trigger
    if (DWT_Delay_Init()) return HAL_ERROR;

    // Perform a device reset using the RESET command
    status = ADS1256_Send_Command(ads, RESET_CMD);
    if (status != HAL_OK) return status;

    // Initialize A/D control register with following settings:
    //  - Digital clock output disabled
    //  - Sensor detect current source disabled
    //  - Gain of 1
    status = ADS1256_Register_Write(ads, ADCON_REG, 0x00 | GAIN_1);
    if (status != HAL_OK) return status;

    // Initialize status register with following settings:
    //  - Most significant bit first
    //  - Auto-calibration enabled
    //  - Analog input buffer disabled
    status = ADS1256_Register_Write(ads, STATUS_REG, 0b11110101);
    if (status != HAL_OK) return status;

    // Initialize data rate register with a data rate of 30 000 sps
    status = ADS1256_Register_Write(ads, DRATE_REG, DRATE_30K_SPS);
    if (status != HAL_OK) return status;

    // Initialize the ADS1256 in single-ended 
    status = ADS1256_Set_Mode(ads, MODE_SINGLE_ENDED);
    if (status != HAL_OK) return status;

    // Initialize the ADS1256 to use analog input channel 0
    status = ADS1256_Set_Channel(ads, CHANNEL_AIN0);
    if (status != HAL_OK) return status;

    return status;
}

/**
 *  HAL_StatusTypeDef ADS1256_Send_Command(ADS1256 *ads, uint8_t command)
 *  
 *  Sends the command byte to the ADS1256 specified by ads over SPI and waits
 *  50 clock periods before returning.  
 * 
 *  Returns a HAL_StatusTypeDef.
*/
__attribute__((optimize("-Ofast"))) HAL_StatusTypeDef ADS1256_Send_Command(ADS1256 *ads, ADS1256_Command command)
{
    // Bring the chip select line low
    ads->csPort->BSRR = (uint32_t)ads->csPin << 16;

    // Initialize the SPI peripheral by setting SPE bit
    ads->spiHandle->Instance->CR1 |= 0x0040;
    
    // Transmit the command over SPI
    if (SPI_Transmit(ads->spiHandle, 1, &command) != 1)
    {
        return HAL_ERROR;
    }

    // Wait 50 master clock periods
    // If we assume fclk is 7.68 MHz, then
    // this delay would be 6.51 us
    DWT_Delay_us(7);

    // Disable SPI using procedure outlined in
    // Section 32.5.9 of Reference Manual 0385:
    // Wait until FTLVL[1:0] is 0b00 and BSY is 0
    while ((ads->spiHandle->Instance->SR & 0x1800) == 0x1800
        || (ads->spiHandle->Instance->SR & 0x0080) == 0x0080);

    // Disable SPI by clearing SPE bit (bit 6)
    ads->spiHandle->Instance->CR1 &= 0xFFBF;

    // Bring the chip select line high
    ads->csPort->BSRR = ads->csPin;

    return HAL_OK;
}

/**
 *  HAL_StatusTypeDef ADS1256_Register_Read(ADS1256 *ads, uint8_t regAddr, uint8_t *inBuffer)
 *  
 *  Read the single byte contents of the register pointed to by regAddr into
 *  the inBuffer byte array of size 1 using the spiHandle of ads.  
 * 
 *  Returns a HAL_StatusTypeDef.
*/
__attribute__((optimize("-Ofast"))) HAL_StatusTypeDef ADS1256_Register_Read(ADS1256 *ads, ADS1256_Register regAddr, uint8_t *inBuffer)
{
    HAL_StatusTypeDef status;

    // Build the full command to be sent over SPI
    uint8_t cmdSeq[2];
    cmdSeq[0] = RREG_CMD_1 & (regAddr & 0x0F);
    cmdSeq[1] = RREG_CMD_2; /* Only allow for reading 1 register at a time */ 

    // Bring the chip select line low
    ads->csPort->BSRR = (uint32_t)ads->csPin << 16;

    // Initialize the SPI peripheral by setting SPE bit
    ads->spiHandle->Instance->CR1 |= 0x0040;

    // Send the command sequence indicating the start
    // register cmdSeq[0] and how many registers should
    // be read cmdSeq[1]
    if (SPI_Transmit(ads->spiHandle, 2, &cmdSeq[0]) != 2)
    {
        return HAL_ERROR;
    }

    // Wait 50 master clock periods
    // If we assume fclk is 7.68 MHz, then
    // this delay would be 6.51 us
    DWT_Delay_us(7);

    // Read the data byte contents of the requested register
    if (SPI_Receive(ads->spiHandle, 1, inBuffer) != 1)
    {
        return HAL_ERROR;
    }

    // Disable SPI using procedure outlined in
    // Section 32.5.9 of Reference Manual 0385:
    // Wait until FTLVL[1:0] is 0b00 and BSY is 0
    while ((ads->spiHandle->Instance->SR & 0x1800) == 0x1800
        || (ads->spiHandle->Instance->SR & 0x0080) == 0x0080);

    // Disable SPI by clearing SPE bit (bit 6)
    ads->spiHandle->Instance->CR1 &= 0xFFBF;

    // Bring the chip select line high
    ads->csPort->BSRR = ads->csPin;

    return status;
}

/**
 *  HAL_StatusTypeDef ADS1256_Register_Write(ADS1256 *ads, uint8_t regAddr, uint8_t data)
 *  
 *  Write the single byte, data, to the contents of the register pointed to by
 *  regAddr on the ADS1256 communicating through ads' spiHandle.
 * 
 *  Returns a HAL_StatusTypeDef.
*/
__attribute__((optimize("-Ofast"))) HAL_StatusTypeDef ADS1256_Register_Write(ADS1256 *ads, ADS1256_Register regAddr, uint8_t data)
{
    HAL_StatusTypeDef status;

    // Build the full command to be sent over SPI
    uint8_t cmdSeq[3];
    cmdSeq[0] = WREG_CMD_1 & (regAddr & 0x0F);
    cmdSeq[1] = WREG_CMD_2; /* Only allow for writing to 1 register at a time */ 
    cmdSeq[2] = data;

    // Bring the chip select line low
    ads->csPort->BSRR = (uint32_t)ads->csPin << 16;

    // Initialize the SPI peripheral by setting SPE bit
    ads->spiHandle->Instance->CR1 |= 0x0040;

    // Send the command sequence indicating the start
    // register cmdSeq[0], how many registers should
    // be written to cmdSeq[1] (only 1 for now), and the
    // byte to be written to that register cmdSeq[2]
    if (SPI_Transmit(ads->spiHandle, 3, &cmdSeq[0]) != 3)
    {
        return HAL_ERROR;
    }

    // Disable SPI using procedure outlined in
    // Section 32.5.9 of Reference Manual 0385:
    // Wait until FTLVL[1:0] is 0b00 and BSY is 0
    while ((ads->spiHandle->Instance->SR & 0x1800) == 0x1800
        || (ads->spiHandle->Instance->SR & 0x0080) == 0x0080);

    // Disable SPI by clearing SPE bit (bit 6)
    ads->spiHandle->Instance->CR1 &= 0xFFBF;

    // Bring the chip select line high
    ads->csPort->BSRR = ads->csPin;

    return status;
}

/**
 *  HAL_StatusTypeDef ADS1256_Set_Mode(ADS1256 *ads, ADS1256_Mode mode)
 * 
 *  Change the current operating mode of the ADS1256 to one specified in
 *  the ADS1256_Mode enum (single-ended or differential). If the user is changing 
 *  from differential to signle-ended mode, then the negative input channel
 *  is automatically changed to AINCOM.
 * 
 *  Returns a HAL_StatusTypeDef.
*/
HAL_StatusTypeDef ADS1256_Set_Mode(ADS1256 *ads, ADS1256_Mode mode)
{
    HAL_StatusTypeDef status = HAL_OK;
    if (ads->mode != mode)
    {
        ads->mode = mode;
        if (mode == MODE_SINGLE_ENDED)
        {
            // Change the ads' mode and set the negative input channel to AINCOM
            // while preserving the current positive input channel
            uint8_t mux = 0x00;
            status = ADS1256_Register_Read(ads, MUX_REG, &mux);
            status = ADS1256_Register_Write(ads, MUX_REG, mux & 0x8);
        }
    }

    return status;
}

/**
 *  HAL_StatusTypeDef ADS1256_Set_Channel(ADS1256 *ads, ADS1256_Channel pChannel)
 *  
 *  Change the positive analog input channel of the ADS1256 to pChannel. If the ADS
 *  is configured in differential input mode, then the corresponding negative input channel
 *  will be automatically selected, if one exists. If the ADS1256 is configered in single-ended
 *  input mode, then the AINCOM channel will be selected as the negative input channel.
 * 
 *  Returns a HAL_StatusTypeDef.
*/
HAL_StatusTypeDef ADS1256_Set_Channel(ADS1256 *ads, ADS1256_Channel pChannel)
{
    HAL_StatusTypeDef status;

    // If the ADC is configured in differential mode, then automatically
    // set its negative input channel to the adjacent analog channel, otherwise
    // set the negative input channel to AINCOM.
    uint8_t nChannel;
    if (ads->mode == MODE_DIFF)
    {
        // There are only 4 differential channels, and they are
        //  AIN0 - AIN1
        //  AIN2 - AIN3
        //  AIN4 - AIN5
        //  AIN6 - AIN7
        // So we must first check to make sure pChannel is 0, 2, 4, or 6
        switch (pChannel)
        {
        case CHANNEL_AIN0:
            nChannel = CHANNEL_AIN1;
            break;
        case CHANNEL_AIN2:
            nChannel = CHANNEL_AIN3;
            break;
        case CHANNEL_AIN4:
            nChannel = CHANNEL_AIN5;
            break;
        case CHANNEL_AIN6:
            nChannel = CHANNEL_AIN7;
            break;
        default:
            // If the user has entered a channel that can not be used
            // peroperly in differential mode, return an error
            return HAL_ERROR;
        }
    }
    else
    {
        nChannel = CHANNEL_AINCOM;
    }

    // Change the channels in the MUX register
    status = ADS1256_Register_Write(ads, MUX_REG, (pChannel << 4) | nChannel);
    if (status != HAL_OK) return status;

    // Issue a SYNC/WAKEUP command to restart conversion process
    status = ADS1256_Send_Command(ads, SYNC_CMD);
    if (status != HAL_OK) return status;
    DWT_Delay_us(4);
    status = ADS1256_Send_Command(ads, WAKEUP_CMD);
    if (status != HAL_OK) return status;

    return status;
}

/**
 *  HAL_StatusTypeDef ADS1256_System_Cal(ADS1256 *ads)
 *  
 *  Perform a self calibration on the ADS1256 pointed
 *  to by ads.
 * 
 *  Returns a HAL_StatusTypeDef.
*/
HAL_StatusTypeDef ADS1256_Self_Cal(ADS1256 *ads)
{
    HAL_StatusTypeDef status;

    // Send the command to perform a self calibration
    status = ADS1256_Send_Command(ads, SELFCAL_CMD);

    // Wait until the DRDY pin is brought low
    while ((ads->rdyPort->IDR & ads->rdyPin) != 0);

    return status;
}