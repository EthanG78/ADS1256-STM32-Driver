/**
 *      ADS1256 STM32F7 SPI Driver
 * 
 *      Author:     Ethan Garnier
 *      Date:       May 13th, 2024
*/
#include "ADS1256.h"
#include "util.h"

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

    // Turn off digital clock output (it isn't reset with RESET command)
    // by clearing bit 6 and 7 of the ADCON register
    status = ADS1256_Register_Write(ads, ADCON_REG, 0, 0b00011111);
    if (status != HAL_OK) return status;

    // Initialize status register with following settings:
    //  - Most significant bit first
    //  - Auto-calibration enabled
    //  - Analog input buffer disabled
    // NOTE: Any of these bits can be changed manually
    // using the ADS1256_Register_Write command after init.
    status = ADS1256_Register_Write(ads, STATUS_REG, 0, 0b11110101);
    if (status != HAL_OK) return status;

    // Perform a full self-calibration (gain and offset cal)
    status = ADS1256_Send_Command(ads, SELFCAL_CMD);
    return status;
}

__attribute__((optimize("-Ofast"))) HAL_StatusTypeDef ADS1256_Send_Command(ADS1256 *ads, uint8_t command)
{
    // NOTE: CS must stay low
}

// NOTE: Only allow for contents of a single register to be read at a time
__attribute__((optimize("-Ofast"))) HAL_StatusTypeDef ADS1256_Register_Read(ADS1256 *ads, uint8_t regAddr, uint8_t *inBuffer)
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

__attribute__((optimize("-Ofast"))) HAL_StatusTypeDef ADS1256_Register_Write(ADS1256 *ads, uint8_t regAddr, uint8_t data)
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