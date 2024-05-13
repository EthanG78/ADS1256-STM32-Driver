/**
 *      ADS1256 STM32F7 SPI Driver
 * 
 *      Author:     Ethan Garnier
 *      Date:       May 13th, 2024
*/
#include "ADS1256.h"

HAL_StatusTypeDef ADS1256_Init(ADS1256 *ads, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin, GPIO_TypeDef *rdyPort, uint16_t rdyPin)
{
    HAL_StatusTypeDef status;

    // Initialize the ADS1256 struct
    ads->spiHandle = spiHandle;
    ads->inputBuffer[0] = 0x00;
    ads->inputBuffer[1] = 0x00;
    ads->inputBuffer[2] = 0x00;
    ads->csPort = csPort;
    ads->csPin = csPin;
    ads->rdyPort = rdyPort;
    ads->rdyPin = rdyPin;

    // Perform a device reset using the RESET command
    status = ADS1256_Send_Command(ads, RESET_CMD);
    if (status != HAL_OK) return status;

    // Turn off digital clock output (it isn't reset with RESET command)
    // by clearing bit 6 and 7 of the ADCON register
    uint8_t clkOutOff = 0b00011111;
    status = ADS1256_Register_Write(ads, ADCON_REG, 0, &clkOutOff);
    if (status != HAL_OK) return status;

    // Initialize status register with following settings:
    //  - Most significant bit first
    //  - Auto-calibration enabled
    //  - Analog input buffer disabled
    // NOTE: Any of these bits can be changed manually
    // using the ADS1256_Register_Write command after init.
    uint8_t statusInit = 0b11110101;
    status = ADS1256_Register_Write(ads, STATUS_REG, 0, &statusInit);
    if (status != HAL_OK) return status;

    // Perform a full self-calibration (gain and offset cal)
    status = ADS1256_Send_Command(ads, SELFCAL_CMD);
    return status;
}

HAL_StatusTypeDef ADS1256_Send_Command(ADS1256 *ads, uint8_t command)
{
    // NOTE: CS must stay low
}

HAL_StatusTypeDef ADS1256_Register_Read(ADS1256 *ads, uint8_t regAddr, uint8_t nRegRead)
{

}

HAL_StatusTypeDeg ADS1256_Register_Write(ADS1256 *ads, uint8_t regAddr, uint8_t nRegWrite, uint8_t *data);
{

}