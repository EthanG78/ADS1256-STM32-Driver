/**
 *      ADS1256 STM32F7 SPI Driver
 * 
 *      Author:     Ethan Garnier
 *      Date:       May 13th, 2024
*/
#ifndef ADS1256_SPI_DRIVER_H
#define ADS1256_SPI_DRIVER_H

#include "stm32f7xx_hal.h" /* Needed for structure defs */
#include "stdint.h"

/* ADS1256 Register Addresses */
typedef enum
{
    MUX_REG         =   0x01,
    ADCON_REG       =   0x02,
    DRATE_REG       =   0x03,
    IO_REG          =   0x04,
    OFC0_REG        =   0x05,
    OFC1_REG        =   0x06,
    OFC2_REG        =   0x07,
    FSC0_REG        =   0x08,   
    FSC1_REG        =   0x09,
    FSC2_REG        =   0x0A,
} ADS1256_Register;

/* ADS1256 SPI Command Bytes */
typedef enum
{
    WAKEUP_CMD      =   0x00,
    RDATA_CMD       =   0x01,
    RDATAC_CMD      =   0x03,
    SDATAC_CMD      =   0x0F,
    RREG_CMD_1      =   0x10,    /* low 4 bits are reg address to read*/
    RREG_CMD_2      =   0x00,    /* low 4 bits are number regs to read - 1*/
    WREG_CMD_1      =   0x50,    /* low 4 bits are reg address to write*/
    WREG_CMD_2      =   0x00,    /* low 4 bits are number regs to write - 1*/
    SELFCAL_CMD     =   0xF0,
    SELFOCAL_CMD    =   0xF1,
    SELFGCAL_CMD    =   0xF2,
    SYSOCAL_CMD     =   0xF3,
    SYSGCAL_CMD     =   0xF4,
    SYNC_CMD        =   0xFC,
    STANDBY_CMD     =   0xFD,
    RESET_CMD       =   0xFE,
    WAKEUP_CMD      =   0xFF,
} ADS1256_Command;

/* ADS1256 Sensor Struct */
typedef struct
{ 
    // STM32F7 HAL specific handler for SPI communication
    SPI_HandleTypeDef *spiHandle;

    // Port where the chip select GPIO pin is located
    GPIO_TypeDef *csPort;

    // 16 bit pin number of the chip select GPIO pin (active low)
    uint16_t csPin;

    // Port where the data ready (DRDY) GPIO pin is located
    GPIO_TypeDef *rdyPort;

    // 16 bit pin number of the data ready (DRDY) GPIO pin (active low)
    uint16_t rdyPin;
} ADS1256;

HAL_StatusTypeDef ADS1256_Init(ADS1256 *ads, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin, GPIO_TypeDef *rdyPort, uint16_t rdyPin);
HAL_StatusTypeDef ADS1256_Send_Command(ADS1256 *ads, ADS1256_Command command);
HAL_StatusTypeDef ADS1256_Register_Read(ADS1256 *ads, ADS1256_Register regAddr, uint8_t *inBuffer);
HAL_StatusTypeDef ADS1256_Register_Write(ADS1256 *ads, ADS1256_Register regAddr, uint8_t data);


#endif