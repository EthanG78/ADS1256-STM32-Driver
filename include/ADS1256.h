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
#define STATUS_REG      0x00
#define MUX_REG         0x01
#define ADCON_REG       0x02
#define DRATE_REG       0x03
#define IO_REG          0x04
#define OFC0_REG        0x05
#define OFC1_REG        0x06
#define OFC2_REG        0x07
#define FSC0_REG        0x08   
#define FSC1_REG        0x09
#define FSC2_REG        0x0A

/* ADS1256 SPI Command Bytes */
#define WAKEUP_CMD      0x00
#define RDATA_CMD       0x01
#define RDATAC_CMD      0x03
#define SDATAC_CMD      0x0F
#define RREG_CMD_1      0x10    /* low 4 bits are reg address to read*/
#define RREG_CMD_2      0x00    /* low 4 bits are number regs to read - 1*/
#define WREG_CMD_1      0x50    /* low 4 bits are reg address to write*/
#define WREG_CMD_2      0x00    /* low 4 bits are number regs to write - 1*/
#define SELFCAL_CMD     0xF0
#define SELFOCAL_CMD    0xF1
#define SELFGCAL_CMD    0xF2
#define SYSOCAL_CMD     0xF3
#define SYSGCAL_CMD     0xF4
#define SYNC_CMD        0xFC
#define STANDBY_CMD     0xFD
#define RESET_CMD       0xFE
#define WAKEUP_CMD      0xFF

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
HAL_StatusTypeDef ADS1256_Send_Command(ADS1256 *ads, uint8_t command);
HAL_StatusTypeDef ADS1256_Register_Read(ADS1256 *ads, uint8_t regAddr, uint8_t *inBuffer);
HAL_StatusTypeDef ADS1256_Register_Write(ADS1256 *ads, uint8_t regAddr, uint8_t data);


#endif