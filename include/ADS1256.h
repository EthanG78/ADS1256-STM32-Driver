/**
 *      ADS1256 STM32F7 SPI Driver
 * 
 *      Author:     Ethan Garnier
 *      Date:       2024
*/
#ifndef ADS1256_SPI_DRIVER_H
#define ADS1256_SPI_DRIVER_H

#include "stm32f7xx_hal.h" /* Needed for structure defs */
#include "stdint.h"

#define ADS1256_ID      3
#define VREF            5
#define BIT_RANGE       0x7FFFFF
#define MASTER_CLK      7680000UL

/* Default register values for initialization */
typedef enum
{
    ADCON_DEFAULT   =   0x00,   /* CLKOUT: Disabled, SDCS: Disabled, PGA: 1 */
    STATUS_DEFAULT  =   0x06,   /* ORDER: MSB, ACAL: Enabled, BUFEN: Enabled */
    DRATE_DEFAULT   =   0xF0,   /* 30k Samples per Second */
    MUX_DEFAULT     =   0x08,   /* AINp: AIN0, AINb: AINCOM */
} ADS1256_Register_Defaults;

/* ADS1256 Register Addresses */
typedef enum
{
    STATUS_REG      =   0x00,
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
    RREG_CMD_1      =   0x10,   /* low 4 bits are reg address to read*/
    RREG_CMD_2      =   0x00,   /* low 4 bits are number regs to read - 1*/
    WREG_CMD_1      =   0x50,   /* low 4 bits are reg address to write*/
    WREG_CMD_2      =   0x00,   /* low 4 bits are number regs to write - 1*/
    SELFCAL_CMD     =   0xF0,
    SELFOCAL_CMD    =   0xF1,
    SELFGCAL_CMD    =   0xF2,
    SYSOCAL_CMD     =   0xF3,
    SYSGCAL_CMD     =   0xF4,
    SYNC_CMD        =   0xFC,
    STANDBY_CMD     =   0xFD,
    RESET_CMD       =   0xFE,
} ADS1256_Command;

/* ADS1256 Gain Settings */
typedef enum
{
    GAIN_1          =   0,
    GAIN_2,
    GAIN_4,
    GAIN_8,
    GAIN_16,
    GAIN_32,
    GAIN_64
} ADS1256_Gain;

typedef enum
{
    DRATE_30K_SPS   =   0xF0,
    DRATE_15K_SPS   =   0xE0,
    DRATE_7_5K_SPS  =   0xD0,
    DRATE_3_75k_SPS =   0xC0,
    DRATE_2K_SPS    =   0xB0,
    DRATE_1K_SPS    =   0xA1,
    DRATE_500_SPS   =   0x92,
    DRATE_100_SPS   =   0x82,
    DRATE_60_SPS    =   0x72,
    DRATE_50_SPS    =   0x63,
    DRATE_30_SPS    =   0x53,
    DRATE_25_SPS    =   0x43,
    DRATE_15_SPS    =   0x33,
    DRATE_10_SPS    =   0x23,
    DRATE_5_SPS     =   0x13,
    DRATE_2_5_SPS   =   0x03
} ADS1256_Data_Rate;

typedef enum
{
    CHANNEL_AIN0    =   0x00,
    CHANNEL_AIN1,
    CHANNEL_AIN2,
    CHANNEL_AIN3,
    CHANNEL_AIN4,
    CHANNEL_AIN5,
    CHANNEL_AIN6,
    CHANNEL_AIN7,
    CHANNEL_AINCOM,
} ADS1256_Channel;

typedef enum
{
    MODE_SINGLE_ENDED = 1,
    MODE_DIFF,
} ADS1256_Mode;

/* ADS1256 Sensor Struct */
typedef struct
{ 
    // STM32F7 HAL specific handler for SPI communication
    SPI_HandleTypeDef *spiHandle;

    // Current ADC mode of operation
    ADS1256_Mode mode;

    // Port where the chip select GPIO pin is located
    GPIO_TypeDef *csPort;

    // 16 bit pin number of the chip select GPIO pin (active low)
    uint16_t csPin;

    // Port where the data ready (DRDY) GPIO pin is located
    GPIO_TypeDef *rdyPort;

    // 16 bit pin number of the data ready (DRDY) GPIO pin (active low)
    uint16_t rdyPin;

    // Port where the reset GPIO pin is located
    GPIO_TypeDef *resetPort;

    // 16 bit pin number of the reset GPIO pin (active low)
    uint16_t resetPin;
} ADS1256;

HAL_StatusTypeDef ADS1256_Init(ADS1256 *ads, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin, GPIO_TypeDef *rdyPort, uint16_t rdyPin, GPIO_TypeDef *resetPort, uint16_t resetPin);
HAL_StatusTypeDef ADS1256_Verify_Config(ADS1256 *ads);
void ADS1256_Hardware_Reset(ADS1256 *ads);
HAL_StatusTypeDef ADS1256_Software_Synchronize(ADS1256 *ads);
HAL_StatusTypeDef ADS1256_Send_Command(ADS1256 *ads, ADS1256_Command command);
HAL_StatusTypeDef ADS1256_Register_Read(ADS1256 *ads, ADS1256_Register regAddr, uint8_t *inBuffer);
HAL_StatusTypeDef ADS1256_Register_Write(ADS1256 *ads, ADS1256_Register regAddr, uint8_t data);
HAL_StatusTypeDef ADS1256_Set_Mode(ADS1256 *ads, ADS1256_Mode mode);
HAL_StatusTypeDef ADS1256_Set_Channel(ADS1256 *ads, ADS1256_Channel pChannel);
HAL_StatusTypeDef ADS1256_Self_Cal(ADS1256 *ads);
HAL_StatusTypeDef ADS1256_Read_ID(ADS1256 *ads, uint8_t *id);
HAL_StatusTypeDef ADS1256_Read_Data(ADS1256 *ads, uint32_t *outputCode);
HAL_StatusTypeDef ADS1256_Read_Voltage(ADS1256 *ads, float *voltage);
HAL_StatusTypeDef ADS1256_Read_Continuous(ADS1256 *ads, uint32_t outputArrSize, uint32_t *outputArr);
HAL_StatusTypeDef ADS1256_Read_Dual_Channel(ADS1256 *ads, ADS1256_Channel channel1, uint32_t *channel1Output, ADS1256_Channel channel2, uint32_t *channel2Output);
#endif
