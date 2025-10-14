/**
 *      ADS1256 STM32F7 SPI Driver
 *
 *      Author:     Ethan Garnier
 *      Date:       2024
 */
#include "lockin_amplification/ADS1256.h"
#include "math.h"

#define __ADS_SELECT(__ADS__) ((__ADS__)->csPort->BSRR = (uint32_t)((__ADS__)->csPin << 16))
#define __ADS_UNSELECT(__ADS__) ((__ADS__)->csPort->BSRR = (__ADS__)->csPin)

/**
 *  uint32_t _dwt_delay_init(void)
 *
 *  Initialization code for the Data Watchpoint Trigger. This is
 *  required for us to count clock cycles and achieve precise
 *  timings and delays in driver code. This code has been taken
 *  from the following article by Khaled Magdy:
 *  https://deepbluembedded.com/stm32-delay-microsecond-millisecond-utility-dwt-delay-timer-delay/
 *
 *  Returns 0 on success, 1 indicating error.
 */
static uint32_t
_dwt_delay_init (void)
{
    /* Disable TRC */
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
    /* Enable TRC */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;

    // Magic fix for when debugging https://stackoverflow.com/a/37345912
    DWT->LAR = 0xC5ACCE55;

    /* Disable clock cycle counter */
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
    /* Enable  clock cycle counter */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // 0x00000001;

    /* Reset the clock cycle counter value */
    DWT->CYCCNT = 0;

    /* 3 NO OPERATION instructions */
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");

    /* Check if clock cycle counter has started */
    if (DWT->CYCCNT)
    {
        return 0; /*clock cycle counter started*/
    }
    else
    {
        return 1; /*clock cycle counter not started*/
    }
}

/**
 *  void _delay_us(volatile uint32_t au32_microseconds)
 *
 *  Wait au32_microseconds. Count is enabled by the Data Watchpoint
 *  Trigger, therefore the above DWT_Delay_Init(void) must be called
 *  successfully before using this function. This code has been taken
 *  from the following article by Khaled Magdy:
 *  https://deepbluembedded.com/stm32-delay-microsecond-millisecond-utility-dwt-delay-timer-delay/
 */
static void
_delay_us (volatile uint32_t au32_microseconds)
{
    uint32_t au32_initial_ticks = DWT->CYCCNT;
    uint32_t au32_ticks = (HAL_RCC_GetHCLKFreq () / 1000000);
    au32_microseconds *= au32_ticks;
    while ((DWT->CYCCNT - au32_initial_ticks) < au32_microseconds - au32_ticks)
        ;
}

/**
 *  HAL_StatusTypeDef _spi_disable(SPI_HandleTypeDef *spiHandle)
 *
 *  Disable the provided STM32 SPI handler based on the standard
 *  SPI master procedure outlined in Section 32.5.9 of Reference
 *  Manual 0385.
 *
 *  Returns a HAL_StatusTypeDef indicating success or failure.
 */
static HAL_StatusTypeDef
_spi_disable (SPI_HandleTypeDef *spiHandle)
{
    // 1. Wait for FIFO Tx buffer to finish transmitting
    // by waiting until FTLVL[1:0] is 0b00
    while ((spiHandle->Instance->SR & 0x1800))
        ;

    // 2. Wait until BSY flag is 0
    while ((spiHandle->Instance->SR & 0x0080))
        ;

    // 3. Disable SPI by clearing SPE bit (bit 6)
    spiHandle->Instance->CR1 &= ~0x0040;

    // 4. Flush FIFO Rx buffer until FRLVL[1:0] is 0b00
    volatile uint8_t tempReg = 0x00;
    while ((spiHandle->Instance->SR & 0x0600))
    {
        tempReg = (volatile uint8_t)spiHandle->Instance->DR;
        (void)tempReg; // Avoids GCC unused warning
    }

    return HAL_OK;
}

/**
 *  uint8_t _spi_transmit_byte(SPI_HandleTypeDef *spiHandle, uint8_t byte)
 *
 *  Transmit a single byte of data over STM32 SPI interface specified by
 *  spiHandle. This function is optimized using the -Ofast compilation flag to
 *  make it as fast as possible.
 *
 *  Returns a HAL_StatusTypeDef indicating success
 */
static HAL_StatusTypeDef
_spi_transmit_byte (SPI_HandleTypeDef *spiHandle, uint8_t byte)
{
    __HAL_LOCK (spiHandle);

    // Enable SPI by setting SPE bit (bit 6)
    spiHandle->Instance->CR1 |= 0x0040;

    // Wait until the SPI transmit buffer is empty
    while (!(spiHandle->Instance->SR & 0x0002))
        ;

    // Send the byte to the data register
    *((volatile uint8_t *)(&(spiHandle->Instance->DR))) = byte;

    _spi_disable (spiHandle);

    __HAL_UNLOCK (spiHandle);

    return HAL_OK;
}

/**
 *  uint8_t _spi_receive_bytes(SPI_HandleTypeDef *spiHandle, uint8_t nBytes, uint8_t *rxBuff)
 *
 *  Receive nBytes of data the STM32 SPI interface specified by spiHandle and
 *  store in array pointed to by rxBuff. This function is optimized using the
 *  -Ofast compilation flag to make it as fast as possible. The array pointed to
 *  by rxBuff must be greater than or equal to nBytes in size.
 *
 *  Returns HAL_StatusTypeDef indicating sucess.
 */
static HAL_StatusTypeDef
_spi_receive_bytes (SPI_HandleTypeDef *spiHandle, uint8_t nBytes, uint8_t *rxBuff)
{
    __HAL_LOCK (spiHandle);

    // Set the RXNE event to fire when Rx buffer is 1/4 full (8 bits)
    spiHandle->Instance->CR2 |= 0x1000;

    // Enable SPI by setting SPE bit (bit 6)
    spiHandle->Instance->CR1 |= 0x0040;

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
            *((volatile uint8_t *)(&(spiHandle->Instance->DR))) = 0x00;

            // Clear this flag so if the RXNE event has yet to fire,
            // we do not resend the dummy transmission
            txNext = 0U;
        }

        // Check if Rx buffer is full (bit 0 of status register is set)
        if ((spiHandle->Instance->SR & 0x0001) == 0x0001)
        {
            ((uint8_t *)rxBuff)[bytesRead] = *(volatile uint8_t *)(&(spiHandle->Instance->DR));
            nBytes--;
            bytesRead++;

            // Set this flag so we send another dummy transmission
            // if there is another byte we want to receive
            txNext = 1U;
        }
    }

    _spi_disable (spiHandle);

    __HAL_UNLOCK (spiHandle);

    return HAL_OK;
}

/**
 *  uint8_t _spi_receive_byte(SPI_HandleTypeDef *spiHandle, uint8_t *rxBuff)
 *
 *  Receive a single byte of data from the STM32 SPI interface specified by
 *  spiHandle and store in array pointed to by rxBuff. This function is
 *  optimized using the -Ofast compilation flag to make it as fast as possible.
 *  The array pointed to by rxBuff must greater than 8 bits large.
 *
 *  Returns HAL_StatusTypeDef indicating sucess.
 */
static HAL_StatusTypeDef
_spi_receive_byte (SPI_HandleTypeDef *spiHandle, uint8_t *rxBuff)
{
    __HAL_LOCK (spiHandle);

    // Set the RXNE event to fire when Rx buffer is 1/4 full (8 bits)
    spiHandle->Instance->CR2 |= 0x1000;

    // Enable SPI by setting SPE bit (bit 6)
    spiHandle->Instance->CR1 |= 0x0040;

    // Wait for the Tx buffer to be empty
    while (!(spiHandle->Instance->SR & 0x0002))
        ;

    // Since we are operating in master mode, we must send a dummy
    // transmission to generate a clock cycle to read in this data.
    *((volatile uint8_t *)(&(spiHandle->Instance->DR))) = 0x00;

    // Wait for the Rx buffer to be 1/4 full (have 8 bits)
    while (!(spiHandle->Instance->SR & 0x0001))
        ;

    // Store the byte currently in the SPI data register
    // at the starting address of rxBuff
    *rxBuff = *(volatile uint8_t *)(&(spiHandle->Instance->DR));

    _spi_disable (spiHandle);

    __HAL_UNLOCK (spiHandle);

    return HAL_OK;
}

/**
 *  HAL_StatusTypeDef ADS1256_Init(ADS1256 *ads, SPI_HandleTypeDef *spiHandle,
 *      GPIO_TypeDef *csPort, uint16_t csPin, GPIO_TypeDef *rdyPort, uint16_t rdyPin)
 *
 *  Initialize the ADS1256 struct with spiHandle and the ports/pins
 *  for the active low chip select and data ready GPIO lines. This function
 *  must be called before usage of the other functions in this driver. The
 *  ADS1256 is initialized with the following settings:
 *      - Most significant bit first
 *      - Auto-calibration enabled
 *      - Analog input buffer enabled
 *      - Sensor detect current source disabled
 *      - Input gain of 1
 *      - Data rate of 30 000 SPS
 *      - Single ended analog input on channel 0
 *  Any of these settings may be overwritten using the ADS1256_Register_Write
 *  function.
 *
 *  This function performs a full self-calibration before returning.
 *
 *  Returns a HAL_StatusTypeDef.
 */
HAL_StatusTypeDef
ADS1256_Init (ADS1256 *ads,
              SPI_HandleTypeDef *spiHandle,
              GPIO_TypeDef *csPort,
              uint16_t csPin,
              GPIO_TypeDef *rdyPort,
              uint16_t rdyPin,
              GPIO_TypeDef *resetPort,
              uint16_t resetPin)
{
    HAL_StatusTypeDef status;

    // Initialize the ADS1256 struct
    ads->spiHandle = spiHandle;
    ads->csPort = csPort;
    ads->csPin = csPin;
    ads->rdyPort = rdyPort;
    ads->rdyPin = rdyPin;
    ads->resetPort = resetPort;
    ads->resetPin = resetPin;
    ads->mode = MODE_SINGLE_ENDED;

    // Initialize the Data Watchpoint Trigger
    if (_dwt_delay_init ())
        return HAL_ERROR;

    // Issue a hardware reset using the reset port/pin
    ADS1256_Hardware_Reset (ads);

    // Check to make sure SPI communication is working properly
    // by verifying the device id of the connected ADS1256.
    uint8_t deviceId;
    ADS1256_Read_ID (ads, &deviceId);
    if (deviceId != ADS1256_ID)
        return HAL_ERROR;

    // Initialize A/D control register with following settings:
    //  - Digital clock output disabled
    //  - Sensor detect current source disabled
    //  - Gain of 1
    status = ADS1256_Register_Write (ads, ADCON_REG, ADCON_DEFAULT);
    if (status != HAL_OK)
        return status;

    // Initialize status register with following settings:
    //  - Most significant bit first
    //  - Auto-calibration enabled
    //  - Analog input buffer enabled
    status = ADS1256_Register_Write (ads, STATUS_REG, STATUS_DEFAULT);
    if (status != HAL_OK)
        return status;

    // Wait for auto-calibration to finish
    HAL_Delay (1);

    // Initialize data rate register with a data rate of 30 000 sps
    status = ADS1256_Register_Write (ads, DRATE_REG, DRATE_DEFAULT);
    if (status != HAL_OK)
        return status;

    // Wait for auto-calibration to finish
    HAL_Delay (1);

    // Initialize the ADS1256 to use analog input channel 0
    status = ADS1256_Set_Channel (ads, CHANNEL_AIN0);
    if (status != HAL_OK)
        return status;

    // Wait for auto-calibration to finish
    HAL_Delay (1);

    // Verify the configuration of the above registers
    // by reading back their contents
    status = ADS1256_Verify_Config (ads);
    if (status != HAL_OK)
        return status;

    // Perform a self calibration of the device
    // before finishing initialization.
    return ADS1256_Self_Cal (ads);
}

HAL_StatusTypeDef
ADS1256_Verify_Config (ADS1256 *ads)
{
    HAL_StatusTypeDef status;
    uint8_t regBits;

    // Read the contents of the ADCON register and
    // verify they are configured with:
    //  - Digital clock output disabled
    //  - Sensor detect current source disabled
    //  - Gain of 1
    status = ADS1256_Register_Read (ads, ADCON_REG, &regBits);
    if (status != HAL_OK || regBits != ADCON_DEFAULT)
        return HAL_ERROR;

    // Read the contents of the STATUS register and
    // verify they are configured with:
    //  - Most significant bit first
    //  - Auto-calibration enabled
    //  - Analog input buffer enabled
    status = ADS1256_Register_Read (ads, STATUS_REG, &regBits);
    if (status != HAL_OK || (regBits & 0x0E) != STATUS_DEFAULT)
        return HAL_ERROR;

    // Read the contents of the DRATE register and
    // verify they are configured with a data rate of 30k sps
    status = ADS1256_Register_Read (ads, DRATE_REG, &regBits);
    if (status != HAL_OK || regBits != DRATE_DEFAULT)
        return HAL_ERROR;

    // Read the contents of the MUX register and
    // verify that the ADS1256 is configured using
    // channels AIN0 and AINCOM
    status = ADS1256_Register_Read (ads, MUX_REG, &regBits);
    if (status != HAL_OK || (regBits & 0xF8) != MUX_DEFAULT)
        return HAL_ERROR;

    return HAL_OK;
}

/**
 *  void ADS1256_Hardware_Reset(ADS1256 *ads)
 *
 *  Issue a hardware reset of the ADS1256
 *  using the supplied reset port and pin.
 */
void
ADS1256_Hardware_Reset (ADS1256 *ads)
{
    // Bring the reset pin high
    ads->resetPort->BSRR = ads->resetPin;
    HAL_Delay (200);
    // Bring the reset pin low to initiate a reset
    ads->resetPort->BSRR = (uint32_t)ads->resetPin << 16;
    HAL_Delay (200);
    // Bring the reset pin high again
    ads->resetPort->BSRR = ads->resetPin;

    // Wait until the DRDY pin is brought low
    while ((ads->rdyPort->IDR & ads->rdyPin) != 0)
        ;
}

/**
 *  HAL_StatusTypeDef ADS1256_Software_Synchronize(ADS1256 *ads)
 *
 *  Issue a software SYNC command to
 *  the connected ADS1256 and issue a WAKEUP
 *  command after 24 master clock periods.
 *
 *  Returns a HAL_StatusTypeDef
 */
HAL_StatusTypeDef
ADS1256_Software_Synchronize (ADS1256 *ads)
{
    HAL_StatusTypeDef status;

    // Issue a SYNC/WAKEUP command to restart conversion process
    status = ADS1256_Send_Command (ads, SYNC_CMD);
    if (status != HAL_OK)
        return status;

    _delay_us (4);

    status = ADS1256_Send_Command (ads, WAKEUP_CMD);
    if (status != HAL_OK)
        return status;

    // Wait until the DRDY pin is brought low
    while ((ads->rdyPort->IDR & ads->rdyPin) != 0)
        ;

    return status;
}

/**
 *  HAL_StatusTypeDef ADS1256_Send_Command(ADS1256 *ads, uint8_t command)
 *
 *  Sends the command byte to the ADS1256 specified by ads over SPI and waits
 *  the minimum amount of time before another command is allowed to be sent.
 *
 *  Returns a HAL_StatusTypeDef.
 */
HAL_StatusTypeDef
ADS1256_Send_Command (ADS1256 *ads, ADS1256_Command command)
{
    HAL_StatusTypeDef status;

    // Bring the chip select line low
    __ADS_SELECT (ads);

    // Transmit the command byte to the ADS1256
    status = _spi_transmit_byte (ads->spiHandle, command);

    // Bring the chip select line high
    __ADS_UNSELECT (ads);

    // Based on the command, we must wait either 24 CLKIN periods
    // or until the DRDY line goes low before we can allow another command
    if (command
        & (RREG_CMD_1 | RREG_CMD_2 | WREG_CMD_1 | WREG_CMD_2 | RDATA_CMD | RDATAC_CMD | RESET_CMD
           | SYNC_CMD | WAKEUP_CMD))
    {
        // Wait 24 CLKIN periods (3.125 us)
        _delay_us (4);
    }
    else
    {
        // Wait until the DRDY pin is brought low
        while ((ads->rdyPort->IDR & ads->rdyPin) != 0)
            ;
    }

    return status;
}

/**
 *  HAL_StatusTypeDef ADS1256_Register_Read(ADS1256 *ads, uint8_t regAddr, uint8_t *inBuffer)
 *
 *  Read the single byte contents of the register pointed to by regAddr into
 *  the inBuffer byte array of size 1 using the spiHandle of ads.
 *
 *  Returns a HAL_StatusTypeDef.
 */
HAL_StatusTypeDef
ADS1256_Register_Read (ADS1256 *ads, ADS1256_Register regAddr, uint8_t *inBuffer)
{
    HAL_StatusTypeDef status;

    // Bring the chip select line low
    __ADS_SELECT (ads);

    // Send the first read register command bit or'd with the
    // address of the register we want to read
    status = _spi_transmit_byte (ads->spiHandle, RREG_CMD_1 | regAddr);
    if (status != HAL_OK)
        goto endRead;

    // Send the second read register command indicating we
    // only want to read this register
    status = _spi_transmit_byte (ads->spiHandle, RREG_CMD_2);
    if (status != HAL_OK)
        goto endRead;

    // Wait 50 CLKIN periods (assuming CLKIN = 7.68 MHz this would be 6.51
    // us)
    _delay_us (7);

    // Receive the byte contents of the requested register from ADS1256
    status = _spi_receive_byte (ads->spiHandle, inBuffer);

endRead:
    // Bring the chip select line high
    __ADS_UNSELECT (ads);

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
HAL_StatusTypeDef
ADS1256_Register_Write (ADS1256 *ads, ADS1256_Register regAddr, uint8_t data)
{
    HAL_StatusTypeDef status;

    // Bring the chip select line low
    __ADS_SELECT (ads);

    // Send the first write register command bit or'd with the
    // address of the register we want to write to
    status = _spi_transmit_byte (ads->spiHandle, WREG_CMD_1 | regAddr);
    if (status != HAL_OK)
        goto endWrite;

    // Send the second write register command indicating we
    // only want to write to this register
    status = _spi_transmit_byte (ads->spiHandle, WREG_CMD_2);
    if (status != HAL_OK)
        goto endWrite;

    // Send the data we want to write to the register
    status = _spi_transmit_byte (ads->spiHandle, data);

endWrite:
    // Bring the chip select line high
    __ADS_UNSELECT (ads);

    return status;
}

/**
 *  HAL_StatusTypeDef ADS1256_Set_Mode(ADS1256 *ads, ADS1256_Mode mode)
 *
 *  Change the current operating mode of the ADS1256 to one specified in
 *  the ADS1256_Mode enum (single-ended or differential). If the user is
 *  changing from differential to signle-ended mode, then the negative input
 *  channel is automatically changed to AINCOM.
 *
 *  Returns a HAL_StatusTypeDef.
 */
HAL_StatusTypeDef
ADS1256_Set_Mode (ADS1256 *ads, ADS1256_Mode mode)
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
            status = ADS1256_Register_Read (ads, MUX_REG, &mux);
            if (status != HAL_OK)
                return status;

            status = ADS1256_Register_Write (ads, MUX_REG, mux & 0xF8);
            if (status != HAL_OK)
                return status;

            status = ADS1256_Software_Synchronize (ads);
        }
    }

    return status;
}

/**
 *  HAL_StatusTypeDef ADS1256_Set_Channel(ADS1256 *ads, ADS1256_Channel pChannel)
 *
 *  Change the positive analog input channel of the ADS1256 to pChannel. If the
 *  ADS is configured in differential input mode, then the corresponding
 *  negative input channel will be automatically selected, if one exists. If the
 *  ADS1256 is configered in single-ended input mode, then the AINCOM channel
 *  will be selected as the negative input channel.
 *
 *  Returns a HAL_StatusTypeDef.
 */
HAL_StatusTypeDef
ADS1256_Set_Channel (ADS1256 *ads, ADS1256_Channel pChannel)
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
            // properly in differential mode, return an error
            return HAL_ERROR;
        }
    }
    else
    {
        nChannel = CHANNEL_AINCOM;
    }

    // Change the channels in the MUX register
    status = ADS1256_Register_Write (ads, MUX_REG, (pChannel << 4) | nChannel);
    if (status != HAL_OK)
        return status;

    status = ADS1256_Software_Synchronize (ads);

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
HAL_StatusTypeDef
ADS1256_Self_Cal (ADS1256 *ads)
{
    return ADS1256_Send_Command (ads, SELFCAL_CMD);
}

/**
 *  uint8_t ADS1256_Read_Id(ADS1256 *ads, uint8_t id)
 *
 *  Reads the contents of the ADS1256's STATUS register
 *  and returns the device id found in the upper 4 bits.
 *
 *  Returns a HAL_StatusTypeDef.
 */
HAL_StatusTypeDef
ADS1256_Read_ID (ADS1256 *ads, uint8_t *id)
{
    HAL_StatusTypeDef status;

    // Read the STATUS register where the device ID is stored
    // in the upper 4 bits of the register
    status = ADS1256_Register_Read (ads, STATUS_REG, id);
    if (status != HAL_OK)
        return status;

    // Shift the value of the byte by 4
    *id = *id >> 4;

    return HAL_OK;
}

/**
 *  HAL_StatusTypeDef ADS1256_Read_Data(ADS1256 *ads, uint32_t outputCode)
 *
 *  Read the latest 24-bit conversion result from the ADS1256 into
 *  the outputCode 32 bit unsigned integer
 *
 *  Returns a HAL_StatusTypeDef.
 */
HAL_StatusTypeDef
ADS1256_Read_Data (ADS1256 *ads, uint32_t *outputCode)
{
    HAL_StatusTypeDef status;
    uint8_t inBuffer[3] = { 0 };

    // Wait until the DRDY pin is brought low
    while ((ads->rdyPort->IDR & ads->rdyPin) != 0)
        ;

    // Bring the chip select line low
    __ADS_SELECT (ads);

    status = _spi_transmit_byte (ads->spiHandle, RDATA_CMD);
    if (status != HAL_OK)
        goto endRead;

    // Wait 50 CLKIN periods (assuming CLKIN = 7.68 MHz this would be 6.51 us)
    _delay_us (7);

    // Read the 24 bit conversion results and store in
    // the supplied buffer:
    status = _spi_receive_bytes (ads->spiHandle, 3, inBuffer);

    // Convert the 3 bytes captured from the ADS1256 into
    // a single 24 bit value.
    *outputCode = (((inBuffer[0] & 0x80) ? 0xFF : 0x00) << 24) | ((uint32_t)inBuffer[0] << 16)
                  | ((uint32_t)inBuffer[1] << 8) | inBuffer[2];

endRead:
    // Bring the chip select line high
    __ADS_UNSELECT (ads);

    return status;
}

/**
 *  float ADS1256_Read_Voltage(ADS1256 *ads, float *voltage)
 *
 *  Read the latest conversion result from the ADS1256 and
 *  calculate the analog voltage that was provided.
 *
 *  Returns a HAL_StatusTypeDef
 */
HAL_StatusTypeDef
ADS1256_Read_Voltage (ADS1256 *ads, float *voltage)
{
    HAL_StatusTypeDef status;
    uint8_t PGA, PGAReg;
    uint32_t outputCode;

    // Read the current conversion result
    status = ADS1256_Read_Data (ads, &outputCode);
    if (status != HAL_OK)
        return HAL_ERROR;

    // Read the contents of the ADCON register to get the current
    // value of the Programmable Gain
    status = ADS1256_Register_Read (ads, ADCON_REG, &PGAReg);
    if (status != HAL_OK)
        return HAL_ERROR;
    PGA = pow (2, (PGAReg & 0x07));
    if (PGA > 64)
        PGA = 64;

    // Calculate the input analog voltage from the output code,
    // the reference voltage, the number of code words, and the
    // programmable gain.
    unsigned long denom = (unsigned long)PGA * (unsigned long)BIT_RANGE;
    unsigned long numer = (unsigned long)VREF * outputCode;
    *voltage = (float)numer / denom;

    return HAL_OK;
}
