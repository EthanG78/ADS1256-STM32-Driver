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
HAL_StatusTypeDef ADS1256_Init(ADS1256 *ads, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csPort, uint16_t csPin, GPIO_TypeDef *rdyPort, uint16_t rdyPin, GPIO_TypeDef *resetPort, uint16_t resetPin)
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
    if (DWT_Delay_Init()) return HAL_ERROR;

    // Issue a hardware reset using the reset port/pin
    ADS1256_Hardware_Reset(ads);

    // Check to make sure SPI communication is working properly
    // by verifying the device if of the connected ADS1256. It's okay
    // if it fails a few times, that's why we have the timeout!
    uint8_t deviceId;
    uint32_t timeout = 0;
    do
    {
    	ADS1256_Read_ID(ads, &deviceId);
    	timeout++;
    } while (deviceId != ADS1256_ID && timeout < HAL_MAX_DELAY);

    if (deviceId != ADS1256_ID)
    {
        return HAL_ERROR;
    }

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

    // Verify the configuration of the above registers
    // by reading back their contents
    status = ADS1256_Verify_Config(ads);
    if (status != HAL_OK) return status;
 
    // Perform a self calibration of the device
    // before finishing initialization.
    return ADS1256_Self_Cal(ads);
}

HAL_StatusTypeDef ADS1256_Verify_Config(ADS1256 *ads)
{
	HAL_StatusTypeDef status;
	uint8_t regBits;

	// Read the contents of the ADCON register and
	// verify they are configured with:
    //  - Digital clock output disabled
    //  - Sensor detect current source disabled
    //  - Gain of 1
	status = ADS1256_Register_Read(ads, ADCON_REG, &regBits);
	if (status != HAL_OK || regBits != 0x00) return HAL_ERROR;

	// Read the contents of the STATUS register and
	// verify they are configured with:
    //  - Most significant bit first
    //  - Auto-calibration enabled
    //  - Analog input buffer disabled
	do
	{
		status = ADS1256_Register_Read(ads, STATUS_REG, &regBits);
	} while ((regBits & 0x0E) != 0x04);
	if (status != HAL_OK || (regBits & 0x0E) != 0x04) return HAL_ERROR;

	// Read the contents of the DRATE register and
	// verify they are configured with a data rate of 30k sps
	do
	{
		status = ADS1256_Register_Read(ads, DRATE_REG, &regBits);
	} while (regBits != DRATE_30K_SPS);
	if (status != HAL_OK || regBits != DRATE_30K_SPS) return HAL_ERROR;

	// Read the contents of the MUX register and
	// verify that the ADS1256 is configured using
	// channels AIN0 and AINCOM
	do
	{
		status = ADS1256_Register_Read(ads, MUX_REG, &regBits);
	} while ((regBits & 0xF8) == 0x08);
	if (status != HAL_OK || (regBits & 0xF8) == 0x08) return HAL_ERROR;

	return HAL_OK;
}

/**
 * void ADS1256_Hardware_Reset(ADS1256 *ads) 
 * 
 *  Issue a hardware reset of the ADS1256
 *  using the supplied reset port and pin.
*/
void ADS1256_Hardware_Reset(ADS1256 *ads)
{
    // Bring the reset pin high
    ads->resetPort->BSRR = ads->resetPin;
    HAL_Delay(200);
    // Bring the reset pin low to initiate a reset
    ads->resetPort->BSRR = (uint32_t)ads->resetPin << 16;
    HAL_Delay(200);
    // Bring the reset pin high again
    ads->resetPort->BSRR = ads->resetPin;

	// Wait until the DRDY pin is brought low
	while ((ads->rdyPort->IDR & ads->rdyPin) != 0);
}


/**
 *  HAL_StatusTypeDef ADS1256_Send_Command(ADS1256 *ads, uint8_t command)
 *  
 *  Sends the command byte to the ADS1256 specified by ads over SPI and waits
 *  the minimum amount of time before another command is allowed to be sent.
 *  
 *  Returns a HAL_StatusTypeDef.
*/
HAL_StatusTypeDef ADS1256_Send_Command(ADS1256 *ads, ADS1256_Command command)
{
    HAL_StatusTypeDef status;

    // Bring the chip select line low
    ads->csPort->BSRR = (uint32_t)ads->csPin << 16;

    // Transmit the command byte to the ADS1256
    status = SPI_Transmit_Byte(ads->spiHandle, command);

    // Bring the chip select line high
    ads->csPort->BSRR = ads->csPin;

    // Based on the command, we must wait either 24 CLKIN periods
    // or until the DRDY line goes low before we can allow another command
    if (command & (RREG_CMD_1 | RREG_CMD_2 | WREG_CMD_1 | WREG_CMD_2 | RDATA_CMD | RDATAC_CMD | RESET_CMD | SYNC_CMD))
    {
        // Wait 24 CLKIN periods (3.125 us)
        DWT_Delay_us(4);
    }
    else
    {
        // Wait until the DRDY pin is brought low
        while ((ads->rdyPort->IDR & ads->rdyPin) != 0);
    }

    return status;
}

/**
 *  HAL_StatusTypeDef ADS1256_Register_Read(ADS1256 *ads, uint8_t regAddr, uint8_t *inBuffer)
 *  
 *  Read the single byte contents of the register pointed to by regAddr into
 *  the inBuffer byte array of size 1 using the spiHandle of ads.  
 *
 *  TODO: There is a weird bug where you need to call this twice to actually get
 *  the contents of the register. I think the timings with reading from the SPI
 *  bus is off...
 * 
 *  Returns a HAL_StatusTypeDef.
*/
HAL_StatusTypeDef ADS1256_Register_Read(ADS1256 *ads, ADS1256_Register regAddr, uint8_t *inBuffer)
{
    HAL_StatusTypeDef status;

    // Bring the chip select line low
    ads->csPort->BSRR = (uint32_t)ads->csPin << 16;

    // Send the first read register command bit or'd with the
    // address of the register we want to read
    status = SPI_Transmit_Byte(ads->spiHandle, RREG_CMD_1 | regAddr);
    if (status != HAL_OK) goto endRead;

    // Send the second read register command indicating we
    // only want to read this register
    status = SPI_Transmit_Byte(ads->spiHandle, RREG_CMD_2);
    if (status != HAL_OK) goto endRead;

    // Wait 50 CLKIN periods (assuming CLKIN = 7.68 MHz this would be 6.51 us)
    DWT_Delay_us(7);
    // HAL_Delay(1);
    // Receive the byte contents of the requested register from ADS1256
    status = SPI_Receive_Byte(ads->spiHandle, inBuffer);

endRead: 
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
HAL_StatusTypeDef ADS1256_Register_Write(ADS1256 *ads, ADS1256_Register regAddr, uint8_t data)
{
    HAL_StatusTypeDef status;

    // Bring the chip select line low
    ads->csPort->BSRR = (uint32_t)ads->csPin << 16;

    // Send the first write register command bit or'd with the
    // address of the register we want to write to
    status = SPI_Transmit_Byte(ads->spiHandle, WREG_CMD_1 | regAddr);
    if (status != HAL_OK) goto endWrite;

    // Send the second write register command indicating we
    // only want to write to this register
    status = SPI_Transmit_Byte(ads->spiHandle, WREG_CMD_2);
    if (status != HAL_OK) goto endWrite;

    // Send the data we want to write to the register
    status = SPI_Transmit_Byte(ads->spiHandle, data);

endWrite:
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
    return ADS1256_Send_Command(ads, SELFCAL_CMD);
}

/**
 *  uint8_t ADS1256_Read_Id(ADS1256 *ads, uint8_t id)
 * 
 *  Reads the contents of the ADS1256's STATUS register
 *  and returns the device id found in the upper 4 bits.
 * 
 *  Returns a HAL_StatusTypeDef.
*/
HAL_StatusTypeDef ADS1256_Read_ID(ADS1256 *ads, uint8_t *id)
{
	HAL_StatusTypeDef status;

    // Read the STATUS register where the device ID is stored
    // in the upper 4 bits of the register
	status = ADS1256_Register_Read(ads, STATUS_REG, id);
	if (status != HAL_OK) return status;

	// Shift the value of the byte by 4
	*id = *id >> 4;

	return HAL_OK;
}

/**
 *  HAL_StatusTypeDef ADS1256_Read_Data(ADS1256 *ads, uint8_t *inBuffer)
 * 
 *  Read the latest 24-bit conversion result from the ADS1256 into
 *  the inBuffer byte array. inBuffer must be able to hold 24 bits.
 * 
 *  Returns a HAL_StatusTypeDef.
*/
HAL_StatusTypeDef ADS1256_Read_Data(ADS1256 *ads, uint8_t *inBuffer)
{
    HAL_StatusTypeDef status;

    // Wait until the DRDY pin is brought low 
    while ((ads->rdyPort->IDR & ads->rdyPin) != 0);

    // Bring the chip select line low
    ads->csPort->BSRR = (uint32_t)ads->csPin << 16;

    status = SPI_Transmit_Byte(ads->spiHandle, RDATA_CMD);
    if (status != HAL_OK) goto endRead;
    
    // Wait 50 CLKIN periods (assuming CLKIN = 7.68 MHz this would be 6.51 us)
    DWT_Delay_us(7);

    // Change the polarity of SPI clock as the ADS1256
    // clocks data onto DOUT on rising edge
    // ads->spiHandle->Instance->CR1 &= ~0x0001;

    // Read the 24 bit conversion results and store in
    // the supplied buffer:
    // NOTE: For some reason (probably due to my own incompetence),
    // the data bytes from the ADS1256 appear on the MISO line
    // 3 clock cycles AFTER they should be. I have analyzed this
    // using logic analyzers and it shows that this isn't actually
    // happening on the DOUT line, thus it HAS to be a software issue.
    // I have tried debugging it, and I think it has to do with data
    // being shifted onto DOUT from the ADS1256 on the RISING EDGE
    // as opposed to data being shifted into DIN on the FALLING EDGE.
    // I have tried switching the clock polarity for data reading alone
    // and this BORKS the data.
    //
    // As a temporary solution, I am reading 3 dummy bytes to allow for
    // the actual data to clock in. I am so so sorry.
    status = SPI_Receive_Bytes(ads->spiHandle, 3, inBuffer);
    status = SPI_Receive_Byte(ads->spiHandle, &inBuffer[0]);
    status = SPI_Receive_Byte(ads->spiHandle, &inBuffer[1]);
    status = SPI_Receive_Byte(ads->spiHandle, &inBuffer[2]);

    // Change the polarity of SPI clock back to falling edge
    // ads->spiHandle->Instance->CR1 |= 0x0001;

endRead:
    // Bring the chip select line high
    ads->csPort->BSRR = ads->csPin;

    return status;
}
