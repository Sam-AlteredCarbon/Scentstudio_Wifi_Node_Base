///*
// * ads124s08.c
// *
// *  Created on: Feb 16, 2024
// *      Author: samon
// */
//
//
//#include "ads124s08.h"
//
//
//SPI_BufferTypeDef hspi_ads;
//ADS124S08_HandleTypeDef hads;
//
////extern SPI_HandleTypeDef hspi3; // SPI handle
//
//// Global variables for SPI communication
//volatile transfer_state spiTxComplete = TRANSFER_WAIT;
//volatile transfer_state spiRxComplete = TRANSFER_WAIT;
//
//uint8_t spiTxBuffer[2]; // Buffer for transmit data
//uint8_t spiRxBuffer[20]; // Buffer for received data
//
//// Flag and length for read operation
//volatile bool isReadOperation = false;
//volatile uint8_t expectedRxLength = 0;
//
//
//// Utility functions to control the CS pin
//static void ADS124S08_CS_Low(void) {
//    HAL_GPIO_WritePin(ADS124S08_CS_GPIO_Port, ADS124S08_CS_Pin, GPIO_PIN_RESET);
//}
//
//static void ADS124S08_CS_High(void) {
//    HAL_GPIO_WritePin(ADS124S08_CS_GPIO_Port, ADS124S08_CS_Pin, GPIO_PIN_SET);
//}
//
//static void ADS124S08_CS_Toggle(void) {
//	HAL_GPIO_WritePin(ADS124S08_CS_GPIO_Port, ADS124S08_CS_Pin, GPIO_PIN_RESET);
//	delay(10);
//    HAL_GPIO_WritePin(ADS124S08_CS_GPIO_Port, ADS124S08_CS_Pin, GPIO_PIN_SET);
//}
//
//static void ADS124S08_Start_Measurement(void) {
//    HAL_GPIO_WritePin(ADS124S08_CS_GPIO_Port, ADS124S08_CS_Pin, GPIO_PIN_RESET);
//}
//
//static void ADS124S08_Stop_Measurement(void) {
//    HAL_GPIO_WritePin(ADS124S08_CS_GPIO_Port, ADS124S08_CS_Pin, GPIO_PIN_RESET);
//}
//
//void ADS124S08_Reset(void) {
//	HAL_GPIO_WritePin(ADS124S08_START_GPIO_Port, ADS124S08_START_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(ADS124S08_RESET_GPIO_Port, ADS124S08_RESET_Pin, GPIO_PIN_RESET);
//    delay(500);
//    HAL_GPIO_WritePin(ADS124S08_RESET_GPIO_Port, ADS124S08_RESET_Pin, GPIO_PIN_SET);
//    delay(500);
//}
//
//void ADS124S08_SPI_Tx_IT_Callback(SPI_HandleTypeDef *hspi)
//{
//	if (isReadOperation) {
//		spiTxComplete = TRANSFER_COMPLETE;
//		if (HAL_SPI_Receive_IT(hspi, spiRxBuffer, expectedRxLength) != HAL_OK) {
//			// Handle error
//			ADS124S08_CS_High(); // Ensure CS is set high if reception fails to start
//		}
//	} else {
//		spiTxComplete = TRANSFER_COMPLETE;
////		ADS124S08_CS_High(); // Ensure CS is set high if reception fails to start
//	}
//}
//
//
//void ADS124S08_SPI_Rx_IT_Callback()
//{
//	spiRxComplete = TRANSFER_COMPLETE;
////	ADS124S08_CS_High();
//}
//
////void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
////    if (hspi->Instance == SPI3) {
////    	ADS124S08_SPI_Tx_IT_Callback(hspi);
////    }
////}
////
////void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
////    if (hspi->Instance == SPI3) {
////    	ADS124S08_SPI_Rx_IT_Callback();
////    }
////}
//
////ADS124S08_STATUS ADS124S08_SendCommand_IT(ADS124S08_HandleTypeDef *hads)
////{
////	uint32_t startTime = HAL_GetTick(); // For timeout
////
////	hads->cs_low();
////
////	hads->spiBuffer.spiTxComplete = TRANSFER_WAIT;
////
////	if(HAL_SPI_Transmit_IT(hads->spiBuffer.hspi_driver, hads->spiBuffer.spiTxBuffer, hads->spiBuffer.txLength) != HAL_OK){
////		return ADS124S08_ERROR;
////	}
////
////	while (hads->spiBuffer.spiTxComplete != TRANSFER_COMPLETE) {
////		if (HAL_GetTick() - startTime >= hads->spiBuffer.timeout) {
////			hads->cs_high(); // Deselect the ADS124S08
////			return ADS124S08_TIMEOUT; // Timeout occurred
////		}
////	}
////
////	hads->cs_high();
////	return ADS124S08_SUCCESS;
////}
//
//
//bool ADS124S08_SendCommand_IT(SPI_HandleTypeDef* hspi, uint8_t command,  uint32_t timeout, bool cs_flag)
//{
//	uint32_t startTime = HAL_GetTick(); // For timeout
//
//    if(cs_flag){
//    	ADS124S08_CS_Low();  // Assert CS to select the ADS124S08
//    	delay(1);
//    }
//	spiTxComplete = TRANSFER_WAIT;
//	HAL_SPI_Transmit_IT(hspi, &command, sizeof(uint8_t));  // Transmit the command and data using SPI interrupt
//
//	while (spiTxComplete != TRANSFER_COMPLETE) {
//		if (HAL_GetTick() - startTime >= timeout) {
//			ADS124S08_CS_High(); // Deselect the ADS124S08
//			return false; // Timeout occurred
//		}
//	}
//
//    if(cs_flag){
//    	ADS124S08_CS_High();
//    }
//	return true;
//}
//
//
//bool ADS124S08_WriteRegister_IT(SPI_HandleTypeDef* hspi, uint8_t regAddr, uint8_t value, uint32_t timeout, bool cs_flag) {
//
//	uint32_t startTime = HAL_GetTick(); // For timeout
//    uint8_t txData[3] = {0};
//
//    txData[0] = ADS124S08_CMD_WREG | (regAddr & 0x1F);  // WREG command with register address
//    txData[1] = 0x00;                     // Number of registers to write - 1
//    txData[2] = value;                    // Value to write to the register
//
//    if(cs_flag){
//    	ADS124S08_CS_Low();  // Assert CS to select the ADS124S08
//    	delay(1);
//    }
//    spiTxComplete = TRANSFER_WAIT;
//    HAL_SPI_Transmit_IT(hspi, txData, sizeof(txData));  // Transmit the command and data using SPI interrupt
//
//    while (spiTxComplete != TRANSFER_COMPLETE) {
//		if (HAL_GetTick() - startTime >= timeout) {
//			ADS124S08_CS_High(); // Deselect the ADS124S08
//			return false; // Timeout occurred
//		}
//	}
//
////    command = ADS124S08_CMD_NOP;
////
////    spiTxComplete = TRANSFER_WAIT;
////    HAL_SPI_Transmit_DMA(hspi, command, sizeof(command));  // Transmit the command and data using SPI interrupt
////
////    while (spiTxComplete != TRANSFER_COMPLETE) {
////		if (HAL_GetTick() - startTime >= timeout) {
////			ADS124S08_CS_High(); // Deselect the ADS124S08
////			return false; // Timeout occurred
////		}
////	}
//
//    if(cs_flag){
//    	ADS124S08_CS_High();
//    }
//	return true;
//
//}
//
//bool ADS124S08_ReadRegister_IT(SPI_HandleTypeDef* hspi, uint8_t regAddr, uint8_t numRegs, uint8_t* regData, uint32_t timeout, bool cs_flag) {
//
//	uint32_t startTime = HAL_GetTick();
//
//	uint8_t txData[2] = {0};
//
//    txData[0] = ADS124S08_CMD_RREG | (regAddr & 0x1F);  // RREG command with register address
//    txData[1] = numRegs - 1;              // Number of registers to read - 1
//
//
//    isReadOperation = 0; // Set read operation flag
//	expectedRxLength = numRegs; // Set expected reception length
//
//	memset(spiRxBuffer, 0, expectedRxLength);
//
//	if(cs_flag){
//		ADS124S08_CS_Low();  // Assert CS to select the ADS124S08
//		delay(1);
//
//	}
////	Switch_MISODRDY_PinMode(GPIO_MODE_INPUT);
//
//
//    spiTxComplete = spiRxComplete = TRANSFER_WAIT; // Reset flags
//
//    HAL_SPI_Transmit_IT(hspi, txData, sizeof(txData));  // Transmit the command using SPI interrupt
////    HAL_SPI_TransmitReceive_IT(hspi, txData, spiRxBuffer, sizeof(txData));
//	// Wait for both transmit and receive operations to complete or timeout
//	while (spiTxComplete != TRANSFER_COMPLETE ) {
//		if (HAL_GetTick() - startTime >= timeout) {
//			ADS124S08_CS_High(); // Deselect the ADS124S08
//			return false; // Timeout occurred
//		}
//	}
////	ADS124S08_CS_High();
////////	while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == GPIO_PIN_RESET);
//////
//	delay(1);
////	ADS124S08_CS_Low();
////	Switch_MISODRDY_PinMode(GPIO_MODE_AF_PP);
////	delay(1);
//	startTime = HAL_GetTick();
////	HAL_SPI_TransmitReceive_IT(hspi, &txData[0], spiRxBuffer, expectedRxLength);
//	HAL_SPI_Receive_IT(hspi, spiRxBuffer, expectedRxLength);
//
//	while (spiRxComplete != TRANSFER_COMPLETE) {
//			if (HAL_GetTick() - startTime >= timeout) {
//				ADS124S08_CS_High(); // Deselect the ADS124S08
//				return false; // Timeout occurred
//			}
//		}
//
//	if(cs_flag){
//		ADS124S08_CS_High();
//	}
//    // Copy the received data to the provided output buffer
//    for (uint8_t i = 0; i < expectedRxLength; ++i) {
//    	regData[i] = spiRxBuffer[i];
//    }
//
////    ADS124S08_CS_High(); // Deselect the ADS124S08 after completion
//    return true; // Read operation successful
//}
//
//
//// Function to check if the ADS124S08 is ready
//bool ADS124S08_IsReady(void) {
//
//    uint32_t timeout = 500; // Timeout in milliseconds
//
//    bool readComplete = false;
//
//    ADS124S08_STATUS_Register ads_status;
//
//
//    readComplete = ADS124S08_ReadRegister_IT(&hspi3, ADS124S08_STATUS_REG, 1, &ads_status.byte, timeout, true);
//
////    if (!readComplete)
////    {
////    	return false;
////    }
//
//
//    bool fl_por = ads_status.bits.FL_POR;//status & (1 << 4);
//    bool rdy = ads_status.bits.RDY;//status & (1 << 7);
//
//    if (fl_por)
//    {
//    	ADS124S08_WriteRegister_IT(&hspi3, ADS124S08_STATUS_REG, 0x00, 100, true);
//    }
//
//
//    // Clear the FL_POR flag if set and ensure RDY bit is cleared indicating device is ready
//    return !rdy;
//}
//
//// Function to read the Device ID
//uint8_t ADS124S08_ReadDeviceID(void) {
//
//    uint8_t deviceId = 0xFF; // Default to an invalid value
//
//    bool ReadComplete = false;
//    ADS124S08_SendCommand_IT(&hspi3, ADS124S08_CMD_NOP,  100, true);
//    ReadComplete = ADS124S08_ReadRegister_IT(&hspi3, ADS124S08_ID_REG, 1, &deviceId, 100, true);
//
//
//    if (ReadComplete) {
//        // Extract the Device ID from the received data
//        deviceId = deviceId & 0x07; // Mask to get only DEV_ID bits //spiRxBuffer[1]
//    }
//
//    return deviceId;
//}
//
//bool ADS124S08_ConfigureChannel_IT(SPI_HandleTypeDef* hspi, uint8_t positiveInput, uint8_t negativeInput) {
//
//	InputMultiplexer_Register channel;
//    uint8_t regAddr = ADS124S08_INPMUX_REG;
//    channel.bits.MUXP = positiveInput;
//    channel.bits.MUXN = negativeInput;
//    uint8_t regValue = channel.byte;
//
//    bool status = ADS124S08_WriteRegister_IT(hspi, regAddr, regValue, 100, true);
//
//    return status;
//}
//
//
//bool ADS124S08_EnableInternalReference(SPI_HandleTypeDef* hspi) {
//    VoltageReference_Register refCn;
//
//    // Configure the REFCN register fields
//    refCn.bits.REFCON = 0x01;   // Internal reference on, powers down in PD mode
//    refCn.bits.REFSEL = 0x02;   // Select internal 2.5V reference
//    refCn.bits.REFN_BUF = 0x01; // Disable negative reference buffer
//    refCn.bits.REFP_BUF = 0x01; // Enable positive reference buffer
//    refCn.bits.FL_REF_EN = 0x00; // Disable reference monitor
//
//    uint8_t regAddr = ADS124S08_REF_REG;
//	uint8_t regValue = refCn.byte; // Combine positive and negative inputs
//
//    bool status = ADS124S08_WriteRegister_IT(hspi, regAddr, regValue, 100, true);
//
//    return status;
//}
//
//bool ADS124S08_DisablePGA_IT(SPI_HandleTypeDef* hspi)
//{
//	PGA_Register pga_reg;
//
//    // Configure the PGA register fields
//	pga_reg.bits.PGA_EN = ADS124S08_PGA_DISABLE;   // Disable the PGA
//	pga_reg.bits.DELAY = ADS124S08_PGA_DELAY_14;   // Delay Value
//	pga_reg.bits.GAIN = ADS124S08_PGA_GAIN_1; // Gain Level
//
//    uint8_t regAddr = ADS124S08_PGA_REG;
//	uint8_t regValue = pga_reg.byte; // Combine positive and negative inputs
//
//    bool status = ADS124S08_WriteRegister_IT(hspi, regAddr, regValue, 100, true);
//
//    return status;
//}
//
//
//bool ADS124S08_SetPGA_IT(SPI_HandleTypeDef* hspi, ADS124S08_PGA_Control enable, ADS124S08_PGA_Delay delay, ADS124S08_PGA_Gain gain)
//{
//	PGA_Register pga_reg;
//
//    // Configure the PGA register fields
//	pga_reg.bits.PGA_EN = enable;   // Disable the PGA
//	pga_reg.bits.DELAY = delay;   // Delay Value
//	pga_reg.bits.GAIN = gain; // Gain Level
//
//    uint8_t regAddr = ADS124S08_PGA_REG;
//	uint8_t regValue = pga_reg.byte; // Combine positive and negative inputs
//
//    bool status = ADS124S08_WriteRegister_IT(hspi, regAddr, regValue, 100, true);
//
//    return status;
//}
//
//
//bool ADS124S08_Set_DataRate_IT(SPI_HandleTypeDef* hspi, ADS124S08_DataRate DataRate, ADS124S08_Conversion_Mode conv_mode)
//{
//	DataRate_Register datarate_reg;
//
//	datarate_reg.bits.CLK = 0x00;
//	datarate_reg.bits.FILTER = 0x01;
//	datarate_reg.bits.G_CHOP = 0x00;
//	datarate_reg.bits.MODE = conv_mode;
//	datarate_reg.bits.DR = DataRate;
//
//	uint8_t regAddr = ADS124S08_DATARATE_REG;
//	uint8_t regValue = datarate_reg.byte; // Combine positive and negative inputs
//
//	bool status = ADS124S08_WriteRegister_IT(hspi, regAddr, regValue, 100, true);
//
//	return status;
//}
//
//bool ADS124S08_ReadData_IT(SPI_HandleTypeDef* hspi, uint8_t buffer_len, uint8_t* databuffer, uint32_t timeout, bool cs_flag) {
//
//
//	uint32_t startTime = 0;
//
//    isReadOperation = 0; // Set read operation flag
//	expectedRxLength = buffer_len; // Set expected reception length
////	bool status = false;
////	Switch_MISODRDY_PinMode(GPIO_MODE_INPUT);
//
////	delay(10);
//
//	memset(spiRxBuffer, 0, expectedRxLength);
////	uint8_t command = ADS124S08_CMD_RDATA;
//	if(cs_flag){
//		ADS124S08_CS_Low();  // Assert CS to select the ADS124S08
//		delay(1);
//	}
//
////	status =
//	ADS124S08_SendCommand_IT(hspi, ADS124S08_CMD_RDATA,  100, false);
//	spiRxComplete = TRANSFER_WAIT;
////	HAL_SPI_Transmit_IT(hspi, &command, sizeof(uint8_t));  // Transmit the command and data using SPI interrupt
////
////	startTime = HAL_GetTick();
////	while (spiTxComplete != TRANSFER_COMPLETE) {
////		if (HAL_GetTick() - startTime >= timeout) {
////			ADS124S08_CS_High(); // Deselect the ADS124S08
////			return false; // Timeout occurred
////		}
////	}
//
////	ADS124S08_CS_High();
//	delay(1);
////	ADS124S08_CS_Low();
//	startTime = HAL_GetTick();
//
////	while((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == GPIO_PIN_SET))// && (HAL_GetTick() - startTime >= timeout) );
//
//	// Switch MISO/DRDY pin to SPI mode
////	Switch_MISODRDY_PinMode(GPIO_MODE_AF_PP);
////	delay(1);
//
//	if (HAL_SPI_Receive_IT(hspi, spiRxBuffer, expectedRxLength) != HAL_OK) {
//				// Handle error
//				ADS124S08_CS_High(); // Ensure CS is set high if reception fails to start
//	}
//
//	startTime = HAL_GetTick();
//	while (spiRxComplete != TRANSFER_COMPLETE) {
//		if (HAL_GetTick() - startTime >= timeout) {
//			ADS124S08_CS_High(); // Deselect the ADS124S08
//			return false; // Timeout occurred
//		}
//	}
//
//	if(cs_flag){
//		ADS124S08_CS_High();
//	}
////	if(!status)
////	{
////		return status;
////	}
//    // Copy the received data to the provided output buffer
//    for (uint8_t i = 0; i < expectedRxLength; ++i) {
//    	databuffer[i] = spiRxBuffer[i];
//    }
//
//    return true; // Read operation successful
//}
//
//bool ADS124S08_ReadData_Polling(SPI_HandleTypeDef* hspi, uint8_t buffer_len, uint8_t* databuffer, uint32_t timeout, bool cs_flag) {
//
//
//	uint32_t startTime = 0;
//
//    isReadOperation = 0; // Set read operation flag
//	expectedRxLength = buffer_len; // Set expected reception length
//
//
//	memset(spiRxBuffer, 0, expectedRxLength);
//
//	if(cs_flag){
//		ADS124S08_CS_Low();  // Assert CS to select the ADS124S08
//		delay(1);
//	}
//
////	status =
////	ADS124S08_SendCommand_IT(hspi, ADS124S08_CMD_RDATA,  100, false);
////	spiRxComplete = TRANSFER_WAIT;
//
//	delay(1);
//
//	startTime = HAL_GetTick();
//
//	Poll_DRDY_Pin(1000);
//
//
//	if (HAL_SPI_Receive_IT(hspi, spiRxBuffer, expectedRxLength) != HAL_OK) {
//				// Handle error
//				ADS124S08_CS_High(); // Ensure CS is set high if reception fails to start
//	}
//
//	startTime = HAL_GetTick();
//	while (spiRxComplete != TRANSFER_COMPLETE) {
//		if (HAL_GetTick() - startTime >= timeout) {
//			ADS124S08_CS_High(); // Deselect the ADS124S08
//			return false; // Timeout occurred
//		}
//	}
//
//	if(cs_flag){
//		ADS124S08_CS_High();
//	}
//
//    // Copy the received data to the provided output buffer
//    for (uint8_t i = 0; i < expectedRxLength; ++i) {
//    	databuffer[i] = spiRxBuffer[i];
//    }
//
//    return true; // Read operation successful
//}
//
//
//void Switch_MISODRDY_PinMode(uint8_t mode) {
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//    if(mode == GPIO_MODE_INPUT) {
//        // Configure pin as GPIO input for DRDY detection
//        GPIO_InitStruct.Pin = GPIO_PIN_11; // Assuming pin 6 for MISO/DRDY
//        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//        GPIO_InitStruct.Pull = GPIO_NOPULL;
//        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//    } else {
//        // Configure pin as SPI MISO
//        GPIO_InitStruct.Pin = GPIO_PIN_11; // Assuming pin 6 for MISO/DRDY
//        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//        GPIO_InitStruct.Pull = GPIO_NOPULL;
//        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//        GPIO_InitStruct.Alternate = GPIO_AF6_SPI3; // Alternate function for SPI3
//        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//    }
//}
//
//void Poll_DRDY_Pin(uint16_t timeout)
//{
//	uint32_t startTime = HAL_GetTick();
//	while((HAL_GPIO_ReadPin(ADS124S08_DRDY_GPIO_Port, ADS124S08_DRDY_Pin) == GPIO_PIN_SET) )
//	{
//		if(HAL_GetTick() - startTime >= timeout)
//		{
//			break;
//		}
//	}
//}
//
//int32_t combineADCBytes(uint8_t adcBytes[3]) {
//    // Combine the bytes into a 24-bit integer, initially treating it as unsigned
//    uint32_t adcValue = ((uint32_t)adcBytes[0] << 16) | ((uint32_t)adcBytes[1] << 8) | adcBytes[2];
//
//    // Sign-extend to 32 bits to maintain the sign of the 24-bit number
//    // Check if the 24th bit is set (indicating a negative number in 2's complement)
//    if (adcValue & 0x00800000) {
//        // If it's negative, extend the sign to the 32-bit variable
//        adcValue |= 0xFF000000;
//    }
//
//    // Cast the unsigned value to signed, since we've manually handled sign-extension
//    return (int32_t)adcValue;
//}
//
//// Function to convert raw ADC value to voltage
////double ConvertADCToVoltage(uint8_t adcBytes[3]) {
////
////	int32_t adcValue = combineADCBytes(adcBytes);
////    // Interpret the 24-bit two's complement value
////
////
////
////    // Convert ADC code to voltage
////    double voltage = (double)adcValue * (2 * VREF / ADS_GAIN) / pow(2, 24);
////
////    return voltage;
////}
//
//
///**
// * Converts a 24-bit two's complement ADC value to voltage.
// *
// * @param adcValue The 24-bit ADC value in two's complement format.  float resistance = (rload * vin / vout) - rload;
// * @param vref The reference voltage for the ADC.
// * @param gain The gain setting of the ADC.
// * @return The voltage corresponding to the ADC value.
// */
//float ConvertADCToVoltage(int32_t adcValue, float vref, uint8_t gain) {
//    // Convert 24-bit two's complement to signed 32-bit integer
//    if (adcValue & 0x800000) { // Check if the value is negative
//        adcValue |= 0xFF000000; // Sign-extend to 32 bits
//    }
//
//    // Calculate LSB value
//    float lsb = ((2.0 * vref) / gain) / ADC_RESOLUTION;
////    float lsb = (vref / gain) / ADC_RESOLUTION23;
//
//
//    // Calculate and return the voltage
//    return adcValue * lsb;
//}
//
//
////uint32_t ConvertVoltageToResistance(float voltage, float vin,float rload)
////{
////	float resistance = (RLOAD * VIN / voltage) - RLOAD;
////	return (uint32_t)resistance;
////}
//
//uint32_t ConvertVoltageToResistance(float voltage)
//{
//	float resistance = (RLOAD * VIN / voltage) - RLOAD;
//	return (uint32_t)resistance;
//}
//
//// Populate the ads124s08 handle type def
////bool ADS124S08_MapConfig(ADS124S08_HandleTypeDef *hads, SPI_HandleTypeDef *hspi){
////
//////	hads->spiBuffer.hspi_driver = hspi;
//////	hads->configureChannel = ADS124S08_ConfigureChannel_IT;
//////	hads->convertToVoltage = ConvertADCToVoltage;
//////	hads->enableInternalReference = ADS124S08_EnableInternalReference;
//////	hads->setGainAndDataRate =
////
////}
//
//// TODO: switch the return type from bool to multistate status enum
//bool ADS124S08_Init(ADS124S08_HandleTypeDef *hads)
//{
//
//	bool status = true;
//    // Reset the ADS124S08
//    ADS124S08_Reset();
//    delay(10); // Delay for reset stabilization
//
//    // Reset Configurations
////    status &= ADS124S08_SendCommand_IT(hads->spiBuffer.hspi_driver, ADS124S08_CMD_RESET, 100, true);
//
//    delay(10); // Delay for reset stabilization
//    // Set system configs
//
//	status &= ADS124S08_DisablePGA_IT(hads->spiBuffer.hspi_driver);
//	status &= ADS124S08_Set_DataRate_IT(hads->spiBuffer.hspi_driver, ADS124S08_DATARATE_50SPS, ADS124S08_DATARATE_SINGLE);
//	status &= ADS124S08_EnableInternalReference(hads->spiBuffer.hspi_driver);
////	status &= ADS124S08_IsReady();
//
//	return status;
//}
//
//
///**
//  * @brief  Starts a conversion on a specified channel, reads the data, and converts it to voltage.
//  * @param  hspi: pointer to the SPI handle.
//  * @param  channel_index: index of the channel to be configured and read.
//  * @param  voltage: pointer to the variable where the converted voltage will be stored.
//  * @retval bool: status of the operation (true = success, false = failure).
//  */
//float ADS124S08_StartConversionReadVoltage(SPI_HandleTypeDef* hspi, uint8_t channel_index, bool* status)
//{
////    bool status = true; // Assume success initially
//    uint8_t data[3] = {0}; // Buffer to hold raw ADC data
//    uint8_t imuxreg = 0; // Variable to hold the read register value
//    int32_t adcValue = 0;
//    float voltage = 0.0;
//
//    // Configure the channel
//    ADS124S08_SendCommand_IT(hspi, ADS124S08_CMD_NOP,  100, true);
//	delay(1);
//    *status &= ADS124S08_ConfigureChannel_IT(hspi, channel_index, ADS124S08_MUX_AINCOM);
//    delay(1); // Delay for configuration to take effect
//    ADS124S08_SendCommand_IT(hspi, ADS124S08_CMD_NOP,  100, true);
//	delay(1);
//    *status &= ADS124S08_ReadRegister_IT(hspi, ADS124S08_IDACMUX_REG, 1, &imuxreg, 100, true);
////    delay(1); // Wait for conversion to start
//    // Start the conversion
////    ADS124S08_CS_Toggle();
////	Switch_MISODRDY_PinMode(GPIO_MODE_INPUT);
////	ADS124S08_CS_Low();
//    ADS124S08_SendCommand_IT(hspi, ADS124S08_CMD_NOP,  100, true);
//	delay(1);
//
////	ADS124S08_SendCommand_IT(hspi, ADS124S08_CMD_START, 100);
//    HAL_GPIO_WritePin(ADS124S08_START_GPIO_Port, ADS124S08_START_Pin, GPIO_PIN_SET);
//
//
//    // Read the data rate register
////    *status &= ADS124S08_ReadRegister_IT(hspi, ADS124S08_IDACMUX_REG, 1, &imuxreg, 100, true);
//    delay(50); // Delay to allow for register read
////    ADS124S08_SendCommand_IT(hspi, ADS124S08_CMD_NOP,  100, true);
////	delay(1);
////     while((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == GPIO_PIN_SET))
////	 Switch_MISODRDY_PinMode(GPIO_MODE_AF_PP);
//    // Read the conversion data
////    *status &= ADS124S08_ReadData_IT(hspi, sizeof(data), data, 100, true); // ADS124S08_ReadData_Polling
//    *status &= ADS124S08_ReadData_Polling(hspi, sizeof(data), data, 100, true); // ADS124S08_ReadData_Polling
//    if (*status) { // Proceed if all operations were successful
//    	adcValue = combineADCBytes(data);
//        voltage = ConvertADCToVoltage(adcValue, (float)VREF, ADS_GAIN);
//    }
//    else{
//    	voltage = 0;
//    }
//
////    delay(1); // Delay after reading
//
//    // Stop the conversion
////    ADS124S08_SendCommand_IT(hspi, ADS124S08_CMD_STOP, 100);
//    HAL_GPIO_WritePin(ADS124S08_START_GPIO_Port, ADS124S08_START_Pin, GPIO_PIN_RESET);
//    ADS124S08_CS_High();
////    delay(1); // Ensure conversion is stopped
//
//    return voltage;
//}
////TODO: Add sys control function
//
//bool ADS124S08_Read_Sensor_Array(ADS124S08_HandleTypeDef *hads) //, float voltage_array[], uint8_t num_channels)
//{
//
////	if(voltage_array == NULL)
////	{
////		return false;
////	}
////	if(num_channels > MAX_CHANNELS)
////	{
////		num_channels = MAX_CHANNELS;
////	}
//	if(hads->current_channel >= MAX_CHANNELS)
//	{
//		hads->current_channel = 0;
//	}
//
//	bool status = true;
//
//	hads->results_buffer[hads->current_channel] = ADS124S08_StartConversionReadVoltage(hads->spiBuffer.hspi_driver, hads->current_channel, &status);
//
//	if(status == true)
//	{
//		hads->current_channel++;
//	}
////	for(uint8_t i = 0; i < num_channels; i++ )
////	{
////		voltage_array[i] = ADS124S08_StartConversionReadVoltage(hads->spiBuffer.hspi_driver, i, &status);
////		hads->results_buffer[i] = voltage_array[i]; //ADS124S08_StartConversionReadVoltage(hads->spiBuffer.hspi_driver, i, &status);
////		if(status != true)
////		{
////			return status;
////		}
////	}
//	return status;
//}
//
//
