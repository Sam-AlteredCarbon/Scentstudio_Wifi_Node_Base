///*
// * ads124s08.h
// *
// *  Created on: Feb 16, 2024
// *      Author: samon
// */
//
//#ifndef INC_ADS124S08_H_
//#define INC_ADS124S08_H_
//
//
//#include "stm32u5xx_hal.h"
//#include "stdbool.h"
//#include <stdint.h>
//#include "main.h"
//#include "spi.h"
//
//// Define GPIO pins and SPI interface used for the ADS124S08
//#define ADS124S08_CS_GPIO_Port 										GPIOA
//#define ADS124S08_CS_Pin 											GPIO_PIN_15
//#define ADS124S08_DRDY_GPIO_Port 									GPIOD
//#define ADS124S08_DRDY_Pin 											GPIO_PIN_0
//#define ADS124S08_START_GPIO_Port 									GPIOD
//#define ADS124S08_START_Pin 										GPIO_PIN_1
//#define ADS124S08_RESET_GPIO_Port 									GPIOD
//#define ADS124S08_RESET_Pin GPIO_PIN_3
//
//#define ADC_RESOLUTION 												16777216.0 // 2^24
//#define ADC_RESOLUTION23											8388608.0 // 2^24
//
//#define MAX_CHANNELS												12
//
//#define VREF        	2.5  // Reference voltage value, adjust as per your configuration
//#define ADS_GAIN        1    // Gain setting, adjust as per your configuration
//#define RLOAD			100000
//#define VIN				2.5
////(rload * vin / voltage) - rload;
//
//typedef enum {
//    ADS124S08_CMD_NOP = 0x00,
//    ADS124S08_CMD_WAKEUP = 0x02,
//    ADS124S08_CMD_POWERDOWN = 0x04,
//    ADS124S08_CMD_RESET = 0x06,
//    ADS124S08_CMD_START = 0x08,
//    ADS124S08_CMD_STOP = 0x0A,
//    ADS124S08_CMD_SYOCAL1 = 0x16,
//    ADS124S08_CMD_SYOCAL2 = 0x17,
//    ADS124S08_CMD_SYGCAL = 0x19,
//    ADS124S08_CMD_SFOCAL = 0x1B,
//    ADS124S08_CMD_RDATA = 0x12,
//    ADS124S08_CMD_RREG = 0x20, // Followed by register address and number of registers minus one
//    ADS124S08_CMD_WREG = 0x40, // Followed by register address and number of registers minus one
//} ADS124S08_Commands;
//
//typedef enum {
//    ADS124S08_ID_REG = 0x00,        // Device ID Register
//    ADS124S08_STATUS_REG = 0x01,    // Status Register
//    ADS124S08_INPMUX_REG = 0x02,    // Input Multiplexer Control Register
//    ADS124S08_PGA_REG = 0x03,       // PGA (Gain) Control Register
//    ADS124S08_DATARATE_REG = 0x04,  // Data Rate Control Register
//    ADS124S08_REF_REG = 0x05,       // Reference Control Register
//    ADS124S08_IDACMAG_REG = 0x06,   // IDAC Magnitude Register
//    ADS124S08_IDACMUX_REG = 0x07,   // IDAC Multiplexer Control Register
//    ADS124S08_VBIAS_REG = 0x08,     // Bias Voltage Register
//    ADS124S08_SYS_REG = 0x09,       // System Control Register
//    ADS124S08_OFCAL0_REG = 0x0A,    // Offset Calibration 0 Register
//    ADS124S08_OFCAL1_REG = 0x0B,    // Offset Calibration 1 Register
//    ADS124S08_OFCAL2_REG = 0x0C,    // Offset Calibration 2 Register
//    ADS124S08_FSCAL0_REG = 0x0D,    // Full-Scale Calibration 0 Register
//    ADS124S08_FSCAL1_REG = 0x0E,    // Full-Scale Calibration 1 Register
//    ADS124S08_FSCAL2_REG = 0x0F,    // Full-Scale Calibration 2 Register
//    ADS124S08_GPIODAT_REG = 0x10,   // GPIO Data Register
//    ADS124S08_GPIOCON_REG = 0x11    // GPIO Control Register
//} ADS124S08_Registers;
//
//
//typedef union {
//    struct {
//        uint8_t FL_REF_L0   : 1; // Reference voltage monitor flag, level 0
//        uint8_t FL_REF_L1   : 1; // Reference voltage monitor flag, level 1
//        uint8_t FL_N_RAILN  : 1; // Negative PGA output at negative rail flag
//        uint8_t FL_N_RAILP  : 1; // Negative PGA output at positive rail flag
//        uint8_t FL_P_RAILN  : 1; // Positive PGA output at negative rail flag
//        uint8_t FL_P_RAILP  : 1; // Positive PGA output at positive rail flag
//        uint8_t RDY         : 1; // Device ready flag
//        uint8_t FL_POR      : 1; // Power-on reset (POR) flag
//    } bits;
//    uint8_t byte; // For accessing the entire register as a single byte
//} ADS124S08_STATUS_Register;
//
//typedef enum {
//	 ADS124S08_SYS_MON_DISABLED		=	0x00,
//	 ADS124S08_SYS_MON_PGA_SHORT	=	0x01,
//	 ADS124S08_SYS_MON_TEMP_SEN		=	0x02,
//	 ADS124S08_SYS_MON_AVDD_4		=	0x03,
//	 ADS124S08_SYS_MON_DVDD_4		=	0x04,
//	 ADS124S08_SYS_MON_BRWN_OUT_0_2	=	0x05,
//	 ADS124S08_SYS_MON_BRWN_OUT_1	=	0x06,
//	 ADS124S08_SYS_MON_BRWN_OUT_10	=	0x07
//}ADS124S08_Sys_Monitor_Control;
//
//
//typedef union {
//    struct {
//        uint8_t REFCON  : 2;  // Internal voltage reference configuration
//        uint8_t REFSEL  : 2;  // Reference input selection
//        uint8_t REFN_BUF: 1;  // Negative reference buffer bypass
//        uint8_t REFP_BUF: 1;  // Positive reference buffer bypass
//        uint8_t FL_REF_EN: 2; // Reference monitor configuration
//    } bits;
//    uint8_t byte;  // Entire register value as a single byte
//} VoltageReference_Register;
//
//typedef union {
//    struct {
//        uint8_t DR     : 4; // Data rate selection
//        uint8_t FILTER : 1; // Digital filter selection
//        uint8_t MODE   : 1; // Conversion mode selection
//        uint8_t CLK    : 1; // Clock source selection
//        uint8_t G_CHOP : 1; // Global chop enable
//    } bits;
//    uint8_t byte; // For easy SPI transmission
//} DataRate_Register;
//
//typedef union {
//    struct {
//        uint8_t FL_RAIL_EN	: 1; // PGA output rail flag enable
//        uint8_t PSW		 	: 1; // Low-side power switch
//        uint8_t RESERVED   	: 2; // Reserved Always 0
//        uint8_t IMAG    : 4; // IDAC magnitude selection
//    } bits;
//    uint8_t byte; // For easy SPI transmission
//} ExCurrent1_Register;
//
//
//typedef union {
//    struct {
//        uint8_t I2MUX	: 4; // IDAC2 output channel selection
//        uint8_t I1MUX	: 4; // IDAC1 output channel selection
//
//    } bits;
//    uint8_t byte; // For easy SPI transmission
//} ExCurrent2_Register;
//
//typedef enum {
//    ADS124S08_DATARATE_2_5SPS = 0x00,
//    ADS124S08_DATARATE_5SPS = 0x01,
//    ADS124S08_DATARATE_10SPS = 0x02,
//    ADS124S08_DATARATE_16_6SPS = 0x03,
//    ADS124S08_DATARATE_20SPS = 0x04,  // Default
//    ADS124S08_DATARATE_50SPS = 0x05,
//    ADS124S08_DATARATE_60SPS = 0x06,
//    ADS124S08_DATARATE_100SPS = 0x07,
//    ADS124S08_DATARATE_200SPS = 0x08,
//    ADS124S08_DATARATE_400SPS = 0x09,
//    ADS124S08_DATARATE_800SPS = 0x0A,
//    ADS124S08_DATARATE_1000SPS = 0x0B,
//    ADS124S08_DATARATE_2000SPS = 0x0C,
//    ADS124S08_DATARATE_4000SPS = 0x0D
//} ADS124S08_DataRate;
//
//typedef enum {
//	ADS124S08_DATARATE_CONTINUOUS   = 0x00, // Disable PGA
//	ADS124S08_DATARATE_SINGLE    = 0x01 // Enable PGA
//} ADS124S08_Conversion_Mode;
//
//typedef enum {
//	ADS124S08_DATARATE_SINC_FILTER   = 0x00, // Disable PGA
//	ADS124S08_DATARATE_LOW_LATENCY_FILTER    = 0x01 // Enable PGA
//} ADS124S08_Datarate_Filter;
//
//typedef union {
//    struct {
//        uint8_t GAIN    : 3; // Gain selection
//        uint8_t PGA_EN  : 2; // PGA enable
//        uint8_t DELAY   : 3; // Programmable conversion delay selection
//    } bits;
//    uint8_t byte; // For direct access to the full register value
//} PGA_Register;
//
//typedef enum {
//	ADS124S08_PGA_DISABLE   = 0x00, // Disable PGA
//	ADS124S08_PGA_ENABLE    = 0x01 // Enable PGA
//} ADS124S08_PGA_Control;
//
//
//typedef enum {
//	ADS124S08_PGA_GAIN_1 = 0x00,
//	ADS124S08_PGA_GAIN_2 = 0x01,
//	ADS124S08_PGA_GAIN_4 = 0x02,
//	ADS124S08_PGA_GAIN_8 = 0x03,
//	ADS124S08_PGA_GAIN_16 = 0x04,  // Default
//	ADS124S08_PGA_GAIN_32 = 0x05,
//	ADS124S08_PGA_GAIN_64 = 0x06,
//	ADS124S08_PGA_GAIN_128 = 0x07
//} ADS124S08_PGA_Gain;
//
//
//typedef enum {
//	ADS124S08_PGA_DELAY_14 = 0x00, //Minimum delay  14*tMOD
//	ADS124S08_PGA_DELAY_25 = 0x01, //  delay  25*tMOD
//	ADS124S08_PGA_DELAY_64 = 0x02, //  delay  64*tMOD
//	ADS124S08_PGA_DELAY_256 = 0x03, //  delay  256*tMOD
//	ADS124S08_PGA_DELAY_1024 = 0x04,  //  delay  1024*tMOD
//	ADS124S08_PGA_DELAY_2048 = 0x05, //  delay  2048*tMOD
//	ADS124S08_PGA_DELAY_4096 = 0x06, //  delay  4096*tMOD
//	ADS124S08_PGA_DELAY_1 = 0x07 //  delay  1*tMOD
//} ADS124S08_PGA_Delay;
//
//typedef union {
//    struct {
//        uint8_t MUXN    : 4;  // Negative ADC input selection
//        uint8_t MUXP    : 4;  // Positive ADC input selection
//    } bits;
//    uint8_t byte;  // The whole register as a single byte
//} InputMultiplexer_Register;
//
//typedef enum {
//    ADS124S08_MUX_AIN0 = 0x00,
//    ADS124S08_MUX_AIN1 = 0x01,
//    ADS124S08_MUX_AIN2 = 0x02,
//    ADS124S08_MUX_AIN3 = 0x03,
//    ADS124S08_MUX_AIN4 = 0x04,
//    ADS124S08_MUX_AIN5 = 0x05,
//    ADS124S08_MUX_AIN6 = 0x06,  // ADS124S08 only
//    ADS124S08_MUX_AIN7 = 0x07,  // ADS124S08 only
//    ADS124S08_MUX_AIN8 = 0x08,  // ADS124S08 only
//    ADS124S08_MUX_AIN9 = 0x09,  // ADS124S08 only
//    ADS124S08_MUX_AIN10 = 0x0A, // ADS124S08 only
//    ADS124S08_MUX_AIN11 = 0x0B, // ADS124S08 only
//    ADS124S08_MUX_AINCOM = 0x0C
//} ADS124S08_MUX_Channel;
//
//typedef union {
//    struct {
//        uint8_t SENDSTAT : 1; // STATUS byte enable
//        uint8_t ADS_CRC  : 1; // CRC enable
//        uint8_t TIMEOUT  : 1; // SPI timeout enable
//        uint8_t CAL_SAMP : 2; // Calibration sample size selection
//        uint8_t SYS_MON  : 3; // System monitor configuration
//    } bits;
//    uint8_t byte;
//} SystemControl_Register;
//
//typedef enum {
//	ADS124S08_SYS_SENDSTAT_DISABLE   = 0x00, // Disable PGA
//	ADS124S08_SYS_SENDSTAT_ENABLE    = 0x01 // Enable PGA
//} ADS124S08_Sys_SendStat_Control;
//
//typedef enum {
//	ADS124S08_SYS_CRC_DISABLE   = 0x00, // Disable PGA
//	ADS124S08_SYS_CRC_ENABLE    = 0x01 // Enable PGA
//} ADS124S08_Sys_CRC_Control;
//
//typedef struct {
//	SPI_HandleTypeDef *hspi_driver; // SPI handle
//
//	// Global variables for SPI communication
//	volatile transfer_state spiTxComplete;
//	volatile transfer_state spiRxComplete;
//
//	uint8_t spiTxBuffer[BUFFER_SIZE_4]; // Buffer for transmit data
//	uint8_t spiRxBuffer[BUFFER_SIZE_32]; // Buffer for received data
//
//	uint8_t txLength; // Length of data to send
//	uint32_t timeout; // Timeout for read or write operation
//
//	// Flag and length for read operation
//	volatile bool isReadOperation; // No longer needed remove
//	volatile uint8_t expectedRxLength;
//
//}SPI_BufferTypeDef;
//
//typedef struct {
//    ADS124S08_DataRate dataRate;
//    ADS124S08_PGA_Gain gain;
//    ADS124S08_MUX_Channel positiveInput;
//    ADS124S08_MUX_Channel negativeInput;
//    ADS124S08_Conversion_Mode conversionMode;
//    ADS124S08_Datarate_Filter filterType;
//    bool internalReferenceEnabled;
//} ADS124S08_ConfigTypeDef;
//
//// Struct for managing ADS124S08 ADC
//typedef struct {
//    ADS124S08_ConfigTypeDef config; // Configuration settings
//    SPI_BufferTypeDef spiBuffer; // SPI communication buffer and handles
//
//    float results_buffer[MAX_CHANNELS];
//    uint8_t current_channel;
//
//    // Function pointers for operational methods
//    void (*cs_low)(void);
//    void (*cs_high)(void);
//    void (*reset)(void);
//
////    bool (*configureChannel)(SPI_HandleTypeDef* hspi, uint8_t positiveInput, uint8_t negativeInput);
////    bool (*enableInternalReference)(SPI_HandleTypeDef* hspi);
////    bool (*setGain)(SPI_HandleTypeDef* hspi, ADS124S08_PGA_Control enable, ADS124S08_PGA_Delay delay, ADS124S08_PGA_Gain gain);
////    bool (*setDataRate)(SPI_HandleTypeDef* hspi, ADS124S08_DataRate DataRate, ADS124S08_Conversion_Mode conv_mode);
////    bool (*readData)(SPI_HandleTypeDef* hspi, uint8_t buffer_len, uint8_t* dataBuffer);
////    float (*convertToVoltage)(int32_t adcValue, float vref, uint8_t gain);
//    // Additional operational methods as needed
//} ADS124S08_HandleTypeDef;
//
//
//typedef enum{
//	ADS124S08_SUCCESS,
//	ADS124S08_TIMEOUT,
//	ADS124S08_ERROR
//}ADS124S08_STATUS;
//
//
//
//void ADS124S08_Reset(void);
//void ADS124S08_SPI_Tx_IT_Callback(SPI_HandleTypeDef *hspi);
//void ADS124S08_SPI_Rx_IT_Callback();
//bool ADS124S08_IsReady(void);
//uint8_t ADS124S08_ReadDeviceID(void);
//bool ADS124S08_WriteRegister_IT(SPI_HandleTypeDef* hspi, uint8_t regAddr, uint8_t value, uint32_t timeout, bool cs_flag);
//bool ADS124S08_ReadRegister_IT(SPI_HandleTypeDef* hspi, uint8_t regAddr, uint8_t numRegs, uint8_t* outBuffer, uint32_t timeout, bool cs_flag);
//bool ADS124S08_SendCommand_IT(SPI_HandleTypeDef* hspi, uint8_t command,  uint32_t timeout, bool cs_flag);
//bool ADS124S08_ReadData_IT(SPI_HandleTypeDef* hspi, uint8_t buffer_len, uint8_t* databuffer, uint32_t timeout, bool cs_flag);
//bool ADS124S08_ReadData_Polling(SPI_HandleTypeDef* hspi, uint8_t buffer_len, uint8_t* databuffer, uint32_t timeout, bool cs_flag);
//
//bool ADS124S08_ConfigureChannel_IT(SPI_HandleTypeDef* hspi, uint8_t positiveInput, uint8_t negativeInput);
//bool ADS124S08_EnableInternalReference(SPI_HandleTypeDef* hspi);
//bool ADS124S08_DisablePGA_IT(SPI_HandleTypeDef* hspi); //ADS124S08_PGA_Gain
//
//bool ADS124S08_SetPGA_IT(SPI_HandleTypeDef* hspi, ADS124S08_PGA_Control enable, ADS124S08_PGA_Delay delay, ADS124S08_PGA_Gain gain);
//bool ADS124S08_Set_DataRate_IT(SPI_HandleTypeDef* hspi, ADS124S08_DataRate DataRate, ADS124S08_Conversion_Mode conv_mode);
//
//bool ADS124S08_Init(ADS124S08_HandleTypeDef *hads);
//
//float ADS124S08_StartConversionReadVoltage(SPI_HandleTypeDef* hspi, uint8_t channel_index, bool* status);
//
//bool ADS124S08_Read_Sensor_Array(ADS124S08_HandleTypeDef *hads); //, float voltage_array[], uint8_t num_channels);
//
//
//void Switch_MISODRDY_PinMode(uint8_t mode);
//void Poll_DRDY_Pin(uint16_t timeout);
//
//int32_t combineADCBytes(uint8_t adcBytes[3]);
////double ConvertADCToVoltage(uint8_t adcBytes[3]);
//float ConvertADCToVoltage(int32_t adcValue, float vref, uint8_t gain);
//uint32_t ConvertVoltageToResistance(float voltage);
//
////double ConvertADCToVoltage(int32_t adcValue);
//#endif /* INC_ADS124S08_H_ */
