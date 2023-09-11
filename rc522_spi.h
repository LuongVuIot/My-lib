// RC522.h
/*	Must be set SPI crc polynomial = 0x1021(CRC16_CCITT) in MX_SPI_Init() in main.c. Because it also is rc522 crc polynomial. The crc init value of stm32f1 is fixed 0xff.
*/
#include "main.h"

typedef enum {
	MI_OK = 0,
	MI_NOTAGERR,
	MI_ERR,
	MI_TIMEOUT,
}RC522Status_e;

typedef enum {
	MODEM_IDLE = 0,
	MODEM_WAITSEND,
	MODEM_TXWAIT,
	MODEM_TRANSMITTING,
	MODEM_RXWAIT,
	MODEM_WAITDATA,
	MODEM_RECEIVING,
}RC522ModemStatus_e;

// define choose slave pin
#define RC522_CS_GPIO		GPIOA
#define RC522_CS_PIN		GPIO_PIN_4

// page 0: Command and status reg
#define RC522_CommandReg		0x01
#define RC522_ComlEnReg			0x02		// enable and disable interrupt request control bits
#define RC522_DivlEnReg			0x03		// __
#define RC522_ComIrqReg			0x04		// interrupt request bits
#define RC522_DivIrqReg			0x05		// __
#define RC522_ErrorReg			0x06		// error bits showing the error status of the last command executed
#define RC522_Status1Reg		0x07		// communication status bits - contain status bits of CRC, int and FIFO
#define RC522_Status2Reg		0x08		// 
#define RC522_FIFODataReg		0x09		// buffer register
#define RC522_FIFOLevelReg	0x0A		// number of bytes stored in FIFO 				
#define RC522_WaterLevelReg	0x0B		// Warning related to number of bytes stored in FIFO
#define RC522_ControlReg		0x0C		// Timer start/stop and last bits receved 
#define RC522_BitFramingReg	0x0D		// Start send, FIFO data align.
#define RC522_CollReg				0x0E		// first bit-collision 

// page 1: Command reg
#define RC522_ModeReg					0x11		// define general mode for transmiting and receiving
#define RC522_TxModeReg				0x12		// define transmission rate, data frame
#define RC522_RxModeReg				0x13		// define reception rate, data frame
#define RC522_TxControlReg		0x14		// control RF signal on pin TX1 and TX2.
#define RC522_TxASKReg				0x15		// don't know what it is used for.
#define RC522_TxSelReg				0x16		// select the internal source for the analog module (antenna)
#define RC522_RxSelReg				0x17		// select the internal receiver settings
#define RC522_RxThresholdReg	0x18		// select threshold for the bit decoder
#define RC522_DemodReg				0x19		// demudulator settings.
#define RC522_MfTxReg					0x1C		// additional response time in MIFARE communication.
#define RC522_MfRxReg					0x1D		// generate parity bit.
#define RC522_SerialSpeedReg	0x1F		// baudrate of UART

// page 2: Configuration reg
#define RC522_CRCMResultReg		0x21		// MSB value of CRC caculation.
#define RC522_CRCLResultReg		0x22		// LSB value of CRC caculation.
#define RC522_ModWidthReg 		0x23		// Miller modulation width
#define RC522_RFCfgReg 				0x26		// set the receiver gain
#define RC522_GsNReg					0x27		// conductance of antenna
#define RC522_CWGsPReg				0x28		// __
#define RC522_ModGsPReg				0x29		// __
#define RC522_TModeReg 				0x2A		// timer configuration 
#define RC522_TPrescalerReg		0x2B		// timer prescaler
#define RC522_TCounterHValReg 0x2E		// timer value higher bits
#define RC522_TCounterLValReg	0x2F		// timer value lower bits

// page 3: Test reg

// RC522 basic command (CommandReg[3:0])
#define RC522_Idle						0b0000		// no action, cancel current command execution
#define RC522_Mem							0b0001		// stores
#define RC522_GenRandomID			0b0010		// genarate 10 bytes ramdom number which will overwire to the first 10 bytes of 25 bytes' internal buffer.
#define RC522_CalcCRC					0b0011		// active CRC or perform a self test
#define RC522_Transmit				0b0100		// tranfer the FIFO content. Note: all relevant register must be set for data transmission.
#define RC522_NoCmdChange			0b0111		// used to modify CommandReg register bits without effecting the current command.
#define RC522_Receive					0b1000		// active the receiver circuits and wating for data to be received.
#define RC522_TRANSCEIVE			0b1100		// continuously repeat the transmission data from FIFO and the reception data from the RF field.
#define RC522_MFAuthent				0b1110		// performs the MIFARE standard authentication as a reader
#define RC522_SoftReset				0b1111		// reset RC522

// MIFARE classic 1K command
#define PICC_REQA							0x26		// Request Command Type A. Type A is commnuniacation protocol between PCD(proximity coupling device) and PICC
																			// After MIFARE card was POR, it will respose ATQA(answer to resquest) to answer REQA or WUPA command. This(ATQA) will show Sales Type of the card. MF1S50yyX_V1 page 15/36 
#define PICC_WUPA							0x52		// REQA command makes PICC from IDLE state to READY state; WUPA makes PICC from IDLE and HALT to READY state. Prepare for anticoll or selection.
#define PICC_ANTICOLL_SEL			0x93		// Anticollision and select cascade level 1.

// The functions
void RC522_Init();	// Initialize RC522
RC522Status_e RC522_Status(uint8_t* id, uint8_t* type);						// check the card ID whether it was detected or not.
RC522Status_e RC522_compare(uint8_t CardID, uint8_t CompareID);		// Compare two ID number. Show whether they are the same or not.
void RC522_WriteReg(uint8_t addr, uint8_t val);										// change register's value.
uint8_t RC522_ReadReg(uint8_t addr);															// get register's value.
void RC522_SetBit(uint8_t addr, uint8_t bit);											// set only a bit.
void RC522_SetBitMask(uint8_t addr, uint8_t bitMask); 						// use bit mask by 'or' bitwise operator.
void RC522_ClearBit(uint8_t addr, uint8_t bit);										// clear a bit.
void RC522_ClearBitMask(uint8_t addr, uint8_t bitMask);						// use bit mask by 'and' bitwise operator.
void RC522_AntennaOn();
void RC522_AntennaOff();
void RC522_Reset();
RC522Status_e RC522_GetATQA();																		// ref AN10834 5/19 Figure1.
void RC522_CS_Write(GPIO_PinState PinState);
RC522Status_e RC522_Request(uint8_t reqMode, uint8_t TagType);
uint8_t RC522_Transceive(uint8_t dataSend, uint8_t TxAlign, uint8_t *receiveBuff, uint8_t RxAlign);		// TxAlign: Numbers of last bits will be send, value 0-7; RxAlign: possition of first bit will be stored at, value:0,1,7.

