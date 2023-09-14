// rc522_spi.c

#include "rc522_spi.h"

extern SPI_HandleTypeDef hspi1;
// extern uint8_t err;
void RC522_Init(void)	// Initialize RC522
{
	RC522_CS_Write(GPIO_PIN_SET);		// CS pin is SET - disable SPI communication
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);		// Reset pin RST
	HAL_Delay(100);		// 8.8.1 rst pin low at least 100ns. 100 ms will be certainly.
	RC522_Reset();

	//timer setup
  /*RC522_WriteReg(RC522_TPrescalerReg, 0xA8);				// 0xA8=168= timer prescaler. Combination with timer reload value = 3999 creates 50 ms delay.
	RC522_WriteReg(RC522_TReloadLReg, 0x9F);					// 3999%256 = 159 set reload value = 3999.
	RC522_WriteReg(RC522_TReloadHReg, 0x0F);					// 3999/256 = 15
	
	RC522_SetBit(RC522_RxModeReg, 3);									// an invalid received data stream (less than 4 bits received) will be ignored and the receiver remains active
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_Delay(1000);*/
	// RC522_SetBitMask(RC522_ComIEnReg, 0x07);				// pin IRQ is a standard CMOS output pin
		
}

void RC522_WriteReg(uint8_t addr, uint8_t val)
{
	HAL_StatusTypeDef transmitStatus;
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	
	RC522_CS_Write(GPIO_PIN_RESET);		// CS pin is low - start SPI communication
	
	addr = (addr << 1)&0x7F;									// bit7: R/W (1/0), bit6-1: register address, bit0 = 0.
	
	// transmit address register target first
	transmitStatus = HAL_SPI_Transmit(&hspi1, &addr, 1, 100);		
	
	// whether error | busy | timeout happens or not.
	if(transmitStatus != HAL_OK)
	{
		Error_Handler();		// modify it in main.c
	}
	
	// then transmit value into that register.
	HAL_SPI_Transmit(&hspi1, &val, 1, 100);		
	
	if(transmitStatus != HAL_OK)
	{
		Error_Handler();
	}
	
	RC522_CS_Write(GPIO_PIN_SET);		// CS pin is high - stop communication.
}

uint8_t RC522_ReadReg(uint8_t addr)
{
	uint8_t dummyAddr = 0x80;
	uint8_t val;
	HAL_StatusTypeDef transmitStatus;
	
	RC522_CS_Write(GPIO_PIN_RESET);
	
	addr = ((addr<<1)&0x7F)|0x80;
	
	transmitStatus = HAL_SPI_Transmit(&hspi1, &addr, 1, 100);
	if(transmitStatus != HAL_OK)
	{
		Error_Handler();
	}
	transmitStatus = HAL_SPI_Transmit(&hspi1, &dummyAddr, 1, 100);
	if(transmitStatus != HAL_OK)
	{
		Error_Handler();
	}
	
	// the address1 according to the data0 - refer RC522 datasheet, page 9, table 6.
	//transmitStatus = HAL_SPI_TransmitReceive(&hspi1, &dummyAddr, &val, 1, 100);
	transmitStatus = HAL_SPI_Receive(&hspi1, &val, 1, 100);
	if(transmitStatus != HAL_OK)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
		Error_Handler();
	}
	
	RC522_CS_Write(GPIO_PIN_SET);
	
	return val;
}
uint8_t RC522_ReadBit(uint8_t addr, uint8_t bit)
{
	return (RC522_ReadReg(addr)>>bit)&1;
}

void RC522_SetBit(uint8_t addr, uint8_t bit)
{
	RC522_WriteReg(addr, RC522_ReadReg(addr)|1<<bit);
}

void RC522_SetBitMask(uint8_t addr, uint8_t bitMask)
{
	RC522_WriteReg(addr, RC522_ReadReg(addr)|bitMask);
}

void RC522_ClearBit(uint8_t addr, uint8_t bit)
{
	RC522_WriteReg(addr, RC522_ReadReg(addr)&~(1<<bit));
}

void RC522_ClearBitMask(uint8_t addr, uint8_t bitMask)
{
	RC522_WriteReg(addr, RC522_ReadReg(addr)&bitMask);
}

void RC522_AntennaOn(void)
{
	RC522_SetBitMask(RC522_TxControlReg, 0x03);
}

void RC522_AntennaOff(void)
{
	RC522_ClearBitMask(RC522_TxControlReg, 0xFC);
}

// soft reset
void RC522_Reset(void)
{
	RC522_WriteReg(RC522_CommandReg, 0x0f);
}

void RC522_CS_Write(GPIO_PinState PinState)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, PinState);
}

// Request command. the card activation procedure starts with a Request command (REQA or REQB)(ref. AN10834 - 4/19 - 2.1). In this lib, i use REQA.
uint8_t RC522_GetATQA(void)	
{
	RC522Status_e Status;
	uint8_t ATQABuffer[2];
	
	Status = RC522_Transceive(0x26, 0x07, ATQABuffer, 0);		// 0x26 = REQA command code, 7 Means 7 last bits of the last byte will be transmitted. Because REQA command has 7 bits. ATQA response contain 16bits.
	if(Status == MI_OK)		return 1;
	else									return 0;
		
}

RC522Status_e RC522_Transceive(uint8_t dataSend,uint8_t TxAlign, uint8_t *receiveBuff, uint8_t RxAlign)
{
	uint8_t n;	// for FIFO level
	RC522_SetBitMask(RC522_ComIEnReg, 0x21);								// allows the timer and receive interrupt request.	
	RC522_SetBitMask(RC522_ComIrqReg, 0x7F);								// clear all of interrupt flag bits.
	RC522_SetBitMask(RC522_BitFramingReg, TxAlign|(RxAlign<<4));
	RC522_WriteReg(RC522_FIFODataReg, dataSend);
	RC522_SetBitMask(RC522_CommandReg, RC522_TRANSCEIV);	
	RC522_SetBit(RC522_BitFramingReg, 7);										// Start to transfer data. Combination with Transceive command 
	RC522_SetBit(RC522_ControlReg, 6);											// Start timer immediately
	if(RC522_ReadBit(RC522_ComIrqReg,0) && (!RC522_ReadBit(RC522_ComIrqReg,5)))		//	timer interrupt is set and Rx interrupt is still zero.
		return MI_TIMEOUT;	
	if(RC522_ReadBit(RC522_ComIrqReg,5)){
		for(n=RC522_ReadReg(RC522_FIFOLevelReg); n>0; --n)
			*(receiveBuff+n) = RC522_ReadReg(RC522_FIFODataReg);
		return MI_OK;
	}
	return MI_ERR;
}
