// rc522_spi.c

#include "rc522_spi.h"

extern SPI_HandleTypeDef hspi1;

void RC522_Init()	// Initialize RC522
{
	RC522_CS_Write(GPIO_PIN_SET);		// CS pin is SET - disable SPI communication
	
	//timer setup
  RC522_WriteReg(RC522_TPrescalerReg, 0x43);	// 0x43=67= timer prescaler. Combination with timer reload value = 99 creates 1 ms delay.
	RC522_Reset();
}

void RC522_WriteReg(uint8_t addr, uint8_t val)
{
	HAL_StatusTypeDef transmitStatus;
	
	RC522_CS_Write(GPIO_PIN_RESET);		// CS pin is low - start SPI communication
	
	addr = addr << 1;									// bit7: R/W (1/0), bit6-1: register address, bit0 = 0.
	
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
	uint8_t dummyAddr = 0x00;
	uint8_t val;
	HAL_StatusTypeDef transmitStatus;
	
	RC522_CS_Write(GPIO_PIN_RESET);
	
	transmitStatus = HAL_SPI_Transmit(&hspi1, &addr, 1, 100);
	if(transmitStatus != HAL_OK)
	{
		Error_Handler();
	}
	
	// the address1 according to the data0 - refer RC522 datasheet, page 9, table 6.
	transmitStatus = HAL_SPI_TransmitReceive(&hspi1, &dummyAddr, &val, 1, 100);
	if(transmitStatus != HAL_OK)
	{
		Error_Handler();
	}
	
	RC522_CS_Write(GPIO_PIN_SET);
	
	return val;
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

void RC522_AntennaOn()
{
	RC522_SetBitMask(RC522_TxControlReg, 0x03);
}

void RC522_AntennaOff()
{
	RC522_ClearBitMask(RC522_TxControlReg, 0xFC);
}

// soft reset
void RC522_Reset()
{
	RC522_WriteReg(RC522_CommandReg, 0x0f);
}

void RC522_CS_Write(GPIO_PinState PinState)
{
	HAL_GPIO_WritePin(RC522_CS_GPIO, RC522_CS_PIN, PinState);
}

// Request command. the card activation procedure starts with a Request command (REQA or REQB)(ref. AN10834 - 4/19 - 2.1). In this lib, i use REQA.
RC522Status_e RC522_GetATQA()	
{
	RC522Status_e Status;

	RC522_WriteReg(RC522_BitFramingReg, 0x07);	// TxLastBits[2:0] = 7. Means 7 last bits of the last byte will be transmitted. Because REQA command has 7 bits.
	RC522_WriteReg(RC522_FIFODataReg, 0x26);		// Write REQA command code into FIFO.
	RC522_WriteReg(RC522_CommandReg, 0xC);			// 
}

uint8_t RC522_Transceive(uint8_t dataSend,uint8_t TxAlign, uint8_t *receiveBuff, uint8_t RxAlign)
{
	RC522_SetBitMask(RC522_BitFramingReg, TxAlign|(RxAlign<<4));
	RC522_WriteReg(RC522_FIFODataReg, dataSend);
	RC522_SetBitMask(RC522_CommandReg, RC522_TRANSCEIVE);	
	RC522_SetBit(RC522_BitFramingReg, 7);			// Start to transfer data. Combination with Transceive command 
	
	// reading rfid web - toCard()
}