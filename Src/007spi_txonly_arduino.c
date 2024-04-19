/*
 * 007spi_txonly_arduino.c
 *
 *  Created on: Feb 5, 2024
 *      Author: benja
 */

#include <string.h>
#include "stm32f401xx.h"

#define BTN_PRESSED				0
#define BTN_UNPRESSED			1

//PB15 --> MOSI
//PB14 --> MISO
//PB13 --> SCLK
//PB12 --> NSS
//alt. function mode: 5

void delay(void){
	for (uint32_t i = 0 ; i < 500000 ; i ++);
}

void SPI2_GPIOInits(){
	GPIO_Handle_t spiPins;

	spiPins.pGPIOx = GPIOB;
	spiPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	spiPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	spiPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	spiPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	spiPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	spiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&spiPins);

	//MOSI
	spiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&spiPins);

	//MISO (unneeded)
	/*spiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&spiPins);*/

	//NSS
	spiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&spiPins);
}

void SPI2_Inits(void){
	SPI_Handle_t spi2Handle;
	spi2Handle.pSPIx = SPI2;
	spi2Handle.spiConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	spi2Handle.spiConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	spi2Handle.spiConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //16 MHz APB1 clock / 8 = 2 MHz SCLK
	spi2Handle.spiConfig.SPI_DFF = SPI_DFF_8BITS;
	spi2Handle.spiConfig.SPI_CPOL = SPI_CPOL_LOW;
	spi2Handle.spiConfig.SPI_CPHA = SPI_CPHA_LOW;
	spi2Handle.spiConfig.SPI_SSM = SPI_SSM_DI; //hardware slave management enabled for NSS pin

	SPI_Init(&spi2Handle); //initializes SPI2 peripheral
	spi2Handle.pSPIx->CR2 |= (1 << SPI_CR2_SSOE); //forces CR1 in master mode
}

void GPIO_ButtonInit(void){
	//button configuration (PC13)
	GPIO_Handle_t gpioBtn;
	gpioBtn.pGPIOx = GPIOC;
	gpioBtn.GPIO_PinConfig.GPIO_PinNumber = 13;
	gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	gpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&gpioBtn);
	//note that clock is enabled in GPIO_Init
}

int main(void){
	char strTx[] = "Hello world"; //Tx buffer

	GPIO_ButtonInit();

	SPI2_GPIOInits();
	SPI2_Inits();
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1){
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_UNPRESSED);
		delay();

		SPI_PeripheralControl(SPI2, ENABLE); //enable peripheral

		uint8_t dataLen = strlen(strTx);
		SPI_SendData(SPI2, &dataLen, 1);//send length of following data (length metadata is only 1 byte long)

		SPI_SendData(SPI2, (uint8_t*) strTx, strlen(strTx)); //send data

		while ( SPI_GetFlagStatus(SPI2, SPI_TXE_BSY_FLAG) ); //check if SPI is not busy

		SPI_PeripheralControl(SPI2, DISABLE); //disable peripheral
	}
	return 0;
}
