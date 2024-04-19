/*
 * stm32f401xx_spi_driver.c
 *
 *  Created on: Jan 28, 2024
 *      Author: benja
 */

#include "stm32f401xx_spi_driver.h"

//helper functions
static void SPI_TXE_Interrupt_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_RXNE_Interrupt_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_OVR_ERR_Interrupt_Handle(SPI_Handle_t *pSPIHandle);

/* peripheral clock setup */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t enOrDi){
	if(enOrDi == ENABLE){
		if (pSPIx == SPI1){
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2){
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4){
			SPI4_PCLK_EN();
		}
	}
	else {
		if (pSPIx == SPI1){
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2){
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3){
			SPI3_PCLK_DI();
		}
		else if (pSPIx == SPI4){
			SPI4_PCLK_DI();
		}
	}
}

/* init and deinit */
void SPI_Init(SPI_Handle_t *pSPIHandle){
	//peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//config. SPI_CR1 register

	uint32_t tempReg = 0;

	//1. config device mode
	tempReg |= pSPIHandle->spiConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. config bus
	if (pSPIHandle->spiConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		//reset BIDI mode
		tempReg &= ~(1 << SPI_CR1_BIDI_MODE);
	}
	else if (pSPIHandle->spiConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		//set BIDI mode
		tempReg |= (1 << SPI_CR1_BIDI_MODE);
	}
	else if (pSPIHandle->spiConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		//clear BIDI mode and set RX-only bit
		tempReg &= ~(1 << SPI_CR1_BIDI_MODE);
		tempReg |= (1 << SPI_CR1_RX_ONLY);
	}

	//3. config serial clock speed
	tempReg |= pSPIHandle->spiConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. configure DFF
	tempReg |= pSPIHandle->spiConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure CPOL
	tempReg |= pSPIHandle->spiConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. configure CPHA
	tempReg |= pSPIHandle->spiConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempReg;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx){
	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	}
	else if (pSPIx == SPI4) {
		SPI4_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flag){
	if (pSPIx->SR & flag) return FLAG_SET;
	return FLAG_RESET;
}

/* data send and receive (blocking)*/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len){
	while(len > 0){ //blocks until all bytes have been transferred
		//wait until TXE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//check DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF) ){
			//config for 16-bit DFF
			pSPIx->DR = *( (uint16_t*) pTxBuffer); //typecast into uint16_t pointer first, then dereference to convert 8-bit to 16-bit
			len-=2;
			pTxBuffer+=2; //increment by 2 because 2 bytes of data has been transferred
		}
		else {
			//config for 8-bit DFF
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++; //increment by 1 because 1 byte of data has been transferred

		}
	}
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len){
	while(len > 0){ //blocks until all bytes have been transferred
		//wait until RXNE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//check DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF) ){
			//config for 16-bit DFF
			*( (uint16_t*) pRxBuffer) = pSPIx->DR; //typecast into uint16_t pointer first, then dereference to convert 8-bit to 16-bit
			len-=2;
			pRxBuffer+=2; //increment by 2 because 2 bytes of data has been transferred
		}
		else {
			//config for 8-bit DFF
			*pRxBuffer = pSPIx->DR;
			len--;
			pRxBuffer++; //increment by 1 because 1 byte of data has been transferred

		}
	}
}

/* IRQ config. and ISR handling */
void SPI_IRQInterruptConfig(uint8_t irqNumber, uint8_t enOrDi){
	if (enOrDi == ENABLE){ //sets IRQ bit corresponding to IRQ number in NVIC
		if (irqNumber <= 31){ //0 - 31
			//config ISER0 register
			*NVIC_ISER0 |= (1 << irqNumber);
		}
		else if (irqNumber > 31 && irqNumber < 64){ //32 - 63
			//config ISER1 register
			*NVIC_ISER1 |= (1 << (irqNumber % 32) );
		}
		else if (irqNumber >= 64 && irqNumber < 96){ //84 - 95
			//config ISER2 register
			*NVIC_ISER2 |= (1 << (irqNumber % 64) );
		}
	}
	else if (enOrDi == DISABLE){
		if (irqNumber <= 31){ //0 - 31
			//config ICER0 register
			*NVIC_ICER0 |= (1 << irqNumber);
		}
		else if (irqNumber > 31 && irqNumber < 64){ //32 - 63
			//config ICER1 register
			*NVIC_ICER1 |= (1 << (irqNumber % 32) );
		}
		else if (irqNumber >= 64 && irqNumber < 96){ //84 - 95
			//config ISER2 register
			*NVIC_ICER2 |= (1 << (irqNumber % 64) );
		}
	}
}

void SPI_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority){
	//store IPR register
	uint8_t iprx = irqNumber / 4;
	uint8_t iprx_section = irqNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (irqPriority << shift_amount);//irqPriority is 8-bit, shift it to section's 4 MSBs (for STM32)
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len){//gets things ready for when a TXE interrupt arrives
	if (pSPIHandle->txState != SPI_BSY_IN_TX){
		//save Tx buffer address and length in global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->txLen = len;

		//mark the SPI state as busy in transmission (Tx) so no other code can take over the same SPI peripheral until transmission ends
		pSPIHandle->txState = SPI_BSY_IN_TX;

		//set TXEIE control bit to get interrupt when the TXE flag bit is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		//data transmission will be handled by the ISR code
	}
	return pSPIHandle->txState;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len){
	if (pSPIHandle->rxState != SPI_BSY_IN_RX){
		//save Rx buffer address and length in global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->rxLen = len;

		//mark the SPI state as busy in receive (Rx) so no other code can take over the same SPI peripheral until receive ends
		pSPIHandle->rxState = SPI_BSY_IN_RX;

		//set RXNEIE control bit to get interrupt when the RXNE flag bit is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		//data transmission will be handled by the ISR code in application layer
	}
	return pSPIHandle->rxState;
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
	uint8_t tempFlag1, tempFlag2;

	//check for TXE (transmit)
	tempFlag1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE); //check TXE flag
	tempFlag2 = pSPIHandle->pSPIx->SR & (1 << SPI_CR2_TXEIE); //check TXE interrupt flag

	if (tempFlag1 && tempFlag2){
		//handle TXE
		SPI_TXE_Interrupt_Handle(pSPIHandle);
	}

	//check for RXE (receive)
	tempFlag1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE); //check RXNE flag
	tempFlag2 = pSPIHandle->pSPIx->SR & (1 << SPI_CR2_RXNEIE); //check RXNE interrupt flag

	if (tempFlag1 && tempFlag2){
		//handle RXNE
		SPI_RXNE_Interrupt_Handle(pSPIHandle);
	}

	//check for OVR (overrun error)
	tempFlag1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR); //check RXNE flag
	tempFlag2 = pSPIHandle->pSPIx->SR & (1 << SPI_CR2_ERRIE); //check RXNE interrupt flag

	if (tempFlag1 && tempFlag2){
		//handle RXNE
		SPI_OVR_ERR_Interrupt_Handle(pSPIHandle);
	}
}

/* en/di SPIx peripheral */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t enOrDi){
	if(enOrDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/* configures SSI for non-multi-master scenario */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t enOrDi){
	if(enOrDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/* configures SSOE for non-multi-master scenario */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t enOrDi){
	if(enOrDi == ENABLE){
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/* helper functions */
static void SPI_TXE_Interrupt_Handle(SPI_Handle_t *pSPIHandle){ //actually transmits data
	//check DFF bit in CR1
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) ){
		//config for 16-bit DFF
		pSPIHandle->pSPIx->DR = *( (uint16_t*) pSPIHandle->pTxBuffer); //typecast into uint16_t pointer first, then dereference to convert 8-bit to 16-bit
		pSPIHandle->txLen-=2;
		pSPIHandle->pTxBuffer+=2; //increment by 2 because 2 bytes of data have been transferred
	}
	else {
		//config for 8-bit DFF
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->txLen--;
		pSPIHandle->pTxBuffer++; //increment by 1 because 1 byte of data has been transferred
	}

	if (pSPIHandle->txLen == 0){
		SPI_CloseTransmission(pSPIHandle);

		//call callback to inform application that Tx is over
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void SPI_RXNE_Interrupt_Handle(SPI_Handle_t *pSPIHandle){
	//check DFF bit in CR1
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) ){
		//config for 16-bit DFF
		*( (uint16_t*) pSPIHandle->pRxBuffer ) = (uint16_t) pSPIHandle->pSPIx->DR; //typecast into uint16_t pointer first, then dereference to convert 8-bit to 16-bit
		pSPIHandle->rxLen-=2;
		pSPIHandle->pRxBuffer+=2; //increment by 2 because 2 bytes of data has been transferred
	}
	else {
		//config for 8-bit DFF
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->rxLen--;
		pSPIHandle->pRxBuffer++; //increment by 1 because 1 byte of data has been transferred
	}

	if (pSPIHandle->rxLen == 0){
		//close SPI reception and inform application that Rx is over
		SPI_CloseReception(pSPIHandle);
		//call callback to inform application that Rx is over
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void SPI_OVR_ERR_Interrupt_Handle(SPI_Handle_t *pSPIHandle){
	if (pSPIHandle->txState != SPI_BSY_IN_TX){ //check first if application can transmit. if so, don't clear because it might need the data in DR and SR
		SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	}

	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	//close SPI transmission and inform application that Rx is over
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);//close SPI transmission

	//reset public member variables
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->rxLen = 0;
	pSPIHandle->rxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	//close SPI reception and inform application that Rx is over
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);//close SPI transmission

	//reset public member variables
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->rxLen = 0;
	pSPIHandle->rxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t appEv){

}
