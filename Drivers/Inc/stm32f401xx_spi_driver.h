/*
 * stm32f401xx_spi_driver.h
 *
 *  Created on: Jan 28, 2024
 *      Author: benja
 */

#ifndef INC_STM32F401XX_SPI_DRIVER_H_
#define INC_STM32F401XX_SPI_DRIVER_H_

#include "stm32f401xx.h"

/* Config. struct for SPIx peripheral */

typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/* Handle struct for SPIx peripheral */

typedef struct{
	SPI_RegDef_t			*pSPIx; //points to base address of SPIx peripheral
	SPI_Config_t			spiConfig;
	uint8_t					*pTxBuffer;
	uint8_t					*pRxBuffer;
	uint32_t				txLen;
	uint32_t				rxLen;
	uint8_t					txState;
	uint8_t					rxState;
}SPI_Handle_t;

/*
 * @SPI_DeviceMode
 */

#define SPI_DEVICE_MODE_SLAVE						0
#define SPI_DEVICE_MODE_MASTER 						1

/*
 * @SPI_BusConfig
 */

#define SPI_BUS_CONFIG_FD							1
#define SPI_BUS_CONFIG_HD							2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY				3

/*
 * @SPI_SclkSpeed
 */

#define SPI_SCLK_SPEED_DIV2							0
#define SPI_SCLK_SPEED_DIV4							1
#define SPI_SCLK_SPEED_DIV8							2
#define SPI_SCLK_SPEED_DIV16						3
#define SPI_SCLK_SPEED_DIV32						4
#define SPI_SCLK_SPEED_DIV64						5
#define SPI_SCLK_SPEED_DIV128						6
#define SPI_SCLK_SPEED_DIV256						7

/*
 * @SPI_DFF
 */

#define SPI_DFF_8BITS								0 //default
#define SPI_DFF_16BITS								1

/*
 * @CPOL
 */

#define SPI_CPOL_HIGH								1
#define SPI_CPOL_LOW								0

/*
 * @CPHA
 */

#define SPI_CPHA_HIGH								1
#define SPI_CPHA_LOW								0

/*
 * @SPI_SSM
 */

#define SPI_SSM_DI									0
#define SPI_SSM_EN									1

/*
 * SPI related status flag macros
 */

#define SPI_RXNE_FLAG								(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG 								(1 << SPI_SR_TXE)
#define SPI_TXE_CHSIDE_FLAG 						(1 << SPI_SR_CHSIDE)
#define SPI_TXE_UDR_FLAG  							(1 << SPI_SR_UDR)
#define SPI_TXE_CRC_ERR_FLAG  						(1 << SPI_SR_CRC_ERR)
#define SPI_TXE_MODF_FLAG  							(1 << SPI_SR_MODF)
#define SPI_TXE_OVR_FLAG  							(1 << SPI_SR_OVR)
#define SPI_TXE_BSY_FLAG  							(1 << SPI_SR_BSY)
#define SPI_TXE_FRE_FLAG  							(1 << SPI_SR_FRE)

/*
 * SPI application states
 */

#define SPI_READY									0
#define SPI_BSY_IN_TX								1
#define SPI_BSY_IN_RX								2

/*
 * possible SPI application events
 */

#define SPI_EVENT_TX_CMPLT							0
#define SPI_EVENT_RX_CMPLT							1
#define SPI_EVENT_OVR_ERR							2
#define SPI_EVENT_CRC_ERR							3

/* peripheral clock setup */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t enOrDi);

/* init and deinit */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/* return the status of a flag bit */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flag);

/* clears OVR flag */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);

/* close Tx */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);

/* close Rx */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/* application callback */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t appEv);//appEv - app event

/* data send and receive */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);

/* IRQ config. and ISR handling */
void SPI_IRQInterruptConfig(uint8_t irqNumber, uint8_t enOrDi);
void SPI_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/* en/di SPIx peripheral */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t enOrDi);

/* configures SSI for non-multi-master scenario */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t enOrDi);

/* configures SSOE for non-multi-master scenario */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t enOrDi);

#endif /* INC_STM32F401XX_SPI_DRIVER_H_ */
