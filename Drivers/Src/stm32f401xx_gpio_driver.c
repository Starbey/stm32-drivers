/*
 * stm32401xx_gpio._driver.c
 *
 *  Created on: Jan 21, 2024
 *      Author: benja
 */

#include "stm32f401xx_gpio_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  void
 *
 * @Note              -  n/a

 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enOrDi){
	if(enOrDi == ENABLE){
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		}
	}
	else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - initializes registers of a GPIO port
 *
 * @*pGPIOHandle      - pointer to GPIO port handle
 *
 * @return            -  void
 *
 * @Note              -  n/a

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp = 0; //temp register

	//enable peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//configure mode of GPIO pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){ //non-it mode
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //multiplied by 2 bc pin modes are 2 bits wide
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear pin register before setting
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else { //it mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//configure the falling trigger selection register (FTSR)
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clear corresponding RTSR bit before setting FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//configure the rising trigger selection register (RTSR)
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clear corresponding FTSR bit before setting RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//configure both FTSR and RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//configure the GPIO port selection in SYSFCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portNum = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1]= portNum << (4 * temp2);

		//enable the EXTI interrupt delivery using the interrupt mask register (IMR)
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0;

	//configure speed of GPIO pin
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//configure pull-up/pull-down settings
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//configure the output type
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//configure the alternate functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~( 0xF << (4 * temp2) );
		pGPIOHandle->pGPIOx->AFR[temp1] |= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2) );
	}
}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - enables or disables peripheral clock for the given GPIO port
 *
 * @*pGPIOx        	  - pointer to GPIO peripheral base address
 *
 * @return            -  void
 *
 * @Note              -  n/a

 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB) {
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOC) {
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOD) {
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOE) {
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOH) {
		GPIOA_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  -  GPIO_ReadFromInputPin
 *
 * @brief             -  reads and returns input value of a GPIO pin
 *
 * @*pGPIOx        	  -  pointer to GPIO peripheral (port) base address
 * @pinNumber		  -  pin number on port
 *
 * @return            -  uint8_t: value of pin
 *
 * @Note              -  n/a

 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber){
	return ( (uint8_t) (pGPIOx->IDR >> pinNumber) & 0x1 ); //returns LSB
}

/*********************************************************************
 * @fn      		  -  GPIO_ReadFromInputPin
 *
 * @brief             -  reads and returns input value of a GPIO port
 *
 * @*pGPIOx        	  -  pointer to GPIO peripheral (port) base address
 *
 * @return            -  uint16_t: value of port
 *
 * @Note              -  n/a

 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	return ( (uint16_t) pGPIOx->IDR ); //returns entire 16-bit port
}


void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value){
	if (value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << pinNumber);
	}
	else{
		pGPIOx->ODR &= ~(1 << pinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){
	pGPIOx->ODR = value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber){
	pGPIOx->ODR ^= (1 << pinNumber);
}

/* IRQ config and ISR handling */
void GPIO_IRQInterruptConfig(uint8_t irqNumber, uint8_t enOrDi){
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

void GPIO_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority){
	//store IPR register
	uint8_t iprx = irqNumber / 4;
	uint8_t iprx_section = irqNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (irqPriority << shift_amount);//irqPriority is 8-bit, shift it to section's 4 MSBs (for STM32)
}

void GPIO_IRQHandling(uint8_t pinNumber){
	//clear EXTI pending register corresponding to the pin
	if (EXTI->PR & (1 << pinNumber) ) EXTI->PR |= (1 << pinNumber);
}

