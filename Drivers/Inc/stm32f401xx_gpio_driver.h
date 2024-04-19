/*
 * stm32401xx_gpio_driver.h
 *
 *  Created on: Jan 21, 2024
 *      Author: benja
 */

#ifndef INC_STM32F401XX_GPIO_DRIVER_H_
#define INC_STM32F401XX_GPIO_DRIVER_H_

#include "stm32f401xx.h"

/* GPIO pin config settings */

typedef struct {
	uint8_t GPIO_PinNumber;				/*!< possible values from @GPIO_PIN_NUMBERS >*/
	uint8_t GPIO_PinMode;				/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;				/*!< possible values from @GPIO_PIN_OUTPUT_SPEEDS >*/
	uint8_t GPIO_PinPuPdControl;		/*!< possible values from @GPIO_PIN_PUPD_CONFIGS >*/
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/* GPIO pin handle structure */

typedef struct {
	GPIO_RegDef_t *pGPIOx; 		/*!< pointer to the base address of the GPIO port of the pin >*/
	GPIO_PinConfig_t GPIO_PinConfig; /*!< holds GPIO pin config settings >*/
}GPIO_Handle_t;

/* @GPIO_PIN_NUMBERS */
/* GPIO pin numbers */
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15

/* @GPIO_PIN_MODES */
/* GPIO pin possible modes */
//Non-interrupt modes
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3

//Interrupt modes
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6

/* @GPIO_PIN_OUTPUT_TYPES */
/* GPIO pin possible output types */
#define GPIO_OP_TYPE_PP			0
#define GPIO_OP_TYPE_OD			1

/* @GPIO_PIN_OUTPUT_SPEEDS */
/* GPIO pin possible output speeds */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

/* @GPIO_PIN_PUPD_CONFIGS */
/* GPIO pin pull-up/pull-down configurations */
#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2


/************ APIs supported by this driver. For more info, check the function defns in the source file. ************/


/* peripheral clock setup */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enOrDi);

/* init and deinit */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/* data read and write */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

/* IRQ config and ISR handling */
void GPIO_IRQInterruptConfig(uint8_t irqNumber, uint8_t enOrDi);
void GPIO_IRQPriorityConfig(uint8_t irqNumber, uint32_t irqPriority);
void GPIO_IRQHandling(uint8_t pinNumber);



#endif /* INC_STM32F401XX_GPIO_DRIVER_H_ */

