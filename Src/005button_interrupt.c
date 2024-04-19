/*
 * 005button_interrupt.c
 *
 *  Created on: Jan 23, 2024
 *      Author: benja
 */


/*
 * 003led_button_ext.c
 *
 *  Created on: Jan 22, 2024
 *      Author: benja
 */

#include <string.h>
#include "stm32f401xx.h"

#define BTN_PRESSED 			0

void delay(void){
	for (uint32_t i = 0 ; i < 250000 ; i ++);
}

int main(void){
	GPIO_Handle_t gpioLed, gpioBtn;
	memset(&gpioLed, 0, sizeof(gpioLed));
	memset(&gpioBtn, 0, sizeof(gpioBtn));

	//LED configuration (PA8)
	gpioLed.pGPIOx = GPIOA;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpioLed);

	//button configuration (PB12)
	gpioBtn.pGPIOx = GPIOB;
	gpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT; //falling edge
	gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&gpioBtn);

	//IRQ configs
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI10_15, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI10_15, ENABLE); //PB12 will trigger interrupt

	while(1);

	return 0;
}

void EXTI15_10_IRQHandler(void){
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_12);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
}
