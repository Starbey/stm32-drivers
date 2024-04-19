/*
 * 002led_button.c
 *
 *  Created on: Jan 21, 2024
 *      Author: benja
 */

#include "stm32f401xx.h"

#define BTN_PRESSED 			0

void delay(void){
	for (uint32_t i = 0 ; i < 250000 ; i ++);
}

int main(void){
	GPIO_Handle_t gpioLed, gpioBtn;

	//LED configuration (PA5)
	gpioLed.pGPIOx = GPIOA;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = 5;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpioLed);

	//button configuration (PC13)
	gpioBtn.pGPIOx = GPIOC;
	gpioBtn.GPIO_PinConfig.GPIO_PinNumber = 13;
	gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	gpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&gpioBtn);

	while (1){
		if (GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) == BTN_PRESSED) {
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		}
	}

	return 0;
}

