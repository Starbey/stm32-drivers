/*
 * 001led_toggle.c
 *
 *  Created on: Jan 21, 2024
 *      Author: benja
 */

#include "stm32f401xx.h"

void delay(void){
	for (uint32_t i = 0 ; i < 500000 ; i ++);
}

int main(void){
	GPIO_Handle_t gpioLed;

	//working with PA5
	gpioLed.pGPIOx = GPIOA;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = 5;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpioLed);

	while (1){
		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
		delay();
	}

	return 0;
}


