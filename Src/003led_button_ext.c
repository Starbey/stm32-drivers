/*
 * 003led_button_ext.c
 *
 *  Created on: Jan 22, 2024
 *      Author: benja
 */

#include "stm32f401xx.h"
#include <string.h>

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
	gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	gpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&gpioBtn);

	while (1){
		if (GPIO_ReadFromInputPin(GPIOB,GPIO_PIN_NO_12) == BTN_PRESSED) {
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
		}
	}

	return 0;
}
