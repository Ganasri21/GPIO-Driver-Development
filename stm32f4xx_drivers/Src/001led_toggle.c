/*
 * 001led_toggle.c
 *
 *  Created on: Nov 18, 2024
 *      Author: ganasri.s
 */

#include "stm32f407xx.h"

void delay(void)
{
	for (uint32_t i = 0; i < 5000000 ; ++i );
}

int main(void)
{

	GPIO_Handle_t Gpioled;

	Gpioled.pGPIOx = GPIOD;
	Gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	Gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&Gpioled);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_15);
		delay();
	}
	return 0;
}

