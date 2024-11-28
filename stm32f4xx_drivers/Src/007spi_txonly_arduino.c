/*
 * 007spi_txonly_arduino.c
 *
 *  Created on: Nov 25, 2024
 *      Author: ganasri.s
 */

#include <string.h>
#include "stm32f407xx.h"

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

/*
 * SPI2 (Alternate Functionality of Port B)
 * PB14 --> MISO
 * PB15 --> MOSI
 * PB13 --> SCLK
 * PB12 --> NSS
 * ALT function mode - 5
 */

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	// Open drain configuration - not req for SPI, but it is needed for I2C as the specification says it
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;      // generates SCLK of 2MHz
	SPI2handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_DI;   // H/W slave Mgt enabled for NSS pin

	SPI_Init(&SPI2handle);
}

// If no slave - no need of MISO & NSS

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;
	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);
}

int main(void)
{
	char user_data[] = "AbCd";    // user buffer

	GPIO_ButtonInit();

	SPI2_GPIOInits();  // func to initialize GPIO pins to behave as SPI2 pins

	SPI2_Inits();  // func for peripheral configuration

	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay(); // to avoid debouncing issue

		// enable SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		// first send length information of the message to slave
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);

		//to send data
		SPI_SendData(SPI2, (uint8_t*) user_data, strlen(user_data));

		// before disabling SPI2 peripheral - confirm that SPI is not busy
		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

		// disabling SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}


