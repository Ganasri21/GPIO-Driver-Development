/*
 * 006spi_tx_testing.c
 *
 *  Created on: Nov 22, 2024
 *      Author: ganasri.s
 */

#include <string.h>
#include "stm32f407xx.h"

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

	// always enable the clock before making changes to it!!!!!
//	GPIO_PeriClockControl(SPIPins.pGPIOx, ENABLE);
//	SPIPins.pGPIOx->MODER   &= 0x00000000;
//	SPIPins.pGPIOx->OSPEEDR &= 0x00000000;
//	SPIPins.pGPIOx->OTYPER  &= 0x00000000;
//	SPIPins.pGPIOx->PUPDR   &= 0x00000000;
//	SPIPins.pGPIOx->AFR[0]  &= 0x00000000;
//	SPIPins.pGPIOx->AFR[1]  &= 0x00000000;


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
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
//	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;      // generates SCLK of 8MHz
	SPI2handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_EN;   // S/W slave Mgt enabled for NSS pin

	SPI_Init(&SPI2handle);
}

// If no slave - no need of MISO & NSS

int main(void)
{
	char user_data[] = "HELLO WORLD!";    // user buffer

	SPI2_GPIOInits();  // func to initialize GPIO pins to behave as SPI2 pins

	SPI2_Inits();  // func for peripheral configuration

	/*
	 * NOTE - By default SPI peripheral is disabled!!!
	 * When SPI peripheral is enabled, the SPI peripheral will be busy in data communication
	 * and SPI peripheral will not accept any changes made to the control registers.
	 * So, it is better to do all control register configuration while SPI peripheral is disable!!!!
	 */

	// This enables NSS signal internally high and avoids MODF error!!
	SPI_SSIConfig(SPI2, ENABLE);

	// enable SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t*) user_data, strlen(user_data));

	// before disabling SPI2 peripheral - confirm that SPI is not busy
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

	// disabling SPI2 peripheral
	SPI_PeripheralControl(SPI2, DISABLE);
	while(1); // infinite loop to hang the application

	return 0;
}
