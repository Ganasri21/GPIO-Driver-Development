/*
 * 008spi_cmd_handling.c
 *
 *  Created on: Nov 26, 2024
 *      Author: ganasri.s
 */

#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

//command codes
#define COMMAND_LED_CTRL       0x50
#define COMMAND_SENSOR_READ    0x51
#define COMMAND_LED_READ       0x52
#define COMMAND_PRINT          0x53
#define COMMAND_ID_READ        0x54

#define LED_ON     1
#define LED_OFF    0

// Arduino Analog pins
#define ANALOG_PIN0      0
#define ANALOG_PIN1      1
#define ANALOG_PIN2      2
#define ANALOG_PIN3      3
#define ANALOG_PIN4      4
#define ANALOG_PIN5      5

//Arduino LED
#define LED_PIN          8

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

	// always enable the clock before making changes to it!!!!!
		GPIO_PeriClockControl(SPIPins.pGPIOx, ENABLE);
		SPIPins.pGPIOx->MODER   &= 0x00000000;
		SPIPins.pGPIOx->OSPEEDR &= 0x00000000;
		SPIPins.pGPIOx->OTYPER  &= 0x00000000;
		SPIPins.pGPIOx->PUPDR   &= 0x00000000;
		SPIPins.pGPIOx->AFR[0]  &= 0x00000000;
		SPIPins.pGPIOx->AFR[1]  &= 0x00000000;


	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

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
	SPI2handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
	SPI2handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_DI;   // H/W slave Mgt enabled for NSS pin

	SPI_Init(&SPI2handle);
}

// If no slave - no need of MISO & NSS

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn, GpioLed;
	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// always enable the clock before making changes to it!!!!!
		GPIO_PeriClockControl(GPIOBtn.pGPIOx, ENABLE);
//		GPIOBtn.pGPIOx->MODER   = 0x00000000;
		GPIOBtn.pGPIOx->OSPEEDR = 0x00000000;
		GPIOBtn.pGPIOx->OTYPER  = 0x00000000;
		GPIOBtn.pGPIOx->PUPDR   = 0x00000000;
		GPIOBtn.pGPIOx->AFR[0]  = 0x00000000;
		GPIOBtn.pGPIOx->AFR[1]  = 0x00000000;


	GPIO_Init(&GPIOBtn);

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);
//	GPIO_PeriClockControl(GPIOD,ENABLE);

//	GpioLed.pGPIOx->MODER   &= 0x00000000;
	GpioLed.pGPIOx->OSPEEDR = 0x00000000;
	GpioLed.pGPIOx->OTYPER  = 0x00000000;
	GpioLed.pGPIOx->PUPDR   = 0x00000000;
	GpioLed.pGPIOx->AFR[0]  = 0x00000000;
	GpioLed.pGPIOx->AFR[1]  = 0x00000000;


	GPIO_Init(&GpioLed);
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == (uint8_t) 0xF5)
	{
		//ack
		return 1;
	}
	return 0;
}

int main(void)
{
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	//initialise_monitor_handles();

	printf("Application is running\n");

	GPIO_ButtonInit();

	SPI2_GPIOInits();  // func to initialize GPIO pins to behave as SPI2 pins

	SPI2_Inits();  // func for peripheral configuration

	printf("SPI Init. done\n");

	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay(); // to avoid debouncing issue

		// enable SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//1. CMD_LED_CTRL <pin no(1)>  <value(1)>

		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		//send command
		SPI_SendData(SPI2, &commandcode, 1);
		/*
		 * NOTE - In SPI communication, when Master / slave send 1 byte
		 * it also receives 1 byte in return
		 * This transmissio of 1 byte resulted 1 garbage byte collection
		 * in Rx buffer of the master and RXNE flag is set.
		 * So, dummy read is done to clear the flag
		 */

		//To do dummy read to clear off RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);


		// send some dummy bits (1 byte) to fetch response from slave - "Shift register logic"
		SPI_SendData(SPI2, &dummy_write, 1);

		// read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		// to verify whether received response is ACK or NACK
		if ( SPI_VerifyResponse(ackbyte) )
		{
			// send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args, 2);

			// dummy read
			SPI_ReceiveData(SPI2,args,2);
			printf("COMMAND_LED_CTRL Executed\n");

		} //end of COMMAND_LED_CTRL



		//2. CMD_SENOSR_READ   <analog pin number(1) >

//		wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_SENSOR_READ;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);


		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = ANALOG_PIN0;

			//send arguments
			SPI_SendData(SPI2,args,1); //sending one byte of

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI2,&dummy_write,1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2,&analog_read,1);
			printf("COMMAND_SENSOR_READ %d\n",analog_read);
		}

		//3.  CMD_LED_READ 	 <pin no(1) >

				//wait till button is pressed
				while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

				//to avoid button de-bouncing related issues 200ms of delay
				delay();

				commandcode = COMMAND_LED_READ;

				//send command
				SPI_SendData(SPI2,&commandcode,1);

				//do dummy read to clear off the RXNE
				SPI_ReceiveData(SPI2,&dummy_read,1);

				//Send some dummy byte to fetch the response from the slave
				SPI_SendData(SPI2,&dummy_write,1);

				//read the ack byte received
				SPI_ReceiveData(SPI2,&ackbyte,1);

				if( SPI_VerifyResponse(ackbyte))
				{
					args[0] = LED_PIN;

					//send arguments
					SPI_SendData(SPI2,args,1); //sending one byte of

					//do dummy read to clear off the RXNE
					SPI_ReceiveData(SPI2,&dummy_read,1);

					//insert some delay so that slave can ready with the data
					delay();

					//Send some dummy bits (1 byte) fetch the response from the slave
					SPI_SendData(SPI2,&dummy_write,1);

					uint8_t led_status;
					SPI_ReceiveData(SPI2,&led_status,1);
					printf("COMMAND_READ_LED %d\n",led_status);

				}

				//4. CMD_PRINT 		<len(2)>  <message(len) >

				//wait till button is pressed
				while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

				//to avoid button de-bouncing related issues 200ms of delay
				delay();

				commandcode = COMMAND_PRINT;

				//send command
				SPI_SendData(SPI2,&commandcode,1);

				//do dummy read to clear off the RXNE
				SPI_ReceiveData(SPI2,&dummy_read,1);

				//Send some dummy byte to fetch the response from the slave
				SPI_SendData(SPI2,&dummy_write,1);

				//read the ack byte received
				SPI_ReceiveData(SPI2,&ackbyte,1);

				uint8_t message[] = "Hello ! How are you ??";
				if( SPI_VerifyResponse(ackbyte))
				{
					args[0] = strlen((char*)message);

					//send arguments
					SPI_SendData(SPI2,args,1); //sending length

					//do dummy read to clear off the RXNE
					SPI_ReceiveData(SPI2,&dummy_read,1);

					delay();

					//send message
					for(int i = 0 ; i < args[0] ; i++){
						SPI_SendData(SPI2,&message[i],1);
						SPI_ReceiveData(SPI2,&dummy_read,1);
					}

					printf("COMMAND_PRINT Executed \n");

				}

				//5. CMD_ID_READ
				//wait till button is pressed
				while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0) );

				//to avoid button de-bouncing related issues 200ms of delay
				delay();

				commandcode = COMMAND_ID_READ;

				//send command
				SPI_SendData(SPI2,&commandcode,1);

				//do dummy read to clear off the RXNE
				SPI_ReceiveData(SPI2,&dummy_read,1);

				//Send some dummy byte to fetch the response from the slave
				SPI_SendData(SPI2,&dummy_write,1);

				//read the ack byte received
				SPI_ReceiveData(SPI2,&ackbyte,1);

				uint8_t id[11];
				uint32_t i=0;
				if( SPI_VerifyResponse(ackbyte))
				{
					//read 10 bytes id from the slave
					for(  i = 0 ; i < 10 ; i++)
					{
						//send dummy byte to fetch data from slave
						SPI_SendData(SPI2,&dummy_write,1);
						SPI_ReceiveData(SPI2,&id[i],1);
					}

					id[10] = '\0';

					printf("COMMAND_ID : %s \n",id);

				}


		// before disabling SPI2 peripheral - confirm that SPI is not busy
		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

		// disabling SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}
