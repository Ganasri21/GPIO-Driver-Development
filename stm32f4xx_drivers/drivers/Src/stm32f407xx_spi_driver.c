/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Nov 22, 2024
 *      Author: ganasri.s
 */

#include "stm32f407xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

 /*
  * Peripheral Clock Setup
  */
/******************************************************************
 * @fn              -  SPI_PeriClockControl
 *
 * @brief           -  This function enables or disables peripheral clock
 *
 * @param[in]       -  Base address of SPI peripheral
 * @param[in]       -  ENABLE or DISABLE Macros
 * @param[in]       -
 *
 * @return          -  none
 *
 * @Note            -  none

 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}else if (pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}else if (pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}


/*
 * Init and De-init
 */
/******************************************************************
 * @fn              -  SPI_Init
 *
 * @brief           -
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -  none
 *
 * @Note            -  none

 */

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//first Configure SPI_CR1 registers
	uint32_t tempreg = 0;

	// 1. configure device mode
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. configure Bus_Config
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDI mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDI mode should be set
		tempreg |= ( 1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);

		//RXONLY bit must be set
		tempreg |= ( 1 << SPI_CR1_RXONLY);
	}

	// 3. configure the SPI serial clock speed(Baud Rate)
	tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. Configure DFF
	tempreg |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	// 5. Configure CPOL
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	// 6. Configure CPHA
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	// 7. Enable SSM
	tempreg |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/******************************************************************
 * @fn              -  SPI_DeInit
 *
 * @brief           -
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -  none
 *
 * @Note            -  none

 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}else if (pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Data Send & Receive
 */
/******************************************************************
 * @fn              -  SPI_SendData
 *
 * @brief           -
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -  none
 *
 * @Note            -  This is a Blocking call

 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	// Polling Based or Blocking Call
	while(Len > 0)
	{
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2.check DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bit DFF
			//1. Load DR with 1 byte of data & increment the Buffer address
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			Len--;
			Len--;  // 2 bytes of Data
			(uint16_t*) pTxBuffer++;

		}else{

			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;   // 1 byte of Data
			pTxBuffer++;
		}
	}
}

/******************************************************************
 * @fn              -  SPI_PeriClockControl
 *
 * @brief           -  This function enables or disables peripheral clock
 *
 * @param[in]       -  Base address of SPI peripheral
 * @param[in]       -  ENABLE or DISABLE Macros
 * @param[in]       -
 *
 * @return          -  none
 *
 * @Note            -  none

 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
			{
				//1. wait until RXNE is set
				while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)  == (uint8_t)FLAG_RESET );

				//2. check the DFF bit in CR1
				if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
				{
					//16 bit DFF
					//1. load the data from DR to Rxbuffer address
					 *((uint16_t*)pRxBuffer) = pSPIx->DR ;
					Len--;
					Len--;
					(uint16_t*)pRxBuffer++;
				}else
				{
					//8 bit DFF
					*(pRxBuffer) = pSPIx->DR ;
					Len--;
					pRxBuffer++;
				}
			}
}

/*
  * Peripheral Control Enable
  */
/******************************************************************
 * @fn              -  SPI_PeripheralControl
 *
 * @brief           -  This function enables or disables peripheral control
 *
 * @param[in]       -  Base address of SPI peripheral
 * @param[in]       -  ENABLE or DISABLE Macros
 * @param[in]       -
 *
 * @return          -  none
 *
 * @Note            -  By default SPI peripheral is disabled!!!
 * 					   When SPI peripheral is enabled, the SPI peripheral will be busy in data communication
 * 					   and SPI peripheral will not accept any changes made to the control registers.
 *                     So, it is better to do all control register configuration while SPI peripheral is disable!!!!
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/******************************************************************
 * @fn              -  SPI_SSIConfig
 *
 * @brief           -  This function enables or disables SSI bit
 *
 * @param[in]       -  Base address of SPI peripheral
 * @param[in]       -  ENABLE or DISABLE Macros
 * @param[in]       -
 *
 * @return          -  none
 *
 * @Note            -  NOTE - SSI bit influences NSS state when SSM=1.
 * 						By default SSI=0, so NSS will be pulled low which is not acceptable
 * 						for master when working in non-multi master situation
 * 						This means that the master is in slave mode!!!
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/******************************************************************
 * @fn              -  SPI_SSOEConfig
 *
 * @brief           -  This function enables or disables SSOE bit
 *
 * @param[in]       -  Base address of SPI peripheral
 * @param[in]       -  ENABLE or DISABLE Macros
 * @param[in]       -
 *
 * @return          -  none
 *
 * @Note            -  Helps to control NSS pin toggle in accordance
 * 					   with SPE bit
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}


/*
 * IRQ Configuration and ISR Handling
 */
/******************************************************************
 * @fn              -  SPI_IRQInterruptConfig
 *
 * @brief           -  To configure Interrupt request number
 *
 * @param[in]       -  IRQ Number
 * @param[in]       -  ENABLE or DISABLE Macros
 * @param[in]       -
 *
 * @return          -  none
 *
 * @Note            -  none

 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64)  //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << IRQNumber % 32);
		}else if(IRQNumber >= 64 && IRQNumber < 96)  // 64 to 95
		{
			// program ISER2 register
			*NVIC_ISER2 |= ( 1 << IRQNumber % 64);
			// doubt , ISER2 or ISER3
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64)  //32 to 63
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << IRQNumber % 32);
		}else if(IRQNumber >= 64 && IRQNumber < 96)  // 64 to 95
		{
			// program ICER2 register
			*NVIC_ICER2 |= ( 1 << IRQNumber % 64);
			// doubt , ICER2 or ICER3
		}
	}
}

/******************************************************************
 * @fn              -  SPI_IRQPriorityConfig
 *
 * @brief           -
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -  none
 *
 * @Note            -  none

 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// 1. First lets find out IPR Register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= ( IRQPriority << shift_amount );
}

/******************************************************************
 * @fn              -  SPI_IRQHandling
 *
 * @brief           -
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -  none
 *
 * @Note            -  none

 */

/******************************************************************
 * @fn              -  SPI_SendDataIT
 *
 * @brief           -
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -  none
 *
 * @Note            -  none

 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_TX)
	{
		// 1. Save Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		// 2. Mark the  SPI state as busy in Transmission s that
		//    no other code can take over the same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	// 4. Data Transmission will be handles by the ISR code

	return state;
}

/******************************************************************
 * @fn              -  SPI_ReceiveDataIT
 *
 * @brief           -
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -  none
 *
 * @Note            -  none

 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_RX)
	{
		// 1. Save Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		// 2. Mark the  SPI state as busy in Reception that
		//    no other code can take over the same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3. Enable the RXNEIE control bit to get interrupt whenever RXNE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	// 4. Data Transmission will be handles by the ISR code

	return state;

}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1 , temp2;
	//first lets check for TXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if( temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	// check for RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	// check for ovr flag
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if( temp1 && temp2)
	{
		//handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}

}


// some helper function implementations - private to spi_driver.c
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bit DFF
		//1. Load DR with 1 byte of data & increment the Buffer address
		pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;  // 2 bytes of Data
		(uint16_t*) pSPIHandle->pTxBuffer++;

	}else{

		//8 bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--; // 1 byte of Data
		pSPIHandle->pTxBuffer++;
	}
	if(! pSPIHandle->TxLen)
	{
		// TxLen is zero
		// Close SPI transmission & inform the application that the Tx is over
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}


}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// to do rxing as per dff
	if (pSPIHandle->pSPIx->CR1 & (1 << 11))
	{
		// 16 bit
		*((uint16_t*) pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;
	}else
	{
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}
	if(! pSPIHandle->RxLen)
	{
		// reception is complete
		SPI_CLoseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);   // this prevents interrupts from setting up of TXE flag
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CLoseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	// This is a weak implementation. the application may override the function
}
