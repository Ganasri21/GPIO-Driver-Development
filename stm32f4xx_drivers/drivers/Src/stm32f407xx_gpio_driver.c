/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Nov 15, 2024
 *      Author: ganasri.s
 */


#include "stm32f407xx_gpio_driver.h"

/*
 * Peripheral Clock Setup
 */

/******************************************************************
 * @fn              -  GPIO_PeriClockControl
 *
 * @brief           -  This function enables or disables peripheral clock for a given GPIO Port
 *
 * @param[in]       -  Base address of GPIO peripheral
 * @param[in]       -  ENABLE or DISABLE Macros
 * @param[in]       -
 *
 * @return          -  none
 *
 * @Note            -  none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}



/*
 * Init and De-init
 */

/******************************************************************
 * @fn              - GPIO_Init
 * @brief           - To initialize GPIO port - take pointer to the Handle structure
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -
 *
 * @Note            -

 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;    // temp.register

	//enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. Configure the mode of GPIO pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG )
	{
		// the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));  // Each pin take 2 bites. So, multiply by 2
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);   // clearing
		pGPIOHandle->pGPIOx->MODER |= temp;      // setting

		/*
		 * NOTE - pGPIOHandle->pGPIOx->MODER |= temp; , HERE assignment operation should not be used as it could alter other bits internally
		 * So, always use Bit-wise operation!!!!
		 */

	}else
	{
		// the interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. Configure Falling Trigger Selection Register (FTSR)
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. Configure Raising Trigger Selection Register (RTSR)
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. Configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		//2. Configure GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		//3. enable EXTI interrupt delivery using IMR (Interrupt Mask Register)
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	// 2. Configure the speed

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);   // clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;    // setting
	temp=0;

	// 3. configure the pupd settings

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);   // clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;    // setting
	temp=0;

	// 4. configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (1 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);   // clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;     // setting
	temp=0;


	// 5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN )
	{
		// configure the alt function registers
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));  // clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));      // setting
	}


}

/******************************************************************
 * @fn              - GPIO_DeInit
 * @brief           - To de-initilize GPIO port - sends back to reset state
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -
 *
 * @Note            -

 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

/*
 * Data read and write
 */
/******************************************************************
 * @fn              - GPIO_ReadfromInputPin
 * @brief           -  To read from a particular Input pin
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -  0 or 1
 *
 * @Note            -

 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/******************************************************************
 * @fn              - GPIO_ReadfromInputPort
 * @brief           -  To read from entire Input Port -
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -
 *
 * @Note            -

 */

uint16_t GPIO_ReadfromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t) pGPIOx->IDR;
	return value;
}

/******************************************************************
 * @fn              - GPIO_WriteToOutputPin
 * @brief           - To write to a particular Output pin
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       - 'Value' takes pin set or reset
 *
 * @return          -
 *
 * @Note            -

 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		// write 1 to the output data register at the bit field corresponding to the pin
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		// write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/******************************************************************
 * @fn              - GPIO_WriteToOutputPort
 * @brief           -
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -  None
 *
 * @Note            -

 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)                     // To write to entire Output port
{
	pGPIOx->ODR = Value;
}

/******************************************************************
 * @fn              - GPIO_ToggleOutputPin
 * @brief           - To toggle output pin
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -
 *
 * @Note            -

 */

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
//	pGPIOx->ODR =0;
}

/*
 * ISR Configuration and Interrupt Handling
 */

/******************************************************************
 * @fn              - GPIO_IRQConfig
 * @brief           - To configure Interrupt request number
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -
 *
 * @Note            -

 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn              - GPIO_IRQPriorityConfig
 * @brief           -
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -
 *
 * @Note            -

 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// 1. First lets find out IPR Register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 * iprx_section ) + ( 8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= ( IRQPriority << shift_amount );
}

/******************************************************************
 * @fn              - GPIO_IRQHandling
 * @brief           - To Handle the Interrupt
 *
 * @param[in]       -
 * @param[in]       -
 * @param[in]       -
 *
 * @return          -
 *
 * @Note            -

 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// clear the existing PR register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber))
	{
		// clear
		EXTI->PR |= ( 1 << PinNumber);
	}
}

