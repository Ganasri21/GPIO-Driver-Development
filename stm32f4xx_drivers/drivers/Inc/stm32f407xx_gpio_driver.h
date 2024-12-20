/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Nov 15, 2024
 *      Author: ganasri.s
 */

#ifndef STM32F407XX_GPIO_DRIVER_H_
#define STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
	// uint8 - Used to store an 8 bit value & Pin number ranges from 0 to 15
	uint8_t GPIO_PinNumber;         /*!< possible values form @GPIO_PIN_NUMBERS >*/
	uint8_t GPIO_PinMode;           /*!< possible values form @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;          /*!< possible values form @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;    // GPIO Output Type
	uint8_t GPIO_PinOPType;         // GPIO Pullup-Pulldown
	uint8_t GPIO_PinAltFunMode;     // GPIO Alt.fun Mode
}GPIO_PinConfig_t;

/*
 * This is the Handle structure for GPIO pin
 * A handle structure is a data structure that holds
 * the runtime information of a peripheral, such as current state,
 * error status, or data buffers. It serves as a "handle" or reference
 * to access a peripheral and its settings throughout its operation.
 */

typedef struct
{
	// pointer to hold the base address of the GPIO peripherals
	GPIO_RegDef_t *pGPIOx;               // This holds the base address of GPIO port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;     // This holds GPIO pin configuration structure

}GPIO_Handle_t;              // GPIO_Handle struct - used to initialize GPIO

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0          0
#define GPIO_PIN_NO_1          1
#define GPIO_PIN_NO_2          2
#define GPIO_PIN_NO_3          3
#define GPIO_PIN_NO_4          4
#define GPIO_PIN_NO_5          5
#define GPIO_PIN_NO_6          6
#define GPIO_PIN_NO_7          7
#define GPIO_PIN_NO_8          8
#define GPIO_PIN_NO_9          9
#define GPIO_PIN_NO_10         10
#define GPIO_PIN_NO_11         11
#define GPIO_PIN_NO_12         12
#define GPIO_PIN_NO_13         13
#define GPIO_PIN_NO_14         14
#define GPIO_PIN_NO_15         15


/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN        0
#define GPIO_MODE_OUT       1
#define GPIO_MODE_ALTFN     2
#define GPIO_MODE_ANALOG    3
#define GPIO_MODE_IT_FT     4      // Input - Falling Edge
#define GPIO_MODE_IT_RT     5      // Input - Rising Edge
#define GPIO_MODE_IT_RFT    6      // Input - Rising Edge, Falling Edge Trigger

/*
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP     0      // Output push-pull (reset state)
#define GPIO_OP_TYPE_OD     1      // Output open-drain

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW       0
#define GPIO_SPEED_MEDIUM    1
#define GPIO_SPEED_FAST      2
#define GPIO_SPEED_HIGH      3

/*
 * GPIO pin pull up & pull down configuration macros
 */
#define GPIO_NO_PUPD          0
#define GPIO_PIN_PU           1
#define GPIO_PIN_PD           2

/********  APIs supported by the driver  ******/


/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);      // To enable or disable peripheral clock for a given Base address

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);        // To initialize GPIO port - take pointer to the Handle structure
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);          // To de-initilize GPIO port - sends back to reset state

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);                // To read from a particular Input pin
uint16_t GPIO_ReadfromInputPort(GPIO_RegDef_t *pGPIOx);                                 // To read from entire Input Port - used uint16_t to read from 16 pins
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);    // To write to a particular Output pin - Parameter 'Value' takes pin set or reset
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);                     // To write to entire Output port
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);                    // To toggle output pin

/*
 * ISR Configuration and Interrupt Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);            // To configure Interrupt request number
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);                                              // To Handle the Interrupt



#endif /* STM32F407XX_GPIO_DRIVER_H_ */
