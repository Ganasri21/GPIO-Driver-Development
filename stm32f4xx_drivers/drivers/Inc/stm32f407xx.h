/*
 * stm32f407xx.h
 *
 *  Created on: Nov 14, 2024
 *      Author: ganasri.s
 */

// MCU specific Header file - used by many drivers

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h> // for uint32_t

#define __vo volatile
/*
 * Base Address of FLASH and SRAM memory
 */
#define FLASH_BASEADDR  0x08000000U       // Main Memory
#define SRAM1_BASEADDR  0x20000000U       // 112KB
#define SRAM2_BASEADDR  0x20001C00U       // 112*1024 = 114688 -> 1C00(HEX)
#define ROM_BASEADDR    0x1FFF0000U       // System Memory
#define SRAM            SRAM1_BASEADDR

/*
 *  AHBx & APBx Bus Peripheral Base Addresses
 */

#define PERIPH_BASEADDR      0x40000000U
#define APB1PERIPH_BASEADDR  PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR  0x40010000U      // 0x40000000 + 0x00010000 (offset)
#define AHB1PERIPH_BASEADDR  0x40020000U      // 0x40000000 + 0x00020000 (offset)
#define AHB2PERIPH_BASEADDR  0x50000000U      // 0x40000000 + 0x10000000 (offset)

/*
 * Base Addresses of peripherals which are hanging on AHB1 Bus
 */

#define GPIOA_BASEADDR     (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR     (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR     (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR     (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR     (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR     (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR     (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR     (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR     (AHB1PERIPH_BASEADDR + 0x2000)

#define RCC_BASEADDR       (AHB1PERIPH_BASEADDR + 0x3800)

/*
 * Base Addresses of peripherals which are hanging on APB1 Bus
 */
#define I2C1_BASEADDR       (APB1PERIPH_BASEADDR + 0x5400)  // 0x4000 5400
#define I2C2_BASEADDR       (APB1PERIPH_BASEADDR + 0x5800)  // 0x4000 5800
#define I2C3_BASEADDR       (APB1PERIPH_BASEADDR + 0x5C00)  // 0x4000 5C00
#define SPI2_BASEADDR       (APB1PERIPH_BASEADDR + 0x3800)  // 0x4000 3800
#define SPI3_BASEADDR       (APB1PERIPH_BASEADDR + 0x3C00)  // 0x4000 3C00
#define USART2_BASEADDR     (APB1PERIPH_BASEADDR + 0x4400)  // 0x4000 4400
#define USART3_BASEADDR     (APB1PERIPH_BASEADDR + 0x4800)  // 0x4000 4800
#define UART4_BASEADDR      (APB1PERIPH_BASEADDR + 0x4C00)  // 0x4000 4C00
#define UART5_BASEADDR      (APB1PERIPH_BASEADDR + 0x5000)  // 0x4000 5000

/*
 * Base Addresses of Peripherals which are hanging on APB2 Bus
 */
#define EXTI_BASEADDR       (APB2PERIPH_BASEADDR + 0x3C00)  // 0x4001 3C00
#define SPI1_BASEADDR       (APB2PERIPH_BASEADDR + 0x3000)  // 0x4001 3000
#define SYSCFG_BASEADDR     (APB2PERIPH_BASEADDR + 0x3800)  // 0x4001 3800
#define USART1_BASEADDR     (APB2PERIPH_BASEADDR + 0x1000)  // 0x4001 1000
#define USART6_BASEADDR     (APB2PERIPH_BASEADDR + 0x1400)  // 0x4001 1400

/*******  Peripheral Register Definition  *******/
typedef struct
{
	__vo uint32_t MODER;         // GPIO port mode register
	__vo uint32_t OTYPER;        // GPIO port output type register
	__vo uint32_t OSPEEDR;       // GPIO port output speed register
	__vo uint32_t PUPDR;         // GPIO port pull-up/pull-down register
	__vo uint32_t IDR;           // GPIO port input data register
	__vo uint32_t ODR;           // GPIO port output data register
	__vo uint32_t BSRR;          // GPIO port bit set/reset register
	__vo uint32_t LCKR;          // GPIO port configuration lock register
	__vo uint32_t AFR[2];        // ARF[0] - GPIO alternate function low register, ARF[1] - GPIO alternate function high register
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;            // RCC clock control register
	__vo uint32_t PLLCFGR;       // RCC PLL configuration register
	__vo uint32_t CFGR;          // RCC clock configuration register
	__vo uint32_t CIR;           // RCC clock interrupt register
	__vo uint32_t AHB1RSTR;      // RCC AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR;      // RCC AHB2 peripheral reset register
	__vo uint32_t AHB3RSTR;      // RCC AHB3 peripheral reset register
	uint32_t      RESERVED0;     // Reserved, 0x1C
	__vo uint32_t APB1RSTR;      // RCC APB1 peripheral reset register
	__vo uint32_t APB2RSTR;      // RCC APB2 peripheral reset register
	uint32_t      RESERVED1[2];  // Reserved, 0x28 - 0x2c
	__vo uint32_t AHB1ENR;       // RCC AHB1 peripheral clock enable register
	__vo uint32_t AHB2ENR;       // RCC AHB2 peripheral clock enable register
	__vo uint32_t AHB3ENR;       // RCC AHB3 peripheral clock enable register
	uint32_t      RESERVED2;     // Reserved, 0x3C
	__vo uint32_t APB1ENR;       // RCC APB1 peripheral clock enable register
	__vo uint32_t APB2ENR;       // RCC APB2 peripheral clock enable register
	uint32_t      RESERVED3[2];  // Reserved, 0x48 - 0x4C
	__vo uint32_t AHB1LPENR;     // RCC AHB1 peripheral clock enable in low power mode register
	__vo uint32_t AHB2LPENR;     // RCC AHB2 peripheral clock enable in low power mode register
	__vo uint32_t AHB3LPENR;     // RCC AHB3 peripheral clock enable in low power mode register
	uint32_t      RESERVED4;     // Reserved, 0x5C
	__vo uint32_t APB1LPENR;     // RCC APB1 peripheral clock enable in low power mode register
	__vo uint32_t APB2LPENR;     // RCC APB2 peripheral clock enabled in low power mode
	uint32_t      RESERVED5[2];  // Reserved, 0x68 - 0x6C
	__vo uint32_t BDCR;          // RCC Backup domain control register
	__vo uint32_t CSR;           // RCC clock control & status register
	uint32_t      RESERVED6[2];  // Reserved, 0x78 - 0x7C
	__vo uint32_t SSCGR;         // RCC spread spectrum clock generation register
	__vo uint32_t PLLI2SCFGR;    // RCC PLLI2S configuration register
	__vo uint32_t PLLSAICFGR;    // RCC PLL configuration register
	__vo uint32_t DCKCFGR;       // RCC Dedicated Clock Configuration Register
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;
}RCC_RegDef_t;

/*
 * Peripheral definitions (Peripheral Base Addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA    ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB    ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC    ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD    ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE    ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF    ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG    ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH    ((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI    ((GPIO_RegDef_t*) GPIOI_BASEADDR)
#define RCC      ((RCC_RegDef_t*)  RCC_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()  ( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()  ( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()  ( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()  ( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()  ( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()  ( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()  ( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()  ( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()  ( RCC->AHB1ENR |= (1 << 8) )

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()  ( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()  ( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()  ( RCC->APB1ENR |= (1 << 23) )

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()  ( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()  ( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()  ( RCC->APB1ENR |= (1 << 15) )

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN()  ( RCC->APB2ENR |= (1 << 4) )
#define USART2_PCLK_EN()  ( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN()  ( RCC->APB1ENR |= (1 << 18) )
#define UART4_PCLK_EN()   ( RCC->APB1ENR |= (1 << 19) )
#define USART5_PCLK_EN()  ( RCC->ABP1ENR |= (1 << 20) )
#define USART6_PCLK_EN()  ( RCC->APB2ENR |= (1 << 5) )


/*
 * Clock Enable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()  ( RCC->APB2ENR |= (1 << 14) )

/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()  ( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()  ( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()  ( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()  ( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()  ( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()  ( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()  ( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()  ( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI()  ( RCC->AHB1ENR &= ~(1 << 8) )

/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()  ( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()  ( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()  ( RCC->APB1ENR &= ~(1 << 23) )


/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()  ( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()  ( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()  ( RCC->APB1ENR &= ~(1 << 15) )

/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI()  ( RCC->APB2ENR &= ~(1 << 4) )
#define USART2_PCLK_DI()  ( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DI()  ( RCC->APB1ENR &= ~(1 << 18) )
#define UART4_PCLK_DI()   ( RCC->APB1ENR &= ~(1 << 19) )
#define USART5_PCLK_DI()  ( RCC->AP12ENR &= ~(1 << 20) )
#define USART6_PCLK_DI()  ( RCC->APB2ENR &= ~(1 << 5) )

/*
 * Clock Disable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI()  ( RCC->APB2ENR &= ~(1 << 14) )

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()      do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()      do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()      do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()      do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()      do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOG_REG_RESET()      do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOH_REG_RESET()      do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOI_REG_RESET()      do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

// some generic Macros
#define ENABLE           1
#define DISABLE          0
#define SET              ENABLE
#define RESET            DISABLE
#define GPIO_PIN_SET     SET
#define GPIO_PIN_RESET   RESET


#endif /* INC_STM32F407XX_H_ */
