/*
 * stm32f401xx.h
 *
 *  Created on: Jan 20, 2024
 *      Author: benja
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include <stddef.h>
#include <stdint.h>

#define __vo 					volatile
#define __weak					__attribute__ ((weak))

/* base addresses of Flash and SRAM memories */

/*********** PROCESSOR SPECIFIC DETAILS ************/
//Cortex MX NVIC ISERx register addresses
#define NVIC_ISER0					( (__vo uint32_t*) 0xE000E100 )
#define NVIC_ISER1					( (__vo uint32_t*) 0xE000E104 )
#define NVIC_ISER2					( (__vo uint32_t*) 0xE000E108 )
#define NVIC_ISER3					( (__vo uint32_t*) 0xE000E10C )

//Cortex MX NVIC ICERx register addresses
#define NVIC_ICER0 					( (__vo uint32_t*) 0XE000E180 )
#define NVIC_ICER1					( (__vo uint32_t*) 0XE000E184 )
#define NVIC_ICER2  				( (__vo uint32_t*) 0XE000E188 )
#define NVIC_ICER3					( (__vo uint32_t*) 0XE000E18C )


//Cortex MX processor priority base address
#define NVIC_PR_BASE_ADDR			( (__vo uint32_t*) 0xE000E400 )

//Cortex MX NVIC number of priority bits implemented in priority register
#define NO_PR_BITS_IMPLEMENTED		4

#define FLASH_BASEADDR				0x08000000U
#define SRAM1_BASEADDR				0x20000000U
#define SRAM2_BASEADDR				(SRAM1_BASEADDR + 0x1C000U)
#define ROM_BASEADDR				0x1FFF0000U
#define SRAM_BASEADDR 				SRAM1_BASEADDR

#define PERIPH_BASEADDR				0x40000000U
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR			0x40010000U
#define AHB1PERIPH_BASEADDR 		0x40020000U
#define AHB2PERIPH_BASEADDR			0x50000000U

/* base addresses of peripherals hanging on AHB1 bus */

#define GPIOA_BASEADDR 			AHB1PERIPH_BASEADDR
#define GPIOB_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1000U)
#define GPIOH_BASEADDR 			(AHB1PERIPH_BASEADDR + 0x1C00U)

#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800U)

/* base addresses of peripherals hanging on APB1 bus */
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)

/* base addresses of peripherals hanging on APB2 bus */
#define EXTI_BASEADDR 			(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR 			(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR 		(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)

/****** PERIPHERAL REGISTER DEFN STRUCTURES ******/

typedef struct{
	__vo uint32_t MODER;			/*!< GPIO port mode register, */
	__vo uint32_t OTYPER;		/*!< GPIO port output type register, */
	__vo uint32_t OSPEEDR;		/*!< GPIO port output speed register, */
	__vo uint32_t PUPDR;			/*!< GPIO port pull-up/pull-down register, */
	__vo uint32_t IDR;			/*!< GPIO port input data register, */
	__vo uint32_t ODR;			/*!< GPIO port output data register, */
	__vo uint32_t BSRR;			/*!< GPIO port bit set/reset register, */
	__vo uint32_t LCKR;			/*!< GPIO port configuration lock register, */
	__vo uint32_t AFR[2];		/*!< AFR[0]: GPIO port alternate function low register,
	 	 	 	 	 	 	 	 AFR[1]: GPIO port alternate function high register */
}GPIO_RegDef_t;

typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

typedef struct{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t RESERVED0[2];
	__vo uint32_t CMPCR;
	__vo uint32_t RESERVED1[2];
	__vo uint32_t CFGR;
}SYSCFG_RegDef_t;

typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;

/* peripheral definitions (peripheral base addresses typecasted to xxx_RegDef_t) */

#define GPIOA					( (GPIO_RegDef_t*) GPIOA_BASEADDR )
#define GPIOB					( (GPIO_RegDef_t*) GPIOB_BASEADDR )
#define GPIOC					( (GPIO_RegDef_t*) GPIOC_BASEADDR )
#define GPIOD					( (GPIO_RegDef_t*) GPIOD_BASEADDR )
#define GPIOE					( (GPIO_RegDef_t*) GPIOE_BASEADDR )
#define GPIOH					( (GPIO_RegDef_t*) GPIOH_BASEADDR )

#define RCC						( (RCC_RegDef_t*) RCC_BASEADDR )

#define EXTI					( (EXTI_RegDef_t*) EXTI_BASEADDR )

#define SYSCFG					( (SYSCFG_RegDef_t*) SYSCFG_BASEADDR )

#define SPI1 					( (SPI_RegDef_t*) SPI1_BASEADDR )
#define SPI2 					( (SPI_RegDef_t*) SPI2_BASEADDR )
#define SPI3 					( (SPI_RegDef_t*) SPI3_BASEADDR )
#define SPI4 					( (SPI_RegDef_t*) SPI4_BASEADDR )

/* clock enable macros for GPIOx peripherals */
#define GPIOA_PCLK_EN()			RCC->AHB1ENR |= (1 << 0)
#define GPIOB_PCLK_EN()			RCC->AHB1ENR |= (1 << 1)
#define GPIOC_PCLK_EN()			RCC->AHB1ENR |= (1 << 2)
#define GPIOD_PCLK_EN()			RCC->AHB1ENR |= (1 << 3)
#define GPIOE_PCLK_EN()			RCC->AHB1ENR |= (1 << 4)
#define GPIOH_PCLK_EN()			RCC->AHB1ENR |= (1 << 7)

/* clock enable macros for I2Cx peripherals */
#define I2C1_PCLK_EN()			(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()			(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()			(RCC->APB1ENR |= (1 << 23))

/* clock enable macros for SPIx peripherals */
#define SPI1_PCLK_EN()			(RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1 << 15) )
#define SPI4_PCLK_EN()			(RCC->APB2ENR |= (1 << 13) )

/* clock enable macros for USARTx peripherals */
#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define USART6_PCLK_EN()		(RCC->APB2ENR |= (1 << 5))

/* clock enable macros for SYSCFG peripherals */
#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1 << 14))

/* clock disable macros for GPIOx peripherals */
#define GPIOA_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 7))

/* clock disable macros for I2Cx peripherals */
#define I2C1_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 23))

/* clock disable macros for SPIx peripherals */
#define SPI1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 15) )
#define SPI4_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 13) )

/* clock disable macros for USARTx peripherals */
#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 4))
#define USART6_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 5))


/* clock disable macros for SYSCFG peripherals */
#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 14))

/* macros to reset GPIOx peripherals */
#define GPIOA_REG_RESET()				do{ ( RCC->AHB1RSTR |= (1 << 0) ); ( RCC->AHB1RSTR &= ~(1 << 0) ); }while(0) //sets and resets reset register
#define GPIOB_REG_RESET()				do{ ( RCC->AHB1RSTR |= (1 << 1) ); ( RCC->AHB1RSTR &= ~(1 << 1) ); }while(0) //sets and resets reset register
#define GPIOC_REG_RESET()				do{ ( RCC->AHB1RSTR |= (1 << 2) ); ( RCC->AHB1RSTR &= ~(1 << 2) ); }while(0) //sets and resets reset register
#define GPIOD_REG_RESET()				do{ ( RCC->AHB1RSTR |= (1 << 3) ); ( RCC->AHB1RSTR &= ~(1 << 3) ); }while(0) //sets and resets reset register
#define GPIOE_REG_RESET()				do{ ( RCC->AHB1RSTR |= (1 << 4) ); ( RCC->AHB1RSTR &= ~(1 << 4) ); }while(0) //sets and resets reset register
#define GPIOH_REG_RESET()				do{ ( RCC->AHB1RSTR |= (1 << 7) ); ( RCC->AHB1RSTR &= ~(1 << 7) ); }while(0) //sets and resets reset register

#define SPI1_REG_RESET() 				do{ ( RCC->APB2RSTR |= (1 << 12) ); ( RCC->APB2RSTR&= ~(1 << 12) ); }while(0) //sets and resets reset register
#define SPI2_REG_RESET()				do{ ( RCC->APB1RSTR |= (1 << 14) ); ( RCC->APB1RSTR &= ~(1 << 14) ); }while(0) //sets and resets reset register
#define SPI3_REG_RESET()				do{ ( RCC->APB1RSTR |= (1 << 15) ); ( RCC->APB1RSTR &= ~(1 << 15) ); }while(0) //sets and resets reset register
#define SPI4_REG_RESET()				do{ ( RCC->APB2RSTR |= (1 << 13) ); ( RCC->APB2RSTR &= ~(1 << 13) ); }while(0) //sets and resets reset register

#define GPIO_BASEADDR_TO_CODE(x) 		(x == GPIOA) ? 0:\
										(x == GPIOB) ? 1:\
									    (x == GPIOC) ? 2:\
									    (x == GPIOD) ? 3:\
									    (x == GPIOH) ? 7:0

/****** IRQ number macros ******/
/* GPIO IRQ numbers */
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI10_15		40

/* SPI IRQ numbers */
#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51

/* IRQ priority level macros */
#define NVIC_IRQ_PRI0			0
#define NVIC_IRQ_PRI1			1
#define NVIC_IRQ_PRI2			2
#define NVIC_IRQ_PRI3			3
#define NVIC_IRQ_PRI4			4
#define NVIC_IRQ_PRI5			5
#define NVIC_IRQ_PRI6			6
#define NVIC_IRQ_PRI7			7
#define NVIC_IRQ_PRI8			8
#define NVIC_IRQ_PRI9			9
#define NVIC_IRQ_PRI10			10
#define NVIC_IRQ_PRI11			11
#define NVIC_IRQ_PRI12			12
#define NVIC_IRQ_PRI13			13
#define NVIC_IRQ_PRI14			14
#define NVIC_IRQ_PRI15			15
#define NVIC_IRQ_PRI16			16

/* generic macros */
#define ENABLE 					1
#define DISABLE 				0
#define SET						ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET
#define FLAG_SET				SET
#define FLAG_RESET				RESET

/************* bit position macros of SPIx peripheral *************/
/* CR1 */
#define SPI_CR1_CPHA 			0
#define SPI_CR1_CPOL 			1
#define SPI_CR1_MSTR 			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE 			6
#define SPI_CR1_LSB_FIRST 		7
#define SPI_CR1_SSI 			8
#define SPI_CR1_SSM 			9
#define SPI_CR1_RX_ONLY 	 	10
#define SPI_CR1_DFF 			11
#define SPI_CR1_CRC_NEXT 		12
#define SPI_CR1_CRC_EN			13
#define SPI_CR1_BIDI_OE 		14
#define SPI_CR1_BIDI_MODE 		15

/* CR2 */
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

/* SR */
#define SPI_SR_RXNE				0
#define SPI_SR_TXE 				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRC_ERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8

typedef struct{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	uint32_t RESERVED0[2];
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	uint32_t RESERVED2[2];
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	uint32_t RESERVED4[2];
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLL12SCFGR;
	uint32_t RESERVED7;
	__vo uint32_t DCKCFGR;
}RCC_RegDef_t;

/* peripheral specific headers */
#include "stm32f401xx_gpio_driver.h"
#include "stm32f401xx_spi_driver.h"



#endif /* INC_STM32F401XX_H_ */
