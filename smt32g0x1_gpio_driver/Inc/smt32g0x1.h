/*
 * smt32g0x1.h
 *
 *  Created on: Dec 27, 2023
 *      Author: Ryzen
 */

#ifndef SMT32G0X1_H_
#define SMT32G0X1_H_

#include <stdint.h>
#include <stddef.h>


#define __vo  volatile
#define __weak __attribute__((weak))

/*************************** PROCESSOR SPECIFIC DETAILS *********************/

#define SET_BIT(REG, MASK)		((REG |= MASK))
#define CLEAR_BIT(REG, MASK)	((REG &= ~MASK))

/*
 * Base address of the NVIC
 */
#define NVIC_BASEADDR					0xE000E100

/*
 * NVIC Register structure
 * this should be in another header file specific to the m4 coretx (e.g. core_m4.h)
 */
typedef struct{
	__vo uint32_t ISER[8];		/*!< Interrupt Set-enable registers,		Address offset: 0x00-0x1C	*/
	uint32_t RESERVED0[24];		/*!< Reserved, 0x20-0x7C*/
	__vo uint32_t ICER[8];		/*!< Interrupt Set-clear registers,			Address offset: 0x80-0x9C	*/
	uint32_t RESERVED1[24];		/*!< Reserved, 0xA0-0xFC*/
	__vo uint32_t ISPR[8];		/*!< Interrupt Set-pending registers,		Address offset: 0x100-0x11C	*/
	uint32_t RESERVED2[24];		/*!< Reserved, 0x120-0x17C*/
	__vo uint32_t ICPR[8];		/*!< Interrupt Clear-pending registers,		Address offset: 0x180-0x19C	*/
	uint32_t RESERVED3[24];		/*!< Reserved, 0x1A0-0x1FC*/
	__vo uint32_t IABR[8];		/*!< Interrupt Active Bit registers,		Address offset: 0x200-0x21C	*/
	uint32_t RESERVED4[56];		/*!< Reserved, 0x220-0x2FC*/
	__vo uint8_t IPR[240];		/*!< Interrupt Priority registers,			Address offset: 0x300-0x3EF	*/
	uint32_t RESERVED5[644];	/*!< Reserved, 0x3F0-0xDFC*/
	__vo uint32_t STIR;			/*!< Software Trigger Interrupt register,	Address offset: 0xE00		*/
}NVIC_RegDef_t;

#define NVIC			((NVIC_RegDef_t*)	NVIC_BASEADDR)
//NOTE: the below is what is shown in the video,
// for consistency, I will use the method we have been using below (as shown above)
// In the generated code, these definitions would be stored in the CMSIS Include core_cm4.h file
///*
// * ARM Cortex-M4 Processor NVIC ISERx register addresses
// */
//#define NVIC_ISER0_BASEADDR			((__vo uint32_t*) 0xE000E100)
//#define NVIC_ISER1_BASEADDR			((__vo uint32_t*) 0xE000E104)
//#define NVIC_ISER2_BASEADDR			((__vo uint32_t*) 0xE000E108)
//#define NVIC_ISER3_BASEADDR			((__vo uint32_t*) 0xE000E10C)
//
///*
// * ARM Cortex-M4 Processor NVIC ICERx register addresses
// */
//#define NVIC_ICER0_BASEADDR			((__vo uint32_t*) 0xE000E180)
//#define NVIC_ICER1_BASEADDR			((__vo uint32_t*) 0xE000E184)
//#define NVIC_ICER2_BASEADDR			((__vo uint32_t*) 0xE000E188)
//#define NVIC_ICER3_BASEADDR			((__vo uint32_t*) 0xE000E18C)


/*
 * MCU specific Macros
 */
#define NO_PR_BITS_IMPLEMENTED		4


/*
 * Base addresses of FLASH and SRAM memories
 */
#define FLASH_BASEADDR					0x080000000U //Base address for the flash memory (size: 1024KB)
#define SRAM1_BASEADDR					0x200000000U //Base address to SRAM1 (Size: 112KB)
#define SRAM2_BASEADDR					0x2001C000U //Base address for SRAM 2 (SRAM1_BASEADDR + 112KB)
#define ROM_BASEADDR					0x1FFF0000U //System Memory (size: 30KB)
#define SRAM							SRAM1_BASEADDR

/*
 * Base addresses of peripheral buses
 */
#define PERIPH_BASEADDR					0x40000000U //Base address of peripherals
#define APB1PERIPH_BASEADDR				PERIPH_BASEADDR //Base address of ABP bus is peripheral base
#define APB2PERIPH_BASEADDR				0x40010000U //Base address of ABP bus
#define AHB1PERIPH_BASEADDR				0x40020000U //Base address of AHB1 bus
#define AHB2PERIPH_BASEADDR				0x50000000U // Base address of AHB2 bus

/*
 * Base addresses of peripherals hanging on the AHB1 bus
 * Only interested in GPIO
 */
#define GPIOA_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0000UL)
#define GPIOB_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0400UL)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0800UL)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0C00UL)
#define GPIOE_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1000UL)
#define GPIOF_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1400UL)
#define GPIOG_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1800UL)

#define RCC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x3800UL)

#define SYSCFG_BASEADDR					(APB2PERIPH_BASEADDR + 0x3800UL)

#define EXTI_BASEADDR					(APB2PERIPH_BASEADDR + 0x3C00UL)

/*****************************peripheral register definition structures*******************************/
/*
 * Note: Registers of a peripheral are specific to an MCU
 * e.g.: Numbers of registers of SPI peripheral of STM32F4x family MCUs may be different (more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your device RM
 */

/*
 * Peripheral register definition structure for EXTI
 */
typedef struct {
	__vo uint32_t IMR;	/*!< EXTI interrupt mask register,				Address offset: 0x00	*/
	__vo uint32_t EMR;	/*!< EXTI event mask register,					Address offset: 0x4	*/
	__vo uint32_t RTSR;	/*!< EXTI rising trigger selection register,	Address offset: 0x08	*/
	__vo uint32_t FTSR;	/*!< EXTI falling trigger selection register,	Address offset: 0x0C	*/
	__vo uint32_t SWIER;/*!< EXTI software interrupt event register,	Address offset: 0x10	*/
	__vo uint32_t PR;	/*!< EXTI pending register,						Address offset: 0x14	*/
} EXTI_RegDef_t;

/*
 * Peripheral register definition structure for GPIO
 */
typedef struct {
	__vo uint32_t MODER;	/*!< GPIO Port mode register,				Address offset: 0x00	*/
	__vo uint32_t OTYPER;	/*!< GPIO Port output type register,		Address offset: 0x04	*/
	__vo uint32_t OSPEEDR;	/*!< GPIO Port output speed register,		Address offset: 0x08	*/
	__vo uint32_t PUPDR;	/*!< GPIO Port pull-up/pull-down register,	Address offset: 0x0C	*/
	__vo uint32_t IDR;		/*!< GPIO Port input data register,			Address offset: 0x10	*/
	__vo uint32_t ODR;		/*!< GPIO Port output data register,		Address offset: 0x14	*/
	__vo uint32_t BSRR;		/*!< GPIO Port bit set/reset register,		Address offset: 0x18	*/
	__vo uint32_t LCKR;		/*!< GPIO Port configuration lock register,	Address offset: 0x1C	*/
	__vo uint32_t AFRL;		/*!< GPIO alternate function low registers, Address offset: 0x20	*/
	__vo uint32_t AFRH;		/*!< GPIO alternate function high register, Address offset: 0x24	*/
	__vo uint32_t BRR;		/*!< GPIO port bit reset register, 			Address offset: 0x28	*/
} GPIO_RegDef_t;


/*
 * Peripheral register definition structure for RCC
 */
typedef struct {
	__vo uint32_t CR;			/*!< RCC Clock control register,									Address offset: 0x00	*/
	__vo uint32_t ICSCR;		/*!< RCC Internal clock source calibration register,				Address offset: 0x04	*/
	__vo uint32_t CFGR;			/*!< RCC Clock configuration register,								Address offset: 0x08	*/
	__vo uint32_t PLLCFGR;		/*!< RCC PLL configuration register,								Address offset: 0x0C	*/
	uint32_t RESERVED0;			/*!< RCC Reserved, 0x1C	,											Address offset: 0x10	*/
	__vo uint32_t CRRCR;		/*!< RCC clock recovery RC register,								Address offset: 0x14	*/
	__vo uint32_t CIER;			/*!< RCC Clock interrupt enable register,							Address offset: 0x18	*/
	__vo uint32_t CIFR;			/*!< RCC Clock interrupt flag register,								Address offset: 0x1C	*/
	__vo uint32_t CICR;			/*!< RCC Clock interrupt clear register,							Address offset: 0x20	*/
	__vo uint32_t IOPRSTR;		/*!< RCC I/O port reset register,									Address offset: 0x24	*/
	__vo uint32_t AHBRSTR;		/*!< RCC AHB peripheral reset register, 							Address offset: 0x28	*/
	__vo uint32_t APBRSTR1;		/*!< RCC APB peripheral reset register 1,							Address offset: 0x2C	*/
	__vo uint32_t APBRSTR2;		/*!< APB peripheral reset register 2,								Address offset: 0x30	*/
	__vo uint32_t IOPENR;		/*!< RCC I/O port clock enable register,							Address offset: 0x34	*/
	__vo uint32_t AHBENR;		/*!< RCC AHB peripheral clock enable register,						Address offset: 0x38	*/
	__vo uint32_t APBENR1;		/*!< RCC APB peripheral clock enable register 1,					Address offset: 0x3C	*/
	__vo uint32_t APBENR2;		/*!< RCC APB peripheral clock enable register 2,					Address offset: 0x40	*/
	__vo uint32_t IOPSMENR;		/*!< RCC I/O port in Sleep mode clock enable register,				Address offset: 0x44	*/
	__vo uint32_t AHBSMENR;		/*!< RCC AHB peripheral clock enable in Sleep/Stop mode register,	Address offset: 0x48	*/
	__vo uint32_t APBSMENR1;	/*!< RCC APB peripheral clock enable in Sleep/Stop mode register 1,	Address offset: 0x4C	*/
	__vo uint32_t APBSMENR2;	/*!< RCC APB peripheral clock enable in Sleep/Stop mode register 2,	Address offset: 0x50	*/
	__vo uint32_t CCIPR;		/*!< RCC Peripherals independent clock configuration register,		Address offset: 0x54	*/
	__vo uint32_t CCIPR2;		/*!< RCC Peripherals independent clock configuration register 2,	Address offset: 0x58	*/
	__vo uint32_t BDCR;			/*!< RCC RTC domain control register								Address offset: 0x5C	*/
	__vo uint32_t CSR;			/*!< RCC Control/status register,									Address offset: 0x60	*/
} RCC_RegDef_t;


/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct {
	__vo uint32_t MEMRMP;		/*!< SYSCFG memory remap register,						Address offset: 0x00	*/
	__vo uint32_t PMC;			/*!< SYSCFG peripheral mode configuration register,		Address offset: 0x04	*/
	__vo uint32_t EXTICR[4];	/*!< SYSCFG external interrupt configuration register,	Address offset: 0x08-0x14	*/
	uint32_t RESERVED[2];		/*!< Reserved, 0x18-0x1C	*/
	__vo uint32_t CMPCR;		/*!< SYSCFG compensation cell control register,			Address offset: 0x20	*/
} SYSCFG_RegDef_t;

/*
 * Peripheral definitions (peripheral base addresses typecasted as xxx_RegDef_t)
 */

#define EXTI				((EXTI_RegDef_t*) EXTI_BASEADDR)

#define GPIOA				((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF				((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG				((GPIO_RegDef_t*) GPIOG_BASEADDR)

/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))

/*
 * Clock enable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= ( 1 << 14 ))

/*
 * Clock disable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))

/*
 * Clock disable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0)); }while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1)); }while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2)); }while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3)); }while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4)); }while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5)); }while(0)
#define GPIOG_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6)); }while(0)

#define GPIO_BASEADDR_TO_CODE(addr)		( (addr==GPIOA) ? 0 :\
										(addr==GPIOB) ? 1 :\
										(addr==GPIOC) ? 2 :\
										(addr==GPIOD) ? 3 :\
										(addr==GPIOE) ? 4 :\
										(addr==GPIOF) ? 5 : 0)

/*
 * Macros for IRQ numbers
 * maybe move these to a different section
 */

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_I2C3_EV		72
#define IRQ_NO_I2C3_ER		73

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51

#define IRQ_NO_USART1		37
#define IRQ_NO_USART2		38
#define IRQ_NO_USART3		39
#define IRQ_NO_UART4		52
#define IRQ_NO_UART5		53
#define IRQ_NO_USART6		71

/*
 * macros for all possible NVIC priority levels
 * maybe move these to a different section
 */
#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15


/*
 * generic macros
 */
#define ENABLE		1
#define DISABLE		0
#define SET			ENABLE
#define RESET		DISABLE


#include "stm32f407xx_gpio_driver.h"



#endif /* SMT32G0X1_H_ */
