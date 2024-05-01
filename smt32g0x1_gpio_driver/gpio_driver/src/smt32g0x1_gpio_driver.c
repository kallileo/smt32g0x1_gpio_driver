/*
 * smt32g0x1_gpio_driver.c
 *
 *  Created on: Apr 28, 2024
 *      Author: Ryzen
 */


#include "stm32g0x1.h"
#include "smt32g0x1_gpio_driver.h"


/*********************************************************************
 * @fn      		  - GPIO_PortClockEnblOrDsbl
 *
 * @brief             - This function enables or disables port clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */
/*
 * Port Clock control
 */

void GPIO_PortClockEnblOrDsbl(GPIO_RegDef_t *pGPIOx, uint8_t EnblOrDsbl)
{
	if (EnblOrDsbl == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			(RCC_RegDef_t->IOPENR |= (1 << 0));
		}
		else if (pGPIOx == GPIOB)
		{
			(RCC_RegDef_t->IOPENR |= (1 << 1));
		}
		else if (pGPIOx == GPIOC)
		{
			(RCC_RegDef_t->IOPENR |= (1 << 2));
		}
		else if (pGPIOx == GPIOD)
		{
			(RCC_RegDef_t->IOPENR |= (1 << 3));
		}
		else if (pGPIOx == GPIOE)
		{
			(RCC_RegDef_t->IOPENR |= (1 << 4));
		}
		else if (pGPIOx == GPIOF)
		{
			(RCC_RegDef_t->IOPENR |= (1 << 5));
		}
	}
	else
		if (pGPIOx == GPIOA)
		{
			(RCC_RegDef_t->IOPENR &= ~(1 << 0));
		}
		else if (pGPIOx == GPIOB)
		{
			(RCC_RegDef_t->IOPENR &= ~(1 << 1));
		}
		else if (pGPIOx == GPIOC)
		{
			(RCC_RegDef_t->IOPENR &= ~(1 << 2));
		}
		else if (pGPIOx == GPIOD)
		{
			(RCC_RegDef_t->IOPENR &= ~(1 << 3));
		}
		else if (pGPIOx == GPIOE)
		{
			(RCC_RegDef_t->IOPENR &= ~(1 << 4));
		}
		else if (pGPIOx == GPIOF)
		{
			(RCC_RegDef_t->IOPENR &= ~(1 << 5));
		}
	}
}



