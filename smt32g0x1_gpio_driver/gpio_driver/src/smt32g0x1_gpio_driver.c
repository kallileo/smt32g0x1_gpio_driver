
/*
 * smt32g0x1_gpio_driver.c
 *
 *  Created on: Apr 28, 2024
 *      Author: Ryzen
 */


#include "stm32g0x1.h"
#include "smt32g0x1_gpio_driver.h"


void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnblOrDsbl)
{
	if (EnblOrDsbl == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_ENBL();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_ENBL();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_ENBL();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_ENBL();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_ENBL();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_ENBL();
		}
	}
	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_ENBL();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_ENBL();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_ENBL();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_ENBL();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_ENBL();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_ENBL();
		}
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	 //enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1 . configure the mode of gpio pin

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= Gpio_Mode_ALTFN)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)); //Clear bit
		pGPIOHandle->pGPIOx->MODER |= temp; //Set bits
	}
	else
	{

	}


	//2. configure pin speed
		temp = 0;
		temp |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)); //Clear bit
		pGPIOHandle->pGPIOx->OSPEEDR |= temp; //Set bits





}



