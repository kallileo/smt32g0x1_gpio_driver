
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
			GPIOA_PCLK_DSBL();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOA_PCLK_DSBL();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOA_PCLK_DSBL();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOA_PCLK_DSBL();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOA_PCLK_DSBL();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOA_PCLK_DSBL();
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

	//3. configure pull up/down resistor
		temp = 0;
		temp |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->PUPDR = temp;

	//4. configure output type
		temp = 0;
		temp |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OTYPER = temp;

	//5. configure alternate mode

}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	//read input
	uint8_t input_status = 0;
	input_status = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x1);
	return input_status;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	//read port
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	//write input
	if (value == RESET)
	{
		pGPIOx->ODR &= (value << PinNumber);
	}
	else
	{
		pGPIOx->ODR |= (value << PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}
