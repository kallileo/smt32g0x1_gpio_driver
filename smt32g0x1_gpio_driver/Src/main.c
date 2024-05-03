/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "stm32g0x1.h"

#define RCC_IOPENR		(*((volatile uint32_t *) 0x40021034))
#define GPIOA_MODER		(*((volatile uint32_t *) 0x50000000))
#define GPIOA_ODR		(*((volatile uint32_t *) 0x50000014))

Gpio_Pin_Num Number;

//void GPIO_PortClockEnblOrDsbl(GPIOA, 1);

void delay()
{
	for (int i = 0; i<500000; i++);
}


int main(void)
{
	/* Initial setup */
	//Enable clock for GPOA
	RCC_IOPENR |= (1<<0);

	//Set PA5 as output
	GPIOA_MODER &= ~(3<<(2*5));
	GPIOA_MODER |= (1<<(2*5));

    /* Loop forever */
	for(;;)
	{
		GPIOA_ODR |= (1<<5);
		delay();
		GPIOA_ODR &= ~(1<<5);
		delay();
	}
}
