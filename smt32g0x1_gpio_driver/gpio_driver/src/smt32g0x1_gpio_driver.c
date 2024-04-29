/*
 * smt32g0x1_gpio_driver.c
 *
 *  Created on: Apr 28, 2024
 *      Author: Ryzen
 */


#include "stm32f407xx_gpio_driver.h"


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
	(RCC_RegDef_t->IOPENR |= (1 << 0))

}
