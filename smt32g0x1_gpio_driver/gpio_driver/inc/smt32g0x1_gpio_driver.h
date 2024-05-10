/*
 * smt32g0x1_gpio_driver.h
 *
 *  Created on: Apr 26, 2024
 *      Author: Ryzen
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32g0x1.h"


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers by using Enumerator instead of Macros
 */
enum Gpio_Pin_Num
{
	Gpio_PinNum_0, Gpio_PinNum_1, Gpio_PinNum_2, Gpio_PinNum_3, Gpio_PinNum_4,
	Gpio_PinNum_5, Gpio_PinNum_6, Gpio_PinNum_7, Gpio_PinNum_8, Gpio_PinNum_9,
	Gpio_PinNum_10, Gpio_PinNum_11, Gpio_PinNum_12, Gpio_PinNum_13, Gpio_PinNum_14,
	Gpio_PinNum_15, Gpio_PinNum_16
};
typedef enum Gpio_Pin_Num Gpio_Pin_Num;

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes by using Enumerator instead of Macros
 */
enum Gpio_Mode
{
	Gpio_Mode_IN, Gpio_Mode_OUT, Gpio_Mode_ALTFN, Gpio_Mode_ANALOG,
	Gpio_Mode_IT_FT, Gpio_Mode_IT_RT, Gpio_Mode_IT_RFT
};
typedef enum Gpio_Mode Gpio_Mode;


/*
 * GPIO pin possible output types
 */
enum Gpio_OutType
{
	Gpio_OutType_PUSHPULL, Gpio_OutType_OPENDRAIN
};
typedef enum Gpio_OutType Gpio_OutType;


/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
enum Gpio_Speed
{
	Gpio_Speed_LOW, Gpio_Speed_MEDIUM, Gpio_Speed_FAST, Gpio_Speed_HIGH
};
typedef enum Gpio_Speed Gpio_Speed;


/*
 * GPIO pin pull up AND pull down configuration enumeration
 */
enum Gpio_PinPuPd
{
	Gpio_PinPuPd_NOPUPD, Gpio_PinPuPd_PU, Gpio_PinPuPd_PD
};
typedef enum Gpio_PinPuPd Gpio_PinPuPd;

/*
 * This is a Configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;			/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;			/*!< possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


typedef struct
{
	GPIO_RegDef_t *pGPIOx;   /*!< This holds the base address of the GPIO port to which the pin belongs >*/
	GPIO_PinConfig_t GPIO_PinConfig;   /*!< This holds GPIO pin configuration settings >*/
}GPIO_Handle_t;


/*
 * Peripheral Clock setup
 */
extern void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


/*
 * Data read and writes
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
