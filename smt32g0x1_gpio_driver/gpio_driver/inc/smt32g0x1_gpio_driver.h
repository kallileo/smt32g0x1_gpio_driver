/*
 * smt32g0x1_gpio_driver.h
 *
 *  Created on: Apr 26, 2024
 *      Author: Ryzen
 */


#include "stm32g0x1.h"


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
/*
#define GPIO_PIN_NO_0  				0
#define GPIO_PIN_NO_1  				1
#define GPIO_PIN_NO_2  				2
#define GPIO_PIN_NO_3  				3
#define GPIO_PIN_NO_4  				4
#define GPIO_PIN_NO_5  				5
#define GPIO_PIN_NO_6  				6
#define GPIO_PIN_NO_7  				7
#define GPIO_PIN_NO_8  				8
#define GPIO_PIN_NO_9  				9
#define GPIO_PIN_NO_10  			10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12  			12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15
*/

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers by using Enumerator instead of Macros
 */
enum Gpio_Pin_Num
{
	Gpio_Pin_Num_0 = 0,
	Gpio_Pin_Num_1,
	Gpio_Pin_Num_2,
	Gpio_Pin_Num_3,
	Gpio_Pin_Num_4,
	Gpio_Pin_Num_5,
	Gpio_Pin_Num_6,
	Gpio_Pin_Num_7,
	Gpio_Pin_Num_8,
	Gpio_Pin_Num_9,
	Gpio_Pin_Num_10,
	Gpio_Pin_Num_11,
	Gpio_Pin_Num_12,
	Gpio_Pin_Num_13,
	Gpio_Pin_Num_14,
	Gpio_Pin_Num_15,
	Gpio_Pin_Num_16
};
typedef enum Gpio_Pin_Num Gpio_Pin_Num;

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
/*
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT     4
#define GPIO_MODE_IT_RT     5
#define GPIO_MODE_IT_RFT    6
*/

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes by using Enumerator instead of Macros
 */
enum Gpio_Mode
{
	Gpio_Mode_In = 0,
	Gpio_Mode_Out,
	Gpio_Mode_ALTFN,
	Gpio_Mode_IT_FT,
	Gpio_Mode_IT_RT,
	Gpio_Mode_IT_RFT
};
typedef enum Gpio_Mode Gpio_Mode;

/*
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP   0
#define GPIO_OP_TYPE_OD   1


/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPOI_SPEED_HIGH			3


/*
 * GPIO pin pull up AND pull down configuration macros
 */
#define GPIO_NO_PUPD   		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

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

/*
 * Port Clock control
 */
void GPIO_PortClockEnblOrDsbl(GPIO_RegDef_t *pGPIOx, uint8_t EnblOrDsbl);

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


/*
 * Data read and write
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
