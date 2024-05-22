/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: May 23, 2024
 *      Author: omega
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_
#include "stm32f411xx.h"  
typedef struct 
{
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_PinAltFuncMode;
    /* data */
}GPIO_PinConfig;


// GPIO struct pin
typedef struct{
    GPIO_RegDef_t* pGPIOx; // hold the address of the gpio port which the pin is belong
    GPIO_PinConfig_t GPIO_PinConfig; // hold setting for pin 
} GPIO_Handle_t;

#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
