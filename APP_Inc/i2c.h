/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c2;

extern I2C_HandleTypeDef hi2c3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

//#define MC20608_I2C_PORT 	GPIOA
//#define MC20608_I2C_CLK_PIN GPIO_PIN_9
//#define MC20608_I2C_SDA_PIN GPIO_PIN_8

//#define IMC20608_I2C_CLK_H    	GPIOA->BSRR = (uint32_t)MC20608_I2C_CLK_PIN
//#define IMC20608_I2C_CLK_L    	GPIOA->BRR = (uint32_t)MC20608_I2C_CLK_PIN

//#define IMC20608_I2C_SDA_H  	GPIOA->BSRR = MC20608_I2C_SDA_PIN
//#define IMC20608_I2C_SDA_L    	GPIOA->BRR  = MC20608_I2C_SDA_PIN

//#define IMC20608_I2C_SDA_READ 	(GPIOA->IDR & MC20608_I2C_SDA_PIN)
void MX_I2C3_DeInit(void);

void MX_I2C2_DeInit(void);

void MX_I2C2_Init(void);
void MX_I2C3_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

