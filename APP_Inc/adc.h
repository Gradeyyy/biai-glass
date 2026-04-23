/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_ADC3_Init(void);
void ProcessADC(void);
/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */
//adc结构体
typedef struct
{
	uint8_t battery_voltage_time_sample_flg;
	uint16_t battery_voltage_time_sample_cnt;
	float battery_voltage_vlaue;
	uint8_t battery_capacity;
}TYPE_BATTERY_VOLTAGE_SAMPLE;
extern volatile TYPE_BATTERY_VOLTAGE_SAMPLE battery_voltage_struct;

#define  BATTERY_VOATAGE_ARRY_MAX  10
#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

