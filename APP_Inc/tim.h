/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_TIM6_Init(void);
void MX_TIM7_Init(void);
/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */
typedef struct
{
	uint8_t time_1_5s_flg;
	uint16_t time_1_5s_cnt;
	uint8_t time_1ms_flg;
	uint16_t time_1ms_cnt;
	uint8_t time_3ms_flg;
	uint16_t time_3ms_cnt;
}TIM_CRTL_STRUCT;
extern volatile TIM_CRTL_STRUCT time_struct;

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

