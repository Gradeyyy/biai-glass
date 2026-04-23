/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "wwdg.h"
#include "stm32g4xx_hal_wwdg.h"
#include "gpio.h"
WWDG_HandleTypeDef hwwdg;

#define WWDG_WINDOW    0x50
#define WWDG_COUNTER   0x7F
void MX_WWDG_Init(void)
{

  /* USER CODE BEGIN WWDG_Init 0 */
     /*  Default WWDG Configuration:
      1] Set WWDG counter to 0x7F  and window to 0x50
      2] Set Prescaler to WWDG_PRESCALER_128

      Timing calculation:
      a) WWDG clock counter period (in ms) = (4096 * WWDG_PRESCALER_128) / (PCLK1 / 1000)
                                           = 3,495 ms
      b) WWDG timeout (in ms) = (0x7F + 1) * 3,495
                              ~= 447,4 ms
      => After refresh, WWDG will expires after 447,4 ms and generate reset if
      counter is not reloaded.
      c) Time to enter inside window
      Window timeout (in ms) = (127 - 80 + 1) * 3,495
                             = 167,7 ms */
  /* USER CODE END WWDG_Init 0 */

  /* USER CODE BEGIN WWDG_Init 1 */

  /* USER CODE END WWDG_Init 1 */
  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_128;
  hwwdg.Init.Window = WWDG_WINDOW;
  hwwdg.Init.Counter = WWDG_COUNTER;
  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN WWDG_Init 2 */
	//__HAL_RCC_WWDG_CLK_ENABLE();
  /* USER CODE END WWDG_Init 2 */

 }
uint8_t wwdg_task(void)
{
	if(HAL_WWDG_Refresh(&hwwdg) != HAL_OK)
	{
		return 1;
	}
	else 
	{
		return 0;
	}
}
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
