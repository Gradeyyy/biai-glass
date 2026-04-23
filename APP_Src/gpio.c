/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
uint8_t gpio_key_flag = 0;
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  static uint8_t led_flg =0;
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  //先打开脑电和陀螺仪等其他的供电的
	TGAT_POWER_WTICH_CTRL_ON();
	POWER_WTICH_CTRL_ON();
	VL53L0X_SD_WTICH_CTRL_ON();
	EEG_TYPE_CTRL_ON();
	IMC82060_INIT_WTICH_CTRL_OFF();
	LED_RED_OFF();
	LED_GREEN_OFF();
	LED_BULE_OFF();
  
  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);
  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = BULE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(BULE_LED_GPIO_Port, &GPIO_InitStruct);
    /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(RED_LED_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = POW_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(POW_SW_GPIO_Port, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = TGAT_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(TGAT_SW_GPIO_Port, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = EEG_TYPE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(EEG_TYPE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = VL53L0X_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VL53L0X_INT_GPIO_Port, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = ICM20608_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(ICM20608_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = VL53L0X_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(VL53L0X_SD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = OPT3001_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OPT3001_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = BULE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init( BULE_LED_GPIO_Port, &GPIO_InitStruct);

//	EEG_TYPE_CTRL_OFF();
//	EEG_TYPE_CTRL_ON();
}
void key_input_nomal(void)
{

	GPIO_InitTypeDef GPIO_initStruct = {0};
//	static uint8_t led_flg =0;
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_initStruct.Pin = KEY_Pin;
	GPIO_initStruct.Mode = GPIO_MODE_INPUT;
	GPIO_initStruct.Pull = GPIO_PULLUP;
	GPIO_initStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_initStruct);
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
}
void key_input_exit(void)
{

	GPIO_InitTypeDef GPIO_initStruct = {0};
//	static uint8_t led_flg =0;
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_initStruct.Pin = KEY_Pin;
	GPIO_initStruct.Mode =GPIO_MODE_IT_FALLING;
	GPIO_initStruct.Pull = GPIO_PULLUP;
	GPIO_initStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_initStruct);
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
//上电的时候不能立刻中断使能初始化按键，要等到确定为松开的时候才能使能按键
void key_input_test(void)
{

	GPIO_InitTypeDef GPIO_initStruct = {0};
//	static uint8_t led_flg =0;
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_initStruct.Pin = KEY_Pin;
	GPIO_initStruct.Mode =GPIO_MODE_IT_FALLING;
	GPIO_initStruct.Pull = GPIO_PULLUP;
	GPIO_initStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_initStruct);

}
/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
