/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_rcc.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_tim_ex.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_exti.h"
#include "stm32g4xx_hal_dma.h"
#include "stm32g4xx_hal_pwr.h"
#include "stm32g4xx_hal_pwr_ex.h"
#include "stm32g4xx_hal_cortex.h"
#include "stm32g4xx_hal_flash_ex.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_uart.h"
#include "stm32g4xx_hal_i2c.h"
#include "stm32g4xx_hal_i2c_ex.h"
#include "stm32g4xx_hal_qspi.h"
#include "stm32g4xx_hal_spi.h"
#include "stm32g4xx_hal_spi_ex.h"
//#include "stm32g4xx_hal_legacy.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
//pc11
#define GREEN_LED_Pin GPIO_PIN_11
#define GREEN_LED_GPIO_Port GPIOC
//PC12
#define BULE_LED_Pin GPIO_PIN_12
#define BULE_LED_GPIO_Port GPIOC
//PA12
#define RED_LED_Pin GPIO_PIN_12
#define RED_LED_GPIO_Port GPIOA
//PG10
#define NRST_Pin GPIO_PIN_10
#define NRST_GPIO_Port GPIOG
//PC13
#define KEY_Pin GPIO_PIN_13
#define KEY_GPIO_Port GPIOC
//PB12
#define POW_SW_Pin GPIO_PIN_12
#define POW_SW_GPIO_Port GPIOB
//PC9
#define TGAT_SW_Pin GPIO_PIN_9
#define TGAT_SW_GPIO_Port GPIOC
//PB14
#define EEG_TYPE_Pin GPIO_PIN_14
#define EEG_TYPE_GPIO_Port GPIOB
//PB10
#define VL53L0X_INT_Pin GPIO_PIN_10
#define VL53L0X_INT_GPIO_Port GPIOB
//PB2
#define VL53L0X_SD_Pin GPIO_PIN_5
#define VL53L0X_SD_GPIO_Port GPIOA
//PC6
#define OPT3001_INT_Pin GPIO_PIN_6
#define OPT3001_INT_GPIO_Port GPIOC
//PD2
#define ICM20608_INT_Pin GPIO_PIN_2
#define ICM20608_INT_GPIO_Port GPIOD


#define QSPI1_CSS_Pin GPIO_PIN_2
#define QSPI1_CSS_GPIO_Port GPIOA

#define QSPI1_MISO_Pin GPIO_PIN_0
#define QSPI1_MISO_GPIO_Port GPIOB

#define QSPI1_CLK_Pin GPIO_PIN_3
#define QSPI1_CLK_GPIO_Port GPIOA

#define QSPI1_MOSI_Pin GPIO_PIN_1
#define QSPI1_MOSI_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
enum Device_State{
	Device_OFF = 0,
	Device_ON,
	Device_SLEEP,
	Device_WAKEUP
};
extern volatile enum Device_State deviceState;

enum KEY_State{
	KEY_IDLE = 0,
	KEY_SHORT_CLICK,
	KEY_LONG_CLICK
};//GPIOA->IDR & GPIO_PIN_0//
#define KEY_MODE_WTICH_CTRL() 	HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)
#define LED_GREEN_TOG() 		HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin)
#define LED_GREEN_ON() 			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin,GPIO_PIN_RESET)
#define LED_GREEN_OFF() 		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin,GPIO_PIN_SET)
//低电平有效 蓝灯
#define LED_BULE_TOG() 			HAL_GPIO_TogglePin(BULE_LED_GPIO_Port, BULE_LED_Pin)
#define LED_BULE_ON() 			HAL_GPIO_WritePin(BULE_LED_GPIO_Port, BULE_LED_Pin,GPIO_PIN_RESET)
#define LED_BULE_OFF() 			HAL_GPIO_WritePin(BULE_LED_GPIO_Port, BULE_LED_Pin,GPIO_PIN_SET)
//低电平有效 红灯
#define LED_RED_TOG() 			HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin)
#define LED_RED_ON() 			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin,GPIO_PIN_RESET)
#define LED_RED_OFF() 			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin,GPIO_PIN_SET)
//电源供电开关处理 低电平导通
#define POWER_WTICH_CTRL_ON() 	HAL_GPIO_WritePin(POW_SW_GPIO_Port, POW_SW_Pin,GPIO_PIN_SET)
#define POWER_WTICH_CTRL_OFF() 	HAL_GPIO_WritePin(POW_SW_GPIO_Port, POW_SW_Pin,GPIO_PIN_RESET)

//电源供电开关处理 低电平导通
#define TGAT_POWER_WTICH_CTRL_ON() 		HAL_GPIO_WritePin(TGAT_SW_GPIO_Port, TGAT_SW_Pin,GPIO_PIN_SET)
#define  TGAT_POWER_WTICH_CTRL_OFF() 	HAL_GPIO_WritePin(TGAT_SW_GPIO_Port, TGAT_SW_Pin,GPIO_PIN_RESET)

//eeg脑电数据包类型
#define EEG_TYPE_CTRL_ON() 		HAL_GPIO_WritePin(EEG_TYPE_GPIO_Port, EEG_TYPE_Pin,GPIO_PIN_SET)
#define  EEG_TYPE_CTRL_OFF() 	HAL_GPIO_WritePin(EEG_TYPE_GPIO_Port, EEG_TYPE_Pin,GPIO_PIN_RESET)
//关闭测距仪的sd
#define  VL53L0X_SD_WTICH_CTRL_ON() 		HAL_GPIO_WritePin(VL53L0X_SD_GPIO_Port, VL53L0X_SD_Pin,GPIO_PIN_SET)
#define  VL53L0X_SD_WTICH_CTRL_OFF() 	HAL_GPIO_WritePin(VL53L0X_SD_GPIO_Port, VL53L0X_SD_Pin,GPIO_PIN_RESET)
//测距仪的sd
#define  IMC82060_INIT_WTICH_CTRL_ON() 		HAL_GPIO_WritePin(ICM20608_INT_GPIO_Port, ICM20608_INT_Pin,GPIO_PIN_SET)
#define  IMC82060_INIT_WTICH_CTRL_OFF() 	HAL_GPIO_WritePin(ICM20608_INT_GPIO_Port, ICM20608_INT_Pin,GPIO_PIN_RESET)
//flash片选
#define FlASH_SPI_CSS_H 	HAL_GPIO_WritePin(QSPI1_CSS_GPIO_Port, QSPI1_CSS_Pin,GPIO_PIN_SET)
#define FlASH_SPI_CSS_L  	HAL_GPIO_WritePin(QSPI1_CSS_GPIO_Port, QSPI1_CSS_Pin,GPIO_PIN_RESET)
 //flash时钟
#define FlASH_SPI_SCK_H 	HAL_GPIO_WritePin(QSPI1_CLK_GPIO_Port, QSPI1_CLK_Pin,GPIO_PIN_SET)
#define FlASH_SPI_SCK_L  	HAL_GPIO_WritePin(QSPI1_CLK_GPIO_Port, QSPI1_CLK_Pin,GPIO_PIN_RESET)
 //flash mosi
#define FlASH_SPI_MOSI_H 	HAL_GPIO_WritePin(QSPI1_MOSI_GPIO_Port, QSPI1_MOSI_Pin,GPIO_PIN_SET)
#define FlASH_SPI_MOSI_L  	HAL_GPIO_WritePin(QSPI1_MOSI_GPIO_Port, QSPI1_MOSI_Pin,GPIO_PIN_RESET)
//flash  miso
 #define READ_FlASH_SPI_MISO() 	HAL_GPIO_ReadPin(QSPI1_MISO_GPIO_Port, QSPI1_MISO_Pin)
//#define FlASH_SPI_MISO_H 	HAL_GPIO_WritePin(QSPI1_MISO_GPIO_Port, QSPI1_MISO_Pin,GPIO_PIN_SET)
//#define FlASH_SPI_MISO_L  	HAL_GPIO_WritePin(QSPI1_MISO_GPIO_Port, QSPI1_MISO_Pin,GPIO_PIN_RESET)
// 
void Delay_us(uint32_t nus);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
