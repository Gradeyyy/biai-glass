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
#include "adc.h"
#include "main.h"
#include "gpio.h"
//#include "data.h"
volatile uint16_t ADC_T[BATTERY_VOATAGE_ARRY_MAX]={0};

volatile TYPE_BATTERY_VOLTAGE_SAMPLE battery_voltage_struct ={
						.battery_voltage_time_sample_flg = 1,// 开机上电先采集8组数据后取平均值标志位，后面滑动采集电压
						.battery_voltage_time_sample_cnt = 0,// 定时器计时
						.battery_voltage_vlaue =0.0,
						.battery_capacity = 0
						};
ADC_HandleTypeDef hadc3;

/* ADC2 init function */
void MX_ADC3_Init(void)
{

  ADC_ChannelConfTypeDef sConfig = {0};

//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC345;
    PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* ADC2 clock enable */
    __HAL_RCC_ADC345_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PA7     ------> ADC2_IN4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC345_CLK_ENABLE();

    /**ADC2 GPIO Configuration
    PA7     ------> ADC2_IN4
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13);

  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
}

void ProcessADC(void)
{
	HAL_StatusTypeDef sta;
	static uint8_t low_vlotage_time_cnt =0;
	uint32_t Adc_Value= 0;
	uint32_t Max_AdcValue = 0;
	uint32_t Min_AdcValue = 0;
	static uint8_t adc_idex=0;
	uint32_t sum = 0x0;

	if(battery_voltage_struct.battery_voltage_time_sample_flg == 1)
	{
//		battery_voltage_struct.battery_voltage_time_sample_flg = 0;
		sta = HAL_ADC_Start(&hadc3);
		if (sta == HAL_BUSY)
		{
//			printf("adc start busy. \r\n");
		}
		else if (sta == HAL_OK) {
			//printf("adc start success. \r\n");
		}
		else
		{
			return;
//			printf("adc start failed. \r\n");
		}
		HAL_ADC_PollForConversion(&hadc3, 10);
		Adc_Value = HAL_ADC_GetValue(&hadc3);
		ADC_T[adc_idex]= Adc_Value;
		adc_idex=(adc_idex>=BATTERY_VOATAGE_ARRY_MAX-1)?BATTERY_VOATAGE_ARRY_MAX:adc_idex+1;
		if(adc_idex == BATTERY_VOATAGE_ARRY_MAX)
		{
			adc_idex = BATTERY_VOATAGE_ARRY_MAX - 1;
			battery_voltage_struct.battery_voltage_time_sample_flg = 0;

			for(uint8_t j=0;j<BATTERY_VOATAGE_ARRY_MAX;j++)
			{
				ADC_T[j] =ADC_T[j+1]; //滑动滤波，去掉第一个数据，每次填充都是最后一个数据
//				sum=sum+ADC_T[j];

			}
			Min_AdcValue = ADC_T[0];
			Max_AdcValue = ADC_T[0];
			for(uint8_t j=0;j<BATTERY_VOATAGE_ARRY_MAX-1;j++) //去掉最大最小值，求平均
			{
				if(Max_AdcValue <= ADC_T[j])
					Max_AdcValue = ADC_T[j];
				if(Min_AdcValue >= ADC_T[j])
					Min_AdcValue  = ADC_T[j];
				sum=sum+ADC_T[j];
			}

			sum = (sum-Max_AdcValue-Min_AdcValue)/(BATTERY_VOATAGE_ARRY_MAX-3);

			battery_voltage_struct.battery_voltage_vlaue = (float) (((sum) & 0xFFF)* 5.072/4095); //5.072V V2.0对应的是20k分压电阻，6.9V V1.0对应的是是通过实际的电池电压测出来，除以采集到的数据得到
//			printf(" adc_value %d,vol:%lf v \r\n", sum,battery_voltage_struct.battery_voltage_vlaue);
			if(battery_voltage_struct.battery_voltage_vlaue > 4.2)
			{
				battery_voltage_struct.battery_voltage_vlaue = 4.2;
			}
			battery_voltage_struct.battery_capacity = ((battery_voltage_struct.battery_voltage_vlaue-3.4)/(0.8))*100;

			if (battery_voltage_struct.battery_voltage_vlaue <= 3.4)
			{
				low_vlotage_time_cnt++;
				if(low_vlotage_time_cnt > 2) //低电压超过5次以上认为是电池快没电了
				{
					deviceState = Device_OFF;//关机
					low_vlotage_time_cnt =0;
				}
			}
			else
			{
				low_vlotage_time_cnt =0;
			}

		}
	}

		//printf("volVale:%d,  vol:%.2f v \r\n", volValue, volFloat);
//		volFloat = (float) (volValue & 0xFFF) * 3.3 / 4096;
		//printf("volVale:%x,  vol:%lf v \r\n", volValue, volFloat);
	}


//}


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
