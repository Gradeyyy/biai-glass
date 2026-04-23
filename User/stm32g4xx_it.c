/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32g4xx_it.h"
#include "key_scan.h"
//#include "LT8960Ldrv.h"
#include "adc.h"
#include "work_drive.h"
#include "usart.h"
#include "td5322a.h"
#include "tim.h"
#include "stm32_ota.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
//extern TIM_HandleTypeDef htim6;
//extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */
// EEG 变量
extern uint8_t receive_buffer_size;
extern uint8_t resFlag;
extern uint8_t receiveBuffer[];
extern uint8_t receiveBufferTem[];

extern uint8_t receiveBuffer2[];
extern uint8_t receiveBuffer2Tem[];


// 计时变量
extern uint16_t count10ms;
extern uint8_t flag_1ms;
extern uint8_t flag_userTime;
extern uint8_t flag_timeStart;
extern uint32_t flag_3ms;

// 按键计时
uint16_t flag_key_3ms;
uint8_t  flag_key_timeStart;

extern uint8_t flag_userTime_2;
extern uint8_t flag_timeStart_2;

extern enum KEY_State keyState;

uint16_t timeCount2 = 0;

uint16_t timeCount = 0;

uint32_t tick_3ms =0;

//extern volatile uint8_t gpio_key_exit_flg;/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
	NVIC_SystemReset();//软件复位
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  ble_uart_rx_msg.timeout++;
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	volatile uint8_t data_temp =0;
  /* USER CODE END USART1_IRQn 0 */
//	static int16_t receive_data_length =0;
//	if(__HAL_UART_GET_IT(&huart3,UART_IT_RXNE) != RESET)  //是否发生中断
//	{
//		__HAL_UART_CLEAR_IT(&huart3,UART_IT_RXNE);
////////		HAL_UART_Receive_IT(&huart1, &data_temp, 1);
//		data_temp = (uint8_t)huart3.Instance->RDR;
//		td5322a_struct.uart_receive_data_buffer[td5322a_struct.uart_length++] =data_temp;
//		tagt_struct.uart_length++;
////		tagt_struct.uart_receive_time_flg = 1; //开始串口超时接收发送，如果发的数据不对，发完了，认为数据不齐需要清零,开机程序会发送两个字节的aa过来，不知道什么原因
////		

////		if(receiveBuffer[0] == 0xaa)
////		{		
//////			if(*((uint16_t *)receiveBuffer) == EEG_UART_HAND_FRAME)
////			{
////				if((receive_data_length-4) == receiveBuffer[2])
////				{
////					tagt_struct.uart_flag = 1;
////					tagt_struct.uart_length = receive_data_length;	//把获取的数据传给蓝牙芯片				
//////					memcpy(tagt_struct.uart_receive_data_buffer,receiveBuffer,tagt_struct.uart_length);
//////					memset(receiveBuffer,0,receive_data_length);
////					tagt_struct.uart_receive_time_flg =0;			
////					receive_data_length = 0;
////				}
////				
////			}

////		}
//	}
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}
 extern uint8_t flg;

void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */
//	flg = 2;

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

//void DMA1_Channel2_IRQHandler(void)
//{
//  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

//  /* USER CODE END DMA1_Channel4_IRQn 0 */
//  HAL_DMA_IRQHandler(&hdma_usart3_rx);
//  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

//  /* USER CODE END DMA1_Channel4_IRQn 1 */
//}
/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC3 channel underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);

  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}
void TIM7_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
//	static uint8_t i =0;
//	i++;
//	if(i>5)
//	{
//		i =0;
//	}
  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	tick_3ms++;
	static 	uint16_t sLed_time_cnt = 0;
	static 	uint8_t sLed_time_flg = 0;
	static uint8_t sBle_light_flg =0;
	static uint8_t s_cnt= 0;
//	static uint8_t mcu_uart_send_msg_state= 0;
  /* USER CODE END TIM6_DAC_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim6);
	if (htim == &htim6) 
	{

	//	flag_1ms++;
	//	开机的情况下或者在低功耗的日常模式依然可以进来，识别灯状态
		if(deviceState == Device_ON) //绿灯500ms闪烁一次
		{
//			flag_1ms++;  // 3ms主要是对飞碟陀螺仪控制等时间进行处理
	/////////////////////global_ctrl_device.key_mode_state ==TRAINING_MODE &&/////2.4G模式训练模式下的灯状态亮灯情况	sLed_time_flg == 0&&				sLed_time_flg = 1;
			
			if( global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.eye_muscle_training_mode == 1)
			{

				sLed_time_cnt = 0;
				LED_BULE_OFF();
				LED_GREEN_ON();

			}
			else if(global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.cervical_training_mode == 1)
			{
	//				sLed_time_cnt = 0;
				sLed_time_cnt++;
				if(sLed_time_cnt >= 2)
				{
					sLed_time_cnt = 0;
					LED_BULE_OFF();
					sLed_time_flg = 0;
					LED_GREEN_TOG();
				}

			}
			else if(global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.mental_training_mode == 1)
			{
				sLed_time_cnt ++;
				if(sLed_time_cnt >= 10)
				{
					sLed_time_cnt = 0;
					sLed_time_flg = 0;
					LED_BULE_OFF();
					LED_GREEN_TOG();
				}
			}
	//////////////////////////////////////////////////////////////////
	/////////////////////蓝牙模式，即时模式3分钟的时间处理&&sBle_light_flg == 0				sBle_light_flg = 1;
			//蓝牙未连上的情况常亮
			else if(global_ctrl_device.key_mode_state == IMMEDIATE_MODE &&td5322a_struct.work_role_connect_flg == 0)
			{
				LED_BULE_ON();
				sLed_time_flg =0;
				sLed_time_cnt =0;
				LED_GREEN_OFF();
			}
			else if(global_ctrl_device.key_mode_state == IMMEDIATE_MODE &&td5322a_struct.work_role_connect_flg == 1)
			{
				sLed_time_cnt++;
				if(sLed_time_cnt >= 10)
				{
					LED_BULE_TOG();
					LED_GREEN_OFF();
					sLed_time_cnt =0;
				}
			}
			//蓝牙连上后，快
			else if(global_ctrl_device.key_mode_state == DAILY_MODE)
			{
				sLed_time_cnt++;
				if(sLed_time_cnt >= 20)
				{
					LED_BULE_TOG();
					LED_GREEN_OFF();
					sLed_time_cnt =0;
				}
			}

			//低功耗蓝牙模式切换定时器
			if(global_ctrl_device.key_mode_time_flg == 1)
			{
				global_ctrl_device.key_mode_time_cnt++;
			}
			else
			{
				global_ctrl_device.key_mode_time_cnt =0;
			}
			//低功耗定时器
			if(global_ctrl_device.low_power_consumption_time_flg == 1)
			{
				global_ctrl_device.low_power_consumption_time_cnt++;
			}
			else
			{
				global_ctrl_device.low_power_consumption_time_cnt =0;
			}
			//向前向有飞行执行时间
			if(global_ctrl_device.eye_muscle_training_mode_ctrl_fly_time_flg == 1)
			{
				global_ctrl_device.eye_muscle_training_mode_ctrl_fly_time_cnt++;
			}
			else
			{
				global_ctrl_device.eye_muscle_training_mode_ctrl_fly_time_cnt = 0;
			}

		
			
			//脑电飞行准备时间
			if(global_ctrl_device.mental_training_mode_ctrl_prepare_fly_time_flg ==1)
			{
				global_ctrl_device.mental_training_mode_ctrl_prepare_fly_time_cnt++;
			}
			else
			{
				global_ctrl_device.mental_training_mode_ctrl_prepare_fly_time_cnt =0;
			}
			//脑电悬停起飞后等待过程向有飞行执行时间
			if(global_ctrl_device.mental_training_mode_ctrl_fly_time_flg == 1)
			{
				global_ctrl_device.mental_training_mode_ctrl_fly_time_cnt++;
			}
			else
			{
				global_ctrl_device.mental_training_mode_ctrl_fly_time_cnt = 0;
			}
			if(global_ctrl_device.eeg_uart_init_time_flg == 1)
			{
				global_ctrl_device.eeg_uart_init_time_cnt++;
			}
			else
			{
				global_ctrl_device.eeg_uart_init_time_cnt = 0;
			}
		}
		else
		{
			sLed_time_cnt = 0;
		}
	}
	//定时器2，20ms进一次中断
	else if(htim == &htim7)
	{
	//飞碟飞行时间定时器
		if(global_ctrl_device.power_ctrl_key_flg ==1)
		{
			global_ctrl_device.power_ctrl_key_cnt++;

		}
		else
		{
			global_ctrl_device.power_ctrl_key_cnt = 0;
		}
		if(deviceState == Device_ON) //
		{
			flag_1ms++; 	
			//主机模式计时，寻找有没有从机
			if(battery_voltage_struct.battery_voltage_time_sample_cnt >= 50)
			{
				battery_voltage_struct.battery_voltage_time_sample_cnt = 0;
				battery_voltage_struct.battery_voltage_time_sample_flg = 1;
			}
			else
			{
				battery_voltage_struct.battery_voltage_time_sample_cnt ++;
			}
					//20ms采集一次数据
		
			if(global_ctrl_device.ctrl_fly_time_flg ==1)
			{
				global_ctrl_device.ctrl_fly_time_cnt++;
			}
			else
			{
				global_ctrl_device.ctrl_fly_time_cnt =0;
			}
			if(td5322a_struct.uart_send_time_flg == 1)
			{
				td5322a_struct.uart_send_time_cnt++;
				
			}
			else
			{
				td5322a_struct.uart_send_time_cnt = 0;
			}
			if(global_ctrl_device.ble_mode_time_flg == 1)
			{
				global_ctrl_device.ble_mode_time_cnt++;
				
				if(s_cnt >50 && td5322a_struct.work_role_connect_flg == 0)
				{
					td5322a_uart_to_bel_command(command_td5322a_set_bel_central_role_scan);
					s_cnt =0;
				}
				else
				{
					s_cnt++;
				}
			}
			else
			{
				global_ctrl_device.ble_mode_time_cnt =0;
				s_cnt=0;
			}
		}

		
	}
	
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
//	uint8_t uart3_receive_data_temp =0;
	static uint8_t state =0;
	if (UartHandle == &huart1) {
//			resBuffer = (uint16_t)READ_REG(huart2.Instance -> RDR);
//			resFlag = 1;
//			WRITE_REG(huart1.Instance -> TDR, resBuffer);
		for(int i = 0; i < receive_buffer_size; i++)
		{
			receiveBufferTem[i]  = receiveBuffer[i];
		}
		HAL_UART_Receive_IT(&huart1, receiveBuffer, receive_buffer_size);
		resFlag = 1;
	}
	else if(UartHandle == &huart3) 
	{
		ble_uart_rx_msg.flag = 1;
		ble_uart_rx_msg.timeout = 0;
		ble_uart_rx_msg.data[ble_uart_rx_msg.length++] = td5322a_struct.data;

		td5322a_struct.uart_receive_data_buffer[td5322a_struct.uart_length++] = td5322a_struct.data;
		
//		uart_fill_cbuffer(&uart3_struct,&td5322a_struct.data,1);

//		for(uint8_t i =0;i <td5322a_struct.uart_length;i++)
//			{
//				printf("%02x",td5322a_struct.data);
//			}
		if(td5322a_struct.uart_length>=1023)
			td5322a_struct.uart_length =0;
		if(td5322a_struct.wrok_mode == 0 && td5322a_struct.uart_flag ==0)
		{
			switch(state)
			{
				case 0:
					if(td5322a_struct.data ==0x0d)
					{
						state =1;
//						td5322a_struct.uart_receive_data_buffer[td5322a_struct.uart_length++] = td5322a_struct.data;
					}

					break;
					case 1:
					if(td5322a_struct.data ==0x0a)
					{
						state =2;
//						td5322a_struct.uart_receive_data_buffer[td5322a_struct.uart_length++] = td5322a_struct.data;

					}

					break;
					case 2:
					if(td5322a_struct.data ==0x0d)
					{
						state =3;
					}
					else
					{
						;
					}
//					td5322a_struct.uart_receive_data_buffer[td5322a_struct.uart_length++] = td5322a_struct.data;

					break;
					case 3:
					if(td5322a_struct.data ==0x0a)
					{
						state =0;
//						td5322a_struct.uart_receive_data_buffer[td5322a_struct.uart_length++] = td5322a_struct.data;
						td5322a_struct.uart_receive_data_buffer[td5322a_struct.uart_length++] = '\0';  // 添加字符串结束符
						td5322a_struct.uart_flag =1;
					}		
					break;
					
			}
		}
		else if( td5322a_struct.wrok_mode == 1)
		{
//			printf("%02x",td5322a_struct.data);
		}
		
		
		HAL_UART_Receive_IT(&huart3, &td5322a_struct.data, 1);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint16_t key_state = 0;
	static uint8_t key_system_on =0;
//	RESET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)
	if (GPIO_Pin == KEY_Pin)//RESET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)&&
	{
		if( RESET == HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) && global_ctrl_device.key_press_off_flg == 0x01  && global_ctrl_device.gpio_key_exit_flg ==0)
		{
//			key_system_on = 1;
//			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
///			deviceState = Device_ON;
			global_ctrl_device.gpio_key_exit_flg = 1;
		}
	}
}
/* USER CODE END 1 */
