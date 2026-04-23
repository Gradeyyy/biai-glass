/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "work_drive.h"
#include "flash_rw.h"
#include "vl53l0x.h"
#include "vl53l0x_cali.h"
#include "ICM_20608.h"
#include "algorithm.h"
#include "opt3001.h"
#include "key_scan.h"
#include "wwdg.h"
#include "ZD25WQ32C.h"
#include "td5322a.h"
#include "stm32_ota.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t flg = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile enum  Device_State deviceState = Device_OFF;

enum KEY_State keyState = KEY_IDLE;

extern uint8_t flag_change;
//uint8_t state = 0;
//extern WWDG_HandleTypeDef hwwdg;
extern VL53L0X_Dev_t vl53l0x_dev;//设备I2C数据参数
extern _vl53l0x_adjust Vl53l0x_adjust; //校准数据24c02写缓存区(用于在校准模式校准数据写24c02)

extern void Flash_Read(uint32_t *pBuffer, uint32_t NumToRead);
extern void Flash_Write(uint64_t *pBuffer, uint32_t  NumToWrite);
extern uint8_t receiveBuffer2[];
extern uint8_t receiveBufferTem2[];

uint64_t Flash_Buffer[10]={0x1122334455667788,0x2345678900000000,0x33334444,0x44445555,0x55556666,0x66667777,0x77778888,0x88889999,0x99990000,0x10101010};//要写入到STM32 FLASH的字符串数组
uint32_t Read_Buffer[20];      //Flash读取缓存数组
uint32_t  Read_buffer_cali[6];  // 读Flash校验数据

// 读Flash数据重组到校�????
void Data_Restructure(void)
{
//	uint8_t  adjustok;                    //校准成功标志0XAA，已校准;其他，未校准
//	uint8_t  isApertureSpads;             //ApertureSpads
//	uint8_t  VhvSettings;                 //VhvSettings
//	uint8_t  PhaseCal;                    //PhaseCal
//	uint32_t XTalkCalDistance;            //XTalkCalDistance
//	uint32_t XTalkCompensationRateMegaCps;//XTalkCompensationRateMegaCps
//	uint32_t CalDistanceMilliMeter;       //CalDistanceMilliMeter
//	int32_t  OffsetMicroMeter;            //OffsetMicroMeter
//	uint32_t refSpadCount;                //refSpadCount
	Vl53l0x_adjust.adjustok = (uint8_t)(Read_buffer_cali[0]&0xFF);
	Vl53l0x_adjust.isApertureSpads = (uint8_t)((Read_buffer_cali[0]>>8) &0xFF);
	Vl53l0x_adjust.VhvSettings = (uint8_t)((Read_buffer_cali[0]>>16)&0xFF);
	Vl53l0x_adjust.PhaseCal = (uint8_t)((Read_buffer_cali[0]>>24)&0xFF);
	Vl53l0x_adjust.XTalkCalDistance = Read_buffer_cali[1];
	Vl53l0x_adjust.XTalkCompensationRateMegaCps = Read_buffer_cali[2];
	Vl53l0x_adjust.CalDistanceMilliMeter = Read_buffer_cali[3];
	Vl53l0x_adjust.OffsetMicroMeter = Read_buffer_cali[4];
	Vl53l0x_adjust.refSpadCount = Read_buffer_cali[5];
}


//测试FLASH读写
void Test_Flash(void)
{
  Flash_Write(Flash_Buffer, sizeof(Flash_Buffer));
  Flash_Read(Read_Buffer, sizeof(Flash_Buffer));
  for(uint8_t i=0;i<20;i++)
  {
    printf("Read_Buffer[%d]:%lX\r\n", i, Read_Buffer[i]);
  }
}

void Delay_us(uint32_t nus)
{
  uint32_t Delay = nus * 84/4;
	do{
		__NOP();
	}while(Delay--);
}

HAL_StatusTypeDef config_ob(void)
{
	HAL_StatusTypeDef status;

	FLASH_OBProgramInitTypeDef OBInit;
	HAL_FLASHEx_OBGetConfig(&OBInit);

	if (OBInit.USERConfig & FLASH_OPTR_nSWBOOT0) {
		HAL_FLASH_Unlock();
		HAL_FLASH_OB_Unlock();
		OBInit.OptionType = OPTIONBYTE_USER;
		OBInit.USERType = OB_USER_nSWBOOT0;
		OBInit.USERConfig = OB_BOOT0_FROM_OB;
		if (HAL_FLASHEx_OBProgram(&OBInit) != HAL_OK)
		{
			status = HAL_FLASH_OB_Lock();
			status = HAL_FLASH_Lock();
			return HAL_ERROR;
		}
		HAL_FLASH_OB_Launch();
		status = HAL_FLASH_Lock();
		return HAL_ERROR;
	}
	return HAL_OK;
}

/*
 *函数功能：STM32软复位函数
 */
 void Stm32_SoftReset(void)
 {
   __set_FAULTMASK(1);//禁止所有的可屏蔽中断
   NVIC_SystemReset();//软件复位
 }

void Device_Enter_ON(void)
{
//	NVIC_SystemReset();
	HAL_Init();
	SystemClock_Config();
	uint8_t data_buffer[20] ={0};
	uint32_t uart_data_buffer =0x55;
	static uint8_t state =0;
  MX_GPIO_Init();
  key_input_exit();
  MX_USART2_UART_Init();
  HAL_Delay(100);
//  printf("uart success.\r\n");
////	SPI_FlashInit();
  SPI_Initializes();
  MX_I2C2_Init();
  MX_I2C3_Init();
	MX_USART1_UART_Init();
//	HAL_Delay(100);
	MX_USART3_UART_Init(USART3_BAUD_SETTING_115200);
	HAL_Delay(100);
  HAL_UART_Receive_IT(&huart3, &td5322a_struct.data, 1); 
 // SFLASH_WriteNByte((uint8_t*)"ABCDEF", 0, 6);   //从地址0，连续写入6字节数据(ABCDEF)
//  TIMDelay_Nms(500);
   //读mac地址出来，连续读出6字节数据
  SFLASH_ReadNByte(command_td5322a_connect_mac_address, 0, 6);             
	//  td5322a_uart_to_bel(&uart_data_buffer,1);  验证发送数据是否正常
//	td5322a_uart_to_bel_command(command_td5322a_bel_central_role_disconnect_channle_change);
//   HAL_Delay(100);
//   td5322a_uart_to_bel_command(command_td5322a_bel_central_role_disconnect);
//   //  td5322a_uart_to_bel(&uart_data_buffer,1);  验证发送数据是否正常
//  
//	HAL_Delay(100);
////	HAL_Delay(100);

// 此处蓝牙芯片重置，会修改眼镜的设备mac地址
//	td5322a_uart_to_bel_command(command_td5322a_save_data_resetore);
//	HAL_Delay(100);
//	td5322a_uart_to_bel_command(command_td5322a_set_bel_name);
	HAL_Delay(100);
//	td5322a_uart_to_bel_command(command_td5322a_save_data_reset);
//	td5322a_uart_to_bel_command(command_td5322a_test_ok);
//  开机时候需要作为主模式，查找从模式的飞碟
//	td5322a_uart_to_bel_command(command_td5322a_set_bel_central_role);
	MX_ADC3_Init();
	MX_TIM6_Init();
	MX_TIM7_Init();
	MX_WWDG_Init();
//  /* USER CODE BEGIN 2 */

////  HAL_GPIO_WritePin(POW_LED_GPIO_Port, POW_LED_Pin, GPIO_PIN_RESET);
	state = icm20608_init();
	if(state == 0)
	{
		flg =1;
		printf("icm20608 fail \r\n");

	}
	state = opt3001_init();
	if(state == 0)
	{
		flg =2;
		printf("opt3001 fail \r\n");

	}
//	if(state)
//	{
//	 printf("opt3001 init success.\r\n");
//	}else{
//	 printf("opt3001 init failed.\r\n");
//	}
	//  Test_Flash();
	// 初始化VL53L0x
	state = vl53l0x_init(&vl53l0x_dev);
	if (state ==1)    //vl53l0x初始�???????????????????
	{
		printf("VL53L0X Error!!!\r\n");
		flg = 3;

	}
	else
	{
		printf("VL53L0X OK\r\n");
	}
//HAL_Delay(3000);
//	Flash_Read(Read_buffer_cali, sizeof(Read_buffer_cali));
//	Data_Restructure();
//	if (Vl53l0x_adjust.adjustok != 0xAA) {
//		printf("begin cali .Vl53l0x_adjust.adjustok:%X \r\n",
//				Vl53l0x_adjust.adjustok);
//		printf("===============\r\n");
//		for (uint8_t ii = 0; ii < 6; ii++) {
//			printf("Read_Buffer_cali[%d]:0x%lX \r\n", ii, Read_buffer_cali[ii]);
//		}
//		printf("===============\r\n");
//		vl53l0x_calibration_test(&vl53l0x_dev);
//	} else {
//		;
//		printf("already cali.\r\n");
//	}

	// 测量模式 mode模式配置 0:默认; 1:高精�???????????????????; 2:长距�???????????????????; 3:�??????????????????? --默认模式单次测量 30ms datasheet P24
	vl53l0x_general_start(&vl53l0x_dev, 0);
	//因为在日常模式下，会调用这个函数，所以此刻它是蓝牙模式了，就不需要再检测2.4g，只有
  	Acc_algorithm_init();
//  	HAL_TIM_Base_Start_IT(&htim6);
//	HAL_TIM_Base_Start_IT(&htim7);
	uart_data_buffer =  SFLASH_ReadJEDEC_ID();
	//因为在日常模式下，会调用这个函数，所以此刻它是蓝牙模式了，就不需要再检测2.4g，只有
	if(global_ctrl_device.key_mode_state != DAILY_MODE)
	{
//		reset_tick();
		flag_change = 1;
//		device_init();
	}

//	__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
}




void enter_lowPower(void)
{
//	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI); //真正进入低功耗模

//	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
//	/* Clear all related wakeup flags*/
//	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
//	/* Enable WakeUp Pin PWR_WAKEUP_PIN2 connected to PC13 */
//	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
//	/* Enter the Standby mode */
//	HAL_PWR_EnterSTANDBYMode();
//	HAL_Delay(2000);
//	 __set_FAULTMASK(1);//禁止所有的可屏蔽中断
//   NVIC_SystemReset();//软件复位
}

void Device_Enter_OFF(void)
{
//						HAL_TIM_Base_Stop_IT(&htim6);
//	__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);

	icm_20608_sleep();
	// opt3001
	//						  opt3001_shutdown();
	// LT8960L
//	LT8960L_Sleep();
	opt3001_shutdown();
//	HAL_GPIO_WritePin(VL53L0X_SD_GPIO_Port, VL53L0X_SD_Pin, GPIO_PIN_RESET);
	TGAT_POWER_WTICH_CTRL_OFF();
	POWER_WTICH_CTRL_OFF();
//	__HAL_RCC_ADC12_CLK_DISABLE();
//	__HAL_RCC_USART1_CLK_DISABLE();
//	__HAL_RCC_USART2_CLK_DISABLE();
	__HAL_RCC_GPIOD_CLK_DISABLE();
	__HAL_RCC_GPIOA_CLK_DISABLE();
	__HAL_RCC_GPIOB_CLK_DISABLE();
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI); //真正进入低功耗模
}

const char *BUILD_DATE = __DATE__;
const char *BUILD_TIME = __TIME__;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
//extern volatile uint8_t  gpio_key_exit_flg;
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  HAL_Init();

////  /* USER CODE BEGIN Init */

////  /* USER CODE END Init */

////  /* Configure the system clock */
  SystemClock_Config();

//  /* USER CODE BEGIN SysInit */
////  config_ob();
 	// SCB->VTOR = APP_START_ADDR;
////  HAL_Delay(1000);
//  HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);

	__HAL_RCC_PWR_CLK_ENABLE();
 /* USER CODE END SysInit */
 	key_input_test();
 	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
 //	HAL_Delay(300);
 	  /* Disable all used wakeup sources: PWR_WAKEUP_PIN2 */
 	if((GPIOC->IDR & GPIO_PIN_13) != 0x00U) //没按下
 	{
 		HAL_Delay(20); //关机时候防抖左右&& flg == 0
 		if(((GPIOC->IDR & GPIO_PIN_13) != 0x00U))
 		{
 			if (READ_REG(TAMP->BKP15R) == 1)
 			{
 				WRITE_REG( TAMP->BKP15R, 0x0 );  /* reset back-up register */
 			}
 //			HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
 			HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);
 			/* Clear all related wakeup flags*/
 			__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
 	//		/* Enable WakeUp Pin PWR_WAKEUP_PIN2 connected to PC13 */
 			HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
 			HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
 			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2);
 			HAL_PWR_EnterSTANDBYMode();
 		}
 		/* Enter the Standby mode */

 	}
	HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
	SystemClock_Config();
  /* Initialize all configured peripherals */
   uint8_t data_buffer[20] ={0};
   uint32_t uart_data_buffer =0x55;
	static uint8_t state =0;
//	MX_USART2_UART_Init();
  MX_GPIO_Init();
  key_input_exit();
  MX_USART2_UART_Init();
  HAL_Delay(100);
  printf("build %s %s\r\n",BUILD_DATE, BUILD_TIME);
//  printf("uart success.\r\n");
////	SPI_FlashInit();
  SPI_Initializes();
  MX_I2C2_Init();
  MX_I2C3_Init();
	MX_USART1_UART_Init();
//	HAL_Delay(100);
	MX_USART3_UART_Init(USART3_BAUD_SETTING_115200);
	HAL_Delay(100);
	HAL_UART_Receive_IT(&huart3, &td5322a_struct.data, 1); 
 // SFLASH_WriteNByte((uint8_t*)"ABCDEF", 0, 6);   //从地址0，连续写入6字节数据(ABCDEF)
//  TIMDelay_Nms(500);
   //读mac地址出来，连续读出6字节数据
	SFLASH_ReadNByte(command_td5322a_connect_mac_address, 0, 6);        
	// 此处蓝牙芯片重置，会修改眼镜的设备mac地址
	td5322a_uart_to_bel_command(command_td5322a_save_data_resetore);
	HAL_Delay(100);

	MX_ADC3_Init();
	MX_TIM6_Init();
	MX_TIM7_Init();
	MX_WWDG_Init();
//  /* USER CODE BEGIN 2 */

////  HAL_GPIO_WritePin(POW_LED_GPIO_Port, POW_LED_Pin, GPIO_PIN_RESET);
	state = icm20608_init();
	if(state == 0)
	{
		flg =1;
		printf("icm20608 fail \r\n");

	}
	state = opt3001_init();
	if(state == 0)
	{
		flg =2;
		printf("opt3001 fail \r\n");

	}

	state = vl53l0x_init(&vl53l0x_dev);
	if (state ==1)    //vl53l0x初始�???????????????????
	{
		printf("VL53L0X Error!!!\r\n");
		flg = 3;

	}
	else
	{
		printf("VL53L0X OK\r\n");
	}
//	HAL_Delay(3000);
//	Flash_Read(Read_buffer_cali, sizeof(Read_buffer_cali));
//	Data_Restructure();
//	if (Vl53l0x_adjust.adjustok != 0xAA) {
//		printf("begin cali .Vl53l0x_adjust.adjustok:%X \r\n",
//				Vl53l0x_adjust.adjustok);
//		printf("===============\r\n");
//		for (uint8_t ii = 0; ii < 6; ii++) {
//			printf("Read_Buffer_cali[%d]:0x%lX \r\n", ii, Read_buffer_cali[ii]);
//		}
//		printf("===============\r\n");
//		vl53l0x_calibration_test(&vl53l0x_dev);
//	} else {
//		;
//		printf("already cali.\r\n");
//	}

	// 测量模式 mode模式配置 0:默认; 1:高精�???????????????????; 2:长距�???????????????????; 3:�??????????????????? --默认模式单次测量 30ms datasheet P24
	vl53l0x_general_start(&vl53l0x_dev, 0);
	//因为在日常模式下，会调用这个函数，所以此刻它是蓝牙模式了，就不需要再检测2.4g，只有
  	Acc_algorithm_init();
  	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	uart_data_buffer =  SFLASH_ReadJEDEC_ID();
//	if(flg != 0 && uart_data_buffer != 0xBA6015)
//	{
//		LED_BULE_ON();
//	}
//	else 
//	{
//		LED_GREEN_ON();
//	}
	static uint8_t uart_test_flg =0;
	volatile uint8_t data_buffer_temp[50] ={0};
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 // static uint8_t flg_uart= 0;
	while (1) {
		
		//HAL_Delay(2500);   //该延时用于触发看门狗，测试用
		if(wwdg_task() == 1)
		{
		  NVIC_SystemReset();//软件复位
		}
		else
		{
			key_scan_task(&td5322a_struct);
			switch (deviceState)
			{
				case Device_OFF:
					break;
				case Device_ON:
					work_task();
					break;
				case Device_SLEEP:

					break;
				case Device_WAKEUP:

					break;
				default:
					break;
			}
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 21;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
//	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//	/** Configure the main internal regulator output voltage
//	*/
//	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

//	/** Initializes the RCC Oscillators according to the specified parameters
//	* in the RCC_OscInitTypeDef structure.
//	*/
//	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
//	RCC_OscInitStruct.PLL.PLLN = 85;
//	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
//	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
//	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//	{
//	Error_Handler();
//	}

//	/** Initializes the CPU, AHB and APB buses clocks
//	*/
//	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
//	{
//	Error_Handler();
//	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
