#include "vl53l0x.h"
#include <stdio.h>
//#include "./SysTick/bsp_SysTick.h"


_VL53L0X_adjust Vl53l0x_adjust; 
_VL53L0X_adjust Vl53l0x_data;   
VL53L0X_Dev_t vl53l0x_dev;                    /*设备I2C数据参数*/
//_vl53l0x_adjust Vl53l0x_adjust; //校准数据24c02写缓存区(用于在校准模式校准数据写入24c02)
//_vl53l0x_adjust Vl53l0x_data;   //校准数据24c02读缓存区（用于系统初始化时向24C02读取数据）

VL53L0X_DeviceInfo_t vl53l0x_dev_info;        /*设备ID版本信息*/
uint8_t AjustOK=0;                            /*校准标志位*/
VL53L0X_RangingMeasurementData_t vl53l0x_data;/*测距测量结构体*/
volatile uint16_t Distance_data=0;                         /*保存测距数据*/

/*0：默认;1:高精度;2:长距离;3:高速*/
mode_data Mode_data[]=
{
  {(FixPoint1616_t)(0.25*65536), 
	 (FixPoint1616_t)(18*65536),
	  33000,
	  14,
	  10}, /*默认*/
		
	{(FixPoint1616_t)(0.25*65536) ,
	 (FixPoint1616_t)(18*65536),
	  200000, 
	  14,
	  10},/*高精度*/
		
  {(FixPoint1616_t)(0.1*65536) ,
	 (FixPoint1616_t)(60*65536),
	  33000,
	  18,
	  14},/*长距离*/
	
  {(FixPoint1616_t)(0.25*65536) ,
	 (FixPoint1616_t)(32*65536),
	  20000,
	  14,
	  10},/*高速*/		
};

/**
  * @brief  打印错误信息
  * @param  Status详情看VL53L0X_Error参数的定义
  * @retval 无
  */
void print_pal_error(VL53L0X_Error Status)
{	
	char buf[VL53L0X_MAX_STRING_LENGTH];
	
  /*根据Status状态获取错误信息字符串*/
	VL53L0X_GetPalErrorString(Status,buf);          
	
  /*打印状态和错误信息*/
//  printf("API Status: %i : %s\r\n",Status, buf);  
	
}


/**
  * @brief  配置VL53L0X设备I2C地址
  * @param  dev:设备I2C参数结构体
  * @param  Status：详情看VL53L0X_Error参数的定义
  * @param  newaddr:设备新I2C地址
  * @retval 错误信息
  */
VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev,uint8_t newaddr)
{
	uint16_t Id;
	uint8_t FinalAddress;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t sta=0x00;
	
	FinalAddress = newaddr;
	
  /*新设备I2C地址与旧地址一致,直接退出*/
	if(FinalAddress==dev->I2cDevAddr)            
		return VL53L0X_ERROR_NONE;
  
	/*在进行第一个寄存器访问之前设置I2C标准模式(400Khz)*/
	Status = VL53L0X_WrByte(dev,0x88,0x00);
	if(Status!=VL53L0X_ERROR_NONE) 
	{
    /*设置I2C标准模式出错*/
		sta=0x01;                                
		goto set_error;
	}
	/*尝试使用默认的0x52地址读取一个寄存器*/
	Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
	if(Status!=VL53L0X_ERROR_NONE) 
	{
    /*读取寄存器出错*/
		sta=0x02;                                 
		goto set_error;
	}
	if(Id == 0xEEAA)
	{
		/*设置设备新的I2C地址*/
		Status = VL53L0X_SetDeviceAddress(dev,FinalAddress);
		if(Status!=VL53L0X_ERROR_NONE) 
		{
       /*设置I2C地址出错*/
			sta=0x03;                             
			goto set_error;
		}
		/*修改参数结构体的I2C地址*/
		dev->I2cDevAddr = FinalAddress;
		/*检查新的I2C地址读写是否正常*/
		Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
		if(Status!=VL53L0X_ERROR_NONE) 
		{
      /*新I2C地址读写出错*/
			sta=0x04;                             
			goto set_error;
		}	
	}
	set_error:
	if(Status!=VL53L0X_ERROR_NONE)
	{
    /*打印错误信息*/
		print_pal_error(Status);               
	}
//	if(sta!=0)
//	  printf("sta:0x%x\r\n",sta);
	return Status;
}


/**
  * @brief  vl53l0x复位函数
  * @param  dev:设备I2C参数结构体
  * @retval 无
  */
void vl53l0x_reset(VL53L0X_Dev_t *dev)
{
	uint8_t addr;
  
  /*保存设备原I2C地址*/
	addr = dev->I2cDevAddr; 
  /*失能VL53L0X*/
  LV_DISABLE(LV_XSH_PORT ,LV_XSH_PIN) ;
	HAL_Delay(30);
  
  /*使能VL53L0X,让传感器处于工作(I2C地址会恢复默认0X52)*/
	LV_ENABLE(LV_XSH_PORT ,LV_XSH_PIN) ; 
	HAL_Delay(30);	
  
	dev->I2cDevAddr=0x52;
  
  /*设置VL53L0X传感器原来上电前原I2C地址*/
	vl53l0x_Addr_set(dev,addr);          
	VL53L0X_DataInit(dev);	
}


/**
  * @brief  初始化vl53l0x
  * @param  dev:设备I2C参数结构体
  * @retval 状态信息
  */
VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_Dev_t *pMyDevice = dev;
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  
	
  /*Configure GPIO pin : PtPin */
  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LV_XSH_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LV_XSH_PORT, &GPIO_InitStruct);

  /*I2C地址(上电默认0x52)*/
	pMyDevice->I2cDevAddr = VL53L0X_Addr; 
  /*I2C通信模式*/
	pMyDevice->comms_type = 1; 
  /*I2C通信速率*/
	pMyDevice->comms_speed_khz = 400;                
	
  /*初始化IIC*/
	VL53L0X_i2c_init();  
  
	/*失能VL53L0X*/
	LV_DISABLE(LV_XSH_PORT, LV_XSH_PIN) ;           
	HAL_Delay(30);
  
  /*使能VL53L0X,让传感器处于工作*/
	LV_ENABLE(LV_XSH_PORT, LV_XSH_PIN);              
	HAL_Delay(30);
	
  /*设置VL53L0X传感器I2C地址*/
	vl53l0x_Addr_set(pMyDevice,0x54);                
	if(Status!=VL53L0X_ERROR_NONE) 
	  goto error;
  
  /*设备初始化*/
	Status = VL53L0X_DataInit(pMyDevice);            
	if(Status!=VL53L0X_ERROR_NONE) 
		goto error;
	HAL_Delay(2);
  
  /*获取设备ID信息*/
	Status = VL53L0X_GetDeviceInfo(pMyDevice,&vl53l0x_dev_info);
	if(Status!=VL53L0X_ERROR_NONE) goto error;
//	vl53l0x_calibration_test(&vl53l0x_dev);
  /*已校准*/
	if(Vl53l0x_data.adjustok==0xAA)                  
	  AjustOK=1;
  /*没校准	*/	
	else                                             
	  AjustOK=0;
	
	error:
	if(Status!=VL53L0X_ERROR_NONE)
	{
		print_pal_error(Status);   /*打印错误信息*/                   
		return Status;
	}
  	
	return Status;
}


/**
  * @brief  VL53L0X主测试程序
  * @param  无
  * @retval 无
  */
void vl53l0x_test(void)
{  
   /*vl53l0x初始化*/  
	 while(vl53l0x_init(&vl53l0x_dev)) 
	 {
//		 printf("初始化失败\r\n");
//     printf("请检查连接\r\n");
	 }
//	 printf("初始化成功\r\n");

	 while(1)
	 { 
      vl53l0x_general_test(&vl53l0x_dev);                 
	 }
}

/**
  * @brief  VL53L0X 测量模式配置
  * @param  dev:设备I2C参数结构体
  * @param  mode: 0:默认;1:高精度;2:长距离
  * @retval 状态信息
  */
VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev,uint8_t mode)
{
  VL53L0X_Error status = VL53L0X_ERROR_NONE;
  uint8_t VhvSettings;
  uint8_t PhaseCal;
  uint32_t refSpadCount;
  uint8_t isApertureSpads;

  /*复位vl53l0x(频繁切换工作模式容易导致采集距离数据不准，需加上这一代码)*/
  vl53l0x_reset(dev);                                                                                                
  status = VL53L0X_StaticInit(dev);

  /*已校准好了,写入校准值*/
  if(AjustOK!=0)                                                                                                     
  {
    /*设定Spads校准值*/
    status= VL53L0X_SetReferenceSpads(dev,Vl53l0x_data.refSpadCount,
                                      Vl53l0x_data.isApertureSpads);                  
    if(status!=VL53L0X_ERROR_NONE) goto error;	
    HAL_Delay(2);
    
    /*设定Ref校准值*/
    status= VL53L0X_SetRefCalibration(dev,Vl53l0x_data.VhvSettings,
                                      Vl53l0x_data.PhaseCal);                          
    if(status!=VL53L0X_ERROR_NONE) goto error;
    HAL_Delay(2);
    
    /*设定偏移校准值*/
    status=VL53L0X_SetOffsetCalibrationDataMicroMeter(dev,
                                      Vl53l0x_data.OffsetMicroMeter);                           
    if(status!=VL53L0X_ERROR_NONE) goto error; 
    HAL_Delay(2);
    
    /*设定串扰校准值*/
    status=VL53L0X_SetXTalkCompensationRateMegaCps(dev,
                              Vl53l0x_data.XTalkCompensationRateMegaCps);                  
    if(status!=VL53L0X_ERROR_NONE) goto error;
    HAL_Delay(2);		  
  }
  else
  {
    /*Ref参考校准*/
    status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);                                           
    if(status!=VL53L0X_ERROR_NONE) goto error;
    HAL_Delay(2);
    
    /*执行参考SPAD管理*/
    status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount,
                                                &isApertureSpads);                                
    if(status!=VL53L0X_ERROR_NONE) goto error;
    HAL_Delay(2);		 	 
  }
  
  /*使能单次测量模式*/
  status = VL53L0X_SetDeviceMode(dev,VL53L0X_DEVICEMODE_SINGLE_RANGING);                                             
  if(status!=VL53L0X_ERROR_NONE) goto error;
  HAL_Delay(2);
  
  /*使能SIGMA范围检查*/
  status = VL53L0X_SetLimitCheckEnable(dev,
                                       VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                       1);                                 
  if(status!=VL53L0X_ERROR_NONE) goto error;
  HAL_Delay(2);
  
   /*使能信号速率范围检查*/
  status = VL53L0X_SetLimitCheckEnable(dev,
                                       VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                       1);                          
  if(status!=VL53L0X_ERROR_NONE) goto error;
  HAL_Delay(2);
  
  /*设定SIGMA范围*/
  status = VL53L0X_SetLimitCheckValue(dev,
                                     VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                     Mode_data[mode].sigmaLimit);         
  if(status!=VL53L0X_ERROR_NONE) goto error;
  HAL_Delay(2);
  
  /*设定信号速率范围范围*/
  status = VL53L0X_SetLimitCheckValue(dev,
                                      VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                      Mode_data[mode].signalLimit);  
  if(status!=VL53L0X_ERROR_NONE) goto error;
  HAL_Delay(2);
  
  /*设定完整测距最长时间*/
  status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev,
                                              Mode_data[mode].timingBudget);                         
  if(status!=VL53L0X_ERROR_NONE) goto error;
  HAL_Delay(2);
  
  /*设定VCSEL脉冲周期*/
  status = VL53L0X_SetVcselPulsePeriod(dev, 
                                      VL53L0X_VCSEL_PERIOD_PRE_RANGE, 
                                      Mode_data[mode].preRangeVcselPeriod);   
  if(status!=VL53L0X_ERROR_NONE) goto error;
  HAL_Delay(2);
  
  /*设定VCSEL脉冲周期范围*/
  status = VL53L0X_SetVcselPulsePeriod(dev,
                                       VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 
                                       Mode_data[mode].finalRangeVcselPeriod);

  error:/*错误信息*/
  if(status!=VL53L0X_ERROR_NONE)
  {
    print_pal_error(status);
    return status;
  }
  return status;	
}	

/**
  * @brief  VL53L0X 单次距离测量函数
  * @param  dev:设备I2C参数结构体
  * @param  pdata:保存测量数据结构体
  * @retval 状态信息
  */
VL53L0X_Error vl53l0x_start_single_test(VL53L0X_Dev_t *dev,
                                        VL53L0X_RangingMeasurementData_t *pdata,
                                        char *buf)
{
  VL53L0X_Error status = VL53L0X_ERROR_NONE;
  uint8_t RangeStatus;

  /*执行单次测距并获取测距测量数据*/
  status = VL53L0X_PerformSingleRangingMeasurement(dev, pdata);   
  if(status !=VL53L0X_ERROR_NONE) return status;
  
  /*获取当前测量状态*/
  RangeStatus = pdata->RangeStatus;                              
  memset(buf,0x00,VL53L0X_MAX_STRING_LENGTH);
  /*根据测量状态读取状态字符串*/
  VL53L0X_GetRangeStatusString(RangeStatus,buf);                 
  /*保存最近一次测距测量数据*/
  Distance_data = pdata->RangeMilliMeter;                         

  return status;
}


/**
  * @brief  启动普通测量
  * @param  dev：设备I2C参数结构体
  * @param  mode模式配置 0:默认;1:高精度;2:长距离
  * @retval 无
  */
void vl53l0x_general_start(VL53L0X_Dev_t *dev,uint8_t mode)
{
  /*测试模式字符串字符缓冲区*/
  static char buf[VL53L0X_MAX_STRING_LENGTH]; 
  /*工作状态*/
	VL53L0X_Error Status=VL53L0X_ERROR_NONE;                       
	uint8_t i=0;
	
  /*配置测量模式*/
	while(vl53l0x_set_mode(dev,mode))                              
	{
//		printf("模式设置失败!!!\r\n");
		HAL_Delay(500);
		i++;
		if(i==2)
		{
		  return;
		}
	}
//  printf("模式设置成功!!!\r\n");
  /*执行一次测量	*/
	if(Status == vl53l0x_start_single_test(dev,&vl53l0x_data,buf))
	{
    /*打印测量距离*/
 //   printf("测量距离d: %4imm\r\n",Distance_data);               
  } 
}


/**
  * @brief  工作模式信息函数
  * @param  无
  * @retval 无
  */
void Show_GenTask_Message(void)
{
//  printf("   指令   ------      功能 \r\n");
//  printf("    0     ------    默认测量模式 \r\n");
//  printf("    1     ------    高精度测量模式 \r\n"); 
//  printf("    2     ------    长距离测量模式 \r\n");
//  printf("    3     ------    高速测量模式 \r\n"); 
//  printf("***输入数字指令后，按回车键再发送*** \r\n");    
}

void vl5320x_ble_masg(void)
{
	static uint8_t BUFFER_SEND_RANGE[6] ={0};
	uint32_t chksum = 0;
	BUFFER_SEND_RANGE[0] = 0xAA;
	BUFFER_SEND_RANGE[1] = 0xAA;
	BUFFER_SEND_RANGE[2] = 0x02; //长度
	BUFFER_SEND_RANGE[3] = Distance_data;
	BUFFER_SEND_RANGE[4] = Distance_data >> 8;
	for (int i = 0; i < 0x02; i++) {
		chksum += BUFFER_SEND_RANGE[3+i];
	}
	BUFFER_SEND_RANGE[5] = (~((uint8_t)chksum));
}
/**
  * @brief  vl53l0x普通测量模式测试
  * @param  dev:设备I2C参数结构体
  * @retval 无
  */
void vl53l0x_general_test(VL53L0X_Dev_t *dev)
{
  uint32_t   ch;
  
  /*显示普通测量模式*/
	Show_GenTask_Message();   
	
	while(1)
	{	
 //   scanf("%d",&ch);
//    printf("接收到字符：%d\r\n",ch);
  
    if(ch<=3)
	{
      vl53l0x_general_start(dev,ch);
      Show_GenTask_Message();
    }		
    else  
    {
      /*如果不是指定指令字符，打印提示信息*/
//      printf("请输入合法指令！\r\n"); 
      Show_GenTask_Message();      
    }
  }
}
#define adjust_num 5//校准错误次数

//VL53L0X校准函数
//dev:设备I2C参数结构体
VL53L0X_Error vl53l0x_adjust(VL53L0X_Dev_t *dev) {

	VL53L0X_DeviceError Status = VL53L0X_ERROR_NONE;
	uint32_t refSpadCount = 7;
	uint8_t isApertureSpads = 0;
	uint32_t XTalkCalDistance = 100;
	uint32_t XTalkCompensationRateMegaCps;
	uint32_t CalDistanceMilliMeter = 100;		//100 << 16;
	int32_t OffsetMicroMeter = 0;				// 30000;
	uint8_t VhvSettings = 23;
	uint8_t PhaseCal = 1;
	uint8_t i = 0;

	VL53L0X_StaticInit(dev);   //数值恢复默认,传感器处于空闲状态
	//SPADS校准----------------------------
	spads: HAL_Delay(10);
	printf("The SPADS Calibration Start...\r\n");
	Status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount,
			&isApertureSpads);	//执行参考Spad管理
	if (Status == VL53L0X_ERROR_NONE) {
//		printf("refSpadCount = %d\r\n", refSpadCount);
		Vl53l0x_adjust.refSpadCount = refSpadCount;
		printf("isApertureSpads = %d\r\n", isApertureSpads);
		Vl53l0x_adjust.isApertureSpads = isApertureSpads;
//		printf("The SPADS Calibration Finish...\r\n\r\n");
		i = 0;
	} else {
		i++;
		if (i == adjust_num)
			return Status;
//		printf("SPADS Calibration Error,Restart this step\r\n");
		goto spads;
	}
	//设备参考校准---------------------------------------------------
	ref: HAL_Delay(10);
//	printf("The Ref Calibration Start...\r\n");
	Status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);//Ref参考校准
	if (Status == VL53L0X_ERROR_NONE) {
//		printf("VhvSettings = %d\r\n", VhvSettings);
		Vl53l0x_adjust.VhvSettings = VhvSettings;
//		printf("PhaseCal = %d\r\n", PhaseCal);
		Vl53l0x_adjust.PhaseCal = PhaseCal;
//		printf("The Ref Calibration Finish...\r\n\r\n");
		i = 0;
	} else {
		i++;
		if (i == adjust_num)
			return Status;
//		printf("Ref Calibration Error,Restart this step\r\n");
		goto ref;
	}
	//偏移校准------------------------------------------------
	offset: HAL_Delay(10);
//	printf(
//			"Offset Calibration:need a white target,in black space,and the distance keep 100mm!\r\n");
//	printf("The Offset Calibration Start...\r\n");

	Status = VL53L0X_PerformOffsetCalibration(dev, CalDistanceMilliMeter,
			&OffsetMicroMeter);	//偏移校准
	if (Status == VL53L0X_ERROR_NONE) {
//		printf("CalDistanceMilliMeter = %d mm\r\n", CalDistanceMilliMeter);
		Vl53l0x_adjust.CalDistanceMilliMeter = CalDistanceMilliMeter;
//		printf("OffsetMicroMeter = %d um\r\n", OffsetMicroMeter);
		Vl53l0x_adjust.OffsetMicroMeter = OffsetMicroMeter;
		printf("The Offset Calibration Finish...\r\n\r\n");
		i = 0;
	} else {
		i++;
		if (i == adjust_num)
			return Status;
//printf("Offset Calibration Error,Restart this step\r\n");
		goto offset;
	}
	//串扰校准-----------------------------------------------------
	xtalk: HAL_Delay(20);
//	printf("Cross Talk Calibration:need a grey target\r\n");
//	printf("The Cross Talk Calibration Start...\r\n");
	Status = VL53L0X_PerformXTalkCalibration(dev, XTalkCalDistance,
			&XTalkCompensationRateMegaCps);	//串扰校准
	if (Status == VL53L0X_ERROR_NONE) {
	//	printf("XTalkCalDistance = %d mm\r\n", XTalkCalDistance);
		Vl53l0x_adjust.XTalkCalDistance = XTalkCalDistance;
//		printf("XTalkCompensationRateMegaCps = %d\r\n",
//				XTalkCompensationRateMegaCps);
		Vl53l0x_adjust.XTalkCompensationRateMegaCps =
				XTalkCompensationRateMegaCps;
//		printf("The Cross Talk Calibration Finish...\r\n\r\n");
		i = 0;
	} else {
		i++;
		if (i == adjust_num)
			return Status;
//		printf("Cross Talk Calibration Error,Restart this step\r\n");
		goto xtalk;
	}
//	printf("All the Calibration has Finished!\r\n");
//	printf("Calibration is successful!!\r\n");

	Vl53l0x_adjust.adjustok = 0xAA;	//校准成功
	// 后面再做校准数据保存
	//AT24CXX_Write(0,(uint8_t*)&Vl53l0x_adjust,sizeof(_vl53l0x_adjust));//将校准数据保存到24c02
	//---------------测试代码
//	Vl53l0x_adjust.adjustok = 0xAA;
//	Vl53l0x_adjust.isApertureSpads = 0x11;
//	Vl53l0x_adjust.VhvSettings = 0x22;
//	Vl53l0x_adjust.PhaseCal = 0x33;
//	Vl53l0x_adjust.XTalkCalDistance = 0x44;
//	Vl53l0x_adjust.XTalkCompensationRateMegaCps = 0x55;
//	Vl53l0x_adjust.CalDistanceMilliMeter = 0x66;
//	Vl53l0x_adjust.OffsetMicroMeter = 0x77;
//	Vl53l0x_adjust.refSpadCount = 0x88;
	//-----------------------
//	Flash_Write(&Vl53l0x_adjust, 24);  // 数据保存到FLASH、
	memcpy(&Vl53l0x_data, &Vl53l0x_adjust, sizeof(Vl53l0x_adjust));//将校准数据复制到Vl53l0x_data结构体
	return Status;
}

//vl53l0x校准测试
//dev:设备I2C参数结构体
void vl53l0x_calibration_test(VL53L0X_Dev_t *dev)
{
	VL53L0X_Error status = VL53L0X_ERROR_NONE;
//	uint8_t key = 0;
//	uint8_t i = 0;

	status = vl53l0x_adjust(dev);	//进入校准
	if (status != VL53L0X_ERROR_NONE)	//校准失败
	{
//		printf("Calibration is error!!\r\n");
	} else
//		printf("Calibration is complete!");
	HAL_Delay(500);
}

/*********************************************END OF FILE**********************/
