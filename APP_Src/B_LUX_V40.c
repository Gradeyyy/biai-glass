/*
 * ICM_20608.c
 *
 *  Created on: 4, 5, 2025
 *      Author: lan
 */

//***************************************
// B_LUX_V40采集程序
//****************************************
#include "B_LUX_V40.h"
#include "ICM_20608.h"


/*---------------------------------------------------------------------
 功能描述: 毫秒 不同的工作环境,需要调整此函数
 参数说明: 无	
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_40_delay_ms(uint16_t k)	
{						
		HAL_Delay(k);
}					

/*---------------------------------------------------------------------
 功能描述: 延时5微秒  不同的工作环境,需要调整此函数
 参数说明: 无	
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_40_Delay5us()
{
  delay_us(5);
}

/*---------------------------------------------------------------------
 功能描述: 延时5毫秒  不同的工作环境,需要调整此函数
 参数说明: 无	
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_40_Delay5ms()
{
  HAL_Delay(5);
}

///*---------------------------------------------------------------------
// 功能描述: 起始信号
// 参数说明: 无	
// 函数返回: 无
// ---------------------------------------------------------------------*/
//void B_LUX_40_Start()
//{
//  B_LUX_40_SDA0_H;                         //拉高数据线
//  B_LUX_40_SCL0_H;                         //拉高时钟线
//  B_LUX_40_Delay5us();                     //延时
//  B_LUX_40_SDA0_L;                         //产生下降沿
//  B_LUX_40_Delay5us();                     //延时
//  B_LUX_40_SCL0_L;                         //拉低时钟线
//}

///*---------------------------------------------------------------------
// 功能描述: 停止信号
// 参数说明: 无	
// 函数返回: 无
// ---------------------------------------------------------------------*/
//void B_LUX_40_Stop()
//{
//  B_LUX_40_SDA0_L;                         	//拉低数据线
//  B_LUX_40_SCL0_H;                         	//拉高时钟线
//  B_LUX_40_Delay5us();                     	//延时
//  B_LUX_40_SDA0_H;                         	//产生上升沿
//  B_LUX_40_Delay5us();                     	//延时
//  B_LUX_40_SCL0_L;
//  B_LUX_40_Delay5us();
//}

///*---------------------------------------------------------------------
// 功能描述: 发送应答信号
// 参数说明: ack - 应答信号(0:ACK 1:NAK)
// 函数返回: 无
// ---------------------------------------------------------------------*/
//void B_LUX_40_SendACK(uint8 ack)
//{
//  if (ack&0x01)	B_LUX_40_SDA0_H;						//写应答信号
//  else	B_LUX_40_SDA0_L;
//  
//  B_LUX_40_SCL0_H;                         	//拉高时钟线
//  B_LUX_40_Delay5us();                     	//延时
//  B_LUX_40_SCL0_L;                         	//拉低时钟线
//  //B_LUX_40_Delay5us();
//  //B_LUX_40_SDA0_H;
//  //B_LUX_40_Delay5us();                     	//延时
//}

///*---------------------------------------------------------------------
// 功能描述: 接收应答信号
// 参数说明: 无
// 函数返回: 返回应答信号
// ---------------------------------------------------------------------*/
//uint8 B_LUX_40_RecvACK()
//{
//  uint8 vCy = 0x00;
//  B_LUX_40_SDA0_H;
//  
//  B_LUX_40_SDA0_I;
//  
//  B_LUX_40_SCL0_H;                         //拉高时钟线
//  B_LUX_40_Delay5us();                     //延时
//  
//  
//  vCy |= B_LUX_40_SDA0_DAT;                 //读应答信号
//  
//  B_LUX_40_Delay5us();                     //延时
//  
//  B_LUX_40_SCL0_L;                         //拉低时钟线
//  
//  B_LUX_40_SDA0_O;
//  
//  return vCy;
//}

///*---------------------------------------------------------------------
// 功能描述: 向IIC总线发送一个字节数据
// 参数说明: dat - 写字节
// 函数返回: 无
// ---------------------------------------------------------------------*/
//uint8 B_LUX_40_SendByte(uint8 vDat)
//{
//  uint8 i;
//  
//  for (i=0; i<8; i++)         					//8位计数器
//  {
//    if (vDat&0x80)	B_LUX_40_SDA0_H;
//    else	B_LUX_40_SDA0_L;              //送数据口
//    
//    B_LUX_40_Delay5us();             		//延时
//    B_LUX_40_SCL0_H;                		//拉高时钟线
//    B_LUX_40_Delay5us();             		//延时
//    B_LUX_40_SCL0_L;                		//拉低时钟线
//    //B_LUX_40_Delay5us();             		//延时
//    vDat <<= 1;              						//移出数据的最高位
//  }
//  
//  return B_LUX_40_RecvACK();
//}

///*---------------------------------------------------------------------
// 功能描述: 从IIC总线接收一个字节数据
// 参数说明: 无
// 函数返回: 接收字节
// ---------------------------------------------------------------------*/
//uint8 B_LUX_40_RecvByte()
//{
//  uint8 i;
//  uint8 vDat = 0;
//  B_LUX_40_SDA0_I;
//  
//  B_LUX_40_SDA0_H;                         	//使能内部上拉,准备读取数据,
//  for (i=0; i<8; i++)         	        		//8位计数器
//  {
//    
//    
//    B_LUX_40_SCL0_H;                       	//拉高时钟线
//    B_LUX_40_Delay5us();             				//延时
//    vDat |= B_LUX_40_SDA0_DAT;              	//读数据               
//    B_LUX_40_SCL0_L;                       	//拉低时钟线
//    B_LUX_40_Delay5us();             				//延时
//    
//    if (i<7)
//      vDat <<= 1;
//    	
//  }
//  B_LUX_40_SDA0_O;
//  
//  return vDat;
//}





/*---------------------------------------------------------------------
 功能描述: 写OPT3001
 参数说明: REG_Address - 寄存器地址
				   REG_data - 写寄存器
 函数返回: 无
 ---------------------------------------------------------------------*/
uint8_t  	B_LUX_40_Write(uint8_t vRegAddress, uint16_t vRegData)
{
	uint8_t vRval = 0;
	uint8_t temp_data=0;
	HAL_I2C_Mem_Write(&hi2c2, vRegAddress, B_LUX_40_SlaveAddress, I2C_MEMADD_SIZE_8BIT, &vRegData, 1, 100);
//  B_LUX_40_Start();                                        //起始信号
//  icm20608_write_reg(B_LUX_40_SlaveAddress,vRval);       //发送设备地址+写信号
//	temp_data += vRval;
//  icm20608_write_reg(vRegAddress,vRval);
//	temp_data += vRval;	//内部寄存器地址
//  icm20608_write_reg( (vRegData>>8)&0xFF,vRval);        //内部寄存器数据
//	temp_data += vRval;	
//	icm20608_write_reg( vRegData&0xFF,vRval);             //内部寄存器数据
//	temp_data += vRval;
//  B_LUX_40_Stop();                                         //停止信号
	
	return temp_data;
}

/*---------------------------------------------------------------------
 功能描述: 读OPT3001内部数据
 参数说明: vRegAddress - 寄存器地址
					 vp_RegData - 寄存器数据指针
 函数返回: 读取的寄存器数据
 ---------------------------------------------------------------------*/
uint8_t  	B_LUX_40_read(uint8_t vRegAddress, uint16_t *vp_RegData)
{  
  uint8_t vRval = 0;
	uint8_t tdata_buffer[2] = {0};
	HAL_I2C_Mem_Read(&hi2c2, B_LUX_40_SlaveAddress, vRegAddress, I2C_MEMADD_SIZE_8BIT,&tdata_buffer[0], I2C_MEMADD_SIZE_16BIT, 100);
	HAL_I2C_Mem_Read(&hi2c2, B_LUX_40_SlaveAddress, vRegAddress+1, I2C_MEMADD_SIZE_8BIT,&tdata_buffer[1], I2C_MEMADD_SIZE_16BIT, 100);
	*vp_RegData = *(uint16_t *)tdata_buffer;
	//  B_LUX_40_Start();                                        //起始信号
//	icm20608_write_reg(B_LUX_40_SlaveAddress,vRval);       //发送设备地址+写信号
//	temp_data += vRval;
//		icm20608_write_reg(vRegAddress,vRval);       //发送设备地址+写信号
//	temp_data += vRval;

//  vRval += B_LUX_40_SendByte(B_LUX_40_SlaveAddress+0);     //发送设备地址+读信号
//  vRval += B_LUX_40_SendByte(vRegAddress);                 //内部寄存器地址
//	B_LUX_40_Stop();
//	
//  B_LUX_40_Start();                                        //起始信号
//	icm20608_write_reg(vRegAddress+1,vRval);       //发送设备地址+写信号
//	temp_data += vRval;
//			icm20608_write_reg(vRegAddress,vRval);       //发送设备地址+写信号
//	temp_data += vRval;
//	*vp_RegData = icm20608_read_reg();
//  vRval += B_LUX_40_SendByte(B_LUX_40_SlaveAddress+1);     //发送设备地址+读信号
//  *vp_RegData  = B_LUX_40_RecvByte();                      //接收数据
//	B_LUX_40_SendACK(0);                                     //回应ACK
//	*vp_RegData <<= 8;
//	*vp_RegData |= B_LUX_40_RecvByte();                              //接收数据
//  B_LUX_40_SendACK(1);                                     //回应ACK
//  B_LUX_40_Stop();                                         //停止信号
  
  return vRval;
  
}

/*---------------------------------------------------------------------
 功能描述: 传感器寄存器配置函数
 参数说明: vCfg - 配置参数
 函数返回: 无
 ---------------------------------------------------------------------*/
uint8_t  	B_LUX_40_RegCfg(void)
{
	uint16_t vCfg = 0;
	uint8_t vRval = 0;
	
	//12:15 RN  		- 配置测量光照的范围 见手册20页表9  当配置位1100传感器测量范围自动选择
	//11    CT  		- 测量时间配置 0- 100Ms  1-800Ms
	//10:9  M[1:0]	- 转换模式 00-关闭模式  01 - 单次转换  10、11 - 连续多次转换
	//8     OVF     - 测量光照超出设定的范围或最大测量值 溢出标志
	//7     CRF			- 转换就绪字段 1-转换完成
	//6     FH			- 转换的光照值 大于上限值 置位
	//5     FL			- 转换的光照值 小于下限值 置位
	//4     L				- 中断输出的两种模式  1-窗口模式 这种模式下高限置位和低限置位INT输出  0-滞后模式 高限置位INT输出 具体看手册
	//3     POL			- INT 中断被触发输出极性 0-拉低  1-拉高
	//2     ME 			- 掩码字段
	//0:1   FC			- 超出上限范围故障计数  如果超出次数 大于等于计数设定次数  INT输出中断
	
	vCfg = (0x0C<<12);
	vCfg |= (0x01<<11);
	vCfg |= (0x01<<9);
	vCfg |= (0x01<<4);
	vRval = B_LUX_40_Write(REG_CONFIGURATION, vCfg);
	
	return vRval;
}

/*---------------------------------------------------------------------
 功能描述: 初始化光照传感器
 参数说明: 无
 函数返回: 无
 ---------------------------------------------------------------------*/
uint8_t B_LUX_40_Init()
{
	uint8_t vRval = 0;
	

  B_LUX_40_delay_ms(5);	                                          //延时100ms
  vRval = B_LUX_40_RegCfg();
	B_LUX_40_delay_ms(5);	                                          //延时100ms
  
	return vRval;
}

/*---------------------------------------------------------------------
 功能描述: 光照读取函数
 参数说明: vp_Lux - 光照数据存储指针
 函数返回: 0-成功  >0 -失败
 ---------------------------------------------------------------------*/
uint8_t 	B_LUX_40_GetManufacturerID(uint16_t *vp_ManufacturerID)
{
	uint8_t vRval = 0;
	
	vRval = B_LUX_40_read(REG_MANUFACTURER_ID, vp_ManufacturerID);
	
	return vRval;
}

uint8_t 	B_LUX_40_GetDeviceID(uint16_t *vp_DeviceID)
{
	uint8_t vRval = 0;
	
	vRval = B_LUX_40_read(REG_DEVICE_ID, vp_DeviceID);
	
	return vRval;
}
  
/*---------------------------------------------------------------------
 功能描述: 光照读取函数
 参数说明: vp_Lux - 光照数据存储指针
 函数返回: 0-成功  >0 -失败
 ---------------------------------------------------------------------*/
uint8_t		B_LUX_40_GetLux(uint32_t *vp_Lux)
{
	uint8_t 	vRval = 0;
	uint16_t  vCfg = 0;
	uint16_t  vDat = 0;
	
	uint16_t  vDatE = 0;
	uint16_t  vDatR = 0;
	
	float   vFval = 0.0;
	float   vLsbSize = 0.0;
	float   vFlux = 0;
	
	vRval = B_LUX_40_read(REG_CONFIGURATION, &vCfg);
  vCfg |= (0x01<<9);
	vRval = B_LUX_40_Write(REG_CONFIGURATION, vCfg);							//单次采集光照
	
	vRval = B_LUX_40_read(REG_CONFIGURATION, &vCfg);
	B_LUX_40_delay_ms(900);																				//大于800Ms
	
	vRval = B_LUX_40_read(REG_CONFIGURATION, &vCfg);
	if  ( (vCfg&(0x01<<7)) )																			//光照采集完成
	{
		vRval = B_LUX_40_read(REG_RESULT, &vDat);
		
		vDatE = ((vDat&0xF000)>>12);
		vDatR = (vDat&0x0FFF);
		
		vFval = (0x01<<vDatE);
		vLsbSize = (0.01 * vFval);
		
		vFlux  = (vLsbSize * (float)vDatR);
		*vp_Lux = ((vFlux)*100.0);															//透明外壳不需要矫正 ，
																														//乳白色外壳推荐*1.8矫正 *vp_Lux = ((vFlux*1.8)*100.0)
	}
	else
	{
		vRval = 0x01;  					//光照采集失败
	}
	
	return vRval;
} 

