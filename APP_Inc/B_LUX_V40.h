#ifndef __B_LUX_V40_H
#define __B_LUX_V40_H

/*--------------------------头文件引用--------------------------------*/
//#include "stm32g4xx.h"
#include "main.h"

/*-----------------------------结构体定义-----------------------------*/
 
															
/*-----------------------------宏定义---------------------------------*/
//引脚定义
//#define B_LUX_40_SCL0_O    {\
//															GPIO_InitTypeDef  GPIO_ST; \
//															GPIO_ST.GPIO_Pin = GPIO_Pin_10;\
//															GPIO_ST.GPIO_Mode = GPIO_Mode_Out_OD; \
//															GPIO_ST.GPIO_Speed = GPIO_Speed_50MHz;\
//															GPIO_Init(GPIOB, &GPIO_ST); }									//GPIOB10 开漏输出
//#define B_LUX_40_SCL0_H    GPIO_SetBits(GPIOB, GPIO_Pin_10)
//#define B_LUX_40_SCL0_L    GPIO_ResetBits(GPIOB, GPIO_Pin_10)

//#define B_LUX_40_SCL0_I    {\
//															GPIO_InitTypeDef  GPIO_ST; \
//															GPIO_ST.GPIO_Pin = GPIO_Pin_10;\
//															GPIO_ST.GPIO_Mode = GPIO_Mode_IN_FLOATING; \
//															GPIO_ST.GPIO_Speed = GPIO_Speed_50MHz;\
//															GPIO_Init(GPIOB, &GPIO_ST); }									//GPIOB10 浮空输入
//#define B_LUX_40_SCL0_DAT  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)

//#define B_LUX_40_SDA0_O    {\
//															GPIO_InitTypeDef  GPIO_ST; \
//															GPIO_ST.GPIO_Pin = GPIO_Pin_11;\
//															GPIO_ST.GPIO_Mode = GPIO_Mode_Out_OD; \
//															GPIO_ST.GPIO_Speed = GPIO_Speed_50MHz;\
//															GPIO_Init(GPIOB, &GPIO_ST); }    							//GPIOB11 开漏输出
//#define B_LUX_40_SDA0_H    GPIO_SetBits(GPIOB, GPIO_Pin_11)
//#define B_LUX_40_SDA0_L    GPIO_ResetBits(GPIOB, GPIO_Pin_11)

//#define B_LUX_40_SDA0_I    {\
//															GPIO_InitTypeDef  GPIO_ST; \
//															GPIO_ST.GPIO_Pin = GPIO_Pin_11;\
//															GPIO_ST.GPIO_Mode = GPIO_Mode_IN_FLOATING; \
//															GPIO_ST.GPIO_Speed = GPIO_Speed_50MHz;\
//															GPIO_Init(GPIOB, &GPIO_ST); }    							//GPIOB11 浮空输入
//#define B_LUX_40_SDA0_DAT  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)

//#define B_LUX_40_ADDR_O    {\
//															GPIO_InitTypeDef  GPIO_ST; \
//															GPIO_ST.GPIO_Pin = GPIO_Pin_13;\
//															GPIO_ST.GPIO_Mode = GPIO_Mode_Out_PP; \
//															GPIO_ST.GPIO_Speed = GPIO_Speed_50MHz;\
//															GPIO_Init(GPIOC, &GPIO_ST); }    							//GPIOC13 推免输出
//#define B_LUX_40_ADDR_H    GPIO_SetBits(GPIOC, GPIO_Pin_13)
//#define B_LUX_40_ADDR_L    GPIO_ResetBits(GPIOC, GPIO_Pin_13)

//#define B_LUX_40_INT0_I    {\
//															GPIO_InitTypeDef  GPIO_ST; \
//															GPIO_ST.GPIO_Pin = GPIO_Pin_5;\
//															GPIO_ST.GPIO_Mode = GPIO_Mode_IN_FLOATING; \
//															GPIO_ST.GPIO_Speed = GPIO_Speed_50MHz;\
//															GPIO_Init(GPIOB, &GPIO_ST); }    							//GPIOB5 浮空输入
//#define B_LUX_40_INT0_DAT  GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)


#define	B_LUX_40_SlaveAddress	  		0x88                                                  //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改 拉低0x88  拉高0x8A  接SDA-0x8C 接SCL-0x8E


#define REG_RESULT									0x00						//光照结果
#define REG_CONFIGURATION						0x01						//配置寄存器
#define REG_LOW_LIMIT								0x02						//下限寄存器
#define REG_HIGH_LIMIT							0x03						//上限寄存器
#define REG_MANUFACTURER_ID					0x7E						//厂商编号
#define REG_DEVICE_ID								0x7F						//设备ID

/*-----------------------------函数声明-------------------------------*/
void   	B_LUX_40_delay_ms(uint16_t k);
//uint8_t    B_LUX_40_Init(vid);

uint8_t  	B_LUX_40_Write(uint8_t vRegAddress, uint16_t vRegData);                            		 //写入寄存器数据
uint8_t  	B_LUX_40_read(uint8_t vRegAddress, uint16_t *vp_RegData);                              //读取寄存器数据

uint8_t  	B_LUX_40_RegCfg(void);																										 			 //配置寄存器数据
uint8_t		B_LUX_40_GetLux(uint32_t *vp_Lux);																								 //采集光照数据
uint8_t 	B_LUX_40_GetManufacturerID(uint16_t *vp_ManufacturerID);													 //读取ManufacturerID
uint8_t 	B_LUX_40_GetDeviceID(uint16_t *vp_DeviceID);																			 //读取DeviceID

//------------------------------------
void    B_LUX_40_Delay5us(void);
void    B_LUX_40_Delay5ms(void);
//void    B_LUX_40_Start(void);                                                                 //起始信号
//void    B_LUX_40_Stop(void);                                                                  //停止信号
//void   	B_LUX_40_SendACK(uint8_t ack);                                                         //应答ACK
//uint8_t  	B_LUX_40_RecvACK(void);                                                               //读ack
//uint8_t   B_LUX_40_SendByte(uint8_t dat);                                                        //IIC单个字节写
//uint8_t  	B_LUX_40_RecvByte(void);                                                              //IIC单个字节读

#endif
