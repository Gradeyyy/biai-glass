#ifndef __LT8960Ldrv_H
#define __LT8960Ldrv_H


#include "main.h"

//LT8960L硬件脚位连接和控制 PB8-scl  PB7-sda
//CLK输出1/0   DAT输出1/0    DAT电平状态读取  DAT输入输出方向
///////////////////////////////////////////////////////////////////////////////
#define LT8960L_CLK_H           GPIOB->BSRR = GPIO_PIN_8
#define LT8960L_CLK_L           GPIOB->BRR = GPIO_PIN_8

#define LT8960L_DAT_H           GPIOB->BSRR = GPIO_PIN_7
#define LT8960L_DAT_L           GPIOB->BRR = GPIO_PIN_7
#define LT8960L_DAT_read        HAL_GPIO_ReadPin(GPIOB ,GPIO_PIN_7) // (GPIOA->IDR  & GPIO_PIN_12)

#define LT8960L_DAT_Input()			LT8960L_DAT_InputMode()
#define LT8960L_DAT_Output()		LT8960L_DAT_OutputMode()

//#define Slow_Clock	//LT8960L与MCU低速通讯



///////////////////////////////////////////////////////////////////////////////
//                  根据用户应用，以下部分可能需要修改                       //
///////////////////////////////////////////////////////////////////////////////
//发射功率设置
#define RF_Power					LT8960L_7dBm

//通讯频点
#define RadioFrequency_user	2405
#define RadioFrequency_user2	2450  // ble
#define Test_Channel	RadioFrequency_user-2402
#define Test_Channel2	RadioFrequency_user2-2402   // ble

#define RadioFrequency_1    2476
#define RadioFrequency_2    2459
#define RadioFrequency_3    2467

#define USER_Channel1		RadioFrequency_1-2402
#define USER_Channel2		RadioFrequency_2-2402
#define USER_Channel3		RadioFrequency_3-2402


//发包长度 最大63Byte
#define Packet_Length		12   //Byte

//发包间隔
#define Tx_Interval_mS	10	 //mS
#define Tx_Interval_mS_ble	30	 //mS       ble 30ms

//模式设置
#define Work_Type		4	//1=Tx 0=Rx 2=单载波 3=休眠与唤醒 4=蓝牙发射 5=蓝牙接收

//速率设置
#define Air_rate_1M        // ble
//#define Air_rate_250K
//#define Air_rate_125K
//#define Air_rate_62K5        // 2.4G

//通讯同步字
#define SyncWord	0x03,0x80,0x5A,0x5A
#define SyncWord2 	0xB8,0x1B,0x20,0x5A








//*************************************
//变量与函数声明部分 不建议修改

#define LT8960L_7dBm			0x7830
#define LT8960L_6dBm			0X7930
#define LT8960L_5dBm			0X7A30
#define LT8960L_3_4dBm			0X7B30
#define LT8960L_0_2dBm			0X7C30
#define LT8960L_n1_5dBm			0X7D30
#define LT8960L_n4dBm		  	0X7E30
#define LT8960L_n7dBm		  	0X7F30
#define LT8960L_n9dBm		  	0X3F30
#define LT8960L_n13dBm	  		0X3FB0
#define LT8960L_n19dBm	  		0X3FC0

#define UART_BaudRate	115200
#define LT890L_DIVICE_SEND_BUFFER_LEN 0x40
//typedef struct
//{
//	uint8_t ble_flg;
//	uint8_t _2_4g_flg;
//	uint8_t result_current;
//	uint8_t result_previous;
//	uint8_t *send_buffer;
//}LT8960L_DIVICE_STRUCT;

//extern volatile LT8960L_DIVICE_STRUCT lt8960l_divice_sturct;



extern unsigned char LT8960L_RegH,LT8960L_RegL;

void LT8960L_WriteReg(unsigned char reg, unsigned char H, unsigned char L);
void LT8960L_ReadReg(unsigned char reg);
void LT8960L_WriteBUF(unsigned char reg, unsigned char *pBuf, unsigned char len);
unsigned char LT8960L_ReadBUF(unsigned char reg, unsigned char *pBuf);

unsigned char LT8960L_GetPKT(void);
void LT8960L_INIT(uint8_t sync);
void LT8960L_BLE_INIT(void);
void LT8960L_Change_0x38(void);

void LT8960L_Carrier_Wave(unsigned char FreqChannel);
void LT8960L_Sleep(void);
void LT8960L_Wakeup(void);

unsigned char LT8960L_OpenRx(unsigned char FreqChannel);

void LT8960L_TxData(unsigned char FreqChannel,unsigned char *pBuf,unsigned char length);
unsigned char LT8960L_RxData(unsigned char FreqChannel,unsigned char *pBuf,unsigned char length);

void LT8960L_Tx_BLE_Data(unsigned char *pBuf,unsigned char length,unsigned char ChannelEnable);
unsigned char LT8960L_Rx_BLE_Data(unsigned char *pBuf,unsigned char length,unsigned char ChannelEnable);
#endif
