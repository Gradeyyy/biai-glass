/*
 * taget.h
 *
 *  Created on: 2025.4.8
 *      Author: lan
 */
#ifndef _TAGT_H_
#define _TAGT_H_

#include "stm32g4xx.h"
#include "algorithm.h"
#include "LT8960Ldrv.h"
#include "work_drive.h"
#include "vl53l0x_def.h"
#include "key_scan.h"
#define EEG_UART_HAND_FRAME 	0xAAAA 
// neurosky
#define  RAW_EEG_LENGTH    	  0x04
#define  RES_EEG_LENGTH    	  0x20
#define  RES2_EEG_LENGTH   	  0x15

#define CMD_PPG_LENGTH		 16			// PPG
#define CMD_EEG_RAW_LENGTH   8			// EEG rawdata
#define CMD_EEG_RES2_LENGTH  11			// EEG 
#define CMD_EEG_RES1_LENGTH  36			// EEG 长数据包
#define CMD_ICM20608_LENGTH  18			// ICM20608

#define CMD_LT8906L_BLE_DATA_1 0xBB
#define CMD_LT8906L_BLE_DATA_2 0xBA
//#define  RAW_ICM20608_LENGTH  14
//按键结构体
typedef struct
{	
	uint16_t uart_length; //串口接收的数据长度
	uint8_t  size  ;//串口接收的数据最大限制长度
	uint8_t uart_flag;	//数据接收完标志位	
	uint8_t *uart_receive_data_buffer; //串口接收数据
	uint8_t uart_receive_time_flg;
	uint8_t uart_receive_time_cnt;
	uint8_t eeg_attention_data;
	uint8_t eeg_attention_flg;
	uint8_t eeg_attention_time_flg;	
	uint8_t eeg_attention_time_cnt;
	uint8_t uart_send_vl53l0x_msg_flg;

}TAGT_STRUCT;

extern volatile TAGT_STRUCT  tagt_struct;
void eeg_make_result(LT8960L_DIVICE_STRUCT *plt8960l_device);
void tagt_process(TAGT_STRUCT *eeg_par,LT8960L_DIVICE_STRUCT *plt8960l_device,uint16_t opt3001_data_value,volatile uint16_t vl53l0x_distance_data);
void ble_send_cmd(uint8_t cmd);
void ble_make_result(void);
void lt8960l_ble_msg_deal(LT8960L_DIVICE_STRUCT *plt8960l_device,uint8_t ble_package_idex,uint16_t rawLux_value,uint16_t vl53l0x_distance_data);
#endif

