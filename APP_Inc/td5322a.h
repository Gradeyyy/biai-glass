/*
 * key_scan.h
 *
 *  Created on: 2025.6.5
 *      Author: lan
 */
#ifndef _TD5322A_H_
#define _TD5322A_H_

#include "stm32g4xx.h"
#include "work_drive.h"

#define CRC16_POLY 0x1021
#define CRC16_INIT 0xFFFF

#define TD5322A_BLE_MAC_LENGTH  26
#define TD5322A_BLE_IDLE_MODE 	0x00 //主机模式

#define TD5322A_BLE_CENTRAL_MODE 	0x00 	//主机模式
#define TD5322A_BLE_PERIPHERAL_MODE 0x01	//从机模式
//#define 

//脑电结构体
typedef struct
{	
	uint16_t uart_length; //串口接收的数据长度
	uint8_t  size ;//串口接收的数据最大限制长度
	uint8_t uart_flag;	//数据接收完标志位	
	uint8_t wrok_mode;	//工作模式，0为从机模式，1为主机模式
	uint8_t work_peripheral_role_flg; //从机机模式标志位
	uint8_t work_central_role_flg; //主机模式标志位

	uint8_t work_role_connect_flg; //主从模式连接标志位
	uint8_t data;
	uint8_t *uart_receive_data_buffer; //串口接收数据
	uint8_t uart_receive_time_flg;
	uint8_t uart_receive_time_cnt;
	uint8_t 	uart_send_time_flg;
	uint16_t uart_send_time_cnt;
	int8_t 	uart_resetore_time_flg; //重启复位标志位定时器
	uint16_t uart_resetore_time_cnt;
}TD5322A_STRUCT;
extern volatile TD5322A_STRUCT  td5322a_struct;

extern const uint8_t command_td5322a_save_data_reset[];
extern const uint8_t command_td5322a_save_data_resetore[];

extern const uint8_t command_td5322a_test_ok[];
extern const uint8_t command_td5322a_check_version[];
extern const uint8_t command_td5322a_check_mac[];
extern const uint8_t command_td5322a_check_baud[];
extern const uint8_t command_td5322a_check_uuid[];
extern const uint8_t command_td5322a_check_role[];
//������
extern  const uint8_t command_td5322a_read_ufo_ble_name[];
extern  const uint8_t command_td5322a_set_baud[];
extern  const uint8_t command_td5322a_set_bel_name[];
extern  const uint8_t command_td5322a_set_bel_central_role[]; //主机模式
extern  const uint8_t command_td5322a_set_bel_peripheral_role[]; //从机模式
extern  const uint8_t command_td5322a_set_bel_central_role_scan[]; //主机扫描模式
extern const uint8_t command_td5322a_bel_central_role_connect_success[];
extern const uint8_t command_td5322a_ble_ok[];

extern const uint8_t command_td5322a_bel_central_role_disconnect_channle_change[]; //断开前需要切换成不同at指令
extern const uint8_t command_td5322a_bel_central_role_disconnect[]; //断开连接
extern uint8_t command_td5322a_connect_mac_address[6];
void td5322a_uart_to_bel(const uint8_t *data_buffer, uint16_t data_length);
void td5322a_uart_to_bel_command( const uint8_t data_buffer[]);
void td5322a_bel_to_mcu(uint8_t *data_buffer, uint16_t data_length);
void eeg_uart_to_td5322a(uint8_t *data_buffer, uint16_t data_length);
uint16_t crc16_table_method(uint8_t *data, uint16_t length);
void td5322a_task(void);
uint8_t arr_comper(const uint8_t *data1,const uint8_t *data2);
void mcu_uart_to_td5322a(const uint8_t *data_buffer, uint16_t data_length);

#endif

