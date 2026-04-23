/*
 * work_drive.h
 *
 *  Created on: 2024年8月23日
 *      Author: lang
 */
#ifndef _WORK_DRIVE_H_
#define _WORK_DRIVE_H_

#include "main.h"
#define CMD_TD5322A_BLE_DATA_1 0xBB
#define CMD_TD5322A_BLE_DATA_2 0xBA
#define RECEIVE_BUFFER_LENGTH 36
#define RECEIVE_BUFFER2_LENGTH 36
#define SOFRWARE_VERSION 0x01
//void device_init(void);
void work_task(void);

//蓝牙芯片结构体
typedef struct 
{
	uint8_t result_current;
	uint8_t result_previous;
	uint8_t *send_buffer;
	uint8_t *receice_buffer;

}TD5322A_DIVICE_STRUCT;

extern volatile TD5322A_DIVICE_STRUCT td5322a_divice_sturct;

void tgat_uart_task(void);
uint8_t parse_SW_data(uint8_t data);
void ble_low_power_task(void);
//void ble_send_cmd(uint8_t cmd);
//void task_read_icm20608(void);
//void task_make_acc(void);
//void ble_init(void);
void device_data_power_down(void);
#endif /* INC_WORK_DRIVE_H_ */
