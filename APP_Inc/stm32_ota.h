/*
 * stm32_ota.h
 */
#include "stm32g4xx.h"
#ifndef _STM32_TOTA_H_
#define _STM32_TOTA_H_


#define BOOTLOADER_FLAG_ADDRESS  (SRAM2_BASE + SRAM2_SIZE - 0x100) 
#define BOOTLOADER_FLAG_MAGIC_NUMBER 0xDEADBEEF
#define BOOT_PROGRAM_ADDR 0x8000000
#define APP_START_ADDR  0x8006000  

enum 
{ 
    GET_DEV_VERSION_CMD = 0x01,  
    SET_DEV_INFO_CMD = 0x02,  
    GET_BATTERY_INFO_CMD= 0x03, 
    SET_BATTERY_INFO_CMD= 0x04, 
    EEG_RAW_DATA,  //eeg 原始数据
    EEG_ALG_DATA,  //eeg 算法数据
    EEG_GYRO_DATA,  //陀螺仪数据
    MEASURE_DISTANCE_DATA, // 测距仪数据
    // 拓展补充
    BLE_MTU_LEN_CMD = 0XCF, 
    OTA_CMD_START = 0xD0,  
    OTA_CMD = 0xD1, 
}; 


#define MAX_FRAMES 3
#define MAX_BLE_UART_LEN 255

typedef struct {
	volatile  uint16_t timeout;
	volatile  uint8_t flag;	
	uint16_t length;
	volatile  uint8_t data[MAX_BLE_UART_LEN];
	uint8_t connected;
    uint8_t msg[MAX_FRAMES][MAX_BLE_UART_LEN]; 
    uint8_t msg_len[MAX_FRAMES]; 
    uint8_t put_msg_index;
    uint8_t get_msg_index;
} __attribute__((packed)) ble_uart_msg_t;


extern volatile TD5322A_STRUCT  td5322a_struct;
extern ble_uart_msg_t ble_uart_rx_msg;


#endif

