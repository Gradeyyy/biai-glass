/*
 * td5322a.c
 *
 *  Created on: 2025.6.5
 *      Author: lan
 */
#include "gpio.h"
#include "math.h"
#include "usart.h"
#include "td5322a.h"
#include "i2c.h"
#include "usart.h"
#include "usart.h"
#include "work_drive.h"
#include "key_scan.h"
#include "string.h"
#include "stm32_ota.h"


const char command_td5322a_ble_conn[]="BLE_CONN"; 
const char command_td5322a_ble_notify_en[]="NOTIFY_EN"; 
const char command_td5322a_ble_discon[]="BLE_DISC\r\n";

ble_uart_msg_t ble_uart_rx_msg = {
    .timeout = 0,
    .flag = 0,	
    .length = 0,
    .connected = 0,
    .put_msg_index = 0,
    .get_msg_index = 0,
    };

static void jump_to_bootloader(void)
{
    *((volatile uint32_t *)BOOTLOADER_FLAG_ADDRESS) = BOOTLOADER_FLAG_MAGIC_NUMBER; 

    uint32_t JumpAddress = *(__IO uint32_t*)(BOOT_PROGRAM_ADDR + 4);  
    typedef void (*pFunction)(void);
    pFunction jump = (pFunction)JumpAddress;

    HAL_RCC_DeInit();
    HAL_DeInit();
  
    MX_USART1_UART_DeInit();
    MX_USART2_UART_DeInit();
    MX_USART3_UART_DeInit();
    MX_I2C2_DeInit();
    MX_I2C3_DeInit(); 
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    SCB->VTOR = BOOT_PROGRAM_ADDR;

    __set_MSP(*(__IO uint32_t*)BOOT_PROGRAM_ADDR); 
    jump();  

 
}




void ota_command_process(void)
{
	if(ble_uart_rx_msg.flag == 1 && ble_uart_rx_msg.timeout > 1) {

		uint8_t index;
        index = ble_uart_rx_msg.put_msg_index;
        ble_uart_rx_msg.put_msg_index++;
        if(ble_uart_rx_msg.put_msg_index >= MAX_FRAMES) 
            ble_uart_rx_msg.put_msg_index = 0;

		if(ble_uart_rx_msg.length > MAX_BLE_UART_LEN){
			printf("ble uart msg length=%d error",ble_uart_rx_msg.length);
			ble_uart_rx_msg.length =0;
			return;
		}
        memcpy((uint8_t *)ble_uart_rx_msg.msg[index], (uint8_t *)ble_uart_rx_msg.data, ble_uart_rx_msg.length);
        ble_uart_rx_msg.msg_len[index] = ble_uart_rx_msg.length;

        memset((uint8_t *)ble_uart_rx_msg.data, 0, ble_uart_rx_msg.length);
        ble_uart_rx_msg.length =0;
        ble_uart_rx_msg.flag = 0;
        
		do{
            if(ble_uart_rx_msg.connected == 0){
                for(uint8_t i =0;i < ble_uart_rx_msg.msg_len[ble_uart_rx_msg.get_msg_index];i++) {
                    printf("%c",ble_uart_rx_msg.msg[ble_uart_rx_msg.get_msg_index][i]);
                }

                // if(strstr((char *)ble_uart_rx_msg.msg[ble_uart_rx_msg.get_msg_index], command_td5322a_ble_conn)){
                //     printf("connected\r\n");
                    
                // }else 
                if(strstr((char *)ble_uart_rx_msg.msg[ble_uart_rx_msg.get_msg_index], command_td5322a_ble_notify_en)){
                    ble_uart_rx_msg.connected = 1;
                    printf("notify en\r\n");
                }

            }else{
                printf("xxx\r\n");
                if(strstr((char *)ble_uart_rx_msg.msg[ble_uart_rx_msg.get_msg_index], command_td5322a_ble_discon)){
                    printf("disconnect\r\n");
                    ble_uart_rx_msg.connected = 0;
                }
                
                switch (ble_uart_rx_msg.msg[ble_uart_rx_msg.get_msg_index][0]){
                    case OTA_CMD_START:
                        printf("app to boot\r\n");
                        jump_to_bootloader();
                        break;
                    case GET_DEV_VERSION_CMD:
                        printf("GET_DEV_VERSION_CMD\r\n");
                        break;
                    default:
                        break;
            	}
			}

            ble_uart_rx_msg.get_msg_index++;
            if(ble_uart_rx_msg.get_msg_index >= MAX_FRAMES) 
                ble_uart_rx_msg.get_msg_index = 0;
        }while(ble_uart_rx_msg.get_msg_index != ble_uart_rx_msg.put_msg_index);
	}
}


