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
#include "usart.h"
#include "work_drive.h"
#include "key_scan.h"
#include "string.h"
uint8_t td5322a_uart_receive_buf[1024] = {0};
volatile TD5322A_STRUCT  td5322a_struct ={ 
	.uart_length =0, // UART接收数据长度
	.size =0, // 数据大小
	.uart_flag=0,	// 接收完成标志位
	.wrok_mode=0,	// 工作模式，0为从机模式，1为主机模式
	.work_peripheral_role_flg =TD5322A_BLE_CENTRAL_MODE, // 工作角色模式标志
	.work_central_role_flg = 0,
	.work_role_connect_flg=0, // 连接状态标志
	.data=0,
	.uart_receive_data_buffer = td5322a_uart_receive_buf, // 接收数据缓冲区
	.uart_receive_time_flg = 0,
	.uart_receive_time_cnt = 0,
	.uart_send_time_flg = 0,
	.uart_send_time_cnt = 0,
	.uart_resetore_time_flg =1,
	.uart_resetore_time_cnt =0
}; // TD5322A模块UART通信结构体

// 状态机状态定义
typedef enum {
    STATE_WAIT_HEADER1,  // 等待帧头第一个字节(0x0D)
    STATE_WAIT_HEADER2,  // 等待帧头第二个字节(0x0A)
    STATE_RECEIVE_BODY,  // 接收数据主体
    STATE_WAIT_FOOTER1,  // 等待帧尾第一个字节(0x0D)
} parser_state_t;
// UART解析器结构体
typedef struct {
    parser_state_t state;
    uint8_t *buffer;  // 接收缓冲区(需要确保缓冲区足够大)
    uint16_t index;       // 当前缓冲区索引位置
    uint8_t frame_ready;  // 帧接收完成标志
} uart_parser_t;

// 全局变量声明
//extern uart_parser_t td5322a_parser;

//extern volatile TAGT_STRUCT  tagt_struct;
// 预计算的CRC16表
static const uint16_t crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    // ... 省略部分表值 ...
    0xF1EF, 0xE1CE, 0xD1AD, 0xC18C, 0xB16B, 0xA14A, 0x9129, 0x8108,
    0x70E7, 0x60C6, 0x50A5, 0x4084, 0x3063, 0x2042, 0x1021, 0x0000
};
// 命令定义
const uint8_t command_td5322a_save_data_reset[]="AT+RESET\r\n";
const uint8_t command_td5322a_save_data_resetore[]="AT+RESTORE\r\n";
const uint8_t command_td5322a_test_ok[]="AT\r\n";
const uint8_t command_td5322a_check_version[]="AT+VER\r\n";
const uint8_t command_td5322a_check_mac[]="AT+BLEMAC\r\n";



const uint8_t command_td5322a_check_baud[]="AT+BAUD\r\n";
const uint8_t command_td5322a_check_uuid[]="AT+UUID\r\n";
const uint8_t command_td5322a_check_role[]="AT+ROLE\r\n";
// 蓝牙相关命令
const uint8_t command_td5322a_read_ufo_ble_name[]="FC_Y1024_V01\r\n";
const uint8_t command_td5322a_set_baud[]="AT+BAUD=19200\r\n";
const uint8_t command_td5322a_set_bel_name[]="AT+BLENAME=BIAI_UFO_SN001\r\n";
const uint8_t command_td5322a_set_bel_central_role[]="AT+ROLE=1\r\n"; // 主机模式
const uint8_t command_td5322a_set_bel_peripheral_role[]="AT+ROLE=0\r\n"; // 从机模式
const uint8_t command_td5322a_set_bel_central_role_scan[]="AT+SCAN=1\r\n"; // 主机扫描模式
uint8_t command_td5322a_ble_central_role_connect_mac[]="AT+CONN=5b6b4da2b331\r\n"; // 主机连接指定MAC地址
const uint8_t command_td5322a_ble_central_role_connect_name[] ="7p FC_TD5322A";// MAC地址相关
const uint8_t command_td5322a_ble_central_role_mac[]="5c304ff3a9e7\r\n"; // 主机MAC地址
const uint8_t command_td5322a_central_role_con_success[] ="\r\nBLE_CONN\r\n";	// 主机连接成功
const uint8_t command_td5322a_central_role_disconn_success[] ="\r\nBLE_DISC\r\n";	// 主机断开连接成功

const uint8_t command_td5322a_peripheral_role_con_success[] ="\r\nLE_CONN\r\n"; // 从机连接成功
const uint8_t command_td5322a_central_role_notify_success[] ="\r\nEN_NOTIFY_SUCC\r\n";	// 主机通知成功

const uint8_t command_td5322a_bel_central_role_disconnect_channle_change[]="AT>9\r\n"; // 断开前需要发送的通道切换命令
const uint8_t command_td5322a_bel_central_role_disconnect[]="AT+DISC=0\r\n"; // 断开连接命令
const uint8_t command_td5322a_connect_ok[]={0x42,0x4C,0x45,0x5F,0x43,0x4F,0x4E,0x4E,0x0D,0x0A};
// MAC地址需要按字节依次获取
uint8_t command_td5322a_connect_mac_address[6]={0};
const uint8_t  command_td5322a_ble_central_role_ok[]={0x0D,0x0A,0x42,0x4C,0x45,0x5F,0x43,0x4F,0x4E,0x4E,0x0D,0x0A};// 主机模式通信
const uint8_t command_td5322a_ble_peripheral_role_ok[]="\r\nAT+ROLE=CLIENT\r\n"; // 从机模式通信
const uint8_t command_td5322a_ble_ok[]="\r\nOK\r\n"; // 操作成功
const uint8_t command_td5322a_test_return_ok[]="\r\nok\r\n"; // 测试返回成功

const uint8_t command_td5322a_UFO_return_message[]="123456789\r\n"; // 连接上飞球之后，飞球的每隔100ms返回值


extern uint8_t receiveBuffer2[36];
extern uint8_t receiveBuffer2Tem[];
extern uint8_t flag_change ;
uart_parser_t td5322a_parser ={
	.state = STATE_WAIT_HEADER1,
	.buffer = receiveBuffer2,
	.index = 0,
	.frame_ready =0
};

extern void ota_command_process(void);

// 数组比较
uint8_t arr_comper(const uint8_t *data1,const uint8_t *data2)
{
	for(uint8_t i =0;i<strlen(data1);i++)
	{
		if(data1[i] != data2[i])
		{
			return 0;
		}
	}
	return 1;
}
// 解析输入字节，更新解析状态
void parser_feed_byte(uart_parser_t *parser, uint8_t byte) {
    switch (parser->state) {
        case STATE_WAIT_HEADER1:
            if (byte == 0x0D) {
                parser->state = STATE_WAIT_HEADER2;
            }
            break;

        case STATE_WAIT_HEADER2:
            if (byte == 0x0A) {
                parser->state = STATE_RECEIVE_BODY;
                parser->index = 0;  // 重置缓冲区索引
            } else {
                // 未接收到0x0A，返回等待帧头状态
                parser->state = STATE_WAIT_HEADER1;
            }
            break;

        case STATE_RECEIVE_BODY:
            // 检测到帧尾(0x0D)
            if (byte == 0x0D) {
                parser->state = STATE_WAIT_FOOTER1;
            } else {
                // 存储数据(确保不超出缓冲区大小)
                if (parser->index < sizeof(parser->buffer) - 1) {
                    parser->buffer[parser->index++] = byte;
                } else {
                    // 缓冲区溢出，重置状态
                    parser->state = STATE_WAIT_HEADER1;
                }
            }
            break;

        case STATE_WAIT_FOOTER1:
            if (byte == 0x0A) {
                // 完成一帧数据接收
//                parser->buffer[parser->index] = '\0';  // 添加字符串结束符
                parser->frame_ready = 1;
            } else {
                // 未接收到帧尾，将之前的0x0D和当前字节存入缓冲区
                if (parser->index < sizeof(parser->buffer) - 2) {
                    parser->buffer[parser->index++] = 0x0D;  // 存储接收到的0x0D
                    parser->buffer[parser->index++] = byte;  // 存储当前字节
                    parser->state = STATE_RECEIVE_BODY;
                } else {
                    // 缓冲区溢出，重置状态
                    parser->state = STATE_WAIT_HEADER1;
                }
            }
            break;
    }
}

// TD5322A任务处理函数
void td5322a_task(void)
{
	volatile static uint8_t command_flg= 0;
	volatile uint8_t buffer[256] ={0};
	static uint8_t s_length =0;
	uart_parser_t s_parser;
	static uint8_t connect_flg =0;
	static uint8_t s_role_flg =0;
	static int8_t mcu_uart_send_msg_state = -2;
	
	static uint8_t uart3_flg =0;

//	if(td5322a_struct.uart_resetore_time_cnt > 400)
//	{
//		td5322a_uart_to_bel_command(command_td5322a_save_data_resetore);
//		td5322a_struct.uart_resetore_time_flg =0;
//	}
//	#if 0
//	if(td5322a_struct.uart_resetore_time_flg == 0)
	{
		switch(mcu_uart_send_msg_state) 
		{
			// 发送设置蓝牙名称命令
			case -2:
				if(td5322a_struct.uart_send_time_cnt >5)//100ms 20*5
				{
					td5322a_uart_to_bel_command(command_td5322a_set_bel_name);
					mcu_uart_send_msg_state =-1;
					td5322a_struct.uart_send_time_cnt =0;
				}
				break;
				// 发送保存数据并重置命令
				case -1:
				if(td5322a_struct.uart_send_time_cnt >5)//100ms等待时间
				{
					td5322a_uart_to_bel_command(command_td5322a_save_data_reset);
					mcu_uart_send_msg_state = 0;
					td5322a_struct.uart_send_time_cnt =0;
				}
				break;
			case 0:
				
				if(td5322a_struct.uart_send_time_cnt >20)//400ms等待时间
				{
					if(td5322a_struct.work_role_connect_flg == 1) // 已连接状态，需要发送断开命令
					{
						td5322a_uart_to_bel_command(command_td5322a_bel_central_role_disconnect_channle_change);
						mcu_uart_send_msg_state = 1;							
					}
					else 
					{
						mcu_uart_send_msg_state = 2;								
					}
					td5322a_struct.uart_send_time_cnt = 0;
				}
				break;
				case 1:
					if(td5322a_struct.uart_send_time_cnt > 5)//
					{
						td5322a_uart_to_bel_command(command_td5322a_bel_central_role_disconnect); // 发送断开连接命令
						mcu_uart_send_msg_state = 2;
						td5322a_struct.uart_send_time_cnt =0;
					}
				break;
				case 2:
					if(td5322a_struct.uart_send_time_cnt > 5)//
					{
						if(td5322a_struct.work_peripheral_role_flg == TD5322A_BLE_CENTRAL_MODE)
						{
							td5322a_uart_to_bel_command(command_td5322a_set_bel_central_role);
							printf ("master select \r\n");

						}
						else
						{
							td5322a_uart_to_bel_command(command_td5322a_set_bel_peripheral_role);
							printf ("slave select \r\n");
						}
						td5322a_struct.uart_send_time_cnt =0;
						mcu_uart_send_msg_state = 3;
					}
				break;
				case 3:
					if(td5322a_struct.uart_send_time_cnt > 30)//600ms等待时间
					{						
						td5322a_uart_to_bel_command(command_td5322a_save_data_reset);
						mcu_uart_send_msg_state = 4;
						td5322a_struct.uart_send_time_cnt =0;
						printf ("save_data_reset \r\n");

					}								
				break;
				case 4:
					if(td5322a_struct.uart_send_time_cnt > 100)//20*100ms等待时间
					{
						if(td5322a_struct.work_peripheral_role_flg == TD5322A_BLE_CENTRAL_MODE) // 主机模式，发送扫描命令
						{
							td5322a_uart_to_bel_command(command_td5322a_set_bel_central_role_scan);
							printf ("delay 1s \r\n");
							global_ctrl_device.ble_mode_time_flg = 1; //

//							flag_change =1; // 标记切换为蓝牙模式，准备数据传输
							
						}
						else
						{
							global_ctrl_device.key_mode_state = IMMEDIATE_MODE; // 设置为立即模式
							global_ctrl_device.key_mode_time_flg =1; // 3分钟超时，超时后切换模式
						}
						td5322a_struct.uart_send_time_flg = 0;
						mcu_uart_send_msg_state = 0;// 循环发送命令，等待响应					
					}

				break;
				default:
					break;
				
		}
	}
//	if(uart3_flg ==0)
//	{
//		uart3_flg =1;
//		HAL_UART_Receive_IT(&huart3, &td5322a_struct.data, 1);
//	}
	// 系统启动时默认设置为蓝牙模式，超时未连接则切换
//	if(td5322a_struct.uart_resetore_time_flg == 0)
	{
		if(td5322a_struct.work_peripheral_role_flg == TD5322A_BLE_CENTRAL_MODE && s_role_flg ==0)
		{
			s_role_flg = 1;
			td5322a_struct.uart_send_time_flg =1; // 启动发送超时计时
		}
		// 切换为从机模式
		else if(td5322a_struct.work_peripheral_role_flg == TD5322A_BLE_PERIPHERAL_MODE && s_role_flg == 1)
		{
			s_role_flg = 0;
			printf ("change sever \r\n");
		}
	}
//	cpu_get_cmd_re_cyc(&uart3_struct,&uart_receive_buffer);
	if(flag_change == 1)
	{
		if (td5322a_struct.work_peripheral_role_flg == TD5322A_BLE_CENTRAL_MODE)
		{
			if(global_ctrl_device.ble_mode_time_cnt < 200) //6s
			{
				if(td5322a_struct.work_role_connect_flg == 1)				
				{
						flag_change =0;
						printf("ufo crtl success.\r\n");
//						s_work_peripheral_role_flg = 1; 
//						sEye_traning_mode_flg =0;
						global_ctrl_device.ble_mode_time_flg = 0;
						// 切换到2.4G训练模式
						global_ctrl_device.key_mode_state = TRAINING_MODE;
						global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.eye_muscle_training_mode =1;

				}

			}
			else
			{
//				flag_ble = 0;
	//			LT8960L_BLE_INIT(); // ble 初始化
				flag_change = 0;
				// 切换为从机模式 立即模式
				td5322a_struct.uart_send_time_flg = 1;
				td5322a_struct.work_peripheral_role_flg = TD5322A_BLE_PERIPHERAL_MODE;
				LED_BULE_ON();
				LED_GREEN_OFF();
//				s_work_peripheral_role_flg = 0;
				global_ctrl_device.ble_mode_time_flg =0; // 重置计时
//				global_ctrl_device.key_mode_time_flg =1; //3分钟超时，超时后切换模式
//				global_ctrl_device.key_mode_state = IMMEDIATE_MODE; // 设置为立即模式
				global_ctrl_device.DEVIC_MODE.training_mode= IDEL_MODE;
				printf("turn on ble immediate mode\r\n");
	//			printf("RF rx  failed.\r\n");
			
			}
	}
	
		
	}
//#endif
	ota_command_process();  //OTA 操作
	if(td5322a_struct.uart_flag == 1)
	{
		if(td5322a_struct.wrok_mode ==0) // 从机模式，直接转发数据
		{
			for(uint8_t i =0;i <td5322a_struct.uart_length;i++)
			{
				printf("%c",td5322a_struct.uart_receive_data_buffer[i]);
				buffer[i]= td5322a_struct.uart_receive_data_buffer[i];
			}
//			printf("\r\n");
			if(arr_comper(command_td5322a_test_return_ok,(const uint8_t *)buffer)==1)
			{
				printf("bel ok\r\n");
				if(connect_flg == 1)
				{
					connect_flg = 0;
					td5322a_struct.wrok_mode = 1;
					printf("mode = 1\r\n");
				}		
				
			}

			if(strstr((const uint8_t *)buffer,command_td5322a_ble_central_role_connect_name))
			{
				printf("name FC_TD5322A \r\n");
				// +DEV1:39e3d13c7f04,-53,FC_TD5322A
				memcpy(&command_td5322a_ble_central_role_connect_mac[8], (uint8_t *)&buffer[6], 12);
				printf("connect cmd %s \r\n",command_td5322a_ble_central_role_connect_mac);

				td5322a_struct.work_role_connect_flg = 1;
				td5322a_struct.work_peripheral_role_flg = TD5322A_BLE_CENTRAL_MODE; // 设为主机模式
				td5322a_uart_to_bel_command( command_td5322a_ble_central_role_connect_mac);
				connect_flg =1;
			}
			
			
			char *pos = strstr((const uint8_t *)buffer,command_td5322a_ble_central_role_connect_name);
			if (pos != NULL) {
					int index = 0;
					printf("name FC_TD5322A \r\n");
					for (char *p = pos - 1; p >= buffer; p--) {
							if (*p == ':') {
									index = p - buffer;
									printf("':' index=%d \n", index);
									break;
							}
					}
					if(index && index < 256){
							memcpy(&command_td5322a_ble_central_role_connect_mac[8], (uint8_t *)&buffer[index+1], 12);
							printf("connect cmd %s \r\n",command_td5322a_ble_central_role_connect_mac);
							// td5322a_struct.work_role_connect_flg = 1;
							td5322a_struct.work_peripheral_role_flg = TD5322A_BLE_CENTRAL_MODE; // 设为主机模式
							td5322a_uart_to_bel_command( command_td5322a_ble_central_role_connect_mac);
							connect_flg =1;
					}else{
					printf("no find mac\n");
					}
					// +DEV1:39e3d13c7f04,-53,FC_TD5322A

			}
						
			
//			if (arr_comper(command_td5322a_peripheral_role_con_success,(const uint8_t *)td5322a_struct.uart_receive_data_buffer) == 1)
//			{
//				td5322a_struct.work_peripheral_role_flg = 1; // 设为从机模式
//				td5322a_struct.work_role_connect_flg = 1;  // 连接成功
//				printf("ble peripheral conncet success \r\n");
//				global_ctrl_device.ble_mode_time_flg = 0; // 重置计时
//			}
//			// 只要收到连接成功响应就设置连接标志为1
//			if((strncmp(command_td5322a_central_role_con_success,(const uint8_t *)buffer,strlen(command_td5322a_central_role_con_success) ==0)))
////				|| (strncmp(command_td5322a_peripheral_role_con_success,td5322a_struct.uart_receive_data_buffer,strlen(command_td5322a_peripheral_role_con_success) ==0)))
			if(arr_comper(command_td5322a_central_role_notify_success,(const uint8_t *)buffer) ==1)
			{
//				td5322a_struct.work_peripheral_role_flg = TD5322A_BLE_PERIPHERAL_MODE; // 设为主机模式
				td5322a_struct.work_role_connect_flg = 1;
				global_ctrl_device.ble_mode_time_flg = 0; // 重置计时
				printf("ble conncet success \r\n");
			}
			else if(arr_comper(command_td5322a_central_role_disconn_success,(const uint8_t *)buffer) ==1)
			{
//				td5322a_struct.work_peripheral_role_flg = TD5322A_BLE_CENTRAL_MODE; // 设为主机模式
				td5322a_struct.work_role_connect_flg = 0;
				global_ctrl_device.ble_mode_time_flg = 0; // 重置计时
				printf("ble disconncet success \r\n");
			}
			else if(arr_comper(command_td5322a_central_role_con_success,(const uint8_t *)buffer) ==1)
			{
//				td5322a_struct.work_peripheral_role_flg = TD5322A_BLE_CENTRAL_MODE; // 设为主机模式
				td5322a_struct.work_role_connect_flg = 1;
				global_ctrl_device.ble_mode_time_flg = 0; // 重置计时
				printf("ble conncet success \r\n");
			}
		}
		
		td5322a_struct.uart_length =0;
		memset(td5322a_struct.uart_receive_data_buffer,0,td5322a_struct.uart_length);
		td5322a_struct.uart_flag = 0;
	}
}

/************************************************
函数名称：mcu_uart_to_td5322a
功能描述：通过UART向TD5322A模块发送数据
输入参数：data_buffer --- 发送数据缓冲区
          data_length --- 发送数据长度
返回值：  无
作者：    lan
*************************************************/
static __IO ITStatus UartReady = RESET;
void mcu_uart_to_td5322a(const uint8_t *data_buffer, uint16_t data_length)
{
	if(HAL_UART_Transmit_DMA(&huart3, data_buffer, data_length) != HAL_OK)
	{
		Error_Handler();
	}
//	if(HAL_UART_Transmit(&huart3, data_buffer, data_length,100) != HAL_OK)
//	{
//		Error_Handler();
//	}

}

void eeg_uart_to_td5322a(uint8_t *data_buffer, uint16_t data_length)
{
    // 打印传入的数据内容
    printf("Data received by eeg_uart_to_td5322a: ");
    for (uint16_t i = 0; i < data_length; i++) {
        printf("%02X ", data_buffer[i]);
    }
    printf("\n");
	
//	if(HAL_UART_Transmit_DMA(&huart3, (const uint8_t *)(data_buffer), data_length) != HAL_OK)
//    {
//        printf("uart error \r\n");
//        Error_Handler();
//    }
	
	HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart3, (const uint8_t*)(data_buffer), data_length);

	if (status != HAL_OK) {
		printf("UART ERROR, HAL_UART_Transmit_DMA RETURN: %d \r\n", (int)status); // 打印数值形式
		Error_Handler();
}
}

void td5322a_bel_to_mcu(uint8_t *data_buffer, uint16_t data_length)
{
if (HAL_UART_Receive_DMA(&huart3, (uint8_t *)data_buffer, data_length) != HAL_OK)
  {
    /* Transfer error in reception process */
    Error_Handler();
  }
}
/************************************************
函数名称：td5322a_uart_to_bel_command
功能描述：通过UART向TD5322A模块发送命令
输入参数：data_buffer --- 命令数据缓冲区
返回值：  无
作者：    lan
*************************************************/
void td5322a_uart_to_bel_command(const uint8_t data_buffer[])
{
//	const uint8_t temp_buffer[] ={0};
//	 uint16_t temp_length = strlen(data_buffer);
	mcu_uart_to_td5322a(data_buffer,strlen(data_buffer));

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
	if(UartHandle == &huart3)
		UartReady = SET;

}

// 使用CRC16表计算CRC值
uint16_t crc16_table_method(uint8_t *data, uint16_t length) {
    uint16_t crc = CRC16_INIT;

    for (uint16_t i = 0; i < length; i++) {
        uint8_t index = ((crc >> 8) ^ data[i]) & 0xFF;
        crc = (crc << 8) ^ crc16_table[index];
    }

    return crc;
}