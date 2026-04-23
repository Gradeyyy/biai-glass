/*
 * work_drive.c
 *
 *  Created on: 2025年2月20日
 *      Author: lang
 */
#include "work_drive.h"
#include "usart.h"
//#include "LT8960Ldrv.h"
#include "algorithm.h"
#include "ICM_20608.h"
#include "adc.h"
#include "vl53l0x_gen.h"
#include "key_scan.h"
#include "td5322a.h"
#include "opt3001.h"
#define CMD_PPG_LENGTH		 16			// PPG
#define CMD_EEG_RAW_LENGTH   8			// EEG rawdata
#define CMD_EEG_RES2_LENGTH  11			// EEG 佩戴不好结果包
#define CMD_EEG_RES1_LENGTH  36			// EEG 信号良好结果包
#define CMD_ICM20608_LENGTH  18			// ICM20608

// 陀螺仪
#define	ALGO_LEFT  			3
#define	ALGO_RIGHT 			4
#define	ALGO_UP				5
#define	ALGO_DOWN			6
#define	ALGO_LEFTSIDE		7
#define	ALGO_RIGHTSIDE		8
#define ALGO_IDLE			0

// 陀螺仪结果
uint8_t Result_Algo = 0;
uint8_t Result_Algo_old = 0;

// 专注度结果
uint8_t Result_Attention = 0;
uint8_t Result_data_old = 0;
uint8_t Result_data_new = 0;
uint8_t fly_counter = 0;

// 起飞标志
uint8_t Flag_Fly = 0;
// 是否进行BLE广播标志
uint8_t flag_ble = 0;
uint8_t flag_change = 1;
// 专注临时计数
uint32_t flag_count_temp = 0;
// 有效专注接收标志
uint8_t flag_res_rx = 0;

// 指令维持计数
uint8_t flag_count_continue = 0;
// EEG 结果控制标志 1-起飞 2-降落 3-稳定
uint8_t flag_res_eeg = 0;

// 定时器
uint8_t flag_1ms = 0;

uint8_t flag_userTime = 0;
uint32_t flag_3ms = 0;


uint8_t flag_timeStart = 0;

uint8_t flag_userTime_2 = 1;
uint8_t flag_timeStart_2 = 0;

uint8_t flag_roll_change = 0;

// 电压值
//uint16_t volValue = 0;
//float volFloat = 0.0;

extern struct icm20608_dev_struc icm20608_dev;

extern VL53L0X_RangingMeasurementData_t vl53l0x_data; // 测距测量结构体
extern VL53L0X_Dev_t vl53l0x_dev; // vl53l0x_dev 结构体
extern uint16_t Distance_data; // 方向数据
extern float coef[12];
uint16_t rawLux; // 原始光感


// 定义飞球新的控制协议结构体
typedef struct {
    uint8_t header;     // 帧头 0x99
    uint8_t length;     // 长度 0x08
    uint8_t rc_roll;    // 左右摇杆值 00~80~FF 00最左，80为中位，FF最右
    uint8_t rc_pitch;   // 前后摇杆值 00~80~FF 00最前，80为中位，FF最后
    uint8_t rc_thr;     // 油门摇杆值 00~80~FF 00最小，80为中位，FF最大
    uint8_t rc_yaw;     // 偏航摇杆值 00~80~FF 00最左，80为中位，FF最右
    uint8_t roll_off;   // 左右微调  0~20~40  20为偏移量0
    uint8_t pitch_off;  // 前后微调  0~20~40  20为偏移量0
    uint8_t key_num;    // 按键功能位  0A为一键上升/一键下降，0B为一键下降，2A为一键急停，48为一键校准，01为翻滚，02为旋转，04为校准，08为一键返回
    uint8_t empty;      // 预留
    uint8_t ck;         // 校验位
} FlyBallControlProtocol;

// run action
enum RUN_STATE{
	RUN_STATE_START = 1,
	RUN_STATE_LEFT_ROLL,           // 自运行左
	RUN_STATE_RIGHT_ROLL,          // 自运行右
	RUN_STATE_FOREWORD_PITCH,      // 自运行前
	RUN_STATE_BACKWORD_PITCH,      // 自运行后
	RUN_STATE_LEFT_ROLLOVER,       // 自运行左翻滚
	RUN_STATE_RIGHT_ROLLOVER,      // 自运行右翻滚
	RUN_STATE_FOREWORD_ROLLOVER,   // 自运行前翻滚
	RUN_STATE_BACKWORD_ROLLOVER,   // 自运行后翻滚
	RUN_STATE_UP,                  // 自运行上升
	RUN_STATE_DOWN,                // 自运行下降
	RUN_STATE_LEFT_ROLL_HAND,      // 主动左
	RUN_STATE_RIGHT_ROLL_HAND,     // 主动右
	RUN_STATE_FOREWORD_PITCH_HAND, // 主动前
	RUN_STATE_BACKWORD_PITCH_HAND, // 主动后
	RUN_STATE_UP_HAND,             // 主动上升
	RUN_STATE_DOWN_HAND,           // 主动下降
	RUN_STATE_IDLE = 18,                // 主动后等待
	RUN_STATE_HOLD,				   // 动作后轨迹规划
	RUN_STATE_NONE                 // 空闲
} runState = RUN_STATE_NONE;

uint8_t runStateOld = 0;

// 左roll 控制变量
uint8_t leftCount = 0;
// 指令间隔
uint8_t time_temp = 0;


// EEG data parse
enum PARSER_STATE {
	PARSER_STATE_SYNC = 1,
	PARSER_STATE_SYNC_2,
	PARSER_STATE_PLENGTH,
	PARSER_STATE_PAYLOAD,
	PARSER_STATE_CHKSUM
} parseState = PARSER_STATE_SYNC;
// 解包标志
#define PST_PACKET_CHECKSUM_FAILED  0
#define PST_PACKET_PARSED_SUCCESS   1
#define PST_PACKET_PARSING 			2

// 同步字
const uint8_t PARSER_SYNC_BYTE = 0xAA;
const uint8_t PARSER_SYNC2_BYTE = 0xAA;

// neurosky
#define  RAW_EEG_LENGTH    	  0x04
#define  RES_EEG_LENGTH    	  0x20
#define  RES2_EEG_LENGTH   	  0x15
//#define  RAW_ICM20608_LENGTH  14

uint8_t payloadLength = 0;
uint32_t payloadSum = 0;
uint8_t payloadSumByte = 0;
uint8_t payloadReceivedLength = 0;

// EEG一次处理Buffer长度

//#define RECEIVE_BUFFER_LENGTH (uint16_t)(512*8+36)	
// EEG接收Buffer
uint8_t BUFFER_RECEIVED[36];

// MIM02一次处理Buffer长度

// MIM02 接收Buffer
uint8_t BUFFER2_RECEIVED[64];

/*
 * EEG 接收变量
 * */
uint8_t resFlag = 0;
uint16_t receive_buffer_size = RECEIVE_BUFFER_LENGTH;
uint8_t receiveBuffer[RECEIVE_BUFFER_LENGTH];
uint8_t receiveBufferTem[RECEIVE_BUFFER_LENGTH];

// MIM02 接收变量
uint8_t receive_buffer2_size = RECEIVE_BUFFER2_LENGTH;
uint8_t resFlag2 = 0;
uint8_t receiveBuffer2[RECEIVE_BUFFER2_LENGTH] ={0};
uint8_t receiveBuffer2Tem[RECEIVE_BUFFER2_LENGTH] ={0};

// EEG send cmd
#define CMD_PRO_HEAD1 		 0xAA
#define CMD_PRO_HEAD2  		 0xAA
#define CMD_PRO_SMALL_3		 0x04
#define CMD_PRO_SMALL_4    	 0x80
#define CMD_PRO_BIG_3		 0x20
#define CMD_PRO_BIG_4		 0x02

// 协议指令标志
enum Protocol_Cmd {
	CMD_NONE = 0, CMD_PPG, CMD_ICM20608, CMD_EEG_RAW, CMD_EEG_RES1, CMD_EEG_RES2, CMD_LUX
};
enum Protocol_Cmd protocolCmd = CMD_NONE;
// BLE 发包缓存
uint8_t BUFFER_SEND[70] = { 0 };
// ICM_20608 发包缓存
uint8_t BUFFER_SEND_ACC[18] = { 0 };

// 计时标志
uint16_t count10ms = 0;
// 2.4G发包缓存
uint8_t RF_DATA[64] = { 0 };
// ble广播数据
//uint8_t ble_data[50]={0X02, 0X25,  											//header
//					0x06, 0x05, 0x04, 0x03, 0x02, 0x01,     				//mac address
//					0x02, 0x01, 0x1A,    									//flags
//					0x03, 0x19, 0xc1, 0x03,   								//device
//					0x08, 0x16, 0x11, 0x11, 0x11, 0xb0, 0xb0, 0xb0,0x00,    //uuid
//					0x0E, 0x09, 0x4C, 0x54, 0x38, 0x39, 0x36, 0x30,0x4C,0x76,0x61,0x6C,0x3D,0x78,0x78, //name
//				};
//uint8_t ble_data2[]={0X02, 0X25,  											//header
//					0x06, 0x05, 0x04, 0x03, 0x02, 0x01,     				//mac address
//					0x02, 0x01, 0x1A,    									//flags
//					0x03, 0x19, 0xc1, 0x03,   								//device
//					0x08, 0x16, 0x11, 0x11, 0x11, 0xb0, 0xb0, 0xb0,0x00,    //uuid
//					0x0E, 0xFF, 0x4C, 0x54, 0x38, 0x39, 0x36, 0x30,0x4C,0x76,0x61,0x6C,0x3D,0x00,0x00, //name
//				};
//原始数据组包通过串口透传bel广播数据
//ble_data_buffer 数据包里面包括，0xaa，0xaa作为帧头数据，然后紧接着两个字节的数据长度 512 0xff,0x01(低字节在前高字节在后)个字节数据长度 ，最后两个是crc的校验
//uint8_t ble_data_buffer[1024]={0X02, 0X25,  											//header
//					0x06, 0x05, 0x04, 0x03, 0x02, 0x01,     				//mac address
//					0x02, 0x01, 0x1A,    									//flags
//					0x03, 0x19, 0xc1, 0x03,   								//device
//					0x08, 0x16, 0x11, 0x11, 0x11, 0xb0, 0xb0, 0xb0,0x00,    //uuid
//					0x0E, 0x09, 0x4C, 0x54, 0x38, 0x39, 0x36, 0x30,0x4C,0x76,0x61,0x6C,0x3D,0x78,0x78, //name
//				};
// ble广播通道
enum{
LOW=0, // 37
MID=24, //38
HIGH=78 //39
}ble_chanel;

const char hex2char[] = {
    '0', '1', '2', '3', '4', '5', '6', '7',
    '8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
};

// 解包neurosky数据
uint8_t parse_SW_data(uint8_t data) {
	switch (parseState) {
	case PARSER_STATE_SYNC:
		if (data == PARSER_SYNC_BYTE) {
			parseState = PARSER_STATE_SYNC_2;
			BUFFER_RECEIVED[0] = PARSER_SYNC_BYTE;
		}
		break;
	case PARSER_STATE_SYNC_2:
		if (data == PARSER_SYNC2_BYTE) {
			parseState = PARSER_STATE_PLENGTH;
			BUFFER_RECEIVED[1] = PARSER_SYNC2_BYTE;
		} else {
			parseState = PARSER_STATE_SYNC;
		}
		break;
	case PARSER_STATE_PLENGTH:
		payloadLength = data;
		// ---------------测试用代码--------------------------
//			if (data == 0x33) {
//				payloadReceivedLength = 3;
//			}
//			if (data == 0x03) {
//				payloadReceivedLength = 3;
//			}
//		if ((payloadLength == (uint8_t) RES_EEG_LENGTH)
//						|| (payloadLength == (uint8_t) RES2_EEG_LENGTH))
//		{
//			printf("payloadLength:%d.\n", payloadLength);
//		}
		// -----------------------------------------
		if ((payloadLength == (uint8_t) RAW_EEG_LENGTH)
				|| (payloadLength == (uint8_t) RES_EEG_LENGTH)
				|| (payloadLength == (uint8_t) RES2_EEG_LENGTH)) {
			payloadReceivedLength = 3;
			BUFFER_RECEIVED[2] = data;
			parseState = PARSER_STATE_PAYLOAD;
			payloadSum = 0;

		} else {
			parseState = PARSER_STATE_SYNC;
		}
		break;
	case PARSER_STATE_PAYLOAD:
		BUFFER_RECEIVED[payloadReceivedLength++] = data;
		payloadSum += data;
		if (payloadReceivedLength >= (payloadLength + 3)) {
			parseState = PARSER_STATE_CHKSUM;
		}
		break;
	case PARSER_STATE_CHKSUM:
//			if ((uint8_t)(~((uint8_t)payloadSum)) != data)
//			{
//				parseState = PARSER_STATE_SYNC;
//				return PST_PACKET_CHECKSUM_FAILED;
//			}
		parseState = PARSER_STATE_SYNC;
		BUFFER_RECEIVED[payloadReceivedLength] = data;
		return PST_PACKET_PARSED_SUCCESS;

	}
	return PST_PACKET_PARSING;
}
//// ble广播数据
//uint8_t ble_data[]={0X02, 0X25,  											//header 2
//					0x06, 0x05, 0x04, 0x03, 0x02, 0x01,     				//mac address 6
//					0x02, 0x01, 0x1A,    									//flags 3
//					0x03, 0x19, 0xc1, 0x03,   								//device 4
//					0x08, 0x16, 0x11, 0x11, 0x11, 0xb0, 0xb0, 0xb0,0x00,    //uuid 9
//					0x0E, 0x09, 0x4C, 0x54, 0x38, 0x39, 0x36, 0x30,0x4C,0x76,0x61,0x6C,0x3D,0x78,0x78,}//name 15

void ble_make_result(void)
{
	static uint8_t ble_data[60] ={0};
	rawLux = calculate_lux_int();
//	VL53L0X_PerformSingleRangingMeasurement();
	vl53l0x_start_single_test(&vl53l0x_dev, &vl53l0x_data, ble_data); //执行一次测量
	icm20608_getdata();//从ICM20608 传感器读取加速度计、陀螺仪和温度数据。
	ble_data[0] = 0x02;
	ble_data[1] = 0x25;
	ble_data[2] = 0x01;
	ble_data[3] = 0x02;
	ble_data[4] = 0x03;
	ble_data[5] = 0x04;
	ble_data[6] = 0x05;
	ble_data[7] = 0x06;//	int32_t low_beta_data_temp =0;
	ble_data[8] = 0x2E;
	ble_data[9] = 0x00;
	ble_data[10] = 0xBB;
	ble_data[11] = 0xBD;
	ble_data[12] = BUFFER_SEND[4];
	for (uint8_t i = 0; i < 24; i++){
		ble_data[13+i] = BUFFER_SEND[7+i];
	}

	ble_data[37] = BUFFER_SEND[32];
	ble_data[38] = BUFFER_SEND[34];
	
	ble_data[39] = ((calculate_lux_int() >> 8) & 0xFF);
	ble_data[40] = (calculate_lux_int() & 0xFF);
	ble_data[41] = ((Distance_data >> 8) & 0xFF);
	ble_data[42] = (Distance_data & 0xFF);

	ble_data[43] = icm20608_dev.accel_x_adc;
	ble_data[44] = icm20608_dev.accel_x_adc >> 8;
	ble_data[45] = icm20608_dev.accel_y_adc;
	ble_data[46] = icm20608_dev.accel_y_adc >> 8;
	ble_data[47] = icm20608_dev.accel_z_adc;
	ble_data[48] = icm20608_dev.accel_z_adc >> 8;

	ble_data[49] = icm20608_dev.temp_adc;
	ble_data[50] = icm20608_dev.temp_adc >> 8;

	ble_data[51] = icm20608_dev.gyro_x_adc;
	ble_data[52] = icm20608_dev.gyro_x_adc >> 8;
	ble_data[53] = icm20608_dev.gyro_y_adc;
	ble_data[54] = icm20608_dev.gyro_y_adc >> 8;
	ble_data[55] = icm20608_dev.gyro_z_adc;
	ble_data[56] = icm20608_dev.gyro_z_adc >> 8;
	ble_data[57] = battery_voltage_struct.battery_capacity;
  
	//---------设备代码版本号---------- 
	//ble_data[58] = 0x61; 
	eeg_uart_to_td5322a(ble_data,58);

}

//void ble_make_result2(void)
//{
//	ble_data2[8] = 0x1E;
//	ble_data2[9] = 0xFF;
//	ble_data2[10] = 0xBB;
//	ble_data2[11] = 0xBA;
//	ble_data2[12] = ((rawLux >> 8) & 0xFF);
//	ble_data2[13] = (rawLux & 0xFF);
//	ble_data2[14] = ((Distance_data >> 8) & 0xFF);
//	ble_data2[15] = (Distance_data & 0xFF);
//	ble_data2[16] = ((icm20608_dev.gyro_x_adc >> 8) & 0xFF);
//	ble_data2[17] = (icm20608_dev.gyro_x_adc & 0xFF);
//	ble_data2[18] = ((icm20608_dev.gyro_y_adc >> 8) & 0xFF);
//	ble_data2[19] = (icm20608_dev.gyro_y_adc & 0xFF);
//	ble_data2[20] = ((icm20608_dev.gyro_z_adc >> 8) & 0xFF);
//	ble_data2[21] = (icm20608_dev.gyro_z_adc & 0xFF);
//	ble_data2[22] = battery_voltage_struct.battery_capacity;
//	for (uint8_t i = 0; i < 16; i++) {
//		ble_data2[23+i] = 0;
//	}
//}

//void send_buffer_eeg_raw_to_bel(void)
//{

//	ble_data_buffer[0]= 0x02; 
//	ble_data_buffer[1]= 0X25;	//header
//	ble_data_buffer[2]= 0x06; 
//	ble_data_buffer[3]= 0x05;
//	ble_data_buffer[4]= 0x04; 
//	ble_data_buffer[5]= 0x03;
//	ble_data_buffer[6]= 0x02; 
//	ble_data_buffer[7]= 0x01;	//mac address
//				
//}
void buffer_eeg_raw_deal(void)
{
	static uint8_t ble_data_buffer[1024] ={0};
	static uint16_t send_length = 0;
	ble_data_buffer[0] = 0x02;
	ble_data_buffer[1] = 0x25;
	ble_data_buffer[2] = 0x01;
	ble_data_buffer[3] = 0x02;
	ble_data_buffer[4] = 0x03;
	ble_data_buffer[5] = 0x04;
	ble_data_buffer[6] = 0x05;
	ble_data_buffer[7] = 0x06;
	ble_data_buffer[8] = 0x00;
	ble_data_buffer[9] = 0x02;
	ble_data_buffer[10] = 0xBB;
	ble_data_buffer[11] = 0xBc;
	ble_data_buffer[12+send_length++] = BUFFER_SEND[5];
//	send_length++;
	ble_data_buffer[12+send_length++] = BUFFER_SEND[6];
//	send_length++;
	if(send_length>= 512) //收完512个字节就可以发送一次数据了 原始脑电数据
	{
			eeg_uart_to_td5322a(ble_data_buffer,524);   //128+12
//		printf("send ble eeg masg time \r\n"); //打印eeg次数
		send_length = 0;
	}
		
//	for (uint8_t i = 0; i < 8; i++) {
//		ble_data[send_length++] = BUFFER_SEND[i];
//	}
}
//// 广播包分两个包发送
//void ble_make_result(void)
//{
//	uint32_t chksum = 0;
//
//	ble_data[15] = 0x15;
//	ble_data[16] = 0xFF;
//	ble_data[17] = 0xBB;
//	ble_data[18] = 0xBB;
//	ble_data[19] = 0x10; // 数据包分包标志第一包
//	ble_data[20] = 0x02;
//	ble_data[21] = BUFFER_SEND[4];
//	chksum += 0x10;
//	chksum += 0x02;
//	chksum += BUFFER_SEND[4];
//	for (uint8_t i = 0; i < 14; i++){
//		ble_data[22+i] = BUFFER_SEND[7+i];
//		chksum += BUFFER_SEND[7+i];
//	}
//	ble_data[36] = (~(uint8_t)chksum);
//
//	ble_data2[15] = 0x15;
//	ble_data2[16] = 0xFF;
//	ble_data2[17] = 0xBB;
//	ble_data2[18] = 0xBB;
//	ble_data2[19] = 0x11; // 数据包分包标志第二包
//	ble_data2[20] = 0x02;
//	ble_data2[21] = BUFFER_SEND[4];
//	chksum = 0;
//	chksum += 0x11;
//	chksum += 0x02;
//	chksum += BUFFER_SEND[4];
//	for(uint8_t i = 0; i < 14; i++)
//	{
//		ble_data2[22+i] = BUFFER_SEND[21+i];
//		chksum += BUFFER_SEND[21+i];
//	}
//	ble_data2[36] = (~(uint8_t)chksum);
//}

// 多指令发包 --> 更改为向LT8960L发送指令
uint8_t send_data_buffer_bel[1024] ={0};
void ble_send_cmd(uint8_t cmd) {
	uint8_t i;
	static uint16_t cnt =0;
	switch (cmd) {
	case CMD_PPG:
		for (i = 0; i <= CMD_PPG_LENGTH; i++) {
			printf("%c", (BUFFER_SEND[i] & 0xFF));
		}
		break;
	case CMD_ICM20608:
//			printf("gyro_x:%d        ", icm20608_dev.gyro_x_adc);
//			printf("gyro_y:%d        ", icm20608_dev.gyro_y_adc);
//			printf("gyro_z:%d        \n", icm20608_dev.gyro_z_adc);
//		LT8960L_Tx_BLE_Data(ble_data2, sizeof(ble_data2), 7);
		break;
	case CMD_EEG_RAW:
		buffer_eeg_raw_deal();
		break;
	case CMD_EEG_RES1:
//		for (i = 0; i < CMD_EEG_RES1_LENGTH; i++) {
//			printf("%c",(BUFFER_SEND[i]&0xFF));
//		}
//		printf("ble success\n");
		ble_make_result();
//		LT8960L_Tx_BLE_Data(ble_data, sizeof(ble_data), 7);
//		LT8960L_Tx_BLE_Data(ble_data2, sizeof(ble_data2), 7); // 广播包分包显示
		break;
	case CMD_EEG_RES2:
		for (i = 0; i < CMD_EEG_RES2_LENGTH; i++) {
			printf("%c", (BUFFER_SEND[i] & 0xFF));
		}
		break;
	case CMD_LUX:
//		ble_make_result2();
//		mcu_uart_to_td5322a((const uint8_t *)ble_data2,0x1e);
//		LT8960L_Tx_BLE_Data(ble_data2, sizeof(ble_data2), 7);
		break;
	case CMD_NONE:
		break;
	default:
		break;
	}
}

// 组织2.5G 命令包
/* 控制通讯，在2476MHz 2459MHz 2467MHz，每间隔3ms发一次数据， SyncWord为：0xB8,0x1B,0x20,0x5A
 * 包裹内容：00 00 00 00 00 00（hex）
 * 第一个字节为油门，00为中位，7F最大，80最小
 * 第二个字节为航向，00为中位，5f最左，A0最右
 * 第三个字节为俯仰，00为中位，5f最前，A0最后
 * 第四个字节为横滚，00为中位，5f最左，A0最右
 * 第五个字节为命令码，0A为一键上升/一键下降，0B为一键下降，2A为一键急停，48为一键校准
 * 第六字节为校验，第一个~第五个异或校验
 */



// 原来飞球遥控协议控制代码的发送，适用于2.4gHz遥控协议和蓝牙8字节遥控协议
// void device_data_send(void) {
//	LT8960L_TxData(USER_Channel1, RF_DATA, 6);
//	LT8960L_TxData(USER_Channel2, RF_DATA, 6);
//	LT8960L_TxData(USER_Channel3, RF_DATA, 6);
//	const uint8_t data_buffer[]={0};
//	memcpy(data_buffer,RF_DATA,sizeof(RF_DATA));
// 	eeg_uart_to_td5322a(RF_DATA,sizeof(RF_DATA));
	
// }

// 11 字节的飞球遥控协议的控制代码的发送，适用于11字节的蓝牙遥控协议
// 发送数据函数，使用11字节的新协议
void device_data_send_new(FlyBallControlProtocol *protocol) {
    // 计算校验位
    protocol->ck = protocol->header ^ protocol->length;
    protocol->ck ^= protocol->rc_roll;
    protocol->ck ^= protocol->rc_pitch;
    protocol->ck ^= protocol->rc_thr;
    protocol->ck ^= protocol->rc_yaw;
    protocol->ck ^= protocol->roll_off;
    protocol->ck ^= protocol->pitch_off;
    protocol->ck ^= protocol->key_num;
    protocol->ck ^= protocol->empty;

    // 发送数据
    static uint8_t send_buffer[11];
    send_buffer[0] = protocol->header;
    send_buffer[1] = protocol->length;
    send_buffer[2] = protocol->rc_roll;
    send_buffer[3] = protocol->rc_pitch;
    send_buffer[4] = protocol->rc_thr;
    send_buffer[5] = protocol->rc_yaw;
    send_buffer[6] = protocol->roll_off;
    send_buffer[7] = protocol->pitch_off;
    send_buffer[8] = protocol->key_num;
    send_buffer[9] = protocol->empty;
    send_buffer[10] = protocol->ck;

    eeg_uart_to_td5322a(send_buffer, sizeof(send_buffer));
}


// 校准
void device_data_cali(void) {
    FlyBallControlProtocol protocol = {
        .header = 0x99,
        .length = 0x08,
        .rc_roll = 0x80,
        .rc_pitch = 0x80,
        .rc_thr = 0x80,
        .rc_yaw = 0x80,
        .roll_off = 0x20,
        .pitch_off = 0x20,
        .key_num = 0x80,
        .empty = 0x00
    };
    device_data_send_new(&protocol);
}

// 一键上升
void device_data_up_real(void) {
    FlyBallControlProtocol protocol = {
        .header = 0x99,
        .length = 0x08,
        .rc_roll = 0x80,
        .rc_pitch = 0x80,
        .rc_thr = 0x80,
        .rc_yaw = 0x80,
        .roll_off = 0x20,
        .pitch_off = 0x20,
        .key_num = 0x01,
        .empty = 0x00
    };
    device_data_send_new(&protocol);
}

// 启动
void device_data_start(void) {
    FlyBallControlProtocol protocol = {
        .header = 0x99,
        .length = 0x08,
        .rc_roll = 0x80,
        .rc_pitch = 0x80,
        .rc_thr = 0x80,
        .rc_yaw = 0x80,
        .roll_off = 0x20,
        .pitch_off = 0x20,
        .key_num = 0x00,
        .empty = 0x00
    };
    device_data_send_new(&protocol);
}

// 上升
void device_data_up(void) {
    FlyBallControlProtocol protocol = {
        .header = 0x99,
        .length = 0x08,
        .rc_roll = 0x80,
        .rc_pitch = 0x80, 
        .rc_thr = 0xC1, //200
			
//        .rc_thr = 0xB9, //185 
//				.rc_thr = 0xA0, //160
		
        .rc_yaw = 0x80,
        .roll_off = 0x20,
        .pitch_off = 0x20,
        .key_num = 0x00,
        .empty = 0x00
    };
    device_data_send_new(&protocol);
}

// 脑电一键起飞
void mental_device_up(void) {
    FlyBallControlProtocol protocol = {
        .header = 0x99,
        .length = 0x08,
        .rc_roll = 0x80,
        .rc_pitch = 0x80,
        .rc_thr = 0xFF,
        .rc_yaw = 0x80,
        .roll_off = 0x20,
        .pitch_off = 0x20,
        .key_num = 0x00,
        .empty = 0x00
    };
    device_data_send_new(&protocol);
}

// 下降
void device_data_down(void) {
    FlyBallControlProtocol protocol = {
        .header = 0x99,
        .length = 0x08,
        .rc_roll = 0x80,
        .rc_pitch = 0x80,
				//.rc_thr = 0x00,
        .rc_thr = 0x5D,   //下降值 93
        .rc_yaw = 0x80,
        .roll_off = 0x20,
        .pitch_off = 0x20,
        .key_num = 0x00,
        .empty = 0x00
    };
    device_data_send_new(&protocol);
}

// 关机和没电立刻下降
void device_data_power_down(void) {
    FlyBallControlProtocol protocol = {
        .header = 0x99,
        .length = 0x08,
        .rc_roll = 0x80,
        .rc_pitch = 0x80,
        .rc_thr = 0x00,
        .rc_yaw = 0x80,
        .roll_off = 0x20,
        .pitch_off = 0x20,
        .key_num = 0x02,
        .empty = 0x00
    };
    device_data_send_new(&protocol);
}

// 航向  1-左  0-右
void device_data_roll(uint8_t direct) {
    FlyBallControlProtocol protocol = {
        .header = 0x99,
        .length = 0x08,
        .rc_yaw = 0x80,
        .rc_pitch = 0x80,
        .rc_thr = 0x80,
        .roll_off = 0x20,
        .pitch_off = 0x20,
        .key_num = 0x00,
        .empty = 0x00
    };
    if (direct == 1) {
        protocol.rc_roll = 0xB5;
    } else {
        protocol.rc_roll = 0x46;
    }
    device_data_send_new(&protocol);
}

// 俯仰 1-前  0-后
void device_data_pitch(uint8_t direct) {
    FlyBallControlProtocol protocol = {
        .header = 0x99,
        .length = 0x08,
        .rc_roll = 0x80,
        .rc_thr = 0x80,
        .rc_yaw = 0x80,
        .roll_off = 0x20,
        .pitch_off = 0x20,
        .key_num = 0x00,
        .empty = 0x00
    };
    if (direct == 1) {
        protocol.rc_pitch = 0xB5;
    } else {
        protocol.rc_pitch = 0x46;
    } 
    device_data_send_new(&protocol);
}

// 翻滚 1-左  0-右
void device_data_yaw(uint8_t direct) {
    FlyBallControlProtocol protocol = {
        .header = 0x99,
        .length = 0x08,
        .rc_pitch = 0x80,
        .rc_thr = 0x80,
        .rc_roll = 0x80,
        .roll_off = 0x20,
        .pitch_off = 0x20,
        .key_num = 0x00,
        .empty = 0x00
    };
    if (direct == 1) {
        protocol.rc_yaw = 0x46;
    } else {
        protocol.rc_yaw = 0xB5;
    }
    device_data_send_new(&protocol);
}

// 无头模式
void device_data_nohead(void) {
    FlyBallControlProtocol protocol = {
        .header = 0x99,
        .length = 0x08,
        .rc_roll = 0x80,
        .rc_pitch = 0x80,
        .rc_thr = 0x80,
        .rc_yaw = 0x80,
        .roll_off = 0x20,
        .pitch_off = 0x20,
        .key_num = 0x10,
        .empty = 0x00
    };
    device_data_send_new(&protocol);
}

// 返航
void device_data_back(void) {
    FlyBallControlProtocol protocol = {
        .header = 0x99,
        .length = 0x08,
        .rc_roll = 0x80,
        .rc_pitch = 0x80,
        .rc_thr = 0x80,
        .rc_yaw = 0x80,
        .roll_off = 0x20,
        .pitch_off = 0x20,
        .key_num = 0x08,
        .empty = 0x00
    };
    device_data_send_new(&protocol);
}

// 变速 0-低速 1-中速 2-高速
void device_data_speed(uint8_t speed) {
    FlyBallControlProtocol protocol = {
        .header = 0x99,
        .length = 0x08,
        .rc_roll = 0x80,
        .rc_pitch = 0x80,
        .rc_thr = 0x80,
        .rc_yaw = 0x80,
        .roll_off = 0x20,
        .pitch_off = 0x20,
        .empty = 0x00
    };
    if (speed == 0) {
        protocol.key_num = 0x0F;
    } else if (speed == 1) {
        protocol.key_num = 0x1F;
    } else if (speed == 2) {
        protocol.key_num = 0x2F;
    } else {
        protocol.key_num = 0x00;
    }
    device_data_send_new(&protocol);
}

// 翻滚 0-左翻 1-右翻 2-前翻 3-后翻 (实测 2-右翻  3-左翻)
void device_data_rollover(uint8_t direct) {
    FlyBallControlProtocol protocol = {
        .header = 0x99,
        .length = 0x08,
        .rc_roll = 0x80,
        .rc_pitch = 0x80,
        .rc_thr = 0x80,
        .rc_yaw = 0x80,
        .roll_off = 0x20,
        .pitch_off = 0x20,
        .empty = 0x00
    };
    if (direct == 0) {
        protocol.key_num = 0x40;
    } else if (direct == 1) {
        protocol.key_num = 0x41;
    } else if (direct == 2) {
        protocol.key_num = 0x42;
    } else if (direct == 3) {
        protocol.key_num = 0x43;
    } else {
        protocol.key_num = 0x00;
    }
    device_data_send_new(&protocol);
}

// 空包
void device_data_none(void) {
    FlyBallControlProtocol protocol = {
        .header = 0x99,
        .length = 0x08,
        .rc_roll = 0x80,
        .rc_pitch = 0x80,
        .rc_thr = 0x80,
        .rc_yaw = 0x80,
        .roll_off = 0x20,
        .pitch_off = 0x20,
        .key_num = 0x00,
        .empty = 0x00
    };
    device_data_send_new(&protocol);
}

// 遥控器上电，在2405mhz，发出100包数据， SyncWord为：0x03,0x80,0x5A,0x5A
// 包裹内容：55 E7 B8 1B 20 31（hex）
//void device_init(void) {
//	uint8_t i;
////	LT8960L_INIT(0);
//	RF_DATA[0] = 0x55;
//	RF_DATA[1] = 0xE7;
//	RF_DATA[2] = 0xB8;
//	RF_DATA[3] = 0x1B;
//	RF_DATA[4] = 0x20;
//	RF_DATA[5] = 0x31;
//	for (i = 0; i < 10; i++) {  //10
////		LT8960L_TxData(Test_Channel, RF_DATA, 6);
//	}
////	LT8960L_INIT(1);
////	for (i = 0; i < 5; i++) {
////		device_data_up();
////	}
////	device_data_start();
////	HAL_Delay(3);
////	device_data_nohead();
////	HAL_Delay(3);
//	flag_3ms = 0;
//}

// 取专注度结果
void eeg_make_result(void) {
	static uint8_t Meditation_data_temp = 0;
	Result_data_new = (uint8_t) BUFFER_SEND[32];
	Meditation_data_temp = (uint8_t) BUFFER_SEND[33];
	//测试数据
	static uint8_t attention_temp_flg =0;
//	printf("attention is :%d\n",  Result_data_new);
//	printf("Meditation is :%d\n",  Meditation_data_temp);
//	Result_data_new = 40;
	if ((Result_data_new > Result_data_old) && Result_data_new >= 30 && attention_temp_flg == 0)
	{

		Result_Attention = 1;  // 上升
//		flag_count_temp++;
//		if(!flag_res_rx) // 确认有脑电才起飞
//		{
			attention_temp_flg =1;
			Result_data_old = Result_data_new;
			global_ctrl_device.mental_training_mode_ctrl_fly_time_flg =1;
//			device_data_none();
//			flag_res_rx = 1;
//		}
	}
	else if ((Result_data_new <= Result_data_old) && Result_data_new < 30)
	{
		Result_Attention = 2;  // 下降
		Result_data_old = Result_data_new;
		attention_temp_flg =0;
		global_ctrl_device.mental_training_mode_ctrl_fly_time_flg = 1;


	}
//	else if(Result_data_new == 0 && attention_temp_flg == 0)
//	{
//		if(global_ctrl_device.mental_training_mode_ctrl_fly_time_flg == 0)
//		{
//			Result_Attention = 2;  // 下降
//			global_ctrl_device.mental_training_mode_ctrl_fly_time_flg = 1;
//		}
//	}
//
//	else if(Result_data_new > 30 && global_ctrl_device.eye_muscle_training_mode_ctrl_fly_time_flg == 0 && attention_temp_flg ==0)
//	{
//		Result_Attention = 3;  // 保持不变
//	}

//	if(Result_data_new > 0)
//	{

//	}
//	printf("Result_data_new: %d,  flag_count_temp:%d\n", Result_data_new, flag_count_temp);
}

void reset_time_flag(void)
{
	printf("4ms chufa : Result_ALgo:%d\n", Result_Algo_old);
	if (flag_userTime == 1) {
		Result_Algo_old = 0;
		printf("flag_userTime = 1\n");
		
		
	}
}

void task_make_acc(void) {
	uint32_t chksum = 0;
	BUFFER_SEND_ACC[0] = 0xAA;
	BUFFER_SEND_ACC[1] = 0xAA;
	BUFFER_SEND_ACC[2] = CMD_ICM20608_LENGTH;

	BUFFER_SEND_ACC[3] = icm20608_dev.accel_x_adc;
	BUFFER_SEND_ACC[4] = icm20608_dev.accel_x_adc >> 8;
	BUFFER_SEND_ACC[5] = icm20608_dev.accel_y_adc;
	BUFFER_SEND_ACC[6] = icm20608_dev.accel_y_adc >> 8;
	BUFFER_SEND_ACC[7] = icm20608_dev.accel_z_adc;
	BUFFER_SEND_ACC[8] = icm20608_dev.accel_z_adc >> 8;

	BUFFER_SEND_ACC[9] = icm20608_dev.temp_adc;
	BUFFER_SEND_ACC[10] = icm20608_dev.temp_adc >> 8;

	BUFFER_SEND_ACC[11] = icm20608_dev.gyro_x_adc;
	BUFFER_SEND_ACC[12] = icm20608_dev.gyro_x_adc >> 8;
	BUFFER_SEND_ACC[13] = icm20608_dev.gyro_y_adc;
	BUFFER_SEND_ACC[14] = icm20608_dev.gyro_y_adc >> 8;
	BUFFER_SEND_ACC[15] = icm20608_dev.gyro_z_adc;
	BUFFER_SEND_ACC[16] = icm20608_dev.gyro_z_adc >> 8;

	for (int i = 0; i < CMD_ICM20608_LENGTH-4; i++) {
		chksum += BUFFER_SEND_ACC[3+i];
	}
	BUFFER_SEND_ACC[17] = (~((uint8_t)chksum));
}
//uint8_t ble_data2[]={0X02, 0X25,  											//header
//					0x06, 0x05, 0x04, 0x03, 0x02, 0x01,     				//mac address
//					0x02, 0x01, 0x1A,    									//flags
//					0x03, 0x19, 0xc1, 0x03,   								//device
//					0x08, 0x16, 0x11, 0x11, 0x11, 0xb0, 0xb0, 0xb0,0x00,    //uuid
//					0x0E, 0xFF, 0x4C, 0x54, 0x38, 0x39, 0x36, 0x30,0x4C,0x76,0x61,0x6C,0x3D,0x78,0x78, //name
//				};
//void ble_make_icm20608(void)
//{
//	task_make_acc();
//	ble_data2[15] = 0x15;
//	ble_data2[16] = 0xFF;
//	ble_data2[17] = 0x01;
//	ble_data2[18] = 0x00;
//	for(uint8_t i = 19; i < 19+CMD_ICM20608_LENGTH; i++)
//	{
//		ble_data2[i] = BUFFER_SEND_ACC[i-19];

//	}
//}
//处理蓝牙低功耗任务
void ble_low_power_task(void)
{
	static uint8_t ble_daily_mode_flg =0;
	static uint8_t ble_dally_mode_10min_flg =0;
	static uint8_t ble_mode_change_flg = 0;
	static char buf[VL53L0X_MAX_STRING_LENGTH]; //测试模式字符串字符缓冲区
	VL53L0X_Error Status = VL53L0X_ERROR_NONE; //工作状态
	uint8_t successParse;
	// lux temp
	float lux = 0.0f;
	uint16_t iExponent, iMantissa;
	float final_lux;
	static uint8_t eeg_type_raw_on_flg =0;

//	HAL_UART_Receive_IT(&huart1, receiveBuffer, RECEIVE_BUFFER_LENGTH); //必须要加上这个才能发生中断
	HAL_UART_Receive_IT(&huart1, receiveBuffer, RECEIVE_BUFFER_LENGTH); //必须要加上这个才能发生中断
//	HAL_UART_Receive_IT(&huart3, receiveBuffer2, RECEIVE_BUFFER_LENGTH); //必须要加上这个才能if发生中断
	if(resFlag == 1)
	{
		for (uint16_t i = 0; i < RECEIVE_BUFFER_LENGTH; i++) {

			successParse = parse_SW_data((uint8_t) receiveBufferTem[i]);
//			printf("%c", (receiveBufferTem[i] & 0xFF));
			if (successParse == 1) {
				switch (payloadLength) {
				 //需要EEG RAW DATA
					case RAW_EEG_LENGTH:
						for (uint16_t j = 0; j < CMD_EEG_RAW_LENGTH; j++) {
							BUFFER_SEND[j] = BUFFER_RECEIVED[j];
//							printf("%c", (BUFFER_SEND[j] & 0xFF));
						}
						//日常的发送模式0xa9的发送时间
						if(td5322a_struct.work_peripheral_role_flg == TD5322A_BLE_PERIPHERAL_MODE && (global_ctrl_device.key_mode_state == IMMEDIATE_MODE||(global_ctrl_device.low_power_consumption_time_cnt < 0x320 && global_ctrl_device.key_mode_state == DAILY_MODE)))
						{
							ble_send_cmd(CMD_EEG_RAW);
						}
						break;

				case RES_EEG_LENGTH:
					for (uint16_t j = 0; j < CMD_EEG_RES1_LENGTH; j++) {
						BUFFER_SEND[j] = BUFFER_RECEIVED[j];
//						printf("%c", (BUFFER_SEND[j] & 0xFF));
					}
					//日常的发送模式0xa9的发送时间
					if(td5322a_struct.work_peripheral_role_flg == TD5322A_BLE_PERIPHERAL_MODE && (global_ctrl_device.key_mode_state == IMMEDIATE_MODE||(global_ctrl_device.low_power_consumption_time_cnt < 0x320 && global_ctrl_device.key_mode_state == DAILY_MODE)))
					{
						ble_send_cmd(CMD_EEG_RES1);
						printf("send ble eeg masg time \r\n"); //打印eeg次数
					}
					else if(global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.mental_training_mode ==1)
					{
						eeg_make_result();
					}
					else
					{
						;
					}
					break;

				case RES2_EEG_LENGTH:
					for (uint8_t j = 0; j < CMD_EEG_RES2_LENGTH; j++) {
						BUFFER_SEND[j] = BUFFER_RECEIVED[j];
					}
					if(td5322a_struct.work_peripheral_role_flg == TD5322A_BLE_PERIPHERAL_MODE)
					{
						ble_send_cmd(CMD_EEG_RES2);
//						printf("send ble eeg masg time \r\n"); //打印eeg次数
					}
					break;

				default:
					break;
				}

			}
		}
		resFlag = 0;
	}
	
	//50ms进一次中断，0xe10 20*60*3 0xE10
	//进入日常模式后计时0xEA60 2000进入蓝牙模式的时候这个global_ctrl_device.key_mode_time_flg 置1了，所以进入蓝牙日常模式开始计时即时模式
	//0E10 定时了3分钟 现在延迟采样时间 设置为FFFF
	if(global_ctrl_device.key_mode_time_cnt >0x4650  && global_ctrl_device.key_mode_state == IMMEDIATE_MODE) // 3分钟到了，蓝灯只有在采集的时候灯才会亮，采集完了灭
	{
		global_ctrl_device.key_mode_time_flg = 0; //关掉3分钟的定时器
//		global_ctrl_device.DEVIC_MODE.training_mode = DAILY_MODE; //进入到日常模式 进入到睡眠模式
		global_ctrl_device.key_mode_state = DAILY_MODE; //进入低功耗模式
		global_ctrl_device.low_power_consumption_time_flg = 1; //开启低功耗计时
//		ble_daily_mode_flg =1;
		printf("turn on daily mode\r\n");
		Device_Enter_OFF(); //日常模式作为低功耗模式
	}
	//50ms进一次中断，0x2ee0 20*60*6 1c20 
	//50ms进一次中断，0x4Bd 20*40/5.2
	//在日常模式下，发送2*1000*3ms，然后关掉系统电源进入低功耗模式
	else if(global_ctrl_device.low_power_consumption_time_cnt > 0x320 && ble_dally_mode_10min_flg == 1 &&global_ctrl_device.key_mode_state == DAILY_MODE)
	{
		ble_dally_mode_10min_flg = 0;
		printf("turn low power mode\r\n");
		Device_Enter_OFF();
//		global_ctrl_device.power_ctrl_flg = DEVICE_POWER_OFF;  //系统初始化开机，然后采集数据
	}
	//50ms进一次中断，0x2ee0 20*60*6 1c20 定时器时间10s约为实测52s 如果6分钟 20*60*6/5.2
	//10分钟后采集数据给app手机0x927C0
	else if(global_ctrl_device.low_power_consumption_time_cnt >= 0x081C)
	{
//		global_ctrl_device.power_ctrl_flg = DEVICE_POWER_ON;  //系统初始化开机，然后采集数据
		global_ctrl_device.low_power_consumption_time_cnt = 0;
		Device_Enter_ON();
//		LT8960L_BLE_INIT(); // ble 初始化
		ble_daily_mode_flg = 0;
//					global_ctrl_device.low_power_consumption_time_flg =1;
		ble_dally_mode_10min_flg = 1;
		printf("send 1.5s masg\r\n");
	}
}

// 主任务处理
void work_task(void) {
//	uint8_t successParse;
//	uint8_t i, j;
//	HAL_StatusTypeDef sta;
	uint8_t len = 0;
	uint8_t rx_buffer[6] = {0};
	static uint8_t flag_gyro_10ms = 0; // 陀螺仪10ms标志
	static uint8_t mental_training_time_cnt =0;
//	static char buf[VL53L0X_MAX_STRING_LENGTH]; //测试模式字符串字符缓冲区
//	static float coef[12] = {0.01, 0.02, 0.04, 0.08, 0.16, 0.32, 0.64, 1.28, 2.56, 5.12, 10.24, 20.48};
	static uint8_t sEye_traning_mode_flg= 0; //眼肌控制标志位
	static uint8_t sEye_traning_mode_backward_forward_flg= 0; //眼肌控制标志位
	static uint8_t previous_run_state = RUN_STATE_IDLE;
	static uint8_t mental_training_down_count =0;
	static uint8_t mental_training_up_flg =0;
	static uint8_t s_work_peripheral_role_flg =0 ;
	static uint16_t other_data_time_cnt =0;
	td5322a_task();	
	ProcessADC(); //检测电池电压
	if (flag_1ms > 2)
	{
		flag_1ms = 1;
		flag_gyro_10ms++;
		if(td5322a_struct.work_peripheral_role_flg == TD5322A_BLE_CENTRAL_MODE)
		{
			// 运动轨迹、速度调整
			switch (runState)
			{

				case RUN_STATE_START:
					device_data_up();
					global_ctrl_device.ctrl_fly_time_flg = 1;
					if (global_ctrl_device.ctrl_fly_time_cnt > 399 && global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.mental_training_mode==0) {  // (起飞高度设置值，值越大起飞越高) 地上起飞 > 700
						runState = RUN_STATE_HOLD; //RUN_STATE_DOWN; //RUN_STATE_LEFT_ROLL;
						global_ctrl_device.ctrl_fly_time_flg = 0;
						runStateOld = RUN_STATE_START;
//						printf("RUN STATE:%d start,	flag_count_temp:%ld\n", runState, flag_count_temp);
					}

					if((global_ctrl_device.ctrl_fly_time_cnt > 200) && (global_ctrl_device.ctrl_fly_time_cnt < 260))
					{
						device_data_none();
					}
					break;
//				case RUN_STATE_LEFT_ROLL:
//					device_data_roll(1);
//
//					if (flag_3ms > 170) { // 200
//						runState = RUN_STATE_HOLD; //RUN_STATE_FOREWORD_PITCH;
//						flag_3ms = 0;
//						runStateOld = RUN_STATE_LEFT_ROLL;
//						printf("RUN STATE:%d left,	flag_count_temp:%ld\n", runState, flag_count_temp);
//					}
//
//					break;
				case RUN_STATE_FOREWORD_PITCH:
					device_data_pitch(1);
					global_ctrl_device.ctrl_fly_time_flg =1; //开飞行时间
					//脖子训练
					if (global_ctrl_device.ctrl_fly_time_cnt > 100 &&global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.cervical_training_mode == 1) {
						runState = RUN_STATE_IDLE; //RUN_STATE_UP;
						global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间
						runStateOld = RUN_STATE_FOREWORD_PITCH;
//						printf("RUN STATE:%d foreword,	flag_count_temp:%ld\n", runState, flag_count_temp);
					}
					//眼肌训练
					else if (global_ctrl_device.ctrl_fly_time_cnt > 100 &&global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.eye_muscle_training_mode==1 ) {
						runState = RUN_STATE_HOLD; //RUN_STATE_UP;
						global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间
						runStateOld = RUN_STATE_FOREWORD_PITCH;
//						printf("RUN STATE:%d foreword,	flag_count_temp:%ld\n", runState, flag_count_temp);
					}

					break;
				case RUN_STATE_RIGHT_ROLL:
					device_data_roll(0);
					global_ctrl_device.ctrl_fly_time_flg =1;
					if (global_ctrl_device.ctrl_fly_time_cnt > 100) { // 280
						runState = RUN_STATE_HOLD; //RUN_STATE_DOWN;
						global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间
						runStateOld = RUN_STATE_RIGHT_ROLL;
//						printf("RUN STATE:%d right,	flag_count_temp:%ld\n", runState, flag_count_temp);
					}

					break;
				case RUN_STATE_BACKWORD_PITCH:
					global_ctrl_device.ctrl_fly_time_flg = 1; //开定时器
					device_data_pitch(0);
					if (global_ctrl_device.ctrl_fly_time_cnt >100) {
						runState = RUN_STATE_IDLE;
						runStateOld = RUN_STATE_BACKWORD_PITCH;
						global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间
//						printf("RUN STATE:%d backword,	flag_count_temp:%ld\n", runState, flag_count_temp);
					}
					break;
//				case RUN_STATE_LEFT_ROLLOVER:
//					device_data_rollover(3); // 0
//					runState = RUN_STATE_HOLD; //RUN_STATE_LEFT_ROLL;
//					runStateOld = RUN_STATE_LEFT_ROLLOVER;
//					flag_roll_change = 1;
//					flag_3ms = 0;
//					printf("RUN STATE:%d letfrollover,	flag_count_temp:%ld\n", runState, flag_count_temp);
//
//				break;
//				case RUN_STATE_RIGHT_ROLLOVER:
//					device_data_rollover(2); // 1
//					runState = RUN_STATE_HOLD; //RUN_STATE_LEFT_ROLL;
//					runStateOld = RUN_STATE_RIGHT_ROLLOVER;
//					flag_roll_change = 0;
//					flag_3ms = 0;
//					printf("RUN STATE:%d rightrollover,	flag_count_temp:%ld\n", runState, flag_count_temp);

//				break;
				case RUN_STATE_UP:
					device_data_up();
					global_ctrl_device.ctrl_fly_time_flg =1;
					if (global_ctrl_device.ctrl_fly_time_cnt > 110) { // 270
						runState = RUN_STATE_HOLD; //RUN_STATE_DOWN; //RUN_STATE_RIGHT_ROLL;
						global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间
						runStateOld = RUN_STATE_UP;
//						printf("RUN STATE:%d up,	flag_count_temp:%ld\n", runState, flag_count_temp);
					}
					break;
				case RUN_STATE_DOWN:
					device_data_down();
					global_ctrl_device.ctrl_fly_time_flg =1;
					if (global_ctrl_device.ctrl_fly_time_cnt > 500) {  //700
						runState = RUN_STATE_HOLD; //RUN_STATE_UP;  //RUN_STATE_BACKWORD_PITCH;
						global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间
						runStateOld = RUN_STATE_DOWN;
//						printf("RUN STATE:%d down,	flag_count_temp:%ld\n", runState, flag_count_temp);
					}
					break;
					
				case RUN_STATE_LEFT_ROLL_HAND: //左摇头 向左
					device_data_roll(1);
					global_ctrl_device.ctrl_fly_time_flg = 1;				
					//脖子训练
					if (global_ctrl_device.ctrl_fly_time_cnt > 100 && global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.cervical_training_mode == 1)
					{
						runState = RUN_STATE_IDLE;
						global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间
//						printf("RUN STATE:%d,	flag_count_temp:%ld\n", runState,
//								flag_count_temp);
					}
					//眼肌训练
					else if (global_ctrl_device.ctrl_fly_time_cnt > 100 && global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.eye_muscle_training_mode == 1)
					{
						runState = RUN_STATE_IDLE;
						global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间
//						printf("RUN STATE:%d,	flag_count_temp:%ld\n", runState,
//								flag_count_temp);
					}
					
					//专注度
					else if (global_ctrl_device.ctrl_fly_time_cnt > 100 && global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.mental_training_mode == 1)
					{
						runState = RUN_STATE_IDLE;
						global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间
//						printf("RUN STATE:%d,	flag_count_temp:%ld\n", runState,
//								flag_count_temp);
					}
					break;
				case RUN_STATE_RIGHT_ROLL_HAND://右摇头 向右
					global_ctrl_device.ctrl_fly_time_flg =1; //开定时器
					device_data_roll(0);
					if (global_ctrl_device.ctrl_fly_time_cnt > 100 && global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.cervical_training_mode == 1)
					{
						runState = RUN_STATE_IDLE;
						global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间
					}
					//眼肌训练
					else if (global_ctrl_device.ctrl_fly_time_cnt > 100 && global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.eye_muscle_training_mode == 1)
					{
						runState = RUN_STATE_IDLE;
						global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间
					}
					//专注度
					else if (global_ctrl_device.ctrl_fly_time_cnt > 100 && global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.mental_training_mode == 1)
					{
						runState = RUN_STATE_IDLE;
						global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间
//						printf("RUN STATE:%d,	flag_count_temp:%ld\n", runState,
//								flag_count_temp);
					}
//					printf("RUN STATE:%d,	flag_count_temp:%ld\n", runState,
//							flag_count_temp);
					break;
					//向前处理
				case RUN_STATE_FOREWORD_PITCH_HAND:
					device_data_pitch(1);
					global_ctrl_device.ctrl_fly_time_flg =1; //开定时器
					//脖子训练
					if (global_ctrl_device.ctrl_fly_time_cnt > 100 &&global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.cervical_training_mode == 1)//脖子处理
					{
						runState = RUN_STATE_IDLE;
						global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间 = 0;
//						printf("RUN STATE:%d\n", runState);
					}
					//眼肌
					else if (global_ctrl_device.ctrl_fly_time_cnt > 100 &&global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.eye_muscle_training_mode == 1)//
					{
						runState = RUN_STATE_IDLE;
						global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间
//						printf("RUN STATE:%d\n", runState);
					}
					//专注度
					else if (global_ctrl_device.ctrl_fly_time_cnt > 100 && global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.mental_training_mode == 1)
					{
						runState = RUN_STATE_IDLE;
						global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间
//						printf("RUN STATE:%d,	flag_count_temp:%ld\n", runState,
//								flag_count_temp);
					}
					break;
					//向后处理
				case RUN_STATE_BACKWORD_PITCH_HAND:
					device_data_pitch(0);
					global_ctrl_device.ctrl_fly_time_flg =1; //开定时器
					//脖子训练模式
					if (global_ctrl_device.ctrl_fly_time_cnt > 100 &&global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.cervical_training_mode == 1)
					{
						runState = RUN_STATE_IDLE;
						global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间
//						printf("RUN STATE:%d,	flag_count_temp:%ld\n", runState,
//																		flag_count_temp);
					}
					//眼肌训练模式
					else if (global_ctrl_device.ctrl_fly_time_cnt > 100 &&global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.eye_muscle_training_mode == 1)
					{
						runState = RUN_STATE_IDLE;
						global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间
//						printf("RUN STATE:%d,	flag_count_temp:%ld\n", runState,
//																		flag_count_temp);
					}
					//专注度
					else if (global_ctrl_device.ctrl_fly_time_cnt > 100 && global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.mental_training_mode == 1)
					{
						runState = RUN_STATE_IDLE;
						global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间
//						printf("RUN STATE:%d,	flag_count_temp:%ld\n", runState,
//								flag_count_temp);
					}

					break;
				case RUN_STATE_UP_HAND:
					if(previous_run_state != RUN_STATE_DOWN_HAND &&global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.mental_training_mode== 0) //要等到下降完成后才能执行
					{
						previous_run_state = RUN_STATE_UP_HAND;
						device_data_up();
						global_ctrl_device.ctrl_fly_time_flg =1; //开定时器
						//非脑训练
						if (global_ctrl_device.ctrl_fly_time_cnt > 100) {
							runState = RUN_STATE_IDLE;
							global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间
//							printf("RUN STATE:%d,	flag_count_temp:%ld\n", runState,
//									flag_count_temp);
						}
						//抬头后运动标志位才启动
						if(global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.eye_muscle_training_mode== 1 &&sEye_traning_mode_flg ==0)
						{
							sEye_traning_mode_flg = 1;
						}
					}
//					else if(global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.mental_training_mode== 1)
//					{
//						previous_run_state = RUN_STATE_UP_HAND;
//						device_data_up();
//						global_ctrl_device.ctrl_fly_time_flg = 1; //开定时器
//						if(global_ctrl_device.ctrl_fly_time_cnt > 230 && global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.mental_training_mode== 1)
//						{
//							runState = RUN_STATE_IDLE; //RUN_STATE_DOWN; //RUN_STATE_LEFT_ROLL;
//							Result_Algo = ALGO_IDLE;
//							global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间
////							runStateOld = RUN_STATE_START;
//							printf("mental_training extern  is  ok\n\r");
//						}
//					}
					break;
				case RUN_STATE_DOWN_HAND:
					device_data_down();
					global_ctrl_device.ctrl_fly_time_flg =1; //开定时器
					previous_run_state = RUN_STATE_DOWN_HAND;
					//非脑训练
					if (global_ctrl_device.ctrl_fly_time_cnt > 500 && global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.mental_training_mode== 0) {
						runState = RUN_STATE_IDLE;
						previous_run_state =RUN_STATE_IDLE;
						global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间
//						HAL_Delay(300);

					}
					//脑训练
//					else if(global_ctrl_device.ctrl_fly_time_cnt > 100 && global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.mental_training_mode== 1)
//					{
//						runState = RUN_STATE_IDLE; //RUN_STATE_DOWN; //RUN_STATE_LEFT_ROLL;
//						previous_run_state =RUN_STATE_IDLE;
//						Result_Algo = ALGO_IDLE;
//						global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间
//						runStateOld = RUN_STATE_START;


//					}

//					printf("RUN STATE:%d start,	flag_count_temp:%ld\n", runState, flag_count_temp);
					break;
				case RUN_STATE_IDLE:
					device_data_none();
					previous_run_state = RUN_STATE_IDLE;
					global_ctrl_device.ctrl_fly_time_flg =1; //开定时器
					if (global_ctrl_device.ctrl_fly_time_cnt > 1000) {
//						runState = RUN_STATE_LEFT_ROLL;
						leftCount = 0;
						global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间
						//						printf("RUN STATE:%d,	flag_count_temp:%ld\n", runState,
//								flag_count_temp);
					}
					break;
				case RUN_STATE_HOLD:
					device_data_none();
					global_ctrl_device.ctrl_fly_time_flg =1; //开定时器
					if (global_ctrl_device.ctrl_fly_time_cnt > 300) {  //300
						// 配置运动轨迹
						switch (runStateOld) {
							case RUN_STATE_START:
								runState = RUN_STATE_LEFT_ROLLOVER; //RUN_STATE_LEFT_ROLL;
								break;
							case RUN_STATE_LEFT_ROLL:
								leftCount++;
								if (leftCount == 1 || leftCount == 3)
								{
									runState = RUN_STATE_UP;
								}
								else if (leftCount == 2)
								{
									runState = RUN_STATE_RIGHT_ROLLOVER;//RUN_STATE_FOREWORD_PITCH;
								}
//								else if (leftCount == 3)
//								{
//									runState = RUN_STATE_LEFT_ROLLOVER;
//								}
								else if (leftCount == 4)
								{
									runState = RUN_STATE_RIGHT_ROLLOVER;//RUN_STATE_BACKWORD_PITCH;
									//leftCount = 0;
								}
								break;
							case RUN_STATE_FOREWORD_PITCH:
								runState = RUN_STATE_LEFT_ROLLOVER; //RUN_STATE_LEFT_ROLL;
								break;
							case RUN_STATE_RIGHT_ROLL:
								runState = RUN_STATE_DOWN;
								break;
							case RUN_STATE_BACKWORD_PITCH:
								runState = RUN_STATE_LEFT_ROLLOVER; //RUN_STATE_LEFT_ROLL;
								break;
							case RUN_STATE_LEFT_ROLLOVER:
									runState = RUN_STATE_LEFT_ROLL;
								break;
							case RUN_STATE_RIGHT_ROLLOVER:
								if (leftCount == 4) {
									runState = RUN_STATE_BACKWORD_PITCH;
									leftCount = 0;
								} else {
									runState = RUN_STATE_FOREWORD_PITCH; //RUN_STATE_LEFT_ROLL;
								}
								break;
							case RUN_STATE_UP:
								runState = RUN_STATE_RIGHT_ROLL;
								break;
							case RUN_STATE_DOWN:
								runState = RUN_STATE_LEFT_ROLL;
								break;
							default:
								break;
						}
						global_ctrl_device.ctrl_fly_time_flg = 0; //关飞行时间 = 0;
					}
					break;
				case RUN_STATE_NONE:
//					device_data_none();
					global_ctrl_device.ctrl_fly_time_flg = 0; //关定时器
					break;
				default:
					break;
			}
			// device_data_none();
		}

		// 陀螺仪控制
		if (Result_Algo == ALGO_LEFT || Result_Algo == ALGO_RIGHT
				|| Result_Algo == ALGO_UP || Result_Algo == ALGO_DOWN
				|| Result_Algo == ALGO_LEFTSIDE || Result_Algo == ALGO_RIGHTSIDE) {
//			flag_userTime = 0;
//			printf("flag_userTime = 0 Result_Algo:%d\n", Result_Algo);
			HAL_Delay(5);
//			flag_timeStart = 1;
			Result_Algo_old = Result_Algo;
			switch (Result_Algo) {
			case ALGO_LEFT:
				runState = RUN_STATE_LEFT_ROLL_HAND;
				break;
			case ALGO_RIGHT:
				runState = RUN_STATE_RIGHT_ROLL_HAND;
				break;
			case ALGO_UP:
				runState = RUN_STATE_UP_HAND;
				break;
			case ALGO_DOWN:
				runState = RUN_STATE_DOWN_HAND;
				break;
			case ALGO_LEFTSIDE:
				runState = RUN_STATE_FOREWORD_PITCH_HAND;
				break;
			case ALGO_RIGHTSIDE:
				runState = RUN_STATE_BACKWORD_PITCH_HAND;
				break;
			case ALGO_IDLE:
				break;
			default:
				break;
			}
		}

	}
//	static uint8_t data_buffer_t[] ={0};
	//加操作系统处理
	
//	if(other_data_time_cnt > 300)
//	{
//		other_data_time_cnt =0;
////		rawLux = calculate_lux_int();
////		 vl53l0x_start_single_test(&vl53l0x_dev, &vl53l0x_data, data_buffer_t); //执行一次测量
//	}
	// 陀螺仪采样100Hz
	if(flag_gyro_10ms >= 2)
	{
		flag_gyro_10ms = 0;
		
//		icm20608_getdata();
//			//眼部训练只做向上和向下，左右。起飞即可向前向后是固定，飞行
//		Result_Algo = Acc_algorithm_result(icm20608_dev.gyro_x_adc, icm20608_dev.gyro_y_adc, icm20608_dev.gyro_z_adc);
////		printf("gyro_x_adc:%d        gyro_y_adc:%d        gyro_z_adc:%d         \r\n",icm20608_dev.gyro_x_adc, icm20608_dev.gyro_y_adc, icm20608_dev.gyro_z_adc);
	
//		printf("flag_gyro is ok \r\n");
		//眼部训练
		if(global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.eye_muscle_training_mode == 1)
		{
			icm20608_getdata();
			//眼部训练只做向上和向下，左右。起飞即可向前向后是固定，飞行
			Result_Algo = Acc_algorithm_result(icm20608_dev.gyro_x_adc, icm20608_dev.gyro_y_adc, icm20608_dev.gyro_z_adc);
			//需要先判断起飞
			if(Result_Algo ==ALGO_LEFT||Result_Algo ==ALGO_RIGHT||Result_Algo ==ALGO_UP||Result_Algo ==ALGO_DOWN|| Result_Algo ==ALGO_LEFTSIDE||Result_Algo ==ALGO_RIGHTSIDE)
			{
//				if(sEye_traning_mode_flg== 0 &&Result_Algo ==5) //仅有起飞的时候才会让它控制眼肌控制
//				{
//					sEye_traning_mode_flg =1;
////					printf("up hand\r\n");
//					global_ctrl_device.eye_muscle_training_mode_ctrl_fly_time_flg = 1;
//				}else &&sEye_traning_mode_flg ==1
//				if((Result_Algo ==3||Result_Algo ==4||Result_Algo ==5||Result_Algo ==6 || Result_Algo ==7||Result_Algo ==8))
				{
					//左右的时候，需要清零让他不用走
//					printf("zore hand\r\n");
					if(sEye_traning_mode_flg == 1)//抬头起来了的时候启动定时器飞行
					{
						global_ctrl_device.eye_muscle_training_mode_ctrl_fly_time_flg = 1;
						global_ctrl_device.eye_muscle_training_mode_ctrl_fly_time_cnt = 0;
					}
				}

			}

			//开始飞行&& runState == RUN_STATE_IDLE
			if(global_ctrl_device.eye_muscle_training_mode_ctrl_fly_time_cnt >1100 && sEye_traning_mode_backward_forward_flg ==0  &&sEye_traning_mode_flg == 1)
			{
				Result_Algo = ALGO_LEFTSIDE;
				global_ctrl_device.eye_muscle_training_mode_ctrl_fly_time_cnt =0;
				sEye_traning_mode_backward_forward_flg  =1;
			}
			//&& runState == RUN_STATE_IDLE
			else if(global_ctrl_device.eye_muscle_training_mode_ctrl_fly_time_cnt >1100 &&sEye_traning_mode_flg == 1 && sEye_traning_mode_backward_forward_flg == 1)
			{

				Result_Algo = ALGO_RIGHTSIDE;
				global_ctrl_device.eye_muscle_training_mode_ctrl_fly_time_cnt =0;
				sEye_traning_mode_backward_forward_flg  =0;
			}


		}
		//脖子训练
		else if(global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.cervical_training_mode == 1)
		{
			icm20608_getdata();
			Result_Algo = Acc_algorithm_result(icm20608_dev.gyro_x_adc, icm20608_dev.gyro_y_adc, icm20608_dev.gyro_z_adc);
			if(sEye_traning_mode_flg == 1)
			{
				sEye_traning_mode_backward_forward_flg =0;
				global_ctrl_device.eye_muscle_training_mode_ctrl_fly_time_flg =0;
				sEye_traning_mode_flg =0;
			}

		}
		else if(global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.mental_training_mode == 1)
		{
			icm20608_getdata();
			Result_Algo = Acc_algorithm_result(icm20608_dev.gyro_x_adc, icm20608_dev.gyro_y_adc, icm20608_dev.gyro_z_adc);
			if(Result_Algo == ALGO_UP||Result_Algo == ALGO_DOWN)
			{
				Result_Algo = ALGO_IDLE;
			}
		}
	}
	ble_low_power_task();
	//脑力训练模式 因为这个飞碟的控制的时间是3ms发送一次数据，所以必须是这个才行定时器3ms发送一次
	if(global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.mental_training_mode ==1 && flag_1ms >= 1)
	{
		// 处理专注度&& flag_userTime_2 == 1
		if (Result_Attention == 1 ) // 上升
		{

			if(mental_training_down_count > 6)//下降超过7次才会上一次
			{
				device_data_up();


				if(global_ctrl_device.mental_training_mode_ctrl_fly_time_cnt > 900)
				{
					global_ctrl_device.mental_training_mode_ctrl_fly_time_cnt = 0;// 定时器关
//					Result_Attention =0;
					mental_training_up_flg =0;
					device_data_none();
					mental_training_down_count = 0;
					printf("up is over\n") ;
				}
				else if((global_ctrl_device.mental_training_mode_ctrl_fly_time_cnt > 200) && (global_ctrl_device.mental_training_mode_ctrl_fly_time_cnt < 260))
				{
					device_data_none();
				}
				else
				{
					;
				}
			}
			else if(global_ctrl_device.mental_training_mode_ctrl_fly_time_cnt <100 && global_ctrl_device.mental_training_mode_ctrl_fly_time_flg == 1)
			{
				device_data_none();
			}
			else if(global_ctrl_device.mental_training_mode_ctrl_fly_time_cnt >100)
			{
				global_ctrl_device.mental_training_mode_ctrl_fly_time_flg = 0;
				Result_Attention =0;
				
			}

		}
		else if (Result_Attention == 2) // 下降 && flag_userTime_2 == 1
		{

			device_data_down();
			if(global_ctrl_device.mental_training_mode_ctrl_fly_time_cnt > 300)
			{
				global_ctrl_device.mental_training_mode_ctrl_fly_time_flg = 0;//也需要把眼肌定时器关
				Result_Attention = 0;
				device_data_none();
				//边界处理
				if(mental_training_down_count >= 0xfe)
				{
					mental_training_down_count =0xff;
				}
				else
				{
					mental_training_down_count++;
				}
				printf("down is over\r\n");
			}
		}
		else
		{
			;
		}
	}

}




