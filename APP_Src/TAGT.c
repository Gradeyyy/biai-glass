/*
 * key_scan.c
 *
 *  Created on: 2025.4.8
 *      Author: lan
 */

#include "TAGT.h"
#include "ICM_20608.h"
#include "tim.h"
extern uint8_t receiveBuffer[];
volatile TAGT_STRUCT  tagt_struct ={ 0,0,0,receiveBuffer,0,0,0,0}; //串口接收的数据长度
extern volatile uint16_t flag_1ms;

// ble广播数据
uint8_t ble_data[]={0X02, 0X25,  											//header
					0x06, 0x05, 0x04, 0x03, 0x02, 0x01,     				//mac address
					0x02, 0x01, 0x1A,    									//flags
					0x03, 0x19, 0xc1, 0x03,   								//device
					0x08, 0x16, 0x11, 0x11, 0x11, 0xb0, 0xb0, 0xb0,0x00,    //uuid
					0x0E, 0x09, 0x4C, 0x54, 0x38, 0x39, 0x36, 0x30,0x4C,0x76,0x61,0x6C,0x3D,0x78,0x78, //name
				};
uint8_t ble_data2[]={0X02, 0X25,  											//header
					0x06, 0x05, 0x04, 0x03, 0x02, 0x01,     				//mac address
					0x02, 0x01, 0x1A,    									//flags
					0x03, 0x19, 0xc1, 0x03,   								//device
					0x08, 0x16, 0x11, 0x11, 0x11, 0xb0, 0xb0, 0xb0,0x00,    //uuid
					0x0E, 0xFF, 0x4C, 0x54, 0x38, 0x39, 0x36, 0x30,0x4C,0x76,0x61,0x6C,0x3D,0x00,0x00, //name
				};
uint8_t ble_data3[] ={0X02, 0X25,
						0x06, 0x05, 0x04, 0x03, 0x02, 0x01,     				//mac address
						0x1E, 0xFF, 0x4C, 0x54, 0x38, 0x39, 0x36, 0x30,0x4C,0x76,0x61,0x6C,0x3D,0x00,0x00, //name
						};
/*******************************************************************************
函数功能：脑电芯片处理数据，数据解析等其他的问题，
形     参：TAGT_STRUCT *eeg_par,串口传进来的数据进行处理解析
			uint8_t *algor_result脑电获取到，控制飞碟的运动
			设备数据发送控制飞碟uint8_t *algor_result,
返 回 值：无
注     意：无
*****************************************************************************/
void tagt_process(TAGT_STRUCT *eeg_par,LT8960L_DIVICE_STRUCT *plt8960l_device,uint16_t opt3001_data_value,volatile uint16_t vl53l0x_distance_data)
{
	static uint8_t payloadLength = 0;
//	static uint8_t flag_change =1;
	static uint8_t rx_buffer[6] ={0};
	static uint8_t len = 0;
	static uint8_t runStateOld = RUN_STATE_NONE;
	static uint8_t leftCount = 0;
	static uint8_t flag_roll_change =0;
	static uint8_t algor_result = 0;
	static uint8_t checksum = 0;
	static uint8_t receive_eeg_data_to_ble_flg = 0;
	static uint8_t ble_daily_mode_flg = 0;
	static uint8_t ble_dally_mode_10min_flg = 0;
	//按键按下开机才能正常运动
	if(global_ctrl_device.power_ctrl_flg == DEVICE_POWER_ON ||(global_ctrl_device.power_ctrl_flg == DEVICE_POWER_OFF&&global_ctrl_device.DEVIC_MODE.training_mode == DAILY_MODE))
	{			
		//判断是否为2.4G模式	
		if (plt8960l_device->_2_4g_flg ==0 &&plt8960l_device->ble_flg == 0) 
		{
//			time_struct.time_1_5s_flg = 1; //开1.5s的定时器
			if (time_struct.time_1_5s_cnt  < 500) //Test_Channel
			{
				len = LT8960L_RxData(1,rx_buffer,sizeof(rx_buffer));
				if (len != 0)
				{
//					for(uint8_t i=0;i<(len-1);i++)
//					{
//						checksum^=rx_buffer[i];
//					}				
//						printf("len:%d    RF rx:%x, %x, %x, %x, %x, %x\r\n",len, rx_buffer[0], rx_buffer[1],
//						rx_buffer[2], rx_buffer[3], rx_buffer[4], rx_buffer[5]);
					if(0xaa==rx_buffer[0] || rx_buffer[1] ==0xaa)//||checksum==rx_buffer[len-1]
					{
	//					printf("RF rx  success.\r\n");
//						plt8960l_device->ble_flg = 0;u    
						plt8960l_device->_2_4g_flg = 1;
						time_struct.time_1_5s_flg  = 0;//关掉1.5s的定时器
						printf("RF rx  success.\r\n");
						device_data_up();
						HAL_Delay(3);
						device_data_up();
						flag_3ms = 0;
					}				
					
				}
				device_data_none();
			} 
			else {
				plt8960l_device->ble_flg = 1;
				LT8960L_BLE_INIT(); // ble 初始化
				plt8960l_device->_2_4g_flg = 0;
				time_struct.time_1_5s_flg =0;
				printf("ble success \r\n");
			}
			return;
		}
		
	//	if(flag_1ms >4)
		{
	//		flag_1ms = 1;
	//		printf(" %d,%d,%d",icm20608_dev.gyro_x_adc, icm20608_dev.gyro_y_adc, icm20608_dev.gyro_z_adc);

		}		
		//串口处理数据 任务
		if(eeg_par ->uart_flag == 1)
		{
	//		memcpy(data_buffer,eeg_uart_par->receive_data_buffer,eeg_uart_par->length); //把串口的数据拷贝出来
	//		printf(" 运动结果%d",algor_result);
			payloadLength = eeg_par->uart_length-4;	
			switch (payloadLength) {
					case RES_EEG_LENGTH:
						for (uint8_t j = 0; j < eeg_par->uart_length; j++) {
							plt8960l_device->send_buffer[j] = eeg_par->uart_receive_data_buffer[j];
						}
						if(plt8960l_device->ble_flg == 1)
						{
	//						ble_send_cmd(CMD_EEG_RES1);
							receive_eeg_data_to_ble_flg = 1;
							//发送数据包1 专注度控制
//							lt8960l_ble_msg_deal(plt8960l_device,CMD_LT8906L_BLE_DATA_1,opt3001_data_value,vl53l0x_distance_data);
							//发送数据包2 光照等其他的数据
//							lt8960l_ble_msg_deal(plt8960l_device,CMD_LT8906L_BLE_DATA_2,opt3001_data_value,vl53l0x_distance_data);
							printf("uart eeg ok");


						}
						else
						{
	//						eeg_make_result(plt8960l_device);
						}
						break;

					case RAW_EEG_LENGTH:
	//					for (j = 0; j < CMD_EEG_RES2_LENGTH; j++) {
	//						BUFFER_SEND[j] = BUFFER_RECEIVED[j];
	//					}
	//						ble_send_cmd(CMD_EEG_RES2);
						break;

					default:
						break;
					}
			eeg_par ->uart_flag = 0;
			eeg_par->uart_length = 0;
			
		}
		
		else if(eeg_par->uart_receive_time_cnt >100) //接收串口的数据定时器超过300ms，认为数据发送完了，要不就是数据有问题
		{
			eeg_par ->uart_flag = 0;
			eeg_par->uart_length = 0;
			eeg_par->uart_receive_time_flg =0;
			eeg_par->uart_receive_data_buffer =0;
		}
		//根据陀螺仪的数据判断这个运动方向

		
		//判断蓝牙设备是否成功，不成功的话，是在2.4G操作的情况下控制
//		if(flag_1ms > 2)
		{		
			flag_1ms =1;		
			//选择的是2.4g频率的时候
			if(plt8960l_device->ble_flg == 0 && plt8960l_device->_2_4g_flg == 1)	
			{
				//颈椎训练模式
				if(global_ctrl_device.DEVIC_MODE.training_mode == CERVICAL_TRAINING_MODE)
				{
					icm20608_getdata(&icm20608_dev); //收到脑电信号传来的数据，先进行陀螺仪的数据	
					algor_result = Acc_algorithm_result(icm20608_dev.gyro_x_adc, icm20608_dev.gyro_y_adc, icm20608_dev.gyro_z_adc);
					// 运动轨迹、速度调整
					if (algor_result == ALGO_LEFT || algor_result == ALGO_RIGHT
						|| algor_result == ALGO_UP || algor_result == ALGO_DOWN
						|| algor_result == ALGO_LEFTSIDE || algor_result == ALGO_RIGHTSIDE) 
					{
					flag_userTime = 0;
		//			printf("flag_userTime = 0 Result_Algo:%d\n", Result_Algo);
					flag_timeStart = 1;
		//			Result_Algo_old = &algor_result;
				
			
						switch (algor_result) 
						{
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
						default:
							break;
						}
			}
						
				}
				//眼睛专注度模式
				else if(global_ctrl_device.DEVIC_MODE.training_mode == EYE_MUSCLE_TRAINING_MODE)
				{
					
				}
				//脑电控制模式
				else if(global_ctrl_device.DEVIC_MODE.training_mode == MENTAL_TRAINING_MODE)
				{
					eeg_make_result(plt8960l_device);
				}
			
				switch (runState) 
				{

					
					case RUN_STATE_START:
						device_data_up();
						time_struct.time_3ms_flg =1; //开启定时器
						if (time_struct.time_3ms_cnt> 800) {  // (起飞高度设置值，值越大起飞越高) 地上起飞 > 700
							runState = RUN_STATE_HOLD; //RUN_STATE_DOWN; //RUN_STATE_LEFT_ROLL;
						time_struct.time_3ms_flg = 0; //关定时
							runStateOld = RUN_STATE_START;
								printf("RUN STATE:%d 启动,	\n", runState);
						}
						if((time_struct.time_3ms_cnt > 200) && (time_struct.time_3ms_cnt < 260))
						{
							device_data_none();
						}
						break;
					case RUN_STATE_LEFT_ROLL:
						device_data_roll(1);
						time_struct.time_3ms_flg = 1; //开定时
						if (time_struct.time_3ms_cnt > 170) { // 200
							runState = RUN_STATE_HOLD; //RUN_STATE_FOREWORD_PITCH;
						time_struct.time_3ms_flg = 0; //关定时
							runStateOld = RUN_STATE_LEFT_ROLL;
								printf("RUN STATE:%d 左\n", runState);
						}

						break;
					case RUN_STATE_FOREWORD_PITCH:
						device_data_pitch(1);
						time_struct.time_3ms_flg = 1; //开定时
						if (time_struct.time_3ms_cnt > 220) {
							runState = RUN_STATE_HOLD; //RUN_STATE_UP;
						time_struct.time_3ms_flg = 0; //关定时
							runStateOld = RUN_STATE_FOREWORD_PITCH;
								printf("RUN STATE:%d 前,	", runState);
						}

						break;
					case RUN_STATE_RIGHT_ROLL:
						device_data_roll(0);
						time_struct.time_3ms_flg = 1; //开定时
						if (time_struct.time_3ms_cnt > 310) { // 280
							runState = RUN_STATE_HOLD; //RUN_STATE_DOWN;
						time_struct.time_3ms_flg = 0; //关定时
							runStateOld = RUN_STATE_RIGHT_ROLL;
								printf("RUN STATE:%d 右\n", runState);
						}

						break;
					case RUN_STATE_BACKWORD_PITCH:
						device_data_pitch(0);
						time_struct.time_3ms_flg = 1; //开定时
						if (time_struct.time_3ms_cnt > 220) {
		//						if (flag_roll_change == 0) {
		//							runState = RUN_STATE_FOREWORD_PITCH; //RUN_STATE_LEFT_ROLLOVER;
		//						}else{
		//							runState = RUN_STATE_FOREWORD_PITCH; //RUN_STATE_RIGHT_ROLLOVER;
		//						}
							runState = RUN_STATE_HOLD;
							runStateOld = RUN_STATE_BACKWORD_PITCH;
							time_struct.time_3ms_flg = 0; //关定时
		//						printf("RUN STATE:%d 后,	flag_count_temp:%ld\n", runState, flag_count_temp);
						}
						break;
					case RUN_STATE_LEFT_ROLLOVER:
						device_data_rollover(3); // 0
						runState = RUN_STATE_HOLD; //RUN_STATE_LEFT_ROLL;
						runStateOld = RUN_STATE_LEFT_ROLLOVER;
						flag_roll_change = 1;
//						time_struct.time_3ms_flg = 1; //开定时
							printf("RUN STATE:%d 左翻,	\n", runState);

					break;
					case RUN_STATE_RIGHT_ROLLOVER:
						device_data_rollover(2); // 1
						runState = RUN_STATE_HOLD; //RUN_STATE_LEFT_ROLL;
						runStateOld = RUN_STATE_RIGHT_ROLLOVER;
						flag_roll_change = 0;
//						time_struct.time_3ms_flg = 0; //关定时
							printf("RUN STATE:%d 右翻,	\n", runState);

					break;
					case RUN_STATE_UP:
						device_data_up();
						time_struct.time_3ms_flg = 1; //开定时
						if (time_struct.time_3ms_cnt > 220) 
						{ // 270
							runState = RUN_STATE_HOLD; //RUN_STATE_DOWN; //RUN_STATE_RIGHT_ROLL;
						time_struct.time_3ms_flg = 0; //关定时
							runStateOld = RUN_STATE_UP;
								printf("RUN STATE:%d 上,	", runState);
						}
						break;
					case RUN_STATE_DOWN:
						device_data_down();
						time_struct.time_3ms_flg = 1; //开定时
						if (flag_3ms > 700) 
						{  //700
							runState = RUN_STATE_HOLD; //RUN_STATE_UP;  //RUN_STATE_BACKWORD_PITCH;
						time_struct.time_3ms_flg = 0; //关定时
							runStateOld = RUN_STATE_DOWN;
							printf("RUN STATE:%d 下,	", runState);
						}
						break;

					case RUN_STATE_LEFT_ROLL_HAND:
						device_data_roll(1);
						time_struct.time_3ms_flg = 1; //开定时
						if (time_struct.time_3ms_cnt > 280) 
						{
							runState = RUN_STATE_IDLE;
						time_struct.time_3ms_flg = 0; //关定时
								printf("RUN STATE:%d,\n", runState);
		//								flag_count_temp);
						}
						break;
					case RUN_STATE_RIGHT_ROLL_HAND:
						device_data_roll(0);
						time_struct.time_3ms_flg = 1; //开定时
						if (time_struct.time_3ms_cnt > 280) {
							runState = RUN_STATE_IDLE;
							time_struct.time_3ms_flg = 0; //关定时
							printf("RUN STATE:%d,\n", runState);
						}
						break;
					case RUN_STATE_FOREWORD_PITCH_HAND:
						device_data_pitch(1);
						time_struct.time_3ms_flg = 1; //开定时
						if (time_struct.time_3ms_cnt > 220) {
							runState = RUN_STATE_IDLE;
							time_struct.time_3ms_flg = 0; //关定时
								printf("RUN STATE:%d,	\n", runState);
		//								flag_count_temp);
						}
						break;
					case RUN_STATE_BACKWORD_PITCH_HAND:
						device_data_pitch(0);
						time_struct.time_3ms_flg = 1; //开定时
						if (time_struct.time_3ms_cnt > 220) {
							runState = RUN_STATE_IDLE;
							time_struct.time_3ms_flg = 0; //关定时
								printf("RUN STATE:%d,	\n", runState);
		//								flag_count_temp);
						}
						break;
					case RUN_STATE_UP_HAND:
						device_data_up();
						time_struct.time_3ms_flg = 1; //开定时
						if (time_struct.time_3ms_cnt > 270) {
							runState = RUN_STATE_IDLE;
							time_struct.time_3ms_flg = 0; //关定时
							printf("RUN STATE:%d,	\n", runState);

						}
						break;
					case RUN_STATE_DOWN_HAND:
						device_data_down();
						time_struct.time_3ms_flg = 1; //开定时器
						if (time_struct.time_3ms_cnt > 700) {
							runState = RUN_STATE_IDLE;
							time_struct.time_3ms_flg = 0;
						printf("RUN STATE:%d,	\n", runState);
						}
						break;
					case RUN_STATE_IDLE:
						device_data_none();
						time_struct.time_3ms_flg = 1; //开定时器
						if (time_struct.time_3ms_cnt > 1000) {
							runState = RUN_STATE_LEFT_ROLL;
							leftCount = 0; 
							time_struct.time_3ms_flg = 0; //关定时器
								printf("RUN STATE:%d,\n", runState);
						}
						break;
					case RUN_STATE_HOLD:
						device_data_none();
						time_struct.time_3ms_flg = 1;
						if (time_struct.time_3ms_cnt > 300) {  //300
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
							time_struct.time_3ms_flg = 0;
						}
						break;
					case RUN_STATE_NONE:
						device_data_none();
						time_struct.time_3ms_flg = 0;
						break;
					default:
						break;
			}

		}
			//蓝牙模式
			else if(plt8960l_device->ble_flg == 1 && plt8960l_device->_2_4g_flg == 0)
			{
				if(ble_daily_mode_flg == 0 && receive_eeg_data_to_ble_flg == 1)
				{
						lt8960l_ble_msg_deal(plt8960l_device,CMD_LT8906L_BLE_DATA_1,opt3001_data_value,vl53l0x_distance_data);
						lt8960l_ble_msg_deal(plt8960l_device,CMD_LT8906L_BLE_DATA_1,0,0);
						receive_eeg_data_to_ble_flg = 0;
				}
				//在日常模式下，发送150ms，然后关掉系统电源进入低功耗模式
				else if(global_ctrl_device.low_power_consumption_time_cnt >50 && ble_dally_mode_10min_flg == 1 &&global_ctrl_device.DEVIC_MODE.training_mode == DAILY_MODE)
				{
					ble_dally_mode_10min_flg = 0;
					global_ctrl_device.power_ctrl_flg = DEVICE_POWER_OFF;  //系统初始化开机，然后采集数据					
				}
//				//模式切换
				if(	global_ctrl_device.key_mode_state == 1)
				{
					global_ctrl_device.key_mode_state = IMMEDIATE_MODE;//蓝牙模式日常模式并且开启
					global_ctrl_device.DEVIC_MODE.training_mode = IMMEDIATE_MODE;
//					global_ctrl_device.power_ctrl_key_flg =0;//模式切换的过程中，按键按下的计时需要清零，要不然按下的定时器还在计时，会认为
					global_ctrl_device.key_mode_time_flg =1; //3分钟定时器采集信号定时启动，3分钟后开启低功耗模式，部分功能10分钟采集一次数据上传给app
				}
				// 3分钟到了，蓝灯只有在采集的时候灯才会亮，采集完了灭0xd548
				else if(global_ctrl_device.key_mode_time_cnt >2000 && global_ctrl_device.DEVIC_MODE.training_mode == IMMEDIATE_MODE) // 3分钟到了，蓝灯只有在采集的时候灯才会亮，采集完了灭
				{
					global_ctrl_device.key_mode_time_flg = 0; //关掉3分钟的定时器
					global_ctrl_device.DEVIC_MODE.training_mode = DAILY_MODE; //进入到日常模式 进入到睡眠模式
					global_ctrl_device.key_mode_state = DAILY_MODE;
					global_ctrl_device.power_ctrl_flg = DEVICE_POWER_OFF;
					global_ctrl_device.low_power_consumption_time_flg =1;
					ble_daily_mode_flg =1;			
				}
				//10分钟后采集数据给手机app0x927C0
				else if(global_ctrl_device.low_power_consumption_time_cnt >=2000 )
				{
					global_ctrl_device.power_ctrl_flg = DEVICE_POWER_ON;  //系统初始化开机，然后采集数据
					global_ctrl_device.low_power_consumption_time_cnt = 0;
					ble_daily_mode_flg = 0;
//					global_ctrl_device.low_power_consumption_time_flg =1;
					ble_dally_mode_10min_flg = 1;
				}
				
				
				
			}
		}
	
	
	}
}
/*******************************************************************************
函数功能：脑电芯片处理数据，数据解析等其他的问题，
形     LT8960L_DIVICE_STRUCT *plt8960l_device,蓝牙发送的芯片数据处理
			
返 回 值：无
注     意：无
*****************************************************************************/
void eeg_make_result(LT8960L_DIVICE_STRUCT *plt8960l_device) 
{
	static uint8_t attention_time_cnt =0;
	static uint8_t eeg_attention_current_data = 0x0; 	//脑电专注度当前的值	
	static uint8_t eeg_attention_previous_data = 0x0;	//脑电专注度上一次的值
//	uint8_t ufo_run_st
	eeg_attention_current_data = (uint8_t) plt8960l_device->send_buffer[32];
	if ((eeg_attention_current_data > eeg_attention_previous_data) && eeg_attention_current_data > 30)
	{
			attention_time_cnt++;
			if(attention_time_cnt >100)
			{
				eeg_attention_previous_data = eeg_attention_current_data; 
				tagt_struct.eeg_attention_flg = 1;  // 上升
				runState = RUN_STATE_START;
				attention_time_cnt =0;
//				flag_res_rx = 1;
			}
			
//			if(tagt_struct.eeg_attention_receive_data_flg) // 确认有脑电才起飞
//			{
//				
//			}
	} 
	else if ((eeg_attention_current_data < eeg_attention_previous_data) && plt8960l_device->result_current > 30) 
	{
		if(attention_time_cnt >100)
		{
		
			eeg_attention_previous_data = eeg_attention_current_data; 
			tagt_struct.eeg_attention_flg = 2;  // 下降
			attention_time_cnt = 0;
		}
	} 
	
	else if(eeg_attention_current_data > 0)
	{
		tagt_struct.eeg_attention_flg = 3;  // 保持不变
	}
	// 处理专注度
	if (tagt_struct.eeg_attention_flg == 1 && flag_userTime_2 == 1) // 上升
	{
		///////////////
//		if(fly_counter < 4)
//		{
//			device_data_up();
//			HAL_Delay(3);
//			flag_res_eeg = 1;
//			flag_count_continue = 100;
//
//			Flag_Fly = 1;
//		}
		//////////////////////
		flag_userTime_2 = 0;
		flag_timeStart_2 = 1;
//		fly_counter++;
		tagt_struct.eeg_attention_flg = 0;
		//printf("attention:%d===fly_counter:%d-------1\n",Result_data_new, fly_counter);
	} 
	else if( tagt_struct.eeg_attention_flg == 2  && flag_userTime_2 == 1) // 下降
	{
		///////////////////
//		if (fly_counter > 3)
//		{
//			flag_res_eeg = 2;
//			flag_count_continue = 100;
//			device_data_down();
//			HAL_Delay(3);
//		}
		///////////////////
		flag_userTime_2 = 0;
		flag_timeStart_2 = 1;
//		fly_counter--;
		tagt_struct.eeg_attention_flg = 0;
		//printf("attention:%d===fly_counter:%d-------2\n",Result_data_new, fly_counter);
	}
	
}

// 多指令发包 --> 更改为向LT8960L发送指令
void ble_send_cmd(uint8_t cmd)
{
	uint8_t i;
	switch (cmd) {
	case CMD_PPG:
		for (i = 0; i <= CMD_PPG_LENGTH; i++) {
//			printf("%c", (BUFFER_SEND[i] & 0xFF));
		}
		break;
	case CMD_ICM20608:
//			printf("gyro_x:%d        ", icm20608_dev.gyro_x_adc);
//			printf("gyro_y:%d        ", icm20608_dev.gyro_y_adc);
//			printf("gyro_z:%d        \n", icm20608_dev.gyro_z_adc);
		LT8960L_Tx_BLE_Data(ble_data2, sizeof(ble_data2), 7);
		break;
	case CMD_EEG_RAW:
//		for(i = 0; i < CMD_EEG_RAW_LENGTH;i++)
//		{
//			printf("%c",(BUFFER_SEND[i]&0xFF));
//		}
//		printf("\n");
		break;
	case CMD_EEG_RES1:
//		for (i = 0; i < CMD_EEG_RES1_LENGTH; i++) {
//			printf("%c",(BUFFER_SEND[i]&0xFF));
//		}
//		printf("\n");
//		ble_make_result();
		LT8960L_Tx_BLE_Data(ble_data, sizeof(ble_data), 7);
//		LT8960L_Tx_BLE_Data(ble_data2, sizeof(ble_data2), 7); // 广播包分包显示
		break;
	case CMD_EEG_RES2:
		for (i = 0; i < CMD_EEG_RES2_LENGTH; i++) {
//			printf("%c", (BUFFER_SEND[i] & 0xFF));
		}
		break;
	case CMD_NONE:
		break;
	default:
		break;
	}
}
/*******************************************************************************
函数功能：LT8960L_DIVICE_STRUCT，
形     LT8960L_DIVICE_STRUCT *plt8960l_device,蓝牙发送的芯片数据处理
		uint16_t rawLux_value 感光值
		uint8_t select_ble_package_idex 包选择发送
		uint16_t vl53l0x_distance_data  包数据值
返 回 值：无
注     意：无
*****************************************************************************/
void lt8960l_ble_msg_deal(LT8960L_DIVICE_STRUCT *plt8960l_device,uint8_t ble_package_idex,uint16_t rawLux_value,uint16_t vl53l0x_distance_data)
{
	uint32_t chksum = 0;
	uint8_t send_ble_data[0x40] ={0};
	//	0X02, 0X25, 
	send_ble_data[0] = 0x02;
	send_ble_data[1] = 0x25;
	//	0x06, 0x05, 0x04, 0x03, 0x02, 0x01, //mac address
	send_ble_data[2] = 0x06;
	send_ble_data[3] = 0x05;
	send_ble_data[4] = 0x04;
	send_ble_data[5] = 0x03;
	send_ble_data[6] = 0x02;
	send_ble_data[7] = 0x01;
	 //数据包
	send_ble_data[8] = 0x1E;
	send_ble_data[9] = 0xFF;
	send_ble_data[10] = ble_package_idex;
	send_ble_data[11] = 0xBB;
	if(ble_package_idex == 0xBB) //广播包为1的时候
	{
		//eeg收到信息从第8个数据传到要发送的数据，0xaaaa,0x20),0x02,1b,0x83,0x18
		send_ble_data[12] = plt8960l_device->send_buffer[4]; //信号值
		chksum += plt8960l_device->send_buffer[4];
		for (uint8_t i = 0; i < 24; i++){
			
			send_ble_data[13+i] = plt8960l_device->send_buffer[7+i];
			chksum += plt8960l_device->send_buffer[7+i];
		}
		send_ble_data[37] = plt8960l_device->send_buffer[32];
		send_ble_data[38] = plt8960l_device->send_buffer[34];
		chksum += plt8960l_device->send_buffer[32];
		chksum += plt8960l_device->send_buffer[34];
	}
	else if(ble_package_idex == 0xBA)
	{
		for (uint8_t i = 0; i < 24; i++)
		{
						//eeg收到信息从第8个数据传到要发送的数据，0xaaaa,0x20),0x02,1b,0x83,0x18
			send_ble_data[12+i] = plt8960l_device->send_buffer[7+i];
			chksum += plt8960l_device->send_buffer[7+i];
		}
		send_ble_data[36] = (rawLux_value&0xff);
		send_ble_data[37] = ((rawLux_value&0xff00)>>8);
		send_ble_data[38] = vl53l0x_distance_data&0xff;
		send_ble_data[39] = ((vl53l0x_distance_data&0xff00)>>8);
		chksum += send_ble_data[36];
		chksum += send_ble_data[38];

	}	
	//写寄存器
	LT8960L_Tx_BLE_Data(send_ble_data, sizeof(send_ble_data), 7);

}

