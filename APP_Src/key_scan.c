/*
 * key_scan.c
 *
 *  Created on: 2025.4.2
 *      Author: lan
 */

#include "key_scan.h"
#include "gpio.h"
#include "math.h"
//#include "LT8960Ldrv.h"
#include "tim.h"
#include "adc.h"
#include "work_drive.h"
#include "td5322a.h"
extern uint8_t flag_change;
//extern uint8_t flag_ble;
//��ʼ�������ṹ��
 volatile GLOBAL_CTRL_DEVICE global_ctrl_device ={
										.DEVIC_MODE.training_mode = IDEL_MODE,
										.power_ctrl_flg = DEVICE_POWER_OFF,
										.key_mode_state = IDEL_MODE,
										.key_mode_time_flg =0,
										.key_mode_time_cnt =0,
										.key_press_off_flg = 1,
										.power_ctrl_key_flg = 0,
										.power_ctrl_key_cnt =0,
										.low_power_consumption_time_flg =0,
										.low_power_consumption_time_cnt =0,
										.eye_muscle_training_mode_ctrl_fly_time_flg =0,
										.eye_muscle_training_mode_ctrl_fly_time_cnt =0,
										.mental_training_mode_ctrl_fly_time_flg =0,
										.mental_training_mode_ctrl_fly_time_cnt =0,
										.ctrl_fly_time_flg =0,	//控制飞碟飞行时间定时器标志位
										.ctrl_fly_time_cnt =0,
										.ble_mode_time_flg = 0,	//2.4g和蓝牙选择模式定时器标志位
										.ble_mode_time_cnt= 0,
										.eeg_uart_init_time_flg =0,
										.eeg_uart_init_time_cnt =0,
										.td5322a_uart_receive_flg =0,
										.gpio_key_exit_flg =0
};
/*******************************************************************************
��������:key_scan_task ����������
��    ��:GLOBAL_CTRL_DEVICE*gKey_mode_ctrl->�豸ȫ�ֱ���
�� �� ֵ:��
ע    ��:��
*******************************************************************************/
//volatile uint8_t  gpio_key_exit_flg = 0;
//extern uint8_t gpio_key_flag;
//volatile uint8_t press_key_off = 0x01; //关机按键标志位 必须要等到按键松开的时候才能恢复，要不然
 void key_scan_task(volatile TD5322A_STRUCT *td5322a_par)
{
	//���µ�һ�ν���ѵ��ģʽ�������������оƬ������Ҫ����
	static uint8_t press_key_flg = 0;
//	 GPIO_InitTypeDef GPIO_InitStruct = {0};
	static uint8_t press_short_key =0x0; //短按标志位
	static uint8_t power_on_flg=0x0;
	static uint8_t key_press_state = 0;
	static uint8_t key_press_state_first_flg =0;

	//��������
	switch(key_press_state)
	{
		case 0:
//	&&gKey_mode_ctrl->DEVIC_MODE.training_mode == IDEL_MODE	&&gKey_mode_ctrl->DEVIC_MODE.training_mode == IDEL_MODE	&&  gKey_mode_ctrl->key_mode_state == IDEL_MODE
//			HAL_Delay(5);
			if(global_ctrl_device.gpio_key_exit_flg == 1  && global_ctrl_device.key_press_off_flg == 1)//&& gKey_mode_ctrl->key_mode_state == IDEL_MODE
//			if(deviceState == Device_ON && press_key_off == 1)
			{
//				HAL_Delay(10);//����KEY_MODE_WTICH_CTRL()
//				if(gpio_key_exit_flg == 1  && press_key_off == 1 )
				{
//					gKey_mode_ctrl->DEVIC_MODE.training_mode = TRAINING_MODE;//����ѵ��ģʽ
//					gKey_mode_ctrl->power_ctrl_flg = DEVICE_POWER_ON;
//					gKey_mode_ctrl->key_mode_state = TRAINING_MODE;
//					gKey_mode_ctrl->power_ctrl_key_flg = 1;
///					gKey_mode_ctrl->key_mode_time_flg = 1;//ѵ��ģʽ��ʱ������
					global_ctrl_device.key_press_off_flg = 0;
					global_ctrl_device.eeg_uart_init_time_flg =1;//需要等电压稳定后才重新再初始化一次串口
					global_ctrl_device.gpio_key_exit_flg =0;
					key_press_state_first_flg =1;
					key_press_state =1;
//					time_struct.time_1_5s_flg = 1; //��1.5s�Ķ�ʱ��
//					device_init();
//					 Device_Enter_ON();
					 key_input_nomal();
					LED_BULE_OFF();
					LED_GREEN_ON();
					deviceState = Device_ON;
//					printf("power on\r\n");
					
				}
			}
			//按键松开的时候初始化io读取状态
			else if(global_ctrl_device.gpio_key_exit_flg == 0 && global_ctrl_device.key_press_off_flg == 2 && KEY_MODE_WTICH_CTRL() == SET)
			{
				HAL_Delay(20);//防抖
				 if(global_ctrl_device.gpio_key_exit_flg == 0 && global_ctrl_device.key_press_off_flg == 2 && KEY_MODE_WTICH_CTRL() == SET)
				{
					 global_ctrl_device.key_press_off_flg = 1;
					key_input_exit();
					HAL_Delay(50);

				}
			}
			else
			{
				;
			}
			
		break;
		
		case 1:
 			if(key_press_state_first_flg == 1)
			{
 				key_press_state = 2;
				key_press_state_first_flg =0;
			}
			if(KEY_MODE_WTICH_CTRL() == RESET && global_ctrl_device.key_mode_state != 0 && global_ctrl_device.power_ctrl_key_flg ==0)
			{
				HAL_Delay(10);//防抖
				if(KEY_MODE_WTICH_CTRL() == RESET && global_ctrl_device.key_mode_state != 0 && global_ctrl_device.power_ctrl_key_flg ==0 )
				{
					global_ctrl_device.power_ctrl_key_flg =1; //�������ڳ������߶̰����ƣ�����2s����Ϊ�ػ�����
					press_key_flg = 1; //����������
				}
				else
				{
					;
				}
			}
			//�̰����� ֻ����ѵ��ģʽ��ʱ���������&& gKey_mode_ctrl->key_mode_state != 0
			if(global_ctrl_device.power_ctrl_key_cnt < 75 && KEY_MODE_WTICH_CTRL() == SET  && global_ctrl_device.power_ctrl_key_flg == 1)
			{
				global_ctrl_device.power_ctrl_key_flg = 0;
				press_short_key = 1;
				key_press_state =2;
			}
			//û�г���Ҳû�ж̰�
//			else if(global_ctrl_device.power_ctrl_key_cnt >=200 && global_ctrl_device.power_ctrl_key_cnt  < 500 && KEY_MODE_WTICH_CTRL() == SET)
//			{
//				press_key_flg = 0; //����������
//				global_ctrl_device.power_ctrl_key_flg = 0; //��ʱ��Ҳ������
//				press_short_key = 0;
//				key_press_state =1;
//			}
			//松开 或者因为低电平没电了，也会进入自动关机
			else if((global_ctrl_device.power_ctrl_key_cnt >= 75  && global_ctrl_device.key_press_off_flg == 0)|| (deviceState ==  Device_OFF && battery_voltage_struct.battery_voltage_vlaue <= 3.4))
			{
				//先把灯关了，然后再识别
				LED_BULE_OFF();
				LED_GREEN_OFF();
				if(global_ctrl_device.DEVIC_MODE.training_mode != IDEL_MODE) //判断是否在使用飞碟如果在使用飞碟，则需要先下降到一定的程度才能进入软件复位关机
				{
					for(uint8_t i =0;i<10;i++)
					{
						HAL_Delay(20);
						device_data_down();
					}
				}
				global_ctrl_device.key_mode_state = IDEL_MODE;
				global_ctrl_device.DEVIC_MODE.training_mode = IDEL_MODE;
				press_key_flg = 0; //
				global_ctrl_device.power_ctrl_flg = DEVICE_POWER_OFF;
				global_ctrl_device.power_ctrl_key_flg = 0;
				press_short_key = 0;
				key_press_state =0;
				global_ctrl_device.key_mode_time_flg =0;
				global_ctrl_device.key_press_off_flg = 2;
				global_ctrl_device.low_power_consumption_time_flg =0;
				Device_Enter_OFF();
				HAL_Delay(200); //需要加延时，要不然会有浪涌，导致芯片复位
				NVIC_SystemReset();//软件复位，重新进入低功耗模式
//				Device_Enter_OFF();
//				gpio_key_flag= 0;
//				Device_Enter_OFF();
//				HAL_NVIC_DisableIRQ(EXTI0_IRQn);
//				deviceState = Device_OFF;
			}
			else
			{
				;
			}
			break;
			case 2:
			if( press_key_flg == 1 &&global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.eye_muscle_training_mode == 1 \
				&& global_ctrl_device.key_mode_state == TRAINING_MODE && press_short_key == 1)
			{
				global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.cervical_training_mode = 1;
				global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.eye_muscle_training_mode = 0;
				press_key_flg =0;
				press_short_key = 0;

			}
			//��׵ѵ��ģʽ &&KEY_MODE_WTICH_CTRL() == RESET
			else if( press_key_flg == 1 && global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.cervical_training_mode == 1 \
				&& global_ctrl_device.key_mode_state == TRAINING_MODE && press_short_key == 1)
			{
				global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.mental_training_mode = 1;
				global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.cervical_training_mode = 0;
				press_key_flg =0;
				press_short_key = 0;
			}
			//����ѵ��ģʽKEY_MODE_WTICH_CTRL() == RESET &&
			else if( press_key_flg == 1 && global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.mental_training_mode == 1 \
				&& global_ctrl_device.key_mode_state == TRAINING_MODE && press_short_key == 1)
			{
				global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.eye_muscle_training_mode = 1;
				global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.mental_training_mode = 0;
				press_key_flg =0;
				press_short_key = 0;
			}
			else if(press_key_flg == 1 && global_ctrl_device.DEVIC_MODE.training_mode==IDEL_MODE &&global_ctrl_device.key_mode_state == IMMEDIATE_MODE)
			{
				flag_change = 1;//重新启动
				//从新进入主模式
				td5322a_par->work_peripheral_role_flg = TD5322A_BLE_CENTRAL_MODE;
//				flag_ble = 0;
				//蓝牙模式即时模式需要重新计时
				global_ctrl_device.key_mode_time_flg = 0;
				global_ctrl_device.key_mode_state = IDEL_MODE;
				//2.4g初始化芯片初始化
				Device_Enter_ON();
				//亮绿灯
				LED_BULE_OFF();
				LED_GREEN_ON();
				press_key_flg = 0;
				press_short_key = 0;
				printf("restart select mode\r\n");
			}
			else if(press_key_flg == 1 && global_ctrl_device.DEVIC_MODE.training_mode == IDEL_MODE &&global_ctrl_device.key_mode_state == DAILY_MODE)
			{
//				flag_change  =1;//重新启动
//				flag_ble =0;
				//蓝牙芯片初始化
//				device_init();
//				reset_tick();
				//亮绿灯
//				LED_BULE_OFF();
//				LED_GREEN_ON();
				Device_Enter_ON();
				global_ctrl_device.key_mode_state = IMMEDIATE_MODE;
				global_ctrl_device.low_power_consumption_time_flg =0;
				global_ctrl_device.key_mode_time_flg =1; //
//				LT8960L_BLE_INIT(); // ble 初始化
				press_key_flg = 0;
				press_short_key = 0;
				printf("return immediate mode\r\n");
			}

			else
			{
				;
			}	
			key_press_state = 1;
			default:
			break;
	}
	//��Դ����͹���״̬����ʼ����ʱ����߹ػ�
	
}


