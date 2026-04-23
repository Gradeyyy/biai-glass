/*
 * key_scan.h
 *
 *  Created on: 2025.4.2
 *      Author: lan
 */
#ifndef _KEY_SCAN_H_
#define _KEY_SCAN_H_

#include "stm32g4xx.h"
#include "td5322a.h"
#define IDEL_MODE			(uint8_t)0x00	//����״̬
#define TRAINING_MODE 		(uint8_t)0x01 	//ѵ��ģʽ
#define IMMEDIATE_MODE 		(uint8_t)0x10	//��ʱģʽ
#define DAILY_MODE			(uint8_t)0x20	//�ճ�ģʽ
#define EYE_MUSCLE_TRAINING_MODE 	(uint8_t)0x01 //�ۼ�ѵ��ģʽ
#define CERVICAL_TRAINING_MODE 		(uint8_t)0x02 //��׵ѵ��ģʽ
#define MENTAL_TRAINING_MODE		(uint8_t)0x04 //����ѵ��ģʽ
#define 	DEVICE_POWER_ON 	(uint8_t)0x01 	//�豸�����ϵ�
#define  	DEVICE_POWER_OFF 	(uint8_t)0x00	//�豸���öϵ�
//�����ṹ��
typedef struct
{	
	union  //
	{
			struct 
			{
				volatile uint8_t eye_muscle_training_mode : 1;
				volatile uint8_t cervical_training_mode : 1;
				volatile uint8_t mental_training_mode : 1;
			}TRINING_MODE_STR;
			volatile uint8_t training_mode;		
	}DEVIC_MODE;
	volatile uint8_t power_ctrl_flg;	//��Դ���Ʊ�־λ
	volatile uint8_t key_press_off_flg; //按键松开标志位
	volatile uint8_t power_ctrl_key_flg; // ��Դ�������Ʊ�־
	volatile uint16_t power_ctrl_key_cnt; // ��Դ�������Ƽ�ʱ
	volatile uint8_t gpio_key_exit_flg;
	volatile uint8_t key_mode_state;
	volatile uint8_t key_mode_time_flg; //计算蓝牙模式下的即时模式定时器标志位
	volatile uint16_t key_mode_time_cnt;
	volatile uint8_t low_power_consumption_time_flg; //低功耗定时器标准位
	volatile uint32_t low_power_consumption_time_cnt; //低功耗定时器计时
	volatile uint8_t eye_muscle_training_mode_ctrl_fly_time_flg; //眼肌训练模式下飞行时间标志位
	volatile uint16_t eye_muscle_training_mode_ctrl_fly_time_cnt;
	volatile uint8_t  mental_training_mode_ctrl_prepare_fly_time_flg;	//脑训练模式准备飞行时间标志位
	volatile uint16_t  mental_training_mode_ctrl_prepare_fly_time_cnt;
	volatile uint8_t  mental_training_mode_ctrl_fly_time_flg;	//脑训练模式飞行时间
	volatile uint16_t  mental_training_mode_ctrl_fly_time_cnt;
	volatile uint8_t  ctrl_fly_time_flg;	//控制飞碟飞行时间定时器标志位
	volatile uint16_t  ctrl_fly_time_cnt;
	volatile uint8_t  ble_mode_time_flg;	//2.4g和蓝牙选择模式定时器标志位
	volatile uint16_t  ble_mode_time_cnt;
	volatile uint8_t  	eeg_uart_init_time_flg;	//2.4g和蓝牙选择模式定时器标志位
	volatile uint16_t  eeg_uart_init_time_cnt;
	volatile uint16_t  td5322a_uart_receive_flg;	//td5322a串口接收命令处理
	
}GLOBAL_CTRL_DEVICE;
extern volatile GLOBAL_CTRL_DEVICE global_ctrl_device;

 void key_scan_task(volatile TD5322A_STRUCT *td5322a_par);
#endif

