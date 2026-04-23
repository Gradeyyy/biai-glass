/*
 * algorithm.c
 *
 *  Created on: 2025/2/18
 *      Author: lang
 */
 #include "algorithm.h"
#include "work_drive.h"
#include "key_scan.h"
 #define GYRO_BUFFER_LENGTH  100
 
int16_t gyro_y[GYRO_BUFFER_LENGTH];
uint8_t bufferIndex = 0;
 
int16_t gyro_x[GYRO_BUFFER_LENGTH];
uint8_t bufferIndex_x = 0;

int16_t gyro_z[GYRO_BUFFER_LENGTH];
uint8_t bufferIndex_z = 0;
 
 enum ALGO_RESULT{
	ALGO_RESULT_INIT = 1,
	ALGO_RESULT_LEARNING,
	ALGO_RESULT_LEFT,
	ALGO_RESULT_RIGHT,
	ALGO_RESULT_UP,
	ALGO_RESULT_DOWN,
	ALGO_RESULT_LEFTSIDE,
	ALGO_RESULT_RIGHTSIDE,
	ALGO_RESULT_ERROR
} algoResult = ALGO_RESULT_INIT;
 
void Acc_algorithm_init(void)
{
	bufferIndex = 50;
	bufferIndex_x = 50;
	bufferIndex_z = 50;
	algoResult = ALGO_RESULT_INIT;
}

uint8_t Algorithm_Input(void)
{
	int16_t temData_y_axis[4];
	int16_t temData_x_axis[4];
	int16_t temData_z_axis[4];
	// x轴
	for(int i = 0; i < 4; i++)
	{
		temData_x_axis[i] = gyro_x[GYRO_BUFFER_LENGTH-4+i];
	}
	// y轴
	for(int i = 0; i < 4; i++)
	{
		temData_y_axis[i] = gyro_y[GYRO_BUFFER_LENGTH-4+i];
	}
	// z轴
	for(int i = 0; i < 4; i++)
	{
		temData_z_axis[i] = gyro_z[GYRO_BUFFER_LENGTH-4+i];
	}
	//向下控制
	if((temData_z_axis[1] < temData_z_axis[0]) && (temData_z_axis[2] < temData_z_axis[1]) && (temData_z_axis[3] < temData_z_axis[2]) && temData_z_axis[3] <-13000 )
	{
//		if((temData_x_axis[1] < temData_x_axis[0]) && (temData_x_axis[2] < temData_x_axis[1]) && (temData_x_axis[3] < temData_x_axis[2])&& temData_x_axis[3] <-500)
		{
			return 3;
		}
	}
	//向上控制
	else if((temData_z_axis[1] > temData_z_axis[0]) && (temData_z_axis[2] > temData_z_axis[1]) && (temData_z_axis[3] > temData_z_axis[2]) && temData_z_axis[3] >13000 )
	{
//		if((temData_x_axis[1] > temData_x_axis[0]) && (temData_x_axis[2] > temData_x_axis[1]) && (temData_x_axis[3] > temData_x_axis[2])&& temData_x_axis[0] <3000)
//		{
			return 4;
//		}
	}
	//向左控制
	else if((temData_y_axis[1] > temData_y_axis[0]) && (temData_y_axis[2] > temData_y_axis[1]) && (temData_y_axis[3] > temData_y_axis[2]) &&temData_y_axis[3]> 14000)
	{
		if(global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.eye_muscle_training_mode == 1 &&temData_y_axis[3]> 16000)
		{
			return 2;
		}
		else if(global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.eye_muscle_training_mode == 0 )
		{
			return 2;
		}
	}
	//向右控制
	else if((temData_y_axis[1] < temData_y_axis[0]) && (temData_y_axis[2] < temData_y_axis[1]) && (temData_y_axis[3] < temData_y_axis[2]) &&temData_y_axis[3]< -14000)
	{
		if(global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.eye_muscle_training_mode == 1 &&temData_y_axis[3]< -16000)
//		{
			return 1;
//		}
		else if(global_ctrl_device.DEVIC_MODE.TRINING_MODE_STR.eye_muscle_training_mode == 0)
		{
			return 1;
		}
	}
	//向前控制
	else if((temData_x_axis[1] > temData_x_axis[0]) && (temData_x_axis[2] > temData_x_axis[1]) && (temData_x_axis[3] > temData_x_axis[2]) &&temData_x_axis[3]> 10000)
	{
//		if((temData_x_axis[1] < temData_x_axis[0]) && (temData_x_axis[2] < temData_x_axis[1]) && (temData_x_axis[3] < temData_x_axis[2]) &&temData_y_axis[3]> 500)
//		{
			return 6;
//		}
	}
	//向后制
	else if((temData_x_axis[1] < temData_x_axis[0]) && (temData_x_axis[2] < temData_x_axis[1]) && (temData_x_axis[3] < temData_x_axis[2]) &&temData_x_axis[3]< -10000)
	{
//		if((temData_x_axis[1] < temData_x_axis[0]) && (temData_x_axis[2] < temData_x_axis[1]) && (temData_x_axis[3] < temData_x_axis[2]) &&temData_y_axis[3]> 500)
//		{
			return 5;
//		}
	}


	return 0;
}
 
uint8_t Acc_algorithm_result(int16_t gyro_x_in, int16_t gyro_y_in, int16_t gyro_z_in)
{
	uint8_t flag;
	switch(algoResult){
		case ALGO_RESULT_INIT:
			gyro_y[bufferIndex++] = gyro_y_in;
			gyro_x[bufferIndex_x++] = gyro_x_in;
			gyro_z[bufferIndex_z++] = gyro_z_in;
		
		  if(bufferIndex == GYRO_BUFFER_LENGTH)
				algoResult = ALGO_RESULT_LEARNING;
			break;
		case ALGO_RESULT_LEARNING:
			for(uint8_t i = GYRO_BUFFER_LENGTH-5; i < GYRO_BUFFER_LENGTH-1; i++)
			{
				gyro_y[i] = gyro_y[i+1]; 
				gyro_x[i] = gyro_x[i+1]; 
				gyro_z[i] = gyro_z[i+1]; 
			}
			gyro_y[GYRO_BUFFER_LENGTH-1] = gyro_y_in;
			gyro_x[GYRO_BUFFER_LENGTH-1] = gyro_x_in;
			gyro_z[GYRO_BUFFER_LENGTH-1] = gyro_z_in;
			flag = Algorithm_Input();
			if(flag == 1)
			{
				algoResult = ALGO_RESULT_LEFT;
			}else if(flag == 2)
			{
				algoResult = ALGO_RESULT_RIGHT;
			}
			else if(flag == 3)
			{
				algoResult = ALGO_RESULT_DOWN;
			}
			else if(flag == 4)
			{
				algoResult = ALGO_RESULT_UP;
			}
			else if(flag == 5)
			{
				algoResult = ALGO_RESULT_LEFTSIDE;
			}
			else if(flag == 6)
			{
				algoResult = ALGO_RESULT_RIGHTSIDE;
			}
			break;
		case ALGO_RESULT_LEFT:
			Acc_algorithm_init();
			return ALGO_RESULT_LEFT;
		case ALGO_RESULT_RIGHT:
			Acc_algorithm_init();
			return ALGO_RESULT_RIGHT;
		case ALGO_RESULT_DOWN:
			Acc_algorithm_init();
			return ALGO_RESULT_DOWN;
		case ALGO_RESULT_UP:
			Acc_algorithm_init();
			return ALGO_RESULT_UP;
		case ALGO_RESULT_LEFTSIDE:
			Acc_algorithm_init();
			return ALGO_RESULT_LEFTSIDE;
		case ALGO_RESULT_RIGHTSIDE:
			Acc_algorithm_init();
			return ALGO_RESULT_RIGHTSIDE;
		case ALGO_RESULT_ERROR:
			break;
		default:
			break;
	}
	return algoResult;
}
