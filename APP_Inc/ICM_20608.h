#ifndef _ICM_20608_H
#define _ICM_20608_H

#include "main.h"

struct icm20608_dev_struc
{
	signed int gyro_x_adc;	// 陀螺仪x轴原始值
	signed int gyro_y_adc;	// 陀螺仪y轴原始值
	signed int gyro_z_adc;	// 陀螺仪z轴原始值
	signed int accel_x_adc;	// 加速度计x轴原始值
	signed int accel_y_adc;	// 加速度计y轴原始值
	signed int accel_z_adc;	// 加速度计z轴原始值
	signed int temp_adc;	// 温度原始值

	/* 下面是计算得到的实际值，扩大100倍*/
	signed int gyro_x_act;	// 陀螺仪x轴实际值
	signed int gyro_y_act;	// 陀螺仪y轴实际值
	signed int gyro_z_act;	// 陀螺仪z轴实际值
	signed int accel_x_act;	// 加速度计x轴实际值
	signed int accel_y_act;	// 加速度计y轴实际值
	signed int accel_z_act;	// 加速度计z轴实际值
	signed int temp_act;	// 温度实际值
};

#define MC20608_I2C_PORT 	GPIOA
#define MC20608_I2C_CLK_PIN GPIO_PIN_9
#define MC20608_I2C_SDA_PIN GPIO_PIN_8
//#define IMC20608_I2C_CLK_L    	MC20608_I2C_PORT->BRR = (uint32_t)MC20608_I2C_CLK_PIN;//
//#define IMC20608_I2C_CLK_H    	MC20608_I2C_PORT->BSRR = (uint32_t)MC20608_I2C_CLK_PIN;//
//#define IMC20608_I2C_SDA_READ 	MC20608_I2C_PORT->IDR & MC20608_I2C_SDA_PIN//

//#define IMC20608_I2C_SDA_L    	MC20608_I2C_PORT->BRR = (uint32_t)MC20608_I2C_SDA_PIN;
//#define IMC20608_I2C_SDA_H  		MC20608_I2C_PORT->BSRR = (uint32_t)MC20608_I2C_SDA_PIN;
#define IMC20608_I2C_SDA_H 			HAL_GPIO_WritePin(MC20608_I2C_PORT, MC20608_I2C_SDA_PIN,GPIO_PIN_SET)
#define IMC20608_I2C_SDA_L  		HAL_GPIO_WritePin(MC20608_I2C_PORT, MC20608_I2C_SDA_PIN,GPIO_PIN_RESET)
#define IMC20608_I2C_CLK_L    		HAL_GPIO_WritePin(MC20608_I2C_PORT, MC20608_I2C_CLK_PIN,GPIO_PIN_SET)
#define IMC20608_I2C_CLK_H    		HAL_GPIO_WritePin(MC20608_I2C_PORT, MC20608_I2C_CLK_PIN,GPIO_PIN_RESET)

#define IMC20608_I2C_SDA_READ		HAL_GPIO_ReadPin(MC20608_I2C_PORT,MC20608_I2C_SDA_PIN)


uint8_t icm20608_init(void);
void imc20608_gpio_init(void);
void icm20608_write_reg(unsigned char reg, unsigned char value);
uint8_t icm20608_read_reg(unsigned char reg);
void icm20608_read_len(unsigned char reg, unsigned char *buf, unsigned char len);
void icm20608_getdata(void);
void icm_20608_sleep(void);



#endif
