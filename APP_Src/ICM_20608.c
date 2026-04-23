/*
 * ICM_20608.c
 *
 *  Created on: Sep 10, 2024
 *      Author: lang
 */

#include "ICM_20608.h"
#include "usart.h"

#define ICM20608_IIC_ADDR   0x69  // ICM20608 IIC地址（AD0 = 1， AD0 = 0地址0x68）

#define ICM20608_IIC_READ_ADDR    0xD3
#define ICM20608_IIC_WRITE_ADDR   0xD2
#define ICM20608G_ID	0xAF   //
#define ICM20608D_ID	0xAE   //


struct icm20608_dev_struc icm20608_dev;

extern I2C_HandleTypeDef hi2c2;

/*
 * ICM20608寄存器
 * 复位后所有寄存器为0，除了
 * Register 107(0x6B) Power Management 1  = 0x40
 * Register 117(0x75) WHO_AM_I 			  = 0xAF/0xAE
 * 陀螺仪和加速度自检（与用户自检输出值比较）
 */

// 陀螺仪
#define ICM20_SELF_TEST_X_GYRO		0x00
#define ICM20_SELF_TEST_Y_GYRO		0x01
#define ICM20_SELF_TEST_Z_GYRO		0x02

// 加速度
#define ICM20_SELF_TEST_X_ACCEL		0x0D
#define ICM20_SELF_TEST_Y_ACCEL		0x0E
#define ICM20_SELF_TEST_Z_ACCEL		0x0F

// 陀螺仪静态偏移
#define ICM20_XG_OFFS_USRH			0x13
#define ICM20_XG_OFFS_USRL			0x14
#define ICM20_YG_OFFS_USRH			0x15
#define ICM20_YG_OFFS_USRL			0x16
#define ICM20_ZG_OFFS_USRH			0x17
#define ICM20_ZG_OFFS_USRL			0x18

#define ICM20_SMPLRT_DIV			0x19
#define ICM20_CONFIG				0x1A
#define ICM20_GYRO_CONFIG			0x1B
#define ICM20_ACCEL_CONFIG			0x1C
#define ICM20_ACCEL_CONFIG2			0x1D
#define ICM20_LP_MODE_CFG			0x1E
#define ICM20_ACCEL_WOM_THR			0x1F
#define ICM20_FIFO_EN				0x23
#define ICM20_FSYNC_INT				0x36
#define ICM20_INT_PIN_CFG			0x37
#define ICM20_INT_ENABLE			0x38
#define ICM20_INT_STATUS			0x3A

// 加速度输出
#define ICM20_ACCEL_XOUT_H			0x3B
#define ICM20_ACCEL_XOUT_L			0x3C
#define ICM20_ACCEL_YOUT_H			0x3D
#define ICM20_ACCEL_YOUT_L			0x3E
#define ICM20_ACCEL_ZOUT_H			0x3F
#define ICM20_ACCEL_ZOUT_L			0x40

// 温度输出
#define ICM20_TEMP_OUT_H			0x41
#define ICM20_TEMP_OUT_L			0x42

// 陀螺仪输出
#define ICM20_GYRO_XOUT_H			0x43
#define ICM20_GYRO_XOUT_L			0x44
#define ICM20_GYRO_YOUT_H			0x45
#define ICM20_GYRO_YOUT_L			0x46
#define ICM20_GYRO_ZOUT_H			0x47
#define ICM20_GYRO_ZOUT_L			0x48

#define ICM20_SIGNAL_PATH_RESET		0x68
#define ICM20_ACCEL_INTEL_CTRL		0x69
#define ICM20_USER_CTRL				0x6A
#define ICM20_PWR_MGMT_1			0x6B
#define ICM20_PWR_MGMT_2			0x6C
#define ICM20_FIFO_COUNTH			0x72
#define ICM20_FIFO_COUNTL			0x73
#define ICM20_FIFO_R_W				0x74
#define ICM20_WHO_AM_I				0x75

// 加速度静态偏移
#define ICM20_XA_OFFSET_H			0x77
#define ICM20_XA_OFFSET_L			0x78
#define ICM20_YA_OFFSET_H			0x7A
#define ICM20_YA_OFFSET_L			0x7B
#define ICM20_ZA_OFFSET_H			0x7D
#define ICM20_ZA_OFFSET_L			0x7E

/**
 * 初始化
 */
 void delay_n1us(uint32_t nus)
{
  uint32_t Delay = nus * 84/4;
	do{
		__NOP();
	}while(Delay--);
}
#define I2C_DELAY 4
/**
 * IO初始化
 */
void imc20608_gpio_init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	IMC20608_I2C_SDA_H;
	IMC20608_I2C_CLK_H;	
}

/*******************************************************************************
函数功能：imc20608 i2c启动信号
形    参：无
返 回 值：无
*******************************************************************************/
void imc20608_i2c_start(void)
{
	IMC20608_I2C_SDA_H;
	delay_n1us(I2C_DELAY);
	IMC20608_I2C_CLK_H;
	delay_n1us(I2C_DELAY);
	IMC20608_I2C_SDA_L;
	delay_n1us(I2C_DELAY);
	IMC20608_I2C_CLK_L;
//	IMC20608_I2C_CLK_H;	
//	IMC20608_I2C_CLK_L;
}

/*******************************************************************************
函数功能：imc20608 i2c停止信号
形    参：无
返 回 值：无
*******************************************************************************/
void imc20608_i2c_stop(void)
{
	IMC20608_I2C_SDA_L;
	delay_n1us(I2C_DELAY);
	IMC20608_I2C_CLK_H;
	delay_n1us(I2C_DELAY);
	IMC20608_I2C_SDA_H;
	delay_n1us(I2C_DELAY);
}

/*******************************************************************************
函数功能：bma250 I2C应答信号
形    参：无
返 回 值：无
*******************************************************************************/
void imc20608_i2c_mack(void)
{
	IMC20608_I2C_CLK_L;
	delay_n1us(I2C_DELAY);
	IMC20608_I2C_SDA_L;
	delay_n1us(I2C_DELAY);
	IMC20608_I2C_CLK_H;
	delay_n1us(I2C_DELAY);
	IMC20608_I2C_CLK_L;
	delay_n1us(I2C_DELAY);
	IMC20608_I2C_SDA_H;
	delay_n1us(I2C_DELAY);
}

/*******************************************************************************
函数功能：bma250 I2C非应答信号
形     参：无
返 回 值：无
*******************************************************************************/
void imc20608_i2c_no_mack(void)
{
	IMC20608_I2C_CLK_L;
	delay_n1us(I2C_DELAY);
	IMC20608_I2C_SDA_H;
	delay_n1us(I2C_DELAY);
	IMC20608_I2C_CLK_H;
	delay_n1us(I2C_DELAY);
	IMC20608_I2C_CLK_L;
	delay_n1us(I2C_DELAY);
	IMC20608_I2C_SDA_L;
	delay_n1us(I2C_DELAY);
}

/*******************************************************************************
函数功能：bma250 I2C 检测应答信号
形     参：无
返 回 值：0->无应答，1->有应答
*******************************************************************************/
uint8_t imc20608_i2c_sack(void)
{
	uint8_t flag;
	IMC20608_I2C_CLK_L;
	IMC20608_I2C_SDA_H;
	delay_n1us(I2C_DELAY);
	IMC20608_I2C_CLK_H;
	delay_n1us(I2C_DELAY);
	if(IMC20608_I2C_SDA_READ==0)//如果为0说明有应答
	{
		flag=1;
	}
	else 
	{
		flag=0;
	}
	IMC20608_I2C_CLK_L;
	return(flag);
}

/*******************************************************************************
函数功能：bma250 I2C发送一字节
形     参：u8 buffer发送的字节
返 回 值：无
*******************************************************************************/
void imc20608_i2c_send_byte(uint8_t buffer)
{
	uint8_t index=0;
	uint8_t temp=buffer;
	for(index=0;index<8;index++)
	{
		IMC20608_I2C_CLK_L;
		delay_n1us(I2C_DELAY);
		if((temp&0x80)==0) //判断最高位是0还是1
		{
			IMC20608_I2C_SDA_L;
		}
		else
		{
			IMC20608_I2C_SDA_H;
		}
		delay_n1us(I2C_DELAY);
		IMC20608_I2C_CLK_H;
		delay_n1us(I2C_DELAY);
		temp<<=1;
	}
	IMC20608_I2C_CLK_L;
}

/*******************************************************************************
函数功能：bma250 I2C接收一个字节
形     参：无
返 回 值：接收到的数据
*******************************************************************************/


uint8_t imc20608_i2c_re_byte(void)
{
	uint8_t index=0;
	uint8_t temp=0;
	IMC20608_I2C_SDA_H;	
	for(index=0;index<8;index++)
	{
		temp<<=1;
		
		IMC20608_I2C_CLK_H;
		delay_n1us(I2C_DELAY);
		if(IMC20608_I2C_SDA_READ==1)
		{
			temp=temp|0x01;
		}
		IMC20608_I2C_CLK_L;
		delay_n1us(I2C_DELAY);
	}
	IMC20608_I2C_CLK_L;
	return temp;	
}

/*******************************************************************************
函数功能：bma250 I2C 写入1个字节
形     参：u8 slave_addr->器件地址，u8 data_addr->数据地址，u8 data->数据
返 回 值：0->成功，1->失败
*******************************************************************************/
uint8_t imc20608_write_byte(uint8_t slave_addr,uint8_t data_addr,uint8_t data)
{
	uint8_t flag=1;
	imc20608_i2c_start();//启动I2C
	imc20608_i2c_send_byte(slave_addr);//发送器件地址
	flag=imc20608_i2c_sack();
	if(flag==0) 
		return(1);
	imc20608_i2c_send_byte(data_addr);
	flag=imc20608_i2c_sack();
	if(flag==0) 
		return(1);
	imc20608_i2c_send_byte(data);
	flag=imc20608_i2c_sack();
	if(flag==0) 
		return(1);
	imc20608_i2c_stop();	
	return(0);
}

/*******************************************************************************
函数功能：bma250 I2C 读取N个字节
形     参：u8 slave_addr->器件地址，u8 data_addr->数据地址，u8 *Buffer->写入的数据，u8 no->数据字节数<8
返 回 值：0->失败，1->成功
*******************************************************************************/
uint8_t imc20608_read_nbyte(uint8_t slave_addr,uint8_t data_addr,uint8_t *Buffer,uint16_t no)
{
	uint16_t index;
	uint8_t flag;
	imc20608_i2c_start();//启动I2C
	imc20608_i2c_send_byte(slave_addr);//发送器件地址
	flag=imc20608_i2c_sack();
	if(flag==0) return(0);
	imc20608_i2c_send_byte(data_addr);//发送器件内部地址
	flag=imc20608_i2c_sack();
//	imc20608_i2c_stop();

	if(flag==0) return(0);
	imc20608_i2c_start();
	imc20608_i2c_send_byte(slave_addr+1);
	flag=imc20608_i2c_sack();
	if(flag==0) return(0);
	imc20608_i2c_send_byte(data_addr);//发送器件内部地址
	flag=imc20608_i2c_sack();
//	imc20608_i2c_stop();
	for(index=0;index<no;index++)//读取字节数据
	{
		Buffer[index]=imc20608_i2c_re_byte();//读取数据
		if(index>=no-1)
		{
			imc20608_i2c_no_mack();
		}
		else
		{
			imc20608_i2c_mack();
		}
	}
	imc20608_i2c_stop();
	return(1);
}

uint8_t icm20608_init(void)
{
	volatile unsigned char regvalue;

	icm20608_write_reg(ICM20_PWR_MGMT_1, 0x80); //复位，睡眠模式
	HAL_Delay(100);
	icm20608_write_reg(ICM20_PWR_MGMT_1, 0x01); // 关闭睡眠，自动选择时钟
	HAL_Delay(50);
	//检查ID
	regvalue = icm20608_read_reg(ICM20_WHO_AM_I);
	printf("icm20608 id = %#X\r\n", regvalue);
	if(regvalue != ICM20608D_ID && regvalue != ICM20608G_ID)
		return 0;
	//初始化寄存器
	icm20608_write_reg(ICM20_SMPLRT_DIV, 0x00);    // 输出速率是内部采样率(采样率=1KHz/(1+x) x为此寄存器配置值)
	icm20608_write_reg(ICM20_CONFIG, 0x04); 	   // 陀螺仪低通滤波BW=20Hz
	icm20608_write_reg(ICM20_GYRO_CONFIG, 0);      // 陀螺仪±250dps量程   0x18:陀螺仪±2000dps量程
	icm20608_write_reg(ICM20_ACCEL_CONFIG, 0x00);  // 加速度计±2G量程（0x18:±16G）
	icm20608_write_reg(ICM20_ACCEL_CONFIG2, 0x04); // 加速度计低通滤波BW=21.2Hz
	icm20608_write_reg(ICM20_LP_MODE_CFG, 0x00);   // 关闭低功耗
	icm20608_write_reg(ICM20_FIFO_EN, 0x00);	   // 关闭FIFO
	icm20608_write_reg(ICM20_PWR_MGMT_2, 0x00);    // 打开加速度计和陀螺仪所有轴

	return 1;
}

void icm_20608_sleep(void)
{
	icm20608_write_reg(ICM20_PWR_MGMT_1, 0x40); //进入睡眠模式
	HAL_Delay(100);
}

/**
 * 读取ICM20608寄存器值，IIC接口
 */
uint8_t icm20608_read_reg(unsigned char reg)
{
	unsigned char reg_val;
	HAL_I2C_Mem_Read(&hi2c2, ICM20608_IIC_READ_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &reg_val, 1, 100);
	//printf("iic read reg = %d\r\n", status);
//	imc20608_read_nbyte(ICM20608_IIC_WRITE_ADDR, reg,&reg_val,I2C_MEMADD_SIZE_8BIT);
	return reg_val;
}

/**
 * 写ICM20608寄存器
 * param - reg  : 要读取的寄存器地址
 * param - value: 要写入的值
 * return		:
 */
void icm20608_write_reg(unsigned char reg, unsigned char value)
{
	HAL_I2C_Mem_Write(&hi2c2, ICM20608_IIC_WRITE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
	
	//printf("iic write reg = %d\r\n", status);
//	imc20608_write_byte(ICM20608_IIC_WRITE_ADDR,reg,value);
}

/**
 * 读取ICM20608连续多个寄存器
 * param - reg ： 要读取的寄存器地址
 * param - buf ： 读取到的值
 * param - len ： 读取寄存器个数
 */
void icm20608_read_len(unsigned char reg, unsigned char *buf, unsigned char len)
{
	HAL_I2C_Mem_Read(&hi2c2, ICM20608_IIC_READ_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

/**
 * 获取陀螺仪的分辨率
 */
float icm20608_gyro_scaleget(void)
{
	unsigned char data;
	float gyroscale;

	data = (icm20608_read_reg(ICM20_GYRO_CONFIG) >> 3) & 0x3;
	switch (data) {
		case 0:
			gyroscale = 131;
			break;
		case 1:
			gyroscale = 65.5;
			break;
		case 2:
			gyroscale = 32.8;
			break;
		case 3:
			gyroscale = 16.4;
			break;
		default:
			gyroscale = 16.4;
			break;
	}
	return gyroscale;
}

// 获取加速度计的分辨率
unsigned short icm20608_accel_scaleget(void)
{
	unsigned char data;
	unsigned short accelscale;

	data = (icm20608_read_reg(ICM20_ACCEL_CONFIG) >> 3) & 0x3;
	switch (data) {
		case 0:
			accelscale = 16384;
			break;
		case 1:
			accelscale = 8192;
			break;
		case 2:
			accelscale = 4096;
			break;
		case 3:
			accelscale = 2048;
			break;
		default:
			accelscale = 16384;
			break;
	}
	return accelscale;
}

/**
 * 读取ICM20608的加速度、陀螺仪和温度原始值
 */
void icm20608_getdata(void)
{
	// 定义陀螺仪、加速度计分辨率变量
//	float gyroscale;
//	unsigned short accescale;
	unsigned char data[14];

	icm20608_read_len(ICM20_ACCEL_XOUT_H, data, 14);

//	gyroscale = icm20608_gyro_scaleget();
//	accescale = icm20608_accel_scaleget();

	icm20608_dev.accel_x_adc = (signed short)((data[0] << 8) | data[1]);
	icm20608_dev.accel_y_adc = (signed short)((data[2] << 8) | data[3]);
	icm20608_dev.accel_z_adc = (signed short)((data[4] << 8) | data[5]);
	icm20608_dev.temp_adc    = (signed short)((data[6] << 8) | data[7]);
	icm20608_dev.gyro_x_adc = (signed short)((data[8] << 8) | data[9]);
	icm20608_dev.gyro_y_adc = (signed short)((data[10] << 8) | data[11]);
	icm20608_dev.gyro_z_adc = (signed short)((data[12] << 8) | data[13]);

	// 计算实际值（实际数据扩大100倍）
	// 串口不支持浮点输出，扩大100倍变为整型
//	icm20608_dev.gyro_x_act = ((float)(icm20608_dev.gyro_x_adc)/gyroscale)*100;
//	icm20608_dev.gyro_y_act = ((float)(icm20608_dev.gyro_y_adc)/gyroscale)*100;
//	icm20608_dev.gyro_z_act = ((float)(icm20608_dev.gyro_z_adc)/gyroscale)*100;
//	icm20608_dev.accel_x_act = ((float)(icm20608_dev.accel_x_adc)/accescale)*100;
//	icm20608_dev.accel_y_act = ((float)(icm20608_dev.accel_y_adc)/accescale)*100;
//	icm20608_dev.accel_z_act = ((float)(icm20608_dev.accel_z_adc)/accescale)*100;
//	// 温度计算表达式：寄存器手册，搜索TEMP_degC
//	icm20608_dev.temp_act = (((float)(icm20608_dev.temp_adc) - 25)/326.8+25)*100;
}







