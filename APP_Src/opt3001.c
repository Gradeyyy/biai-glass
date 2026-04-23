/**
  *   lang
  *
  *   2025-02-19
 */

#include "opt3001.h"

extern I2C_HandleTypeDef hi2c3;


static uint8_t device_addr= OPT3001_ADDR1;   // read = (device_addr << 1)+1 = 0x89;   write = (device_addr << 1) = 0x88
static uint16_t config_data= 0xCE10;        //The sensor works in continuous operation mode by default. 0xCE10:连续模式 0xCA10:单次模式
float coef[12] = {0.01, 0.02, 0.04, 0.08, 0.16, 0.32, 0.64, 1.28, 2.56, 5.12, 10.24, 20.48};


void opt3001_writedata(uint8_t register_addr, uint16_t data){

	 uint8_t buffer[2];
	 buffer[0] = (data)>>8;
	 buffer[1] = (data)& 0x00FF;
	 //HAL_I2C_Master_Transmit(&hi2c2, device_addr<<1, buffer, 3, 50);
	 HAL_I2C_Mem_Write(&hi2c3, (device_addr<<1), register_addr, I2C_MEMADD_SIZE_8BIT, buffer, 2, 100);
 }


uint16_t opt3001_readdata(uint8_t register_addr){

	uint8_t readbuffer[2];
	uint16_t received_data;
	HAL_I2C_Mem_Read(&hi2c3, ((device_addr << 1)+1), register_addr, I2C_MEMADD_SIZE_8BIT, readbuffer, 2, 100);
	received_data = (((uint16_t )readbuffer[0] << 8) | readbuffer[1]);
	
	return received_data;
}

uint8_t opt3001_init(void){

	if (HAL_I2C_IsDeviceReady(&hi2c3, device_addr<<1, 100, 20000) != HAL_OK){
		return 0;
	}

	opt3001_writedata(OPT3001_ConfigReg, config_data);
	return 1;
}

void opt3001_shutdown(void)
{
	opt3001_writedata(OPT3001_ConfigReg, 0xC810);

}

/**
* @brief  提取16位二进制数据中一位，返回该位是0或1
* @param	DATA:原始数据
* @param	NUM：数据位置
* @retval 数据值
*/
int Extract_text(uint16_t *DATA, int NUM)
{
  int RE_NUM = 3;
  if (NUM < 0 || NUM > 15)
  {
    printf("提取发生错误");
    while (1)
      ;
    return RE_NUM;
  }

  if (*DATA & (1 << NUM))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

/**
* @brief  提取十六位二进制中的一段，转换为十进制
* @param	DATA:原始数据
* @param	NUM_Start：数据位置开始
* @param	NUM_Stop：数据位置停止
* @retval 数据值
*/
int Binary_To_Decimal(uint16_t *DATA, int NUM_Start, int NUM_Stop)
{
  //函数开启条件：开始和停止位置需要大于0小于15，并且停止位置大于开始位置
  if ((NUM_Start < 0 || NUM_Start > 15) || (NUM_Stop < 0 && NUM_Stop > 15) || (NUM_Start > NUM_Stop))
  {
    printf("转换发送错误\r\n");
    return (-1);
  }
  //读取二进制位
  int NUM_B[16] = {0};
  for (int i = 0; i < ((NUM_Stop - NUM_Start) + 1); i++)
  {
    NUM_B[i] = Extract_text(DATA, NUM_Start + i);
  }
  //转换为十进制
  int NUM_D = 0;
  for (int i = 0; i < ((NUM_Stop - NUM_Start) + 1); i++)
  {
    NUM_D = NUM_D + (NUM_B[i] * pow(2, i));
  }
  return NUM_D;
}

float calculate_lux(void){
	
	uint16_t iExponent, iMantissa;
    uint16_t final_lux;
    uint16_t rawlux;

//    opt3001_writedata(OPT3001_ResultReg, 0x00);
//    HAL_Delay(50);
    rawlux = opt3001_readdata(OPT3001_ResultReg);
    iMantissa = rawlux & 0x0FFF;
    iExponent = (rawlux & 0xF000) >> 12;
//    // final_lux = iMantissa * (0.01  * powf(2, iExponent));
    printf("coef[iExponent]: %lf\r\n", coef[iExponent]);
    printf("iMantissa: %d\r\n", iMantissa);
    final_lux = coef[iExponent] * iMantissa;
//    final_lux =0.01 * rawlux;
//    final_lux = 0.01 * (pow(2, (Binary_To_Decimal(&rawlux, 12, 15)))) * Binary_To_Decimal(&rawlux, 0, 11);
    return final_lux;
}

uint16_t calculate_lux_int(void)
{
	uint16_t rawlux;
    rawlux = opt3001_readdata(OPT3001_ResultReg);
    if(rawlux >0xffff)
    {
    	rawlux = 0xffff;
    }
//    rawLux *= 0.01;
//    if(rawlux < 10000)
//    {
//    	rawlux *= 0.01;
//    }
//    else if( 10000 <= rawlux <= 12000)
//    {
//    	rawlux /= 60;
//    }
//    else if( 12000 <= rawlux <= 15000)
//    {
//    	rawlux /= 20;
//    }
//    else
//    	rawlux /= 10;

	return rawlux;
}

uint16_t read_devid(void){

	uint16_t received_data;
//    opt3001_writedata(OPT3001_DeviceID, 0x00);
//    HAL_Delay(50);
    received_data = opt3001_readdata(OPT3001_DeviceID);

	return received_data;

}

uint16_t read_manufacturer_id(void){

	uint16_t received_data;
//    opt3001_writedata(OPT3001_ManuID, 0x00);
//    HAL_Delay(50);
    received_data = opt3001_readdata(OPT3001_ManuID);

	return received_data;
}
