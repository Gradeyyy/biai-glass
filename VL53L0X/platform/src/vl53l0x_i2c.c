#include "vl53l0x_i2c.h"


//VL53L0X I2C初始化  写地址：0x52 读地址：0x53
I2C_HandleTypeDef hi2c3;
void VL53L0X_i2c_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
//	if(i2cHandle->Instance==I2C2)
//	{
	 PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C3;
	PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	//PB5 AF8 scl3
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF8_I2C3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	//PA8 AF2 sda3
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_I2C3;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* I2C2 clock enable */
	__HAL_RCC_I2C3_CLK_ENABLE();
	HAL_NVIC_SetPriority(I2C3_ER_IRQn, 0, 2);
	HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);

	hi2c3.Instance = I2C3;
	hi2c3.Init.Timing = 0x60505F8C;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK)
	{
		Error_Handler();
	}
	//i2c3
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}
		//i2c3
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
	{
		Error_Handler();
	}
}

//IIC写一个字节数据
uint8_t VL_IIC_Write_1Byte(uint8_t SlaveAddress, uint8_t REG_Address,uint8_t REG_data)
{
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(&hi2c3, SlaveAddress, REG_Address, I2C_MEMADD_SIZE_8BIT, &REG_data, 1, 100);
	return status;
}

//IIC读一个字节数据
uint8_t VL_IIC_Read_1Byte(uint8_t SlaveAddress, uint8_t REG_Address,uint8_t *REG_data)
{
	unsigned char reg_val;
	HAL_I2C_Mem_Read(&hi2c3, (SlaveAddress|0x01), REG_Address, I2C_MEMADD_SIZE_8BIT, &reg_val, 1, 100);
	//printf("iic read reg = %d\r\n", status);
	return reg_val;
}

//IIC写n字节数据
uint8_t VL_IIC_Write_nByte(uint8_t SlaveAddress, uint8_t REG_Address,uint16_t len, uint8_t *buf)
{
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(&hi2c3, SlaveAddress, REG_Address, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
	return status;
}

//IIC读n字节数据
uint8_t VL_IIC_Read_nByte(uint8_t SlaveAddress, uint8_t REG_Address,uint16_t len,uint8_t *buf)
{
	HAL_StatusTypeDef state;
	state = HAL_I2C_Mem_Read(&hi2c3, (SlaveAddress|0x01), REG_Address, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
	return state;
}

//VL53L0X 写多个数据
//address:地址
//index:偏移地址
//pdata:数据指针
//count:长度 最大65535
uint8_t VL53L0X_write_multi(uint8_t address, uint8_t index,uint8_t *pdata,uint16_t count)
{
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(&hi2c3, address, index, I2C_MEMADD_SIZE_8BIT, pdata, count, 100);
	return status;
}


//VL53L0X 读多个数据
//address:地址
//index:偏移地址
//pdata:数据指针
//count:长度 最大65535
uint8_t VL53L0X_read_multi(uint8_t address,uint8_t index,uint8_t *pdata,uint16_t count)
{
	HAL_StatusTypeDef state;
	state = HAL_I2C_Mem_Read(&hi2c3, (address|0x01), index, I2C_MEMADD_SIZE_8BIT, pdata, count, 100);
	return state;
}

//VL53L0X 写1个数据(单字节)
//address:地址
//index:偏移地址
//data:数据(8位)
uint8_t VL53L0X_write_byte(uint8_t address,uint8_t index,uint8_t data)
{
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(&hi2c3, address, index, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
	return status;
}

//VL53L0X 写1个数据(双字节)
//address:地址
//index:偏移地址
//data:数据(16位)
uint8_t VL53L0X_write_word(uint8_t address,uint8_t index,uint16_t data)
{
	uint8_t buffer[2];
	buffer[0] = (uint8_t)((data&0xff00)>>8);
	buffer[1] = (uint8_t)(data&0xff);
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(&hi2c3, address, index, I2C_MEMADD_SIZE_8BIT, buffer, 2, 100);
	return status;
}

//VL53L0X 写1个数据(四字节)
//address:地址
//index:偏移地址
//data:数据(32位)
uint8_t VL53L0X_write_dword(uint8_t address,uint8_t index,uint32_t data)
{
	
//    uint8_t status = STATUS_OK;
//
//    uint8_t buffer[4];
//
//	//将32位数据拆分成8位
//	buffer[0] = (uint8_t)(data>>24);
//	buffer[1] = (uint8_t)((data&0xff0000)>>16);
//	buffer[2] = (uint8_t)((data&0xff00)>>8);
//	buffer[3] = (uint8_t)(data&0xff);
//
//	status = VL53L0X_write_multi(address,index,buffer,4);
//
//	return status;
	uint8_t buffer[4];

	//将32位数据拆分成8位
	buffer[0] = (uint8_t)(data>>24);
	buffer[1] = (uint8_t)((data&0xff0000)>>16);
	buffer[2] = (uint8_t)((data&0xff00)>>8);
	buffer[3] = (uint8_t)(data&0xff);
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(&hi2c3, address, index, I2C_MEMADD_SIZE_8BIT, buffer, 4, 100);
	return status;
}


//VL53L0X 读1个数据(单字节)
//address:地址
//index:偏移地址
//data:数据(8位)
uint8_t VL53L0X_read_byte(uint8_t address,uint8_t index,uint8_t *pdata)
{
	HAL_StatusTypeDef state;
	state = HAL_I2C_Mem_Read(&hi2c3, (address|0x01), index, I2C_MEMADD_SIZE_8BIT, pdata, 1, 100);
	return state;
}

//VL53L0X 读个数据(双字节)
//address:地址
//index:偏移地址
//data:数据(16位)
uint8_t VL53L0X_read_word(uint8_t address,uint8_t index,uint16_t *pdata)
{
//	uint8_t status = STATUS_OK;
//
//	uint8_t buffer[2];
//
//	status = VL53L0X_read_multi(address,index,buffer,2);
//
//	*pdata = ((uint16_t)buffer[0]<<8)+(uint16_t)buffer[1];
//
//	return status;
	uint8_t buffer[2];
	HAL_StatusTypeDef state;
	state = HAL_I2C_Mem_Read(&hi2c3, (address|0x01), index, I2C_MEMADD_SIZE_8BIT, buffer, 2, 100);
	*pdata = ((uint16_t)buffer[0]<<8)+(uint16_t)buffer[1];
	return state;
	
}

//VL53L0X 读1个数据(四字节)
//address:地址
//index:偏移地址
//data:数据(32位)
uint8_t VL53L0X_read_dword(uint8_t address,uint8_t index,uint32_t *pdata)
{
//	uint8_t status = STATUS_OK;
//
//	uint8_t buffer[4];
//
//	status = VL53L0X_read_multi(address,index,buffer,4);
//
//	*pdata = ((uint32_t)buffer[0]<<24)+((uint32_t)buffer[1]<<16)+((uint32_t)buffer[2]<<8)+((uint32_t)buffer[3]);
//
//	return status;
	uint8_t buffer[4];
	HAL_StatusTypeDef state;
	state = HAL_I2C_Mem_Read(&hi2c3, (address|0x01), index, I2C_MEMADD_SIZE_8BIT, buffer, 4, 100);
	*pdata = ((uint32_t)buffer[0]<<24)+((uint32_t)buffer[1]<<16)+((uint32_t)buffer[2]<<8)+((uint32_t)buffer[3]);
	return state;
	
}
