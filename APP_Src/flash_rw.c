/*
 * flash_rw.c
 *
 *  Created on: 2025-02-18
 *      Author: lang
 */
#include "flash_rw.h"
#include "stm32g4xx_hal_flash_ex.h"
#include "stm32g4xx_hal_flash.h"
#include "stdint.h"

#define FLASH_USER_Page 63 //STM32G431=128KB，页数最大63
#define FLASH_USER_START_ADDR  ((uint32_t)0x0801F800) /* Base @ of Page 63, 2 Kbytes */ //(FLASH_BASE + (FLASH_USER_Page * FLASH_PAGE_SIZE)) /* 用户Flash区域的起始@ */
#define FLASH_USER_END_ADDR    ((uint32_t)0x0801FFFF)  //(FLASH_BASE + FLASH_SIZE - 1)                      /* 使用者Flash区域的结束@ */

static FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t PageError = 0;

// 擦除最后一页
void Flash_Erase(void)
{
  /* 填满 EraseInit 结构*/
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES; //页擦除
  EraseInitStruct.Page      = 63;             //擦除页
  EraseInitStruct.NbPages   = 1;             //擦除页数
  //擦除FLASH
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
  {
	  Error_Handler( );
  }
}

void Flash_Write(uint64_t *pBuffer, uint32_t  NumToWrite)
{
  uint16_t  i = 0;
  uint32_t Address = FLASH_USER_START_ADDR;
  //强制转换后写入数组长度向上取整(SIZE+3)/4
  NumToWrite = (NumToWrite + 3) / 4;
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
  Flash_Erase();      //先擦除
  //写入
  while((Address<FLASH_USER_END_ADDR) && (i<NumToWrite))
  {
    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, pBuffer[i]) == HAL_OK)  // (uint64_t)*(pBuffer+i*8)
    {
      Address = Address + 8;
      i++;
    }
    else
      {
      Error_Handler( );
      }
  }
  HAL_FLASH_Lock();   //上锁
}

void Flash_Read(uint32_t *pBuffer, uint32_t NumToRead)
{
  uint16_t i = 0;
  uint32_t Address = FLASH_USER_START_ADDR;
  //强制转换后写入数组长度向上取整(SIZE+3)/4
  NumToRead = (NumToRead + 3) / 4;
  while((Address<FLASH_USER_END_ADDR)&&(i<NumToRead))
  {
    pBuffer[i++]= *(__IO uint32_t *)Address;
    Address = Address + 4;
  }
}
