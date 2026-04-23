/**
 * flash_rw.h
 *
 * flash 操作（存储vl53l0x配置参数，紧存放在最后一页）
 *
 */
#ifndef __FLASH_RW_H__
#define __FLASH_RW_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

void Flash_Write(uint64_t *pBuffer, uint32_t  NumToWrite);
void Flash_Read(uint32_t *pBuffer, uint32_t NumToRead);



#ifdef __cplusplus
}
#endif

#endif

