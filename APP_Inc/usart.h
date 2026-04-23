/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */
typedef struct
{
	uint16_t w_i;				//write position 0
	uint16_t r_i;				//read position 0
	uint8_t  over;				//0->buf未溢出	1->溢出，填充buf时如果溢出置该标志为1，打印该标志后清0
	const uint16_t max;	//buf空间
	uint8_t  *buf;
}UART_CYC_STRUCT;
extern UART_CYC_STRUCT uart3_struct;

typedef struct
{
	uint16_t Length;			//Buffer内实际数据长度,通过它判断Buffer是否为空
	const uint16_t Size;     //Buffer空间大小
	uint8_t *Buffer;
}BUFFER_UNIT_STRUCT;

extern BUFFER_UNIT_STRUCT uart_receive_buffer;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN Private defines */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(FILE *f)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */
/* USER CODE END Private defines */
#define USART3_BAUD_SETTING_115200 115200
#define USART3_BAUD_SETTING_230400 230400
//#define USART3_BAUD_SETTING_230400 576000
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(uint32_t baud);
uint8_t uart_fill_cbuffer(UART_CYC_STRUCT *puart_cbuffer,uint8_t *last_data,uint16_t data_l);
void cpu_get_cmd_re_cyc(UART_CYC_STRUCT* pcyc,BUFFER_UNIT_STRUCT *pcmd);
/* USER CODE BEGIN Prototypes */
void MX_USART1_UART_DeInit(void);
void MX_USART2_UART_DeInit(void);
void MX_USART3_UART_DeInit(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

