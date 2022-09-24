/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdbool.h>

/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
__packed typedef struct 
{
    uint16_t i;
    uint8_t m;
    uint16_t batVoltTot; //0
    int16_t  batCurrent;
    uint16_t soc;
    uint16_t soh;
    uint16_t batIconFlag;
    uint16_t batChargeStatus;
    uint16_t batCellVoltMax;
    uint16_t batCellVoltMin;
    uint16_t batTempMax;
    uint16_t batTempMin;
    uint16_t batCellVolt;
    uint16_t batCellDiffMax;
    uint16_t batTemp;
}bmsInfo_t;
/* USER CODE END Private defines */

void MX_UART4_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
extern bool readBmsRegs();
extern bool readBmsID();
extern uint8_t uart4RxData[10];
extern uint8_t uart4RxLen;



extern uint16_t batVoltTot; //0
extern uint16_t cellCnt;    //1
extern uint16_t soc;
extern uint16_t soh;
extern uint16_t capacity;
extern int16_t batCurrent;
extern uint16_t currentIn;
extern uint16_t boardTemp;
extern uint16_t batTemp1;
extern uint16_t batTemp2;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

