/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */
extern osMessageQId msgIndexQHandle;
/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */
extern HAL_StatusTypeDef ConfigFilter(void);
extern void sndCANMsg(void);
extern void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

typedef struct   //0x184
{
    uint8_t reserve8bit:8;  
    uint8_t reserve8bit1:8;
    
    uint8_t HallFault:1;
    uint8_t phaseLossFault:1;
    uint8_t lowVoltFault:1;
    uint8_t inverterOverTempFault:1;
    uint8_t overVoltFault:1;
    uint8_t overCurrentFault:1;
    uint8_t driveFault:1;
    uint8_t mosfetFault:1;

    uint8_t reserve5bit:5;
    uint8_t throttleFault:1;
    uint8_t rotorLockedFault:1;
    uint8_t motorOverHeatFault:1;
   
}ControlErr_s;


typedef struct  //0x185
{
    int16_t motorTemp;
    int8_t controllerTemp;   
}ControlerTemp_s;


typedef struct   //0x186
{
    int32_t motorRPM;
    uint8_t  throttlePercent;   
}InverterSpd_s;


typedef struct   //0x187
{
    uint32_t reserve:32;
    uint8_t gear:2;
    uint8_t sideStand:1;
    uint8_t brake:1;
}InverterInput_s;

extern void packJaon(void);
extern void initJsonObj(void);

extern char jsonData[800];
extern uint16_t jsonLen;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

