/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include <jansson.h>
#include "usart.h"
uint8_t mosfetFault;
uint8_t driveFault;
uint8_t overCurrentFault;
uint8_t overVoltFault;
uint8_t inverterOverTempFault;
uint8_t lowVoltFault;
uint8_t phaseLossFault;
uint8_t HallFault;

uint8_t motorOverHeatFault;
uint8_t rotorLockedFault;
uint8_t throttleFault;

int16_t motorTemp;
int8_t controllerTemp;   

int32_t motorRPM;
uint8_t  throttlePercent;   

uint8_t gear;
uint8_t sideStand;
uint8_t brake;
    
uint8_t msgIndex=0;;
uint8_t canRxData[8];
CAN_RxHeaderTypeDef rxHeader;

json_t *root;
char* text;
char jsonData[1000];

uint16_t jsonLen=0;
json_t * jsonMsg;
json_t *motorTemp_t;
json_t *controllerTemp_t;
json_t *Battery_Percent_t;
json_t *BatterySOH_t;

json_t *batTemperature_t;
json_t *batCurrent_t;
json_t *batVolt_t;
json_t *motorRPM_t;
json_t *gearInfo_t;
json_t *throttle_t;
json_t *brake_t;
json_t *sideStand_t;
void initJsonObj()
{
    jsonMsg = json_object();
    if(jsonMsg == NULL)
    {
        return;
    }
    motorTemp_t = json_integer(motorTemp);
    controllerTemp_t = json_integer(controllerTemp);
    Battery_Percent_t = json_integer(soc);
    BatterySOH_t = json_integer(soh);
    
    batTemperature_t = json_integer(boardTemp);
    batCurrent_t = json_integer(batCurrent);
    batVolt_t = json_integer(batVoltTot);
    
    motorRPM_t = json_integer(motorRPM);
    gearInfo_t = json_integer(gear);
    throttle_t = json_integer(throttlePercent);
    brake_t = json_integer(brake);
    sideStand_t = json_integer(sideStand);
    
    
  
    json_object_set( jsonMsg,"MotorTemp",motorTemp_t);
    json_object_set( jsonMsg,"InverterTemp",controllerTemp_t);
    
    json_object_set( jsonMsg,"BatPercent",Battery_Percent_t);
    json_object_set( jsonMsg,"BatSOH",BatterySOH_t);
    
    
    json_object_set( jsonMsg,"BatTemp",batTemperature_t);
    json_object_set( jsonMsg,"BatCurrent",batCurrent_t);
    json_object_set( jsonMsg,"BatVolt",batVolt_t);
    
    json_object_set( jsonMsg,"MotorRPM",motorRPM_t);
    json_object_set( jsonMsg,"GearInformation",gearInfo_t);
    json_object_set( jsonMsg,"ThrottlePercent",throttle_t);
    json_object_set( jsonMsg,"BrakeStatus",brake_t);
    json_object_set( jsonMsg,"SideStandInfo",sideStand_t);
    
    
}
void packJaon()
{
    
    json_integer_set( motorTemp_t,motorTemp);
    json_integer_set( controllerTemp_t,controllerTemp);
    json_integer_set( Battery_Percent_t,soc);
    json_integer_set( BatterySOH_t,soh);
    
    json_integer_set( batTemperature_t,boardTemp);
    json_integer_set( batCurrent_t,batCurrent);
    json_integer_set( batVolt_t,batVoltTot);
    json_integer_set( motorRPM_t,motorRPM);
    json_integer_set( gearInfo_t,gear);
    json_integer_set( throttle_t,throttlePercent);
    json_integer_set( brake_t,brake);
    json_integer_set( sideStand_t,sideStand);
    
    
    

    text = json_dumps(jsonMsg,0);
    jsonData[0]=0xDD;
    jsonLen = snprintf(&jsonData[1],200,"%s",text);
    jsonData[jsonLen+1]=0xee;  
    jsonLen+=2;
    free(text);    
}


void updateInverteInfo()
{
    if((rxHeader.StdId == 0x184) &&(rxHeader.DLC>=4))
    {
        ControlErr_s* ptr_s = (ControlErr_s *)canRxData;
        mosfetFault           = ptr_s->mosfetFault;
        driveFault            = ptr_s->driveFault;
        overCurrentFault      = ptr_s->overCurrentFault;
        overVoltFault         = ptr_s->overVoltFault;
        inverterOverTempFault = ptr_s->inverterOverTempFault;
        lowVoltFault          = ptr_s->lowVoltFault;
        phaseLossFault        = ptr_s->phaseLossFault;
        HallFault             = ptr_s->HallFault;

        motorOverHeatFault    = ptr_s->motorOverHeatFault;
        rotorLockedFault      = ptr_s->rotorLockedFault;
        throttleFault         = ptr_s->throttleFault;           
    }
    else if((rxHeader.StdId == 0x185) &&(rxHeader.DLC>=3))
    {
         ControlerTemp_s* ptr_s = (ControlerTemp_s *)canRxData;
         motorTemp   = ptr_s->motorTemp;
         controllerTemp = ptr_s->controllerTemp; 
    }
    else if((rxHeader.StdId == 0x186) &&(rxHeader.DLC>=5))
    {
         InverterSpd_s* ptr_s = (InverterSpd_s *)canRxData;
         motorRPM   = ptr_s->motorRPM;
         throttlePercent = ptr_s->throttlePercent; 
    }
    else if((rxHeader.StdId == 0x187) &&(rxHeader.DLC>=5))
    {
         InverterInput_s* ptr_s = (InverterInput_s *)canRxData;
         gear      = ptr_s->gear;
         sideStand = ptr_s->sideStand; 
         brake = ptr_s->brake; 
    }
    
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

    HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rxHeader,canRxData);
    HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);//toggle red led when recv can msg
    
    updateInverteInfo();

    
}
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 18;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 18;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);\
    
    CLEAR_BIT(canHandle->Instance->MCR,CAN_MCR_SLEEP);

    __HAL_AFIO_REMAP_CAN1_2();
    
    

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

     CLEAR_BIT(canHandle->Instance->MCR,CAN_MCR_SLEEP);
    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
HAL_StatusTypeDef ConfigFilter()
{
    CAN_FilterTypeDef canFilter;
    canFilter.FilterBank = 0;
    canFilter.FilterActivation = ENABLE;
    canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilter.FilterIdHigh = 0;
    canFilter.FilterIdLow = 0;
    canFilter.FilterMaskIdHigh=0x0;
    canFilter.FilterMaskIdLow=0x0;
    canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilter.SlaveStartFilterBank=15;
    
    HAL_CAN_ConfigFilter(&hcan1,&canFilter);
    
    canFilter.FilterBank=15;
    canFilter.SlaveStartFilterBank=15;
    HAL_CAN_ConfigFilter(&hcan2,&canFilter);
    
} 
    
uint8_t data[8];
uint32_t mailbox;
CAN_TxHeaderTypeDef pTxHeader;
void sndCANMsg()
{

    pTxHeader.StdId=0X123;
    pTxHeader.IDE= CAN_ID_STD;
    pTxHeader.DLC =1;
    HAL_CAN_AddTxMessage(&hcan1,&pTxHeader,data,&mailbox);
    HAL_CAN_AddTxMessage(&hcan2,&pTxHeader,data,&mailbox);
}
/* USER CODE END 1 */
