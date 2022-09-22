/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"


/* USER CODE BEGIN 0 */
#include "stdio.h"
#include "string.h"
#include <stdbool.h>

uint8_t uart4RxData[10];
uint8_t uart4RxLen;

uint16_t batPercent;
int16_t batTemperature;
int16_t batCurrent;
uint16_t batVolt;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  HAL_UART_Receive_IT(&huart4,uart4RxData,1);
  __HAL_UART_ENABLE_IT(&huart4,UART_IT_RXNE);
    
}

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{  	
    while ((huart3.Instance->SR & 0X40) == 0); 
    	huart3.Instance->DR = (uint8_t) ch;
    return ch;
}

uint16_t batVoltTot; //0
uint16_t cellCnt;    //1
uint16_t soc;
uint16_t capacity;
uint16_t currentOut;
uint16_t currentIn;
uint16_t boardTemp;
uint16_t batTemp1;
uint16_t batTemp2;
uint16_t tempOfCell_1;
uint16_t tempOfCell_2;
uint16_t tempOfCell_3;
uint16_t tempOfCell_4;
uint16_t tempOfCell_5;
uint16_t tempOfCell_6;
uint16_t tempOfCell_7;
uint16_t tempOfCell_8;
uint16_t tempOfCell_9;
uint16_t tempOfCell_10;
uint16_t tempOfCell_11;
uint16_t tempOfCell_12;
uint16_t tempOfCell_13;
uint16_t tempOfCell_14;
uint16_t tempOfCell_15;
uint16_t tempOfCell_16;
uint16_t tempOfCell_17;
uint16_t tempOfCell_18;
uint16_t tempOfCell_19;
uint16_t tempOfCell_20; 
uint8_t reqBmsData[10]={0x02,0x01,0x00,0x00,};

uint8_t uart1RxStatus =0;

uint8_t bmsRxData[100];

bmsInfo_t* bmsInfoPtr;

unsigned short ModBusCRC16(const void *s, int n)
{ 
    unsigned short c = 0xffff; 
    for(int k=0; k<n; k++)
    { 
        unsigned short b=(((unsigned char*)s)[k]); 
        for(char i=0; i<8; i++)
        {
            c = ((b^c)&1) ? (c>>1)^0xA001 : (c>>1); b>>=1;
        }
    } 
    return (c<<8)|(c>>8);
}

void updateBatInfo()
{
 batVoltTot      = bmsInfoPtr->batVoltTot; //0
 cellCnt         = bmsInfoPtr->cellCnt;    //1
 soc             = bmsInfoPtr ->soc;
 capacity        = bmsInfoPtr->capacity;
 currentOut      = bmsInfoPtr->currentOut;
 currentIn       = bmsInfoPtr->currentIn;
 boardTemp       = bmsInfoPtr->boardTemp;
 batTemp1        = bmsInfoPtr->batTemp1;
 batTemp2        = bmsInfoPtr->batTemp2;
 tempOfCell_1    = bmsInfoPtr->tempOfCell_1;
 tempOfCell_2    = bmsInfoPtr->tempOfCell_2;
 tempOfCell_3    = bmsInfoPtr->tempOfCell_3;
 tempOfCell_4    = bmsInfoPtr->tempOfCell_4;
 tempOfCell_5    = bmsInfoPtr->tempOfCell_5;
 tempOfCell_6    = bmsInfoPtr->tempOfCell_6;
 tempOfCell_7    = bmsInfoPtr->tempOfCell_7;
 tempOfCell_8    = bmsInfoPtr->tempOfCell_8;
 tempOfCell_9    = bmsInfoPtr->tempOfCell_9;
 tempOfCell_10   = bmsInfoPtr->tempOfCell_10;
 tempOfCell_11   = bmsInfoPtr->tempOfCell_11;
 tempOfCell_12   = bmsInfoPtr->tempOfCell_12;
 tempOfCell_13   = bmsInfoPtr->tempOfCell_13;
 tempOfCell_14   = bmsInfoPtr->tempOfCell_14;
 tempOfCell_15   = bmsInfoPtr->tempOfCell_15;
 tempOfCell_16   = bmsInfoPtr->tempOfCell_16;
 tempOfCell_17   = bmsInfoPtr->tempOfCell_17;
 tempOfCell_18   = bmsInfoPtr->tempOfCell_18;
 tempOfCell_19   = bmsInfoPtr->tempOfCell_19;
 tempOfCell_20   = bmsInfoPtr->tempOfCell_20; 
 

 batVoltTot      = __REV16(batVoltTot); //0
 cellCnt         = __REV16(cellCnt);    //1
 soc             = __REV16(soc);
 capacity        = __REV16(capacity);
 currentOut      = __REV16(currentOut);
 currentIn       = __REV16(currentIn);
 boardTemp       = __REV16(boardTemp);
 batTemp1        = __REV16(batTemp1);
 batTemp2        = __REV16(batTemp2);
 tempOfCell_1    = __REV16(tempOfCell_1);
 tempOfCell_2    = __REV16(tempOfCell_2);
 tempOfCell_3    = __REV16(tempOfCell_3);
 tempOfCell_4    = __REV16(tempOfCell_4);
 tempOfCell_5    = __REV16(tempOfCell_5);
 tempOfCell_6    = __REV16(tempOfCell_6);
 tempOfCell_7    = __REV16(tempOfCell_7);
 tempOfCell_8    = __REV16(tempOfCell_8);
 tempOfCell_9    = __REV16(tempOfCell_9);
 tempOfCell_10   = __REV16(tempOfCell_10);
 tempOfCell_11   = __REV16(tempOfCell_11);
 tempOfCell_12   = __REV16(tempOfCell_12);
 tempOfCell_13   = __REV16(tempOfCell_13);
 tempOfCell_14   = __REV16(tempOfCell_14);
 tempOfCell_15   = __REV16(tempOfCell_15);
 tempOfCell_16   = __REV16(tempOfCell_16);
 tempOfCell_17   = __REV16(tempOfCell_17);
 tempOfCell_18   = __REV16(tempOfCell_18);
 tempOfCell_19   = __REV16(tempOfCell_19);
 tempOfCell_20   = __REV16(tempOfCell_20);
}

__packed typedef struct 
{
    uint8_t bmsAdr;
    uint8_t cmd;
    uint16_t start;
    uint16_t cnt;
    uint16_t crc;
}bmsReq_t;
bmsReq_t bmsReq;

uint8_t bmsID[13];

uint8_t bmsTxData[8] ;//   ={0x01,0x03,0x00,0x00,0x00,0x1d,0x85,0xc3};  //big endian


bool readBmsID()
{
    uint16_t crcRet;
   
    memset(bmsRxData,0x00,sizeof(bmsRxData));
    HAL_GPIO_WritePin(DE_RE_485_GPIO_Port,DE_RE_485_Pin,GPIO_PIN_SET);
    bmsReq.bmsAdr = 0x01;
    bmsReq.cmd=0x03; //read regs
    bmsReq.start=1000;
    bmsReq.cnt=13;
    memcpy(bmsTxData,&bmsReq,sizeof(bmsTxData));
    *(uint16_t *)&bmsTxData[2] = __REV16(*(uint16_t *)&bmsTxData[2]);
    *(uint16_t *)&bmsTxData[4] = __REV16(*(uint16_t *)&bmsTxData[4]);
    crcRet = ModBusCRC16(bmsTxData,6);
    *(uint16_t *)&bmsTxData[6]= __REV16(crcRet);
     HAL_UART_Transmit(&huart1,bmsTxData,8,500);  
    HAL_GPIO_WritePin(DE_RE_485_GPIO_Port,DE_RE_485_Pin,GPIO_PIN_RESET);
    uart1RxStatus = HAL_UART_Receive(&huart1,bmsRxData, bmsReq.cnt*2+5,1000);  
    if(uart1RxStatus == HAL_OK)
    {
        printf("read bms ID info ok\r\n");
        HAL_GPIO_TogglePin(LED_Y_GPIO_Port,LED_Y_Pin);//toggle yellow led when recv 485 msg
        memcpy(bmsID,bmsRxData,sizeof(bmsID));
        return true;
    }
    else
    {
        return false;
    }
        
    
}

bool readBmsRegs()
{
    uint16_t crcRet;
   
    memset(bmsRxData,0x00,sizeof(bmsRxData));
    HAL_GPIO_WritePin(DE_RE_485_GPIO_Port,DE_RE_485_Pin,GPIO_PIN_SET);
    bmsReq.bmsAdr = 0x01;
    bmsReq.cmd=0x03; //read regs
    bmsReq.start=0x00;
    bmsReq.cnt=0x7;
    memcpy(bmsTxData,&bmsReq,sizeof(bmsTxData));
    *(uint16_t *)&bmsTxData[2] = __REV16(*(uint16_t *)&bmsTxData[2]);
    *(uint16_t *)&bmsTxData[4] = __REV16(*(uint16_t *)&bmsTxData[4]);
    crcRet = ModBusCRC16(bmsTxData,6);
    *(uint16_t *)&bmsTxData[6]= __REV16(crcRet);
     HAL_UART_Transmit(&huart1,bmsTxData,8,500);  
    HAL_GPIO_WritePin(DE_RE_485_GPIO_Port,DE_RE_485_Pin,GPIO_PIN_RESET);
    uart1RxStatus = HAL_UART_Receive(&huart1,bmsRxData, bmsReq.cnt*2+5,1000);  
    if(uart1RxStatus == HAL_OK)
    {
        printf("read bms info ok\r\n");
        HAL_GPIO_TogglePin(LED_Y_GPIO_Port,LED_Y_Pin);//toggle yellow led when recv 485 msg
        bmsInfoPtr = (bmsInfo_t *)bmsRxData;
        updateBatInfo();
        return true;
    }
    else
    {
        return false;
    }
}

uint8_t bmsRead01Data[8] ={0x01,0x01,0x00,0x00,0x00,0x34,0x3d,0xdd}; //read bms error info
bool readBmsErrInfo()
{
    uint16_t crcRet;
       
    memset(bmsRxData,0x00,sizeof(bmsRxData));
    HAL_GPIO_WritePin(DE_RE_485_GPIO_Port,DE_RE_485_Pin,GPIO_PIN_SET);
    bmsReq.bmsAdr = 0x01;
    bmsReq.cmd=0x01; //read regs
    bmsReq.start=0x00;
    bmsReq.cnt=0x34;
    memcpy(bmsTxData,&bmsReq,sizeof(bmsTxData));
    *(uint16_t *)&bmsTxData[2] = __REV16(*(uint16_t *)&bmsTxData[2]);
    *(uint16_t *)&bmsTxData[4] = __REV16(*(uint16_t *)&bmsTxData[4]);
    crcRet = ModBusCRC16(bmsTxData,6);
    *(uint16_t *)&bmsTxData[6]= __REV16(crcRet);
     HAL_UART_Transmit(&huart1,bmsTxData,8,500);  
    HAL_GPIO_WritePin(DE_RE_485_GPIO_Port,DE_RE_485_Pin,GPIO_PIN_RESET);
    uart1RxStatus = HAL_UART_Receive(&huart1,bmsRxData,(0x34+7)/8+5,10000);  
    if(uart1RxStatus == HAL_OK)
    {
        printf("read bms info ok\r\n");
         HAL_GPIO_TogglePin(LED_Y_GPIO_Port,LED_Y_Pin);//toggle yellow led when recv 485 msg
        bmsInfoPtr = (bmsInfo_t *)bmsRxData;
        updateBatInfo();
        return true;
    }
    else
    {
        return false;
    }
    
}


/* USER CODE END 0 */

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_tx;

/* UART4 init function */
void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}
/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */

  /* USER CODE END UART4_MspInit 0 */
    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX
    */
    GPIO_InitStruct.Pin = EC600_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(EC600_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = EC600_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(EC600_RX_GPIO_Port, &GPIO_InitStruct);

    /* UART4 DMA Init */
    /* UART4_TX Init */
    hdma_uart4_tx.Instance = DMA2_Channel5;
    hdma_uart4_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart4_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_tx.Init.Mode = DMA_NORMAL;
    hdma_uart4_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_uart4_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_uart4_tx);

    /* UART4 interrupt Init */
    HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspInit 1 */

  /* USER CODE END UART4_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = USART_RX_485_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(USART_RX_485_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = USART_TX_485_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USART_TX_485_GPIO_Port, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = Debug_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(Debug_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Debug_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(Debug_RX_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspDeInit 0 */

  /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX
    */
    HAL_GPIO_DeInit(GPIOC, EC600_TX_Pin|EC600_RX_Pin);

    /* UART4 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* UART4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspDeInit 1 */

  /* USER CODE END UART4_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, USART_RX_485_Pin|USART_TX_485_Pin);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, Debug_TX_Pin|Debug_RX_Pin);

  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
