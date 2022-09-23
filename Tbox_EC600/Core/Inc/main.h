/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef __packed struct 
{
    uint8_t head;
    uint8_t  data[20];
    uint8_t tail;
}canMsgType;
extern canMsgType packData;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BatVolt_Adc1_10_Pin GPIO_PIN_0
#define BatVolt_Adc1_10_GPIO_Port GPIOC
#define VOLT12_FLAG_Pin GPIO_PIN_2
#define VOLT12_FLAG_GPIO_Port GPIOA
#define BAT_EN_Pin GPIO_PIN_3
#define BAT_EN_GPIO_Port GPIOA
#define LET_USART_DTR_Pin GPIO_PIN_4
#define LET_USART_DTR_GPIO_Port GPIOA
#define LED_R_Pin GPIO_PIN_5
#define LED_R_GPIO_Port GPIOC
#define LED_Y_Pin GPIO_PIN_1
#define LED_Y_GPIO_Port GPIOB
#define Debug_TX_Pin GPIO_PIN_10
#define Debug_TX_GPIO_Port GPIOB
#define Debug_RX_Pin GPIO_PIN_11
#define Debug_RX_GPIO_Port GPIOB
#define CHRG_STOP_Pin GPIO_PIN_14
#define CHRG_STOP_GPIO_Port GPIOB
#define LTE_RESET_Pin GPIO_PIN_8
#define LTE_RESET_GPIO_Port GPIOA
#define USART_RX_485_Pin GPIO_PIN_9
#define USART_RX_485_GPIO_Port GPIOA
#define USART_TX_485_Pin GPIO_PIN_10
#define USART_TX_485_GPIO_Port GPIOA
#define DE_RE_485_Pin GPIO_PIN_11
#define DE_RE_485_GPIO_Port GPIOA
#define EC600_EN_Pin GPIO_PIN_15
#define EC600_EN_GPIO_Port GPIOA
#define EC600_TX_Pin GPIO_PIN_10
#define EC600_TX_GPIO_Port GPIOC
#define EC600_RX_Pin GPIO_PIN_11
#define EC600_RX_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
