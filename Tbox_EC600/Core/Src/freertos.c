/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "gpio.h"
#include "inv_mpu.h"
#include "can.h"
#include "stdio.h"
#include "string.h"
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t volt12Value=0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId ec600ComTaskHandle;
osThreadId powerControlHandle;
osThreadId batterInfoSyncHandle;
osMessageQId msgIndexQHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartEC600Com(void const * argument);
void powerControlFunc(void const * argument);
void readBatteryInfo(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of msgIndexQ */
  osMessageQDef(msgIndexQ, 16, uint16_t);
  msgIndexQHandle = osMessageCreate(osMessageQ(msgIndexQ), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ec600ComTask */
  osThreadDef(ec600ComTask, StartEC600Com, osPriorityNormal, 0, 256);
  ec600ComTaskHandle = osThreadCreate(osThread(ec600ComTask), NULL);

  /* definition and creation of powerControl */
  osThreadDef(powerControl, powerControlFunc, osPriorityIdle, 0, 128);
  powerControlHandle = osThreadCreate(osThread(powerControl), NULL);

  /* definition and creation of batterInfoSync */
  osThreadDef(batterInfoSync, readBatteryInfo, osPriorityNormal, 0, 128);
  batterInfoSyncHandle = osThreadCreate(osThread(batterInfoSync), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
 
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
 
        //set trigger signal
    
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartEC600Com */
/**
* @brief Function implementing the ec600ComTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEC600Com */
void StartEC600Com(void const * argument)
{
  /* USER CODE BEGIN StartEC600Com */
    osEvent event;
    uint16_t index;
    uint8_t i=0;
    packData.head=0xDD;
    packData.tail=0xEE; 
    for(i=0;i<20;i++)
    {
       packData.data[i]=i;
    }
   initJsonObj();
   
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
    
    packJaon();
    HAL_UART_Transmit_DMA(&huart4,jsonData,jsonLen);
    
  }
  /* USER CODE END StartEC600Com */
}

/* USER CODE BEGIN Header_powerControlFunc */
/**
* @brief Function implementing the powerControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_powerControlFunc */
uint8_t rxCnt=0;
void powerControlFunc(void const * argument)
{
  /* USER CODE BEGIN powerControlFunc */
    uint8_t lpmStartFlag=0;
   uint32_t startTs=0;
   uint8_t powerStatus=0;
    void * par;
    int ret = mpu_init(par);
    if(ret !=0) //MPU error
    {
        HAL_GPIO_WritePin(LED_Y_GPIO_Port,LED_Y_Pin,GPIO_PIN_RESET); // yellow pin on
    }
  /* Infinite loop */
  for(;;)
  {
    sndCANMsg();
    osDelay(100);
    rxCnt = HAL_CAN_GetRxFifoFillLevel(&hcan1,CAN_RX_FIFO0);
    HAL_ADC_Start(&hadc1);
    if(HAL_OK == HAL_ADC_PollForConversion(&hadc1,10))
    {
        volt12Value = HAL_ADC_GetValue(&hadc1);
        volt12Value = volt12Value*3300/4096*13/3;
        
    }
    
    powerStatus = readPowerSupplyStatus();
    if(powerStatus == 1)
    {
        startTs = osKernelSysTick();
        lpmStartFlag=0;
    }  
    else if((powerStatus== 0)&&(lpmStartFlag==0))
    {
        startTs = osKernelSysTick();
        lpmStartFlag =1;
    }
    
    if((osKernelSysTick() - startTs) >200000)  //delay 20 seconds
    {
        //MCU enter stop mode 
          if ( 0 != mpu_lp_motion_interrupt(1000, 1, 40))
          {
              printf("motion detect config err");
          }
          else
          {
              HAL_GPIO_WritePin(GPIOA,EC600_EN_Pin, GPIO_PIN_SET);
              HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
              HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);		//PB0---MEMS INT
              PWR->CR|=1<<2;         
              PWR->CR|=1<<3;          
              HAL_SuspendTick();
              HAL_NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);
              HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
              HAL_NVIC_DisableIRQ(TIM1_TRG_COM_TIM11_IRQn);
              HAL_NVIC_ClearPendingIRQ(TIM1_BRK_TIM9_IRQn);
              HAL_NVIC_ClearPendingIRQ(TIM1_UP_TIM10_IRQn);
              HAL_NVIC_ClearPendingIRQ(TIM1_TRG_COM_TIM11_IRQn);
              HAL_PWR_EnterSTANDBYMode();
          }
      }
         
  }
  /* USER CODE END powerControlFunc */
}

/* USER CODE BEGIN Header_readBatteryInfo */

/**
* @brief Function implementing the batterInfoSync thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_readBatteryInfo */
void readBatteryInfo(void const * argument)
{
  /* USER CODE BEGIN readBatteryInfo */
  /* Infinite loop */
  for(;;)
  {
    
    readBmsRegs();
    osDelay(2000);
    
    readBmsID();
    osDelay(2000);
  }
  /* USER CODE END readBatteryInfo */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

