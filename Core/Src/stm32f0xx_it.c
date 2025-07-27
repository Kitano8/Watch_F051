/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static __IO uint16_t key1_t=0,key1_int=0;
 __IO uint16_t key2_t=0,key2_int=0;
static __IO uint16_t key3_t=0,key3_int=0;
static __IO uint16_t key4_t=0,key4_int=0;

extern __IO uint32_t key1, key1_short, key1_long;
extern __IO uint32_t key2, key2_short, key2_long;
extern __IO uint32_t key3, key3_short, key3_long;
extern __IO uint32_t key4, key4_short, key4_long;

extern __IO uint8_t it_num;

#define BUTT_LONG 1000
#define BUTT_SHORT 150
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
if (key1_int || key2_int || key3_int) {
                     if (BUTTON_0_GPIO_Port->IDR & BUTTON_0_Pin) { key1_t++;}
                else if (!(BUTTON_1_GPIO_Port->IDR & BUTTON_1_Pin)) { key2_t++;}
                else if (!(BUTTON_2_GPIO_Port->IDR & BUTTON_2_Pin)) { key3_t++;}

                if(key1_t>BUTT_SHORT && key1_t<BUTT_LONG && !(BUTTON_0_GPIO_Port->IDR & BUTTON_0_Pin)){ // 100mS короткое нажатие
                  key1_short = 1;
                  key1_t = 0;
                  key1 = 1;
                  key1_int = 0;
                  
                }
                
                if(key2_t>BUTT_SHORT && key2_t<BUTT_LONG && !(BUTTON_1_GPIO_Port->IDR & BUTTON_1_Pin)){ // 100mS короткое нажатие
                  key2_short = 1;
                  key2_t = 0;
                  key2 = 1;
                  key2_int = 0;
                  
                }
                
                if(key3_t>BUTT_SHORT && key3_t<BUTT_LONG && !(BUTTON_2_GPIO_Port->IDR & BUTTON_2_Pin)){ // 100mS короткое нажатие
                  key3_short = 1;
                  key3_t = 0;
                  key3 = 1;
                  key3_int = 0;
                  
                }
                
   }
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 0 and 1 interrupts.
  */
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */
if(__HAL_GPIO_EXTI_GET_IT(BUTTON_0_Pin) != 0x00u)
  { 
    __HAL_GPIO_EXTI_CLEAR_IT(BUTTON_0_Pin);
    
        key1_int = 1;
        key2_int = 0;
        key3_int = 0;
        key4_int = 0;
        
        it_num=1;
  }
  if(__HAL_GPIO_EXTI_GET_IT(BUTTON_1_Pin) != 0x00u)
  { 
    __HAL_GPIO_EXTI_CLEAR_IT(BUTTON_1_Pin);
    
        key1_int = 0;
        key2_int = 1;
        key3_int = 0;
        key4_int = 0;
        
        it_num=2;
  }
  /* USER CODE END EXTI0_1_IRQn 0 */
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */

  /* USER CODE END EXTI0_1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line 2 and 3 interrupts.
  */
void EXTI2_3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_3_IRQn 0 */
  if(__HAL_GPIO_EXTI_GET_IT(BUTTON_2_Pin) != 0x00u)
  { 
    __HAL_GPIO_EXTI_CLEAR_IT(BUTTON_2_Pin);
    
        key1_int = 0;
        key2_int = 0;
        key3_int = 1;
        key4_int = 0;
        
        it_num=3;
  }
  
  /* USER CODE END EXTI2_3_IRQn 0 */
  /* USER CODE BEGIN EXTI2_3_IRQn 1 */

  /* USER CODE END EXTI2_3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */
if(__HAL_GPIO_EXTI_GET_IT(RTC_IRQ_Pin) != 0x00u)
  { 
    __HAL_GPIO_EXTI_CLEAR_IT(RTC_IRQ_Pin);
  }
  /* USER CODE END EXTI4_15_IRQn 0 */
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
