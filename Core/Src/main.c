/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "ste2007.h"
#include "LCD.h"
#include "stm32_ds3231.h"
#include "version.h" 
#include "ftoa.h"
#include "itoa.h"
#include "MAX17055.h"
#include "bmp280.h"
#include "Adafruit_AMG88xx.h"
#include "bilinear.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern tFont Font12,Font18,Font24;
char TxBuffer [20];
char TxBuffer2 [10];
uint8_t BACK_COLOR=0x00, MENU_COLOR=0x01;
__IO uint8_t it_num=0;
uint8_t pressure_buf[96];
extern uint8_t v_buffer[2*14];
__IO uint16_t max_status=0,max_status2=0,max_fstat=0;
 extern __IO uint16_t key2_t,key2_int;  
 
BMP280_HandleTypedef bmp280;
float pressure, temperature, humidity;

// amg8833
int16_t pixelsRaw[64];


#define RESIZE_X 32
#define RESIZE_Y 32


float temp_amg=0.0f;
int16_t amg_porog=30;

int16_t pixelsRawResize[RESIZE_X*RESIZE_Y];

void show_time_screen (uint8_t sec);
void show_bat_screen (void);
void show_teplo_screen (void);
void show_tacho_screen (void);
uint8_t menu_key(void);
void set_charge_current(uint8_t curr);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

void show_time_setup (void);
void show_time (uint8_t pos);

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

_RTC rtc = {
    .Year = 0, .Month = 0, .Date = 0,
    .DaysOfWeek = WEDNESDAY,
    .Hour = 0, .Min = 0, .Sec = 0
};

_RTC rtc_sys = {0};
_RTC rtc_sec = {0};
_RTC rtc_sec_end = {0};

float rtcTemp;
float tf = 0.0f, pf = 0.0f, af = 0.0f, hf = 0.0f;

__IO uint32_t key1, key1_short, key1_long;
__IO uint32_t key2, key2_short, key2_long;
__IO uint32_t key3, key3_short, key3_long;
__IO uint32_t key4, key4_short, key4_long;

extern uint16_t id;

float batt_soc=0.0f,batt_current=0.0f,batt_voltage=0.0f,batt_empty_voltage=0.0f,bat_cycles=0.0f,batt_full_cap=0.0f,bat_res=0.0f;
__IO float design_cup=0.0f,full_cup=0.0f,rem_cap=0.0f,batt_time_empty=0.0f,batt_time_full=0.0f,max_temp=0.0f;
uint16_t adc_batt_voltage=0;
float age=0.0f;

uint8_t mode=1;         // 0 - режим часы, энергосбережение, 1 - режим информации о батарее, 2 - тепловизор, 3 - установка часов
  float pressure=0.0f;
  uint8_t p_count=0;
  uint16_t max_param_check=0;
  
  void parse_time (_RTC *rt);
  
__IO uint32_t key1, key1_short, key1_long;
__IO uint32_t key2, key2_short, key2_long;
__IO uint32_t key3, key3_short, key3_long;
__IO uint32_t key4, key4_short, key4_long;

  uint16_t min_thr=30;
  uint16_t m_count=0;
  uint8_t secundomer=0,charge_current=2;
  
  /* Captured Values */
uint32_t               uwIC2Value1 = 0;
uint32_t               uwIC2Value2 = 0;
uint32_t               uwDiffCapture = 0;

/* Capture index */
uint16_t               uhCaptureIndex = 0;

/* Frequency Value */
float               uwFrequency = 0.0f;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
    __HAL_RCC_PWR_CLK_ENABLE();
    
  ste2007_init();
  clear_screen();
  
  //HAL_TIM_OC_Start(&htim1,TIM_CHANNEL_1);
  //HAL_Delay(100);
  //HAL_TIM_OC_Stop(&htim1,TIM_CHANNEL_1);
  

  parse_time(&rtc_sys);            // 18.06.2025
  
  DS3231_Init(&hi2c1);
  
  DS3231_GetTime(&rtc);
    
  if(rtc.Year<25){
    DS3231_SetTime(&rtc_sys);           // 18.06.2025
  }
  
  amg88xxInit();
  
  amg88xx_sleep();
  
  max_param_check=check_variables();

  max17055_check();
  max17055Init_CM4(max_param_check,0);
  
  if(!max_param_check){
    SaveCalibrationVariables();
  }
  
  bmp280_init_default_params(&bmp280.params);
  bmp280.addr = BMP280_I2C_ADDRESS_0;
  bmp280.i2c = &hi2c1;
  while (!bmp280_init(&bmp280, &bmp280.params)) {
		draw_string("BMP FAIL",1,ADDR_LINE_LCD(0),&Font18,MENU_COLOR);
                HAL_Delay(1000);
                draw_string("BMP FAIL",1,ADDR_LINE_LCD(0),&Font18,BACK_COLOR);
                HAL_Delay(1000);
	}
  

  
  set_charge_current(charge_current);
  
  
  uint8_t regVal=0,led_on=0;
  
  if(mode==2){
        amg88xx_wake_up();
      }
  
  HAL_TIM_IC_MspDeInit(&htim2);
  HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TACHO_POWER_GPIO_Port,TACHO_POWER_Pin,GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
// опрос статуса rtc. общий цикл   
   
    ReadRegister(DS3231_REG_STATUS, &regVal);
      
      if(regVal & DS3231_STA_A1F)
      {
        regVal &= ~DS3231_STA_A1F;
        WriteRegister(DS3231_REG_STATUS, regVal);
      }
      if(regVal & DS3231_STA_A2F)
      {
        regVal &= ~DS3231_STA_A2F;
        WriteRegister(DS3231_REG_STATUS, regVal);
      }
     
// давление. обший цикл    
    bmp280_force_measurement(&bmp280);  // перед каждым измерением
    
    bmp280_read_float(&bmp280, &temperature, &pressure, &humidity);
    
    pressure=convert_pa_to_mm(pressure);
 
     if(m_count<min_thr){ // 30
       if(mode==0 || mode==1){
          m_count++;
       }
      }
      else{
        m_count=0;
        
        if((int32_t)pressure<789 && (int32_t)pressure>739){
          pressure_buf[p_count]=(uint8_t)((pressure-740.0f)/(48.0f/20.0f));  // давление от 740 до 788. 20 пикселей
        }
      if(p_count<95){
        p_count++;
      }
      else{
        p_count=0;
      }
    }
    
// опрос макс17055. разделить по необходимости
    batt_current=getInstantaneousCurrent();
    batt_voltage=getInstantaneousVoltage();
    bat_cycles=getCycles();
    batt_soc=getSOC();
    bat_res=getBat_Res_mOhm();
    //design_cup=getCapacity();
    full_cup=getFullCapRep();
    //rem_cap=getRemainingCapacity();
    //max_temp=get_temperature();
    //batt_time_empty=getTimeToEmpty();
    //batt_time_full=getTimeToFull();
    max_fstat=max17055_get_fstat();
    max_status2=max17055_get_status2();
    
// запрос времени с rtc. общий цикл    
    DS3231_GetTime(&rtc); 
    
    if(mode==3){        // выход из режима установки времени
      mode=4;
    }
    
    if(it_num==1){   // верх слева. режим
      
      clear_screen();
      if(mode==2){
        amg88xx_sleep();
      }
      if(mode>3){
        HAL_GPIO_WritePin(TACHO_POWER_GPIO_Port,TACHO_POWER_Pin,GPIO_PIN_RESET);
        HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
        mode=0;
      }
      else{
        mode++;
      }
      led_on=1;
      if(mode==2){
        amg88xx_wake_up();
      }
      
    }
    if(it_num==3 && mode==1){
      if(charge_current<3){
        charge_current++;
        set_charge_current(charge_current);
      }
    }
    if(it_num==2 && mode==1){
      if(charge_current){
          charge_current--;  
          set_charge_current(charge_current);
      }
    }
    if(it_num==3 && mode==2){
      amg_porog++;
    }
    if(it_num==2 && mode==2){
      amg_porog--;
    }
    if(it_num==2 && mode==0){   // time screen
      if(secundomer<2){
        secundomer++;
      }
      else{
        secundomer=0;
      }
    }
    
    it_num=0;
    
// отображение
  
    if(mode==0){
      if(secundomer==0){
        rtc_sec=rtc;
      }
      else if(secundomer==1){
        rtc_sec_end=rtc;
      }
      else{
        
      }
      show_time_screen(secundomer);     // 05.06.2025
    }
    else if(mode==1){
      show_bat_screen();
    }
    else if(mode==2){
      show_teplo_screen();
    }
    else if(mode==3){
      show_time_setup();
    }
    else if(mode==4){
      HAL_GPIO_WritePin(TACHO_POWER_GPIO_Port,TACHO_POWER_Pin,GPIO_PIN_SET);
      HAL_TIM_IC_MspInit(&htim2);
      HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
      show_tacho_screen();
      HAL_Delay(1000);
      HAL_TIM_IC_MspDeInit(&htim2);
    }
    if(led_on){
      led_on=0;
      HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
      HAL_Delay(1000);
      HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
      SaveCalibrationVariables();
    }
    
    //HAL_Delay(100);
    
   if(mode==0 || mode==1){
    //HAL_Delay(3000);
    
    DS3231_SetAlarm_one_min();
    
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

        // stop here
        SystemClock_Config(); 
        DS3231_ClearAlarm1();
        DS3231_ClearAlarm2();
        
   }
    
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0010020A;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_9BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|CHARGER_42_Pin|CHARGER_100_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TACHO_POWER_GPIO_Port, TACHO_POWER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PF0 PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_0_Pin BUTTON_1_Pin BUTTON_2_Pin */
  GPIO_InitStruct.Pin = BUTTON_0_Pin|BUTTON_1_Pin|BUTTON_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA6 PA8 PA9
                           PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB5
                           PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CHARGER_42_Pin CHARGER_100_Pin */
  GPIO_InitStruct.Pin = CHARGER_42_Pin|CHARGER_100_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TACHO_POWER_Pin */
  GPIO_InitStruct.Pin = TACHO_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TACHO_POWER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RTC_IRQ_Pin */
  GPIO_InitStruct.Pin = RTC_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RTC_IRQ_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void parse_time (_RTC *rt){
  char temp[2];
  
  temp[0]=version_full[0];
  temp[1]=version_full[1];
  rt->Date=atoi(temp);
  
  temp[0]=version_full[3];
  temp[1]=version_full[4];
  rt->Month=atoi(temp);
 
  temp[0]=version_full[8];
  temp[1]=version_full[9];
  rt->Year=atoi(temp);
  
  temp[0]='0';
  temp[1]=version_full[11];
  rt->DaysOfWeek=atoi(temp); 
  
  temp[0]=version_logo[0];
  temp[1]=version_logo[1];
  rt->Hour=atoi(temp);
  
  temp[0]=version_logo[3];
  temp[1]=version_logo[4];
  rt->Min=atoi(temp);
  
  temp[0]=version_logo[6];
  temp[1]=version_logo[7];
  rt->Sec=atoi(temp);
  
}


// нажатие кнопок:
// 1 - 1 верх слева
// 2 - 1 длинное
//3 - 2 низ слева
//4 - 2 длинное,
//5 - 3 низ справа
//6 - 3 длинное
//7 - 4 верх справа
//8 - 4 длинное
uint8_t menu_key(void){
  

        if(key1 || key2 || key3 || key4){
          
        if(key1){
          key1 = 0;
                if(key1_short){     // 1 короткое
                  key1_long = 0;
                  key1_short = 0;
                  return 1;
                  }
                if(key1_long){      // 1 длинное
                  key1_long = 0;
                  key1_short = 0;
                  return 2;
                  }
        }
        if(key2){
          key2 = 0;
                if(key2_short){     // 2 короткое
                  key2_short = 0;
                  key2_long = 0;
                  return 3;
                  }
                if(key2_long){      // 2 длинное
                  key2_long = 0;
                  key2_short = 0;
                  return 4;
                  }
        }
        if(key3){
          key3 = 0;
                if(key3_short){     // 3 короткое
                  key3_long = 0;
                  key3_short = 0;
                  return 5;
                  }
                if(key3_long){      // 3 длинное
                  key3_long = 0;
                  key3_short = 0;
                  return 6;
                  }
        }
        /*
        if(key4){
            key4 = 0;
                if(key4_short){     // 1 короткое
                  key4_long = 0;
                  key4_short = 0;
                  return 7;
                  }
                if(key4_long){      // 1 длинное
                  key4_short = 0;
                  key4_long = 0;
                  return 8;
                  }
        }
        */
      }
      return 0;
}

 uint32_t sec_count=0,min_count=0,hour_count=0,second_count=0;
void show_time_screen (uint8_t sec){
      sec_count=0;
  
      draw_string("0000000000000",1,ADDR_LINE_LCD(0),&Font18,BACK_COLOR);
    
      itoa(rtc.Hour,TxBuffer2,10);
      if(rtc.Hour<10){
        strcat(TxBuffer,"0");
      }
      strcat(TxBuffer,TxBuffer2);
      strcat(TxBuffer,":");
      if(rtc.Min<10){
        strcat(TxBuffer,"0");
      }
      itoa(rtc.Min,TxBuffer2,10);
      strcat(TxBuffer,TxBuffer2);
      
      //
      strcat(TxBuffer," ");
      
      ftoa(pressure,TxBuffer2,1);//"%.2f mmHg"
      strcat(TxBuffer,TxBuffer2);
      
      strcat(TxBuffer," ");
      if(rtc.DaysOfWeek>1){
        itoa(rtc.DaysOfWeek-1,TxBuffer2,10);
      }
      else{     // sunday
        itoa(7,TxBuffer2,10);
      }
      strcat(TxBuffer,TxBuffer2);

      draw_string(TxBuffer,1,ADDR_LINE_LCD(0),&Font18,MENU_COLOR);
      
      strcpy(TxBuffer,"");
    
    

    
    strcpy(TxBuffer,"");
    
    draw_string("00 00 00 0.000",1,ADDR_LINE_LCD(1),&Font18,BACK_COLOR);
    
    if(sec==0){                         // дата
      itoa(rtc.Date,TxBuffer2,10);
      if(rtc.Date<10){
        strcat(TxBuffer,"0");
      }
      strcat(TxBuffer,TxBuffer2);
      strcat(TxBuffer,".");
      itoa(rtc.Month,TxBuffer2,10);
      if(rtc.Month<10){
        strcat(TxBuffer,"0");
      }
      strcat(TxBuffer,TxBuffer2);
      strcat(TxBuffer,".");
      itoa(rtc.Year,TxBuffer2,10);
      if(rtc.Year<10){
        strcat(TxBuffer,"0");
      }
      strcat(TxBuffer,TxBuffer2);
      
    }
    else{                               // секундомер   1 - врем€ от текущего 2 - врем€ от окнечной метки времени
      
      if(sec==1){                               // секундомер запущен и отображаетс€
        
      if(rtc.Date>rtc_sec.Date){
        sec_count+=(rtc.Date-rtc_sec.Date)*3600*24;
      }
        
      if(rtc.Hour>rtc_sec.Hour){
        sec_count+=(rtc.Hour-rtc_sec.Hour)*3600;
      }
      else{
        sec_count+=(rtc.Hour+24-rtc_sec.Hour)*3600;
        sec_count-=3600*24;
      }
      
      if(rtc.Min>rtc_sec.Min){
        sec_count+=(rtc.Min-rtc_sec.Min)*60;
      }
      else{
        sec_count+=(rtc.Min+60-rtc_sec.Min)*60;
        sec_count-=3600;
      }
      
      if(rtc.Sec>rtc_sec.Sec){
        sec_count+=rtc.Sec-rtc_sec.Sec;
      }
      else{
        sec_count+=rtc.Sec+60-rtc_sec.Sec;
        sec_count-=60;
      }
      } 
      else if(sec==2){                          // остановка секундомера, отображение
        
      if(rtc_sec_end.Date>rtc_sec.Date){
        sec_count+=(rtc_sec_end.Date-rtc_sec.Date)*3600*24;
      }
      
      if(rtc_sec_end.Hour>rtc_sec.Hour){
        sec_count+=(rtc_sec_end.Hour-rtc_sec.Hour)*3600;
      }
      else{
        sec_count+=(rtc_sec_end.Hour+24-rtc_sec.Hour)*3600;
        sec_count-=3600*24;
      }
      
      if(rtc_sec_end.Min>rtc_sec.Min){
        sec_count+=(rtc_sec_end.Min-rtc_sec.Min)*60;
      }
      else{
        sec_count+=(rtc_sec_end.Min+60-rtc_sec.Min)*60;
        sec_count-=3600;
      }
      
      if(rtc_sec_end.Sec>rtc_sec.Sec){
        sec_count+=rtc_sec_end.Sec-rtc_sec.Sec;
      }
      else{
        sec_count+=rtc_sec_end.Sec+60-rtc_sec.Sec;
        sec_count-=60;
      }
      } 
      
      hour_count=sec_count/3600;
        
      itoa(hour_count,TxBuffer2,10);
      if(hour_count<10){
        strcat(TxBuffer,"0");
      }
      strcat(TxBuffer,TxBuffer2);
      
      
      strcat(TxBuffer,":");
      
      min_count=(sec_count%3600)/60;
      
      itoa(min_count,TxBuffer2,10);
      if(min_count<10){
        strcat(TxBuffer,"0");
      }
      strcat(TxBuffer,TxBuffer2);
      
      strcat(TxBuffer,":");
      
      second_count=sec_count%60;
      
      itoa(second_count,TxBuffer2,10);
      if(second_count<10){
        strcat(TxBuffer,"0");
      }
      strcat(TxBuffer,TxBuffer2);
    }
      
      strcat(TxBuffer," ");
      
      ftoa(bat_cycles,TxBuffer2,1);
      strcat(TxBuffer,TxBuffer2);
      strcat(TxBuffer,"c");
      
      
            
      draw_string(TxBuffer,1,ADDR_LINE_LCD(1),&Font18,MENU_COLOR);
      
      strcpy(TxBuffer,"");
    
    draw_string("0000000000000",1,ADDR_LINE_LCD(2),&Font18,BACK_COLOR);
    
    itoa((uint16_t)batt_soc,TxBuffer2,10);
    strcat(TxBuffer,TxBuffer2);
    strcat(TxBuffer,"%");
    
    strcat(TxBuffer," ");
    ftoa(batt_current,TxBuffer2,1);
    strcat(TxBuffer,TxBuffer2);
    strcat(TxBuffer,"mA");
    
    draw_string(TxBuffer,1,ADDR_LINE_LCD(2),&Font18,MENU_COLOR);
    strcpy(TxBuffer,"");
    
    strcpy(TxBuffer,"");
    
          // график давлени€
      
      for(uint8_t i=0;i<96;i++){
        v_buffer[0]=0x00;
        v_buffer[1]=0x00;
        v_buffer[2]=0x00;
        set_pixel(0,19-pressure_buf[i],MENU_COLOR,1);//
        send_3_buff(i,ADDR_LINE_LCD(3));
      }
}

void show_bat_screen (void){
  draw_string("0000000000000",1,ADDR_LINE_LCD(0),&Font18,BACK_COLOR);
    
      itoa(rtc.Hour,TxBuffer2,10);
      if(rtc.Hour<10){
        strcat(TxBuffer,"0");
      }
      strcat(TxBuffer,TxBuffer2);
      strcat(TxBuffer,":");
      if(rtc.Min<10){
        strcat(TxBuffer,"0");
      }
      itoa(rtc.Min,TxBuffer2,10);
      strcat(TxBuffer,TxBuffer2);
      
      //
      strcat(TxBuffer," ");
      
      //ftoa(pressure,TxBuffer2,1);//"%.2f mmHg"
      ftoa(bat_res,TxBuffer2,0);
      strcat(TxBuffer,TxBuffer2);
      strcat(TxBuffer,"m");
      
      strcat(TxBuffer," ");
      if(rtc.DaysOfWeek>1){
        itoa(rtc.DaysOfWeek-1,TxBuffer2,10);
      }
      else{     // sunday
        itoa(7,TxBuffer2,10);
      }
      strcat(TxBuffer,TxBuffer2);

      draw_string(TxBuffer,1,ADDR_LINE_LCD(0),&Font18,MENU_COLOR);
      
      strcpy(TxBuffer,"");
    
    

    
    strcpy(TxBuffer,"");
    
    draw_string("00 00 00",1,ADDR_LINE_LCD(1),&Font18,BACK_COLOR);
      
      //ftoa(bat_res,TxBuffer2,1);
      //strcat(TxBuffer,TxBuffer2);
      //strcat(TxBuffer,"m");
    
      itoa((int32_t)full_cup,TxBuffer2,10);
      strcat(TxBuffer,TxBuffer2);
      strcat(TxBuffer,"mh");
      
      strcat(TxBuffer," ");
      
      itoa(charge_current,TxBuffer2,10);
      strcat(TxBuffer,TxBuffer2);
      
      if(max_fstat & FSTAT_RELDT_BIT){
      strcat(TxBuffer,"R");
      }
      if(max_fstat & FSTAT_RELDT2_BIT){
        strcat(TxBuffer,"R");
      }
      if(max_fstat & FSTAT_FQ_BIT){
        strcat(TxBuffer,"Q");
      }
      if(max_fstat & FSTAT_EDET_BIT){
        strcat(TxBuffer,"E");
      }
      if(max_status2 & STATUS2_Hib_BIT){
        strcat(TxBuffer,"H");
      }
      if(max_status2 & STATUS2_FullDet_BIT){
        strcat(TxBuffer,"F");
      }
            
      draw_string(TxBuffer,1,ADDR_LINE_LCD(1),&Font18,MENU_COLOR);
      
      strcpy(TxBuffer,"");
    
    draw_string("0000000000000",1,ADDR_LINE_LCD(2),&Font18,BACK_COLOR);
    itoa((uint16_t)batt_voltage,TxBuffer,10);
    strcat(TxBuffer,"mV");
    
    strcat(TxBuffer," ");
    ftoa(bat_cycles,TxBuffer2,1);
    strcat(TxBuffer,TxBuffer2);
    strcat(TxBuffer,"c");
    draw_string(TxBuffer,1,ADDR_LINE_LCD(2),&Font18,MENU_COLOR);
    strcpy(TxBuffer,"");
    
    draw_string("0000000000000",1,ADDR_LINE_LCD(3),&Font18,BACK_COLOR);
    itoa((uint16_t)batt_soc,TxBuffer2,10);
    strcat(TxBuffer,TxBuffer2);
    strcat(TxBuffer,"%");
    strcat(TxBuffer," ");
    ftoa(batt_current,TxBuffer2,2);
    strcat(TxBuffer,TxBuffer2);
    strcat(TxBuffer,"mA");
    
    
    
    draw_string(TxBuffer,1,ADDR_LINE_LCD(3),&Font18,MENU_COLOR);
    strcpy(TxBuffer2,"");
    strcpy(TxBuffer,"");
}

void show_teplo_screen (void){
    static uint16_t draw_count=0;
    static int16_t old_amg_porog=0;
    //temp_amg=readThermistor();
    readPixelsRaw(pixelsRaw);

    for(uint8_t i=0; i<64; i++){   
      // знаковое число из 12 в 16 бит
      pixelsRaw[i]=pixelsRaw[i]<<4;
      pixelsRaw[i]=pixelsRaw[i]/16;//>>4
      // из формата дополнени€ до двух в знаковое
      if(pixelsRaw[i]<0){
        pixelsRaw[i]=0-((~pixelsRaw[i])+1);     // инверси€ плюс 1
      }
      // из знакового в плавующую точку
      //pixels[i]=(float)(pixelsRaw[i])/4.0f;     // 1 lsb = 0.25 C
    }
    
    resizeBilinearGrey_draw(pixelsRaw,8, 8, RESIZE_X, RESIZE_Y);
    
    if(draw_count>24 || old_amg_porog!=amg_porog){      // 05.06.2025

      draw_count=0;
      old_amg_porog=amg_porog;
      itoa(amg_porog,TxBuffer,10);
      draw_string(TxBuffer,65,ADDR_LINE_LCD(0),&Font18,MENU_COLOR);
    
      ftoa(batt_current,TxBuffer,1);
      draw_string(TxBuffer,65,ADDR_LINE_LCD(1),&Font18,MENU_COLOR);
    
      itoa((uint16_t)batt_voltage,TxBuffer,10);
      draw_string(TxBuffer,65,ADDR_LINE_LCD(2),&Font18,MENU_COLOR);
     }
    else{ 
      draw_count++;
    }
    /*
    resizeBilinearGrey(pixelsRaw,pixelsOut,8, 8, RESIZE_X, RESIZE_Y);
    
    uint8_t data=0;
    uint16_t i=0,a=0,b=0;
  for(b=0;b<RESIZE_X;b++){      // столбцы от 0 до 64
   for(a=0;a<RESIZE_Y/8;a++){   // байты экрана
      lcd_set_row(a);
      lcd_set_col(b);
    for(i=0;i<8;i++){           // обработка одного бита
      if(pixelsOut[i+a*8+b*RESIZE_X]<amg_porog*4){
        data|=0x01<<i;
      }
    }
      LCD_data(data);
      data=0;
   }
  }
  */
}

void show_time_setup (void){
  uint8_t key=0;
  uint8_t temp=0;
  draw_string("SETUP TIME",1,ADDR_LINE_LCD(0),&Font18,MENU_COLOR);
  draw_string("CONT UP",1,ADDR_LINE_LCD(1),&Font18,MENU_COLOR);
  draw_string("EXIT DOWN",1,ADDR_LINE_LCD(2),&Font18,MENU_COLOR);
  
  
  key2_short = 0;
  key2_t = 0;
  key2 = 0;
  key2_int = 0;
  
  do{
    key=it_num;
    HAL_Delay(300);
  }
  while(key!=BUT_UP && key!=BUT_DOWN && key!=BUT_OK);

  if(key==BUT_OK || key==BUT_DOWN){
    it_num=0;
    clear_screen();
    return;
  }
  else if(key==BUT_UP){
    temp=rtc.Date;
    it_num=0;
        do{                         // установка дн€
      key=it_num;
      if(key==BUT_DOWN){        // вниз
        key=0;
        it_num=0;
        if(temp){
          temp--;
        }
      }
      if(key==BUT_UP){       // вверх
        key=0;
        it_num=0;
        temp++;
      }
      if(temp<32 && temp>0){
        rtc.Date=temp;
      }
      show_time(0);
    }
    
    while(key!=BUT_OK);      // OK
    temp=rtc.Month;
    it_num=0;
    do{                         // установка мес€ца
      key=it_num;
      if(key==BUT_DOWN){        // вниз
        key=0;
        it_num=0;
        if(temp){
          temp--;
        }
      }
      if(key==BUT_UP){       // вверх
        key=0;
        it_num=0;
        temp++;
      }
      if(temp<13 && temp>0){
        rtc.Month=temp;
      }
      show_time(1);
    }
    while(key!=BUT_OK);      // OK
    temp=25;
    rtc.Year=temp;
    it_num=0;
        do{                         // установка года
      key=it_num;
      if(key==BUT_DOWN){        // вниз
        key=0;
        it_num=0;
        if(temp>25){
          temp--;
        }
      }
      if(key==BUT_UP){       // вверх
        key=0;
        it_num=0;
        temp++;
      }
      if(temp<99 && temp>24){
        rtc.Year=temp;
      }
      show_time(2);
    }
    while(key!=BUT_OK);      // OK
    temp=rtc.DaysOfWeek;
    it_num=0;
    do{                         // установка дн€ недели
      key=it_num;
      if(key==BUT_DOWN){        // вниз
        key=0;
        it_num=0;
        if(temp){
          temp--;
        }
      }
      if(key==BUT_UP){       // вверх
        key=0;
        it_num=0;
        temp++;
      }
      if(temp<8 && temp>0){
        rtc.DaysOfWeek=temp;
      }
      show_time(3);
    }
    while(key!=BUT_OK);      // OK
    temp=rtc.Hour;
    it_num=0;
    do{                         // установка часов
      key=it_num;
      if(key==BUT_DOWN){        // вниз
        key=0;
        it_num=0;
        if(temp){
          temp--;
        }
      }
      if(key==BUT_UP){       // вверх
        key=0;
        it_num=0;
        temp++;
      }
      if(temp<24){
        rtc.Hour=temp;
      }
      show_time(4);
    }
    while(key!=BUT_OK);      // OK
      temp=rtc.Min;
      it_num=0;
        do{                         // установка минут
      key=it_num;
      if(key==BUT_DOWN){        // вниз
        key=0;
        it_num=0;
        if(temp){
          temp--;
        }
      }
      if(key==BUT_UP){       // вверх
        key=0;
        it_num=0;
        temp++;
      }
      if(temp<60){
        rtc.Min=temp;
      }
      show_time(5);
    }
    while(key!=BUT_OK);      // OK
    temp=rtc.Sec;
    it_num=0;
    do{                         // установка секунд
      key=it_num;
      if(key==BUT_DOWN){        // вниз
        key=0;
        it_num=0;
        if(temp){
          temp--;
        }
      }
      if(key==BUT_UP){       // вверх
        key=0;
        it_num=0;
        temp++;
      }
      if(temp<60){
        rtc.Sec=temp;
      }
      show_time(6);
    }
    while(key!=BUT_OK);      // OK
    DS3231_SetTime(&rtc);
  }
  it_num=0;
  clear_screen();
  strcpy(TxBuffer2,"");
  strcpy(TxBuffer,"");
}

void show_time (uint8_t pos){
  
  strcpy(TxBuffer,"");
  strcpy(TxBuffer2,"");
  
   // 1-€ строка
      draw_string("0000000000000",1,ADDR_LINE_LCD(0),&Font18,BACK_COLOR);
      
      //
      itoa(rtc.Hour,TxBuffer2,10);
      if(rtc.Hour<10){
        strcat(TxBuffer,"0");
      }
      strcat(TxBuffer,TxBuffer2);
      
      strcat(TxBuffer,":");
      if(rtc.Min<10){
        strcat(TxBuffer,"0");
      }
      itoa(rtc.Min,TxBuffer2,10);
      strcat(TxBuffer,TxBuffer2);
      
      strcat(TxBuffer,":");
      
      if(rtc.Sec<10){
        strcat(TxBuffer,"0");
      }
      itoa(rtc.Sec,TxBuffer2,10);
      strcat(TxBuffer,TxBuffer2);
      
      if(pos==4){
        draw_string_inv(TxBuffer,1,ADDR_LINE_LCD(0),&Font18,MENU_COLOR,0);
      }
      else if(pos==5){
        draw_string_inv(TxBuffer,1,ADDR_LINE_LCD(0),&Font18,MENU_COLOR,3);
      }
      else if(pos==6){
        draw_string_inv(TxBuffer,1,ADDR_LINE_LCD(0),&Font18,MENU_COLOR,6);
      }
      else{
        draw_string(TxBuffer,1,ADDR_LINE_LCD(0),&Font18,MENU_COLOR);
      }
      
      strcpy(TxBuffer,"");
      
      
      
      // 2-€ строка
     draw_string("0000000000000",1,ADDR_LINE_LCD(1),&Font18,BACK_COLOR);
      
      itoa(rtc.Date,TxBuffer2,10);
      if(rtc.Date<10){
        strcat(TxBuffer,"0");
      }
      strcat(TxBuffer,TxBuffer2);
      strcat(TxBuffer,".");
      itoa(rtc.Month,TxBuffer2,10);
      if(rtc.Month<10){
        strcat(TxBuffer,"0");
      }
      strcat(TxBuffer,TxBuffer2);
      strcat(TxBuffer,".");
      itoa(rtc.Year,TxBuffer2,10);
      if(rtc.Year<10){
        strcat(TxBuffer,"0");
      }
      strcat(TxBuffer,TxBuffer2);
      
      if(pos==0){
        draw_string_inv(TxBuffer,1,ADDR_LINE_LCD(1),&Font18,MENU_COLOR,0);
      }
      else if(pos==1){
        draw_string_inv(TxBuffer,1,ADDR_LINE_LCD(1),&Font18,MENU_COLOR,3);
      }
      else if(pos==2){
        draw_string_inv(TxBuffer,1,ADDR_LINE_LCD(1),&Font18,MENU_COLOR,6);
      }
      else{
        draw_string(TxBuffer,1,ADDR_LINE_LCD(1),&Font18,MENU_COLOR);
      }
      // 3-€ строка
      strcpy(TxBuffer,"");
      
       draw_string("0000000000000",1,ADDR_LINE_LCD(2),&Font18,BACK_COLOR);
       itoa(rtc.DaysOfWeek,TxBuffer,10);
       if(pos==3){
        draw_string_inv(TxBuffer,1,ADDR_LINE_LCD(2),&Font18,MENU_COLOR,0);
       }
       else{
         draw_string(TxBuffer,1,ADDR_LINE_LCD(2),&Font18,MENU_COLOR);
       }
}

void show_tacho_screen (void){
  
  uint32_t rpm=0;
  uint16_t i=0;
  uint8_t key=0;
  
  draw_string("0000000000000",1,ADDR_LINE_LCD(0),&Font18,BACK_COLOR);
  draw_string("Tacho mode",1,ADDR_LINE_LCD(0),&Font18,MENU_COLOR);
  
  do{
    key=it_num;
    
    if(i<100){
      i++;
      HAL_Delay(10);
    }
    else{
      i=0;
      batt_current=getInstantaneousCurrent();
      batt_soc=getSOC();
      
      strcpy(TxBuffer,"");
    draw_string("0000000000000",1,ADDR_LINE_LCD(3),&Font18,BACK_COLOR);
    itoa((uint16_t)batt_soc,TxBuffer2,10);
    strcat(TxBuffer,TxBuffer2);
    strcat(TxBuffer,"%");
    strcat(TxBuffer," ");
    ftoa(batt_current,TxBuffer2,1);
    strcat(TxBuffer,TxBuffer2);
    strcat(TxBuffer,"mA");
    draw_string(TxBuffer,1,ADDR_LINE_LCD(3),&Font18,MENU_COLOR);
    }

  
    if(rpm!=(uint32_t)(uwFrequency*60.0f)){
  
      draw_string("0000000000000",1,ADDR_LINE_LCD(1),&Font18,BACK_COLOR);
      ftoa(uwFrequency*60.0f,TxBuffer,1);
      draw_string(TxBuffer,1,ADDR_LINE_LCD(1),&Font18,MENU_COLOR);
      rpm=(uint32_t)(uwFrequency*60.0f);
    }
  }
  while(key!=BUT_OK);
  
    strcpy(TxBuffer2,"");
    strcpy(TxBuffer,"");
    uwFrequency=0;
}

void set_charge_current(uint8_t curr){
  if(curr==0){
      HAL_GPIO_WritePin(CHARGER_100_GPIO_Port,CHARGER_100_Pin,GPIO_PIN_SET);      // 44.9мј
      HAL_GPIO_WritePin(CHARGER_42_GPIO_Port,CHARGER_42_Pin,GPIO_PIN_SET);          // 107,8мј
  }
  else if(curr==1){
      HAL_GPIO_WritePin(CHARGER_100_GPIO_Port,CHARGER_100_Pin,GPIO_PIN_RESET);      // 44.9мј
      HAL_GPIO_WritePin(CHARGER_42_GPIO_Port,CHARGER_42_Pin,GPIO_PIN_SET);          // 107,8мј
  }
  else if(curr==2){
      HAL_GPIO_WritePin(CHARGER_100_GPIO_Port,CHARGER_100_Pin,GPIO_PIN_SET);      // 44.9мј
      HAL_GPIO_WritePin(CHARGER_42_GPIO_Port,CHARGER_42_Pin,GPIO_PIN_RESET);          // 107,8мј
  }
  else{
      HAL_GPIO_WritePin(CHARGER_100_GPIO_Port,CHARGER_100_Pin,GPIO_PIN_RESET);      // 44.9мј
      HAL_GPIO_WritePin(CHARGER_42_GPIO_Port,CHARGER_42_Pin,GPIO_PIN_RESET);          // 107,8мј
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    if(uhCaptureIndex == 0)
    {
      /* Get the 1st Input Capture value */
      uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      uhCaptureIndex = 1;
    }
    else if(uhCaptureIndex == 1)
    {
      /* Get the 2nd Input Capture value */
      uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); 

      /* Capture computation */
      if (uwIC2Value2 > uwIC2Value1)
      {
        uwDiffCapture = (uwIC2Value2 - uwIC2Value1); 
      }
      else if (uwIC2Value2 < uwIC2Value1)
      {
        /* 0xffffFFFF is max TIM2_CCRx value */
        uwDiffCapture = ((0xffffFFFF - uwIC2Value1) + uwIC2Value2) + 1;
      }
      else
      {
        /* If capture values are equal, we have reached the limit of frequency
           measures */
        Error_Handler();
      }
      /* Frequency computation: for this example TIMx (TIM2) is clocked by
         APB1Clk */      
      uwFrequency = ((float)HAL_RCC_GetPCLK1Freq()) / uwDiffCapture;
      uhCaptureIndex = 0;
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
