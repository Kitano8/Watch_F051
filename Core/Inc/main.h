/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/*
    09.04.2025  ������ ������� 051. ������� ��������� � ��������� �� ���������) �� �������� ������� ������ ������( ������� ������ ���������� ������, � ��� � ����� ��� ������� ����.
                ���� ��������. max17055 ��������, id ���������, ��� � ���������� ����
    11.04.2025  �������� ��� ��� bmp280
                ������� ��������, � �������� ���(. �� 48 ��� ��� 18,4 ��, �� 8��� ���� 8,4��. �� ���� ��� ��� amg8833, ��� ��� ��� ���������� 3,8��
                ���-�� ������� ���( ����������� ������ ���� ���� ���������
    13.04.2025  �������� ������� max17055
    15.04.2025  ����� ������������� �������
    16.04.2025  ������ ������ ����� ������� � ���� ���������� � ����������� ���, ���������� ������� ��� ����
    19.04.2025  ������� ���������� � �������� ���������������� ���������� -0,5�� �����������, R�� ������� ���������� ����� 56 ���
                0x0800FC00 ����� �������� �������. ���������� �� ����
    20.04.2025  ��������� �� ���� �.�. led- �� ����� �� �������� �� �������, ������� �� gnd ��������, ��� � ���������� 8��
                ���� �����. ������� ��������� ����������� �����������. ������ � ����������� ����, ���� �������� ����������� � ��������� ��� ������
                �������� ��������(
    21.04.2025  ������� HAL_SuspendTick(); ����������� 0,8��. ���� ��� ��������. ������ swd �����������, ������ ��� � �� ����������� ���� �� ��������� ��������
    22.04.2025	�� ��������, �������� ����. ����� ��������� ��������� ������, ����� ���-�� ���������� ���� ���
    23.04.2025  �������� �������, ��� ��������� �������� ������������ ��� �� usb, ���������� ��� ��������� 3,3��, � ������ ���� � ���������� ������� ����� 1��
                �������� ������� ���� � ���, ����������� 1,6-1,7��. � ����� � PWR_MAINREGULATOR_ON ����������� 1,06��.
                � standbuy ������ ����������� 1,4��. � ������ stop �������� �� ������������. ������� ��������, �� ���� ����� ����, ������ ��� ����� ����, ����� �������, �� ����������
                �����!!! option bytes ����������� nRST_STOP � nRST_STBY. ����������� 0,3�� ��� HAL_SuspendTick();
                https://electronix.ru/forum/topic/164660-stm32l100-posle-proshivki-ne-spit/
                �������� ���������� �� 48*48 �������� ����� �����, ������ ���� ��������� ����������, ����� ������� ��������
                ������ ��������� �� ���� �� 64*64 ������� ��� ������. �������� ������ �� 32*32, �����-����� ������ ����������
                ������� ��������� �������
    27.05.2025  �������� �������� � ����������
    29.05.2025	�������� ����� ������� (����� �������), �����������. 
		������� � ����������� ���������� ����������� ������ ����������� ��� ��� ���������.
		��������� ����������� �� �����, ��� ������ ����������� ���� ����� ����. �����������
    05.06.2025  ������ ��������� ����� ��� ������.
                ��������� ����� ��� ������ �����, ���������� ������. �������� ���� ���������� � ������, ���� ��� �������� ������� ���������
    06.06.2025  �������� ���������� �����. ���������� ������� � �����, � �� ����, �������� �������� ����� ����� ������ �� �������, �.�. �� ������� ����� ������, � ����� ��� �� �������. ������� ���
    09.06.2025  ��� ��������� ������� ��� ���������� ������� ��� ���������� 00 ��� 01, �� �����, � ��� ���������� �������� � 25. ����� �� ��������� � ������� ��������� ��������� ����, ������� ����� 149���
                �������� �������� �� tcrt5000, ���� ��������
    16.06.2025  �������� �������� ����� ���������� lm393. ����������� � ������ ��������� ����� 10��, ��-�� ��-����������.
                ��� ����� ������ ����� ������� ������ ������, ����� � ������ ��� ������� ������ ������ ������ � ������.
                �������� ����� ���������, ��������� ����� �������� ����� ��������.
                ������ 2 ����� ����������� 4294967295/8000000=536 ������
                ������ 1 ����� ����������� 65535/8000000=0.008 ������
                �� ������� �������� ������� ������ ���-��? ����� � �� �����
    18.06.2025  ���� ����� � ����� ��������, �� ��� ������ ��� ����� ����������
                �������� � ������ ��� ����������� ����� ��������� ����
    19.06.2025  ��������� ���������. ���������! ������ ���������� ���������
    15.07.2025  ����������� �������, max �������� ������� � ���, ����������� ��������� ������������ ������� ���������.
                ����� ����������� ������ �������. ���� ���������� ������� � ����� 2 - ������������ ���
                ����������� ���������� ������ � ���������� �����, max �������� ���� ��� ����� �/100. �������� tim1 � pa8, ����� ��� ����������� �����?
    22.07.2025  ����� ����������� ������������� ������� ������ �������� �� ������ �������. ����� �������� � 3� ����� ��������� � ����

��� �������� � ����� �������� ���������� ��������� ��������� ��������:
���������� ��� SLEEPDIP � �������� SCB  +
�������� ��� PDDS � �������� PWR_CR     +
�������� ��� WUF � �������� PWR_CSR     �������
��������� ��������� ���������� LPSDSR � �������� PWR_CR +
��������� ������� ��� ���������� ������ �� ������ STOP  +
������ ������� WFI ��� WFE

�� ���������

WFI (�������� ����������) ��� WFE (�������� �������), ����:
� ���������� ��� SLEEPDEEP � �������� ���������� �������� Cortex�-M0
� �������� ��� PDDS � �������� ���������� �������� (PWR_CR)
� �������� ����� ���������� ����������, �������� ��� LPDS � PWR_CR

����������: ��� ����� � ����� ��������� ��� ���� �������� ����� EXTI (� �������� �������� (EXTI_PR)),
��� ���� �������� ���������� ������������ ��������� � ���� ������� RTC ������ ���� ��������. 
� ��������� ������ ��������� ����� � ����� ��������� ������������, � ���������� ��������� ������������.

���� ���������� ���������� ��������� ������� ��������� (������� ����) ����� ������ � ����� ���������, 
�������� ��������� ����� ������ ���� ������� ���������� �� HSI, � ����� ������ ��� HSEON.
� ��������� ������, ���� ����� ������ � ����� ��������� ��� HSEON ������������ �� ������ 1, 
���������� �������� ������� ������� ������������ (CSS) ��� ����������� ������ ���� �������� ���������� 
(�������� ��������� ����������) � �������������� ���� ��� ����� � ����� ���������.

Saved_RCOMP0 112
Saved_TempCo 8766
Saved_FullCapRep 4000
Saved_Cycles 153
Saved_FullCapNom 4607

MCP73831T-2ACI/OT       "KDXX"
Vreg=4.2V 
Ipreg/Ireg=10%           Preconditioning Current Regulation (Trickle Charge Constant-Current Mode) (7,5-12,5%)
Vpth/Vreg=66.5%          Precondition Voltage Threshold Ratio   (64-69%)
Iterm/Ireg=7.5%          Charge Termination Current Ratio        (5.6-9.4%)
Vrth/Vreg=96.5%          Automatic Recharge. Recharge Voltage Threshold Ratio        (94-99%)


    temp_amg=readThermistor();
    
    readPixelsRaw(pixelsRaw);

    for(uint8_t i=0; i<64; i++){
	//pixelsRaw[i] = (pixelsRaw[i]-80)<<4;    
      // �������� ����� �� 12 � 16 ���
      pixelsRaw[i]=pixelsRaw[i]<<4;
      pixelsRaw[i]=pixelsRaw[i]>>4;
      // �� ������� ���������� �� ���� � ��������
      if(pixelsRaw[i]<0){
        pixelsRaw[i]=0-((~pixelsRaw[i])+1);     // �������� ���� 1
      }
      // �� ��������� � ��������� �����
      pixels[i]=(float)(pixelsRaw[i])/4.0f;     // 1 lsb = 0.25 C
    }



������ ������ ������ � ��������

Unused ranges:

         From           To    Size
         ----           --    ----
   0x800'235f   0x800'ffff  0xdca1      56481 ����
  0x2000'0131  0x2000'0137     0x7
  0x2000'0538  0x2000'1fff  0x1ac8      6856 ����

// BMP280
Unused ranges:

         From           To    Size
         ----           --    ----
   0x800'3f81   0x800'ffff  0xc07f      49279
  0x2000'01ee  0x2000'01ef     0x2
  0x2000'05f0  0x2000'1fff  0x1a10      6672

// 19.04.25 ������� ���������� �� ���� 
Unused ranges:

         From           To    Size
         ----           --    ----
   0x800'4caf   0x800'ffff  0xb351      45905
  0x2000'0354  0x2000'0357     0x4
  0x2000'0758  0x2000'1fff  0x18a8      6312

// 23.04.2025 ���������� � ������� 48*48 int16 = 4608 ����
Unused ranges:

         From           To    Size
         ----           --    ----
   0x800'4ebd   0x800'ffff  0xb143      45379
  0x2000'1958  0x2000'1fff   0x6a8      1704

// 23.04.2025 ���������� ��� �������� ������, ������ 128 ���� �� ����� ������
Unused ranges:

         From           To    Size
         ----           --    ----
   0x800'4e41   0x800'ffff  0xb1bf      45503
  0x2000'034c  0x2000'034f     0x4
  0x2000'0750  0x2000'1fff  0x18b0      6320


05.06.2025      ����������

  19'048 bytes of readonly  code memory
   2'027 bytes of readonly  data memory
   1'874 bytes of readwrite data memory

Unused ranges:

         From           To    Size
         ----           --    ----
   0x800'5253   0x800'ffff  0xadad      44461
  0x2000'0354  0x2000'0357     0x4
  0x2000'0758  0x2000'1fff  0x18a8      6312

 15.07.2025 ������ ��������
Unused ranges:

         From           To    Size
         ----           --    ----
   0x800'605b   0x800'ffff  0x9fa5      40869
  0x2000'040c  0x2000'040f     0x4
  0x2000'0810  0x2000'1fff  0x17f0      6128
*/
  
  
  
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
#define BUTTON_0_Pin GPIO_PIN_0
#define BUTTON_0_GPIO_Port GPIOA
#define BUTTON_0_EXTI_IRQn EXTI0_1_IRQn
#define BUTTON_1_Pin GPIO_PIN_1
#define BUTTON_1_GPIO_Port GPIOA
#define BUTTON_1_EXTI_IRQn EXTI0_1_IRQn
#define BUTTON_2_Pin GPIO_PIN_2
#define BUTTON_2_GPIO_Port GPIOA
#define BUTTON_2_EXTI_IRQn EXTI2_3_IRQn
#define LED_Pin GPIO_PIN_4
#define LED_GPIO_Port GPIOA
#define CHARGER_42_Pin GPIO_PIN_11
#define CHARGER_42_GPIO_Port GPIOA
#define CHARGER_100_Pin GPIO_PIN_12
#define CHARGER_100_GPIO_Port GPIOA
#define TACHO_POWER_Pin GPIO_PIN_3
#define TACHO_POWER_GPIO_Port GPIOB
#define RTC_IRQ_Pin GPIO_PIN_4
#define RTC_IRQ_GPIO_Port GPIOB
#define RTC_IRQ_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */
#define ADDR_FLASH_PAGE_0     ((uint32_t)0x08000000) /* Base @ of Page 0, 1 Kbytes */
#define ADDR_FLASH_PAGE_1     ((uint32_t)0x08000400) /* Base @ of Page 1, 1 Kbytes */
#define ADDR_FLASH_PAGE_62    ((uint32_t)0x0800F800) /* Base @ of Page 62, 1 Kbytes */
#define ADDR_FLASH_PAGE_63    ((uint32_t)0x0800FC00) /* Base @ of Page 63, 1 Kbytes */

#define LAST_FLASH_MEMORY_ADDRESS	(ADDR_FLASH_PAGE_63+FLASH_PAGE_SIZE)
#define ReadCalibrationVaraible(offset)  (*(__IO uint32_t*)(CalibrationAddr + offset))
#define CalibrationDone_Offset  ((uint8_t)0 * sizeof(uint32_t))
#define A2_Offset               ((uint8_t)1 * sizeof(uint32_t))
#define B2_Offset               ((uint8_t)2 * sizeof(uint32_t))
#define C2_Offset               ((uint8_t)3 * sizeof(uint32_t))
#define D2_Offset               ((uint8_t)4 * sizeof(uint32_t))
#define E2_Offset               ((uint8_t)5 * sizeof(uint32_t))
#define F2_Offset               ((uint8_t)6 * sizeof(uint32_t))
#define G2_Offset               ((uint8_t)7 * sizeof(uint32_t))
#define I2_Offset               ((uint8_t)8 * sizeof(uint32_t))
#define K2_Offset               ((uint8_t)9 * sizeof(uint32_t))
#define L2_Offset               ((uint8_t)10 * sizeof(uint32_t))
#define Offset(a)               ((uint8_t)a * sizeof(uint32_t))

#define BUT_UP          3
#define BUT_DOWN        2
#define BUT_OK          1

#define TSC_FLASH_COMPLETE       FLASH_COMPLETE

#define TSC_FLASH_FLAG_BSY       ((uint32_t)FLASH_FLAG_BSY)  /*!< FLASH Busy flag */
#define TSC_FLASH_FLAG_EOP       ((uint32_t)FLASH_FLAG_EOP)  /*!< FLASH End of Operation flag */
#define TSC_FLASH_FLAG_PGERR     ((uint32_t)FLASH_FLAG_PGERR)  /*!< FLASH Program error flag */
#define TSC_FLASH_FLAG_WRPERR  ((uint32_t)FLASH_FLAG_WRPERR)  /*!< FLASH Write protected error flag */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
