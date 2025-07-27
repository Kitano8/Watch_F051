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
    09.04.2025  Начало проекта 051. Дисплей заработал и перевёрнут по вертикали) не работает поворот справа налево( придётся полный видеобуфер делать, я так и хотел для больших цифр.
                Часы работают. max17055 работает, id совпадает, ток и напряжение есть
    11.04.2025  Добавляю код для bmp280
                Пишалка работает, а вибратор нет(. на 48 МГц жрёт 18,4 мА, на 8МГц жрет 8,4мА. Всё таки это был amg8833, при его сне получилось 3,8мА
                Что-то зарядки нет( пересобрать проект надо было полностью
    13.04.2025  Дополнил драйвер max17055
    15.04.2025  вывел сопротивление батареи
    16.04.2025  бывает низкий заряд батареи и часы включаются а изображения нет, перепаивал дисплей два раза
    19.04.2025  добавил прерывания и запустил энергосбережение показывает -0,5мА потребление, Rвн батареи показывает около 56 мОм
                0x0800FC00 адрес страницы крайний. сохранение во флеш
    20.04.2025  Подсветки не было т.к. led- на плате не контачит со шлейфом, припаял на gnd соседний, ток с подсветкой 8мА
                Меню делаю. добавил процедуру пробуждения тепловизора. Данные с тепловизора идут, надо дописать отображение и интерфейс его экрана
                перестал засыпать(
    21.04.2025  Добавил HAL_SuspendTick(); потребление 0,8мА. надо ещё почитать. Раньше swd отваливался, сейчас нет и не запускается пока не отключишь отладчик
    22.04.2025	Не засыпает, проверил биты. Нужно пробовать отключать выводы, вроде что-то получалось один раз
    23.04.2025  выключил зарядку, это позволило замерить потребляемый ток от usb, потребляет при измерении 3,3мА, в режиме стоп с остановкой систика ровно 1мА
                Пробовал ставить уход в сон, потребление 1,6-1,7мА. в стопе с PWR_MAINREGULATOR_ON потребление 1,06мА.
                в standbuy режиме потребление 1,4мА. в режиме stop отладчик не отваливается. Отладка работает, но пока режим стоп, данные все равны нулю, когда выходит, то актуальные
                Нашел!!! option bytes перезаписал nRST_STOP и nRST_STBY. потребление 0,3мА без HAL_SuspendTick();
                https://electronix.ru/forum/topic/164660-stm32l100-posle-proshivki-ne-spit/
                Запустил тепловизор на 48*48 пикселей через буфер, сверху вниз правильно показывает, слева направо наоборот
                Сделал отрисовку на лету на 64*64 пикселя без буфера. Уменьшил размер до 32*32, более-менее быстро показывает
                Добавил установку времени
    27.05.2025  Добавить тахометр и секундомер
    29.05.2025	Порвался шлейф немного (сброс крайний), восстановил. 
		Сделать в тепловизоре обновление отображения порога температуры при его изменении.
		Проверить сохраняются ли циклы, при сбросе контроллера цикл равен нулю. Сохраняется
    05.06.2025  Убрать установку часов при сбросе.
                Установку часов при сбросе убрал, секундомер сделал. Тахометр надо встраивать в корпус, пока что лицензия компаса кончилась
    06.06.2025  Поправил вычисление минут. секундомер считает в часах, а не днях, проблему перехода через месяц решить не удастся, т.к. не понятна длина месяца, я думаю это не страшно. оставлю так
    09.06.2025  При установке времени при отключении батареи год показывает 00 или 01, не помню, а при увеличении начинает с 25. Циклы не сохраняет и наверно остальные параметры тоже, ёмкость стала 149мач
                Добавляю тахометр из tcrt5000, пока напрямую
    16.06.2025  Тахометр поставил через компаратор lm393. Потребление в режиме тахометра около 10мА, из-за ик-светодиода.
                При смене экрана нужно очищать буфера вывода, иначе в первый раз выводит старые данные вместе с новыми.
                Добавить режим тахометра, посчитать какой диапазон можно измерить.
                Таймер 2 имеет разрядность 4294967295/8000000=536 секунд
                Таймер 1 имеет разрядность 65535/8000000=0.008 секунд
                На графике давления сделать курсор как-то? может и не нужно
    18.06.2025  Если время в часах сброшено, то при старте его нужно установить
                Добавить в режиме акб возможность смены зарядного тока
    19.06.2025  Настройка тахометра. Заработал! правки интерфейса тахометра
    15.07.2025  Потребление выросло, max перестал уходить в сон, попробовать выключать тактирование таймера тахометра.
                Вывел отображение режима зарядки. надо переводить зарядку в режим 2 - максимальный ток
                Попробовать переводить выводы в аналоговый режим, max засыпает если ток менее С/100. Отключил tim1 и pa8, может ток потребления упадёт?
    22.07.2025  Вывод внутреннего сопротивления батареи вместо давления на экране батареи. убрал задержку в 3С перед переходом в стоп

Для перевода в режим останова необходимо выполнить следующие действия:
Установить бит SLEEPDIP в регистре SCB  +
Очистить бит PDDS в регистре PWR_CR     +
Очистить бит WUF в регистре PWR_CSR     добавил
Настроить регулятор напряжения LPSDSR в регистре PWR_CR +
Настроить событие или прерывание выхода из режима STOP  +
Подать команду WFI или WFE

из референса

WFI (ожидание прерывания) или WFE (ожидание события), пока:
– Установите бит SLEEPDEEP в регистре управления системой Cortex®-M0
– Очистите бит PDDS в регистре управления питанием (PWR_CR)
– Выберите режим регулятора напряжения, настроив бит LPDS в PWR_CR

Примечание: для входа в режим остановки все биты ожидания линии EXTI (в регистре ожидания (EXTI_PR)),
все биты ожидания прерывания периферийных устройств и флаг тревоги RTC должны быть сброшены. 
В противном случае процедура входа в режим остановки игнорируется, и выполнение программы продолжается.

Если приложению необходимо отключить внешний генератор (внешние часы) перед входом в режим остановки, 
источник системных часов должен быть сначала переключен на HSI, а затем очищен бит HSEON.
В противном случае, если перед входом в режим остановки бит HSEON удерживается на уровне 1, 
необходимо включить функцию системы безопасности (CSS) для обнаружения любого сбоя внешнего генератора 
(внешнего тактового генератора) и предотвращения сбоя при входе в режим остановки.

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
      // знаковое число из 12 в 16 бит
      pixelsRaw[i]=pixelsRaw[i]<<4;
      pixelsRaw[i]=pixelsRaw[i]>>4;
      // из формата дополнения до двух в знаковое
      if(pixelsRaw[i]<0){
        pixelsRaw[i]=0-((~pixelsRaw[i])+1);     // инверсия плюс 1
      }
      // из знакового в плавующую точку
      pixels[i]=(float)(pixelsRaw[i])/4.0f;     // 1 lsb = 0.25 C
    }



начало только собрал с дисплеем

Unused ranges:

         From           To    Size
         ----           --    ----
   0x800'235f   0x800'ffff  0xdca1      56481 байт
  0x2000'0131  0x2000'0137     0x7
  0x2000'0538  0x2000'1fff  0x1ac8      6856 байт

// BMP280
Unused ranges:

         From           To    Size
         ----           --    ----
   0x800'3f81   0x800'ffff  0xc07f      49279
  0x2000'01ee  0x2000'01ef     0x2
  0x2000'05f0  0x2000'1fff  0x1a10      6672

// 19.04.25 добавил сохранение во флеш 
Unused ranges:

         From           To    Size
         ----           --    ----
   0x800'4caf   0x800'ffff  0xb351      45905
  0x2000'0354  0x2000'0357     0x4
  0x2000'0758  0x2000'1fff  0x18a8      6312

// 23.04.2025 тепловизор с буфером 48*48 int16 = 4608 байт
Unused ranges:

         From           To    Size
         ----           --    ----
   0x800'4ebd   0x800'ffff  0xb143      45379
  0x2000'1958  0x2000'1fff   0x6a8      1704

// 23.04.2025 тепловизор без большого буфера, только 128 байт на сырые данные
Unused ranges:

         From           To    Size
         ----           --    ----
   0x800'4e41   0x800'ffff  0xb1bf      45503
  0x2000'034c  0x2000'034f     0x4
  0x2000'0750  0x2000'1fff  0x18b0      6320


05.06.2025      секундомер

  19'048 bytes of readonly  code memory
   2'027 bytes of readonly  data memory
   1'874 bytes of readwrite data memory

Unused ranges:

         From           To    Size
         ----           --    ----
   0x800'5253   0x800'ffff  0xadad      44461
  0x2000'0354  0x2000'0357     0x4
  0x2000'0758  0x2000'1fff  0x18a8      6312

 15.07.2025 правки тахометр
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
