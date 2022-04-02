/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os2.h" 
//#include "rl_usb.h"
//#include "rl_fs.h" 
#include "EventRecorder.h"

#include "stdio.h"
#include "string.h"

#include "user_usart.h"
#include "UserApp.h"
#include "ringbuffer.h"
#include "CMDTask.h"
 
#include "user_acc.h"
#include "user_mag.h"

//#include "Parameter.h"
//#include "user_rtc.h"
// 
//#include "UserStorage.h"
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCU_SPI_MISO_3_Pin GPIO_PIN_2
#define MCU_SPI_MISO_3_GPIO_Port GPIOE
#define MCU_SPI_MISO_4_Pin GPIO_PIN_3
#define MCU_SPI_MISO_4_GPIO_Port GPIOE
#define MCU_SPI_nCS_Pin GPIO_PIN_4
#define MCU_SPI_nCS_GPIO_Port GPIOE
#define MCU_SPI_nCS_2_Pin GPIO_PIN_5
#define MCU_SPI_nCS_2_GPIO_Port GPIOE
#define MCU_SPI_nCS_3_Pin GPIO_PIN_6
#define MCU_SPI_nCS_3_GPIO_Port GPIOE
#define MCU_SPI_SCK_Pin GPIO_PIN_13
#define MCU_SPI_SCK_GPIO_Port GPIOE
#define MCU_SPI_MOSI_Pin GPIO_PIN_10
#define MCU_SPI_MOSI_GPIO_Port GPIOB
#define TMP_1_SCK_Pin GPIO_PIN_13
#define TMP_1_SCK_GPIO_Port GPIOB
#define TMP_1_SIO_Pin GPIO_PIN_14
#define TMP_1_SIO_GPIO_Port GPIOB
#define TMP_1_nCS_Pin GPIO_PIN_15
#define TMP_1_nCS_GPIO_Port GPIOB
#define RS485DIR_Pin GPIO_PIN_10
#define RS485DIR_GPIO_Port GPIOD
#define MCU_SPI_MISO_5_Pin GPIO_PIN_6
#define MCU_SPI_MISO_5_GPIO_Port GPIOC
#define MCU_SPI_MISO_6_Pin GPIO_PIN_7
#define MCU_SPI_MISO_6_GPIO_Port GPIOC
#define MCU_SPI_MISO_7_Pin GPIO_PIN_8
#define MCU_SPI_MISO_7_GPIO_Port GPIOC
#define MCU_SPI_MISO_8_Pin GPIO_PIN_9
#define MCU_SPI_MISO_8_GPIO_Port GPIOC
#define LED_R_Pin GPIO_PIN_2
#define LED_R_GPIO_Port GPIOD
#define LED_G_Pin GPIO_PIN_3
#define LED_G_GPIO_Port GPIOD
#define MCU_SPI_MISO_1_Pin GPIO_PIN_0
#define MCU_SPI_MISO_1_GPIO_Port GPIOE
#define MCU_SPI_MISO_2_Pin GPIO_PIN_1
#define MCU_SPI_MISO_2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

#define GPIO_PIN_0_H16                 ((uint32_t)0x00010000)  /* Pin 0 selected    */
#define GPIO_PIN_1_H16                 ((uint32_t)0x00020000)  /* Pin 1 selected    */
#define GPIO_PIN_2_H16                 ((uint32_t)0x00040000)  /* Pin 2 selected    */
#define GPIO_PIN_3_H16                 ((uint32_t)0x00080000)  /* Pin 3 selected    */
#define GPIO_PIN_4_H16                 ((uint32_t)0x00100000)  /* Pin 4 selected    */
#define GPIO_PIN_5_H16                 ((uint32_t)0x00200000)  /* Pin 5 selected    */
#define GPIO_PIN_6_H16                 ((uint32_t)0x00400000)  /* Pin 6 selected    */
#define GPIO_PIN_7_H16                 ((uint32_t)0x00800000)  /* Pin 7 selected    */
#define GPIO_PIN_8_H16                 ((uint32_t)0x01000000)  /* Pin 8 selected    */
#define GPIO_PIN_9_H16                 ((uint32_t)0x02000000)  /* Pin 9 selected    */
#define GPIO_PIN_10_H16                ((uint32_t)0x04000000)  /* Pin 10 selected   */
#define GPIO_PIN_11_H16                ((uint32_t)0x08000000)  /* Pin 11 selected   */
#define GPIO_PIN_12_H16                ((uint32_t)0x10000000)  /* Pin 12 selected   */
#define GPIO_PIN_13_H16                ((uint32_t)0x20000000)  /* Pin 13 selected   */
#define GPIO_PIN_14_H16                ((uint32_t)0x40000000)  /* Pin 14 selected   */
#define GPIO_PIN_15_H16                ((uint32_t)0x80000000)  /* Pin 15 selected   */

#define LED_R_Pin_H16 GPIO_PIN_2_H16
#define LED_G_Pin_H16 GPIO_PIN_3_H16

#define RS485DIR_Pin_H16 GPIO_PIN_10_H16

#define MCU_SPI_nCS_Pin_H16 GPIO_PIN_4_H16
#define MCU_SPI_nCS_2_Pin_H16 GPIO_PIN_5_H16
#define MCU_SPI_nCS_3_Pin_H16 GPIO_PIN_6_H16
#define MCU_SPI_SCK_Pin_H16 GPIO_PIN_13_H16
#define MCU_SPI_MOSI_Pin_H16 GPIO_PIN_10_H16
#define TMP_1_SCK_Pin_H16 GPIO_PIN_13_H16
#define TMP_1_SIO_Pin_H16 GPIO_PIN_14_H16
#define TMP_1_nCS_Pin_H16 GPIO_PIN_15_H16




#define FLASH_RnB_Pin GPIO_PIN_6
#define FLASH_RnB_GPIO_Port GPIOD

#define LED_R_ON   LED_R_GPIO_Port->BSRR = (uint32_t)LED_R_Pin_H16     // L
#define LED_R_OFF  LED_R_GPIO_Port->BSRR = (uint32_t)LED_R_Pin         // H
#define LED_R_FLASH HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin)

#define LED_G_ON   LED_G_GPIO_Port->BSRR = (uint32_t)LED_G_Pin_H16     // L
#define LED_G_OFF  LED_G_GPIO_Port->BSRR = (uint32_t)LED_G_Pin         // H
#define LED_G_FLASH HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin)

#define RS485DIR_Rx RS485DIR_GPIO_Port->BSRR = (uint32_t)RS485DIR_Pin_H16     // L
#define RS485DIR_Tx RS485DIR_GPIO_Port->BSRR = (uint32_t)RS485DIR_Pin         // H


#define MCU_SPI_nCS_L   MCU_SPI_nCS_GPIO_Port ->BSRR = (uint32_t)MCU_SPI_nCS_Pin_H16     // L
#define MCU_SPI_nCS_H  MCU_SPI_nCS_GPIO_Port ->BSRR = (uint32_t)MCU_SPI_nCS_Pin          // H

#define MCU_SPI_nCS_2_L   MCU_SPI_nCS_2_GPIO_Port ->BSRR = (uint32_t)MCU_SPI_nCS_2_Pin_H16     // L
#define MCU_SPI_nCS_2_H  MCU_SPI_nCS_2_GPIO_Port ->BSRR = (uint32_t)MCU_SPI_nCS_2_Pin          // H

#define MCU_SPI_nCS_3_L   MCU_SPI_nCS_3_GPIO_Port ->BSRR = (uint32_t)MCU_SPI_nCS_3_Pin_H16     // L
#define MCU_SPI_nCS_3_H  MCU_SPI_nCS_3_GPIO_Port ->BSRR = (uint32_t)MCU_SPI_nCS_3_Pin          // H

#define MCU_SPI_SCK_L   MCU_SPI_SCK_GPIO_Port ->BSRR = (uint32_t)MCU_SPI_SCK_Pin_H16     // L
#define MCU_SPI_SCK_H  MCU_SPI_SCK_GPIO_Port ->BSRR = (uint32_t)MCU_SPI_SCK_Pin          // H

//#define MCU_SPI_SCK_2_L   MCU_SPI_SCK_2_GPIO_Port ->BSRR = (uint32_t)MCU_SPI_SCK_2_Pin_H16     // L
//#define MCU_SPI_SCK_2_H  MCU_SPI_SCK_2_GPIO_Port ->BSRR = (uint32_t)MCU_SPI_SCK_2_Pin          // H

#define MCU_SPI_MOSI_L   MCU_SPI_MOSI_GPIO_Port ->BSRR = (uint32_t)MCU_SPI_MOSI_Pin_H16     // L
#define MCU_SPI_MOSI_H  MCU_SPI_MOSI_GPIO_Port ->BSRR = (uint32_t)MCU_SPI_MOSI_Pin          // H

#define TMP_1_SCK_L   TMP_1_SCK_GPIO_Port ->BSRR = (uint32_t)TMP_1_SCK_Pin_H16     // L
#define TMP_1_SCK_H  TMP_1_SCK_GPIO_Port ->BSRR = (uint32_t)TMP_1_SCK_Pin          // H

#define TMP_1_SIO_L   TMP_1_SIO_GPIO_Port ->BSRR = (uint32_t)TMP_1_SIO_Pin_H16     // L
#define TMP_1_SIO_H  TMP_1_SIO_GPIO_Port ->BSRR = (uint32_t)TMP_1_SIO_Pin          // H


#define TMP_1_nCS_L   TMP_1_nCS_GPIO_Port ->BSRR = (uint32_t)TMP_1_nCS_Pin_H16     // L
#define TMP_1_nCS_H  TMP_1_nCS_GPIO_Port ->BSRR = (uint32_t)TMP_1_nCS_Pin          // H

#define bit_MCU_SPI_MISO_1  (MCU_SPI_MISO_1_GPIO_Port->IDR&MCU_SPI_MISO_1_Pin)
#define bit_MCU_SPI_MISO_2  (MCU_SPI_MISO_2_GPIO_Port->IDR&MCU_SPI_MISO_2_Pin)
#define bit_MCU_SPI_MISO_3  (MCU_SPI_MISO_3_GPIO_Port->IDR&MCU_SPI_MISO_3_Pin)
#define bit_MCU_SPI_MISO_4  (MCU_SPI_MISO_4_GPIO_Port->IDR&MCU_SPI_MISO_4_Pin)
#define bit_MCU_SPI_MISO_5  (MCU_SPI_MISO_5_GPIO_Port->IDR&MCU_SPI_MISO_5_Pin)
#define bit_MCU_SPI_MISO_6  (MCU_SPI_MISO_6_GPIO_Port->IDR&MCU_SPI_MISO_6_Pin)
#define bit_MCU_SPI_MISO_7  (MCU_SPI_MISO_7_GPIO_Port->IDR&MCU_SPI_MISO_7_Pin)
#define bit_MCU_SPI_MISO_8  (MCU_SPI_MISO_8_GPIO_Port->IDR&MCU_SPI_MISO_8_Pin)


#define OS_TICK_PER_SECOND  1000

#define ADC_Sample_Freq  250     // ADC������
#define ADC_Calc_Freq    5       // ������������

#define ADC1_RAW_CH_Cout       2     // ͨ����
#define ADC1_RAW_BUF_Size      (ADC_Sample_Freq/ADC_Calc_Freq)*ADC1_RAW_CH_Cout*2  // *2=ÿ����¼2�ֽ�
#define ADC1_RAW_BUF_A_Start   0x2001B100  
#define ADC1_RAW_BUF_B_Start   (ADC1_RAW_BUF_A_Start+ADC1_RAW_BUF_Size) 

#define ADC2_RAW_CH_Cout   3
#define ADC2_RAW_BUF_Size      (ADC_Sample_Freq/ADC_Calc_Freq)*ADC2_RAW_CH_Cout*2 
#define ADC2_RAW_BUF_A_Start   (ADC1_RAW_BUF_B_Start+ADC1_RAW_BUF_Size) 
#define ADC2_RAW_BUF_B_Start   (ADC2_RAW_BUF_A_Start+ADC2_RAW_BUF_Size) 

#define ADC4_RAW_CH_Cout   1

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
