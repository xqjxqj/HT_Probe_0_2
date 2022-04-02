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
#include "UserApp.h"

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
#define ADC_FORMAT2_Pin GPIO_PIN_2
#define ADC_FORMAT2_GPIO_Port GPIOE
#define ADC_PWDN_0_Pin GPIO_PIN_3
#define ADC_PWDN_0_GPIO_Port GPIOE
#define ADC_PWDN_1_Pin GPIO_PIN_4
#define ADC_PWDN_1_GPIO_Port GPIOE
#define ADC_MODE0_Pin GPIO_PIN_5
#define ADC_MODE0_GPIO_Port GPIOE
#define ADC_MODE1_Pin GPIO_PIN_6
#define ADC_MODE1_GPIO_Port GPIOE
#define ARM_RESET_Pin GPIO_PIN_10
#define ARM_RESET_GPIO_Port GPIOG
#define ADC_SCLK_Pin GPIO_PIN_13
#define ADC_SCLK_GPIO_Port GPIOB
#define ADC_DOUT_Pin GPIO_PIN_15
#define ADC_DOUT_GPIO_Port GPIOB
#define ADC_SYNC_Pin GPIO_PIN_8
#define ADC_SYNC_GPIO_Port GPIOD
#define ADC_CLK_Pin GPIO_PIN_8
#define ADC_CLK_GPIO_Port GPIOA
#define LED_R_Pin GPIO_PIN_2
#define LED_R_GPIO_Port GPIOD
#define LED_G_Pin GPIO_PIN_3
#define LED_G_GPIO_Port GPIOD
#define GR_Pin GPIO_PIN_3
#define GR_GPIO_Port GPIOB
#define FLOW_Pin GPIO_PIN_4
#define FLOW_GPIO_Port GPIOB
#define PULSE_Pin GPIO_PIN_5
#define PULSE_GPIO_Port GPIOB
#define ADC_CLKDIV_Pin GPIO_PIN_9
#define ADC_CLKDIV_GPIO_Port GPIOB
#define ADC_FORMAT0_Pin GPIO_PIN_0
#define ADC_FORMAT0_GPIO_Port GPIOE
#define ADC_FORMAT1_Pin GPIO_PIN_1
#define ADC_FORMAT1_GPIO_Port GPIOE
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

#define LED_R_ON   LED_R_GPIO_Port->BSRR = (uint32_t)LED_R_Pin_H16     // L
#define LED_R_OFF  LED_R_GPIO_Port->BSRR = (uint32_t)LED_R_Pin         // H
#define LED_R_FLASH HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin)

#define LED_G_ON   LED_G_GPIO_Port->BSRR = (uint32_t)LED_G_Pin_H16     // L
#define LED_G_OFF  LED_G_GPIO_Port->BSRR = (uint32_t)LED_G_Pin         // H
#define LED_G_FLASH HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin)






/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
