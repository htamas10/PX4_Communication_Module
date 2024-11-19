/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
void Update_Main_Menu(void);
void Sys_DelayUs(int us);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define BTN_RIGHT_Pin GPIO_PIN_0
#define BTN_RIGHT_GPIO_Port GPIOC
#define BTN_RIGHT_EXTI_IRQn EXTI0_IRQn
#define BTN_DOWN_Pin GPIO_PIN_1
#define BTN_DOWN_GPIO_Port GPIOC
#define BTN_DOWN_EXTI_IRQn EXTI1_IRQn
#define BTN_UP_Pin GPIO_PIN_2
#define BTN_UP_GPIO_Port GPIOC
#define BTN_UP_EXTI_IRQn EXTI2_IRQn
#define BTN_LEFT_Pin GPIO_PIN_3
#define BTN_LEFT_GPIO_Port GPIOC
#define BTN_LEFT_EXTI_IRQn EXTI3_IRQn
#define LCD_BTN2_Pin GPIO_PIN_1
#define LCD_BTN2_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define IC_EN_Pin GPIO_PIN_6
#define IC_EN_GPIO_Port GPIOA
#define LCD_EN_Pin GPIO_PIN_7
#define LCD_EN_GPIO_Port GPIOA
#define DB0_Pin GPIO_PIN_5
#define DB0_GPIO_Port GPIOC
#define LCD_BTN1_Pin GPIO_PIN_0
#define LCD_BTN1_GPIO_Port GPIOB
#define DB7_Pin GPIO_PIN_10
#define DB7_GPIO_Port GPIOB
#define LCD_RW_Pin GPIO_PIN_12
#define LCD_RW_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_13
#define LCD_RST_GPIO_Port GPIOB
#define DB1_Pin GPIO_PIN_6
#define DB1_GPIO_Port GPIOC
#define DB2_Pin GPIO_PIN_7
#define DB2_GPIO_Port GPIOC
#define DB3_Pin GPIO_PIN_8
#define DB3_GPIO_Port GPIOC
#define DB4_Pin GPIO_PIN_9
#define DB4_GPIO_Port GPIOC
#define USER_LED_Pin GPIO_PIN_12
#define USER_LED_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define BL_PWM_Pin GPIO_PIN_15
#define BL_PWM_GPIO_Port GPIOA
#define LCD_CS1_Pin GPIO_PIN_10
#define LCD_CS1_GPIO_Port GPIOC
#define LCD_CS2_Pin GPIO_PIN_11
#define LCD_CS2_GPIO_Port GPIOC
#define LCD_DI_Pin GPIO_PIN_12
#define LCD_DI_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define DB5_Pin GPIO_PIN_8
#define DB5_GPIO_Port GPIOB
#define DB6_Pin GPIO_PIN_9
#define DB6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */