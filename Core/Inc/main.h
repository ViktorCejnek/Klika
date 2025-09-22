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
#include "stm32wbxx_hal.h"

#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TMC2009_UART.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
enum FSMSTATE
{
	s_Idle, s_Lock, s_Unlock, s_Turning, s_Timeout, s_Soon, s_Finnished, s_Error
};

enum FLAG
{
	no_flag, flag
};

extern enum FLAG f_Unlock;
extern enum FLAG f_Stop;
extern enum FLAG f_Diag;
extern enum FLAG f_Lock;

extern const int32_t speed;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define TMC_speed           (200000)
#define TMC_gear_ratio      (3)
#define TMC_steps_per_rot   (256 * 200 * (TMC_gear_ratio))
#define TMC_time_per_rot    ((TMC_steps_per_rot) * (1000) / (speed*0.715))
#define TMC_timeout         ((3) * (TMC_time_per_rot))
//#define TMC_timeout         4500
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Lock(void);
void Unlock(void);
void Stop(void);

void delay_us (uint16_t us);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void INIT();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MS1_Pin GPIO_PIN_0
#define MS1_GPIO_Port GPIOA
#define MS2_Pin GPIO_PIN_1
#define MS2_GPIO_Port GPIOA
#define SPREAD_Pin GPIO_PIN_4
#define SPREAD_GPIO_Port GPIOA
#define ENN_Pin GPIO_PIN_5
#define ENN_GPIO_Port GPIOA
#define DPH_Pin GPIO_PIN_0
#define DPH_GPIO_Port GPIOB
#define DPL_Pin GPIO_PIN_1
#define DPL_GPIO_Port GPIOB
#define DNH_Pin GPIO_PIN_11
#define DNH_GPIO_Port GPIOA
#define DNL_Pin GPIO_PIN_12
#define DNL_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_15
#define LED_GPIO_Port GPIOA
#define STDBY_Pin GPIO_PIN_3
#define STDBY_GPIO_Port GPIOB
#define DIR_Pin GPIO_PIN_4
#define DIR_GPIO_Port GPIOB
#define STEP_Pin GPIO_PIN_5
#define STEP_GPIO_Port GPIOB
#define INDEX_Pin GPIO_PIN_6
#define INDEX_GPIO_Port GPIOB
#define DIAG_Pin GPIO_PIN_7
#define DIAG_GPIO_Port GPIOB
#define DIAG_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
