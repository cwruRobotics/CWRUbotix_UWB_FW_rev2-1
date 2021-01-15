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
#include "stm32f1xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DWM1000_CS_Pin GPIO_PIN_0
#define DWM1000_CS_GPIO_Port GPIOB
#define DWM1000_RST_Pin GPIO_PIN_1
#define DWM1000_RST_GPIO_Port GPIOB
#define CAN_ID_1_Pin GPIO_PIN_2
#define CAN_ID_1_GPIO_Port GPIOB
#define DWM1000_IRQ_Pin GPIO_PIN_10
#define DWM1000_IRQ_GPIO_Port GPIOB
#define DWM1000_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define CAN_ID_64_Pin GPIO_PIN_12
#define CAN_ID_64_GPIO_Port GPIOB
#define CAN_ID_128_Pin GPIO_PIN_13
#define CAN_ID_128_GPIO_Port GPIOB
#define RNG_LED_Pin GPIO_PIN_14
#define RNG_LED_GPIO_Port GPIOB
#define CAN_LED_Pin GPIO_PIN_15
#define CAN_LED_GPIO_Port GPIOB
#define CAN_ID_2_Pin GPIO_PIN_3
#define CAN_ID_2_GPIO_Port GPIOB
#define CAN_ID_4_Pin GPIO_PIN_4
#define CAN_ID_4_GPIO_Port GPIOB
#define CAN_ID_8_Pin GPIO_PIN_5
#define CAN_ID_8_GPIO_Port GPIOB
#define CAN_ID_16_Pin GPIO_PIN_6
#define CAN_ID_16_GPIO_Port GPIOB
#define CAN_ID_32_Pin GPIO_PIN_7
#define CAN_ID_32_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define DW_IRQn_Pin DWM1000_IRQ_Pin
#define DW_IRQn DWM1000_IRQ_EXTI_IRQn
#define DW_IRQn_Type DW_IRQn
#define DW_RESET_Pin DWM1000_RST_Pin
#define DW_RESET_GPIO_Port DWM1000_RST_GPIO_Port
#define RANGING_LED_GPIO_Port RNG_LED_GPIO_Port
#define RANGING_LED_Pin RNG_LED_Pin
#define DW_NSS_GPIO_Port DWM1000_CS_Pin
#define DW_NSS_Pin DWM1000_CS_Pin
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
