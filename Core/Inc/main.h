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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Cap3_Pin GPIO_PIN_2
#define Cap3_GPIO_Port GPIOE
#define Cap3_EXTI_IRQn EXTI2_IRQn
#define Cap4_Pin GPIO_PIN_3
#define Cap4_GPIO_Port GPIOE
#define Cap4_EXTI_IRQn EXTI3_IRQn
#define Cap5_Pin GPIO_PIN_4
#define Cap5_GPIO_Port GPIOE
#define Cap5_EXTI_IRQn EXTI4_IRQn
#define Cap6_Pin GPIO_PIN_5
#define Cap6_GPIO_Port GPIOE
#define Cap6_EXTI_IRQn EXTI9_5_IRQn
#define Cap7_Pin GPIO_PIN_6
#define Cap7_GPIO_Port GPIOE
#define Cap7_EXTI_IRQn EXTI9_5_IRQn
#define Cap8_Pin GPIO_PIN_13
#define Cap8_GPIO_Port GPIOC
#define Cap8_EXTI_IRQn EXTI15_10_IRQn
#define Cap9_Pin GPIO_PIN_14
#define Cap9_GPIO_Port GPIOC
#define Cap9_EXTI_IRQn EXTI15_10_IRQn
#define Cap10_Pin GPIO_PIN_15
#define Cap10_GPIO_Port GPIOC
#define Cap10_EXTI_IRQn EXTI15_10_IRQn
#define STP2_Pin GPIO_PIN_2
#define STP2_GPIO_Port GPIOA
#define STP1_Pin GPIO_PIN_3
#define STP1_GPIO_Port GPIOA
#define STP1_EN_Pin GPIO_PIN_4
#define STP1_EN_GPIO_Port GPIOA
#define STP1_DIR_Pin GPIO_PIN_5
#define STP1_DIR_GPIO_Port GPIOA
#define Relay2_Pin GPIO_PIN_6
#define Relay2_GPIO_Port GPIOA
#define Relay1_Pin GPIO_PIN_7
#define Relay1_GPIO_Port GPIOA
#define Pwm8_Pin GPIO_PIN_0
#define Pwm8_GPIO_Port GPIOB
#define Pwm7_Pin GPIO_PIN_1
#define Pwm7_GPIO_Port GPIOB
#define STP2_EN_Pin GPIO_PIN_7
#define STP2_EN_GPIO_Port GPIOE
#define STP2_DIR_Pin GPIO_PIN_8
#define STP2_DIR_GPIO_Port GPIOE
#define Pwm6_Pin GPIO_PIN_9
#define Pwm6_GPIO_Port GPIOE
#define Pwm5_Pin GPIO_PIN_11
#define Pwm5_GPIO_Port GPIOE
#define Pwm4_Pin GPIO_PIN_13
#define Pwm4_GPIO_Port GPIOE
#define Pwm3_Pin GPIO_PIN_14
#define Pwm3_GPIO_Port GPIOE
#define Pwm2_Pin GPIO_PIN_10
#define Pwm2_GPIO_Port GPIOB
#define Pwm1_Pin GPIO_PIN_11
#define Pwm1_GPIO_Port GPIOB
#define TowerRed_Pin GPIO_PIN_12
#define TowerRed_GPIO_Port GPIOB
#define TowerAmber_Pin GPIO_PIN_13
#define TowerAmber_GPIO_Port GPIOB
#define TowerGreen_Pin GPIO_PIN_14
#define TowerGreen_GPIO_Port GPIOB
#define RelayJ15_Pin GPIO_PIN_15
#define RelayJ15_GPIO_Port GPIOB
#define RelayJ17_Pin GPIO_PIN_8
#define RelayJ17_GPIO_Port GPIOD
#define RelayJ18_Pin GPIO_PIN_9
#define RelayJ18_GPIO_Port GPIOD
#define Led3_Pin GPIO_PIN_10
#define Led3_GPIO_Port GPIOD
#define Led2_Pin GPIO_PIN_11
#define Led2_GPIO_Port GPIOD
#define Led1_Pin GPIO_PIN_12
#define Led1_GPIO_Port GPIOD
#define Sig5V_Pin GPIO_PIN_14
#define Sig5V_GPIO_Port GPIOD
#define Sig12V_Pin GPIO_PIN_15
#define Sig12V_GPIO_Port GPIOD
#define Sig48V_Pin GPIO_PIN_6
#define Sig48V_GPIO_Port GPIOC
#define Inp1_Pin GPIO_PIN_15
#define Inp1_GPIO_Port GPIOA
#define Inp2_Pin GPIO_PIN_12
#define Inp2_GPIO_Port GPIOC
#define Inp3_Pin GPIO_PIN_0
#define Inp3_GPIO_Port GPIOD
#define Inp4_Pin GPIO_PIN_1
#define Inp4_GPIO_Port GPIOD
#define Inp5_Pin GPIO_PIN_2
#define Inp5_GPIO_Port GPIOD
#define Inp_Pin GPIO_PIN_3
#define Inp_GPIO_Port GPIOD
#define Inp7_Pin GPIO_PIN_4
#define Inp7_GPIO_Port GPIOD
#define Inp8_Pin GPIO_PIN_7
#define Inp8_GPIO_Port GPIOD
#define Inp9_Pin GPIO_PIN_4
#define Inp9_GPIO_Port GPIOB
#define Cap1_Pin GPIO_PIN_0
#define Cap1_GPIO_Port GPIOE
#define Cap1_EXTI_IRQn EXTI0_IRQn
#define Cap2_Pin GPIO_PIN_1
#define Cap2_GPIO_Port GPIOE
#define Cap2_EXTI_IRQn EXTI1_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
