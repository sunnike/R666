/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/** @defgroup Timer_Flag_Enumeration_definition Timer Flags definition
  * @brief  Timer Flags definition
  * @{
  */
typedef enum{
  flag_1ms    = 0x0001, /*!< Specifies the timer "idle" states. This bit-masking is used to set/clear time_states when timer is idle. */
  flag_10ms   = 0x0002, /*!< Specifies the timer "1ms" states. This bit-masking is used to set/clear time_states in every 1ms. */
  flag_20ms   = 0x0004, /*!< Specifies the timer "10ms" states. This bit-masking is used to set/clear time_states in every 100ms. */
  flag_30ms   = 0x0008, /*!< Specifies the timer "100ms" states. This bit-masking is used to set/clear time_states in every 500ms. */
  flag_40ms   = 0x0010,  /*!< Specifies the timer "1s" states. This bit-masking is used to set/clear time_states in every 1s. */
  flag_50ms   = 0x0020, /*!< Specifies the timer "1ms" states. This bit-masking is used to set/clear time_states in every 1ms. */
  flag_100ms  = 0x0040, /*!< Specifies the timer "10ms" states. This bit-masking is used to set/clear time_states in every 100ms. */
  flag_200ms  = 0x0080, /*!< Specifies the timer "100ms" states. This bit-masking is used to set/clear time_states in every 500ms. */
  flag_300ms  = 0x0100,  /*!< Specifies the timer "1s" states. This bit-masking is used to set/clear time_states in every 1s. */
  flag_400ms  = 0x0200, /*!< Specifies the timer "1ms" states. This bit-masking is used to set/clear time_states in every 1ms. */
  flag_500ms  = 0x0400, /*!< Specifies the timer "10ms" states. This bit-masking is used to set/clear time_states in every 100ms. */
  flag_600ms  = 0x0800, /*!< Specifies the timer "100ms" states. This bit-masking is used to set/clear time_states in every 500ms. */
  flag_700ms  = 0x1000,  /*!< Specifies the timer "1s" states. This bit-masking is used to set/clear time_states in every 1s. */
  flag_800ms  = 0x2000, /*!< Specifies the timer "1ms" states. This bit-masking is used to set/clear time_states in every 1ms. */
  flag_900ms  = 0x4000, /*!< Specifies the timer "10ms" states. This bit-masking is used to set/clear time_states in every 100ms. */
  flag_1s     = 0x8000 /*!< Specifies the timer "100ms" states. This bit-masking is used to set/clear time_states in every 500ms. */
}eTimer_Flags;


/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
