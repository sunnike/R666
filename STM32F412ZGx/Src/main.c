
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern volatile unsigned int time_states;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_RTC_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/*================== 1s Routine ================== */
		if (time_states & flag_1s){
			time_states &= ~flag_1s; // Clear flag.
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_0);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_0);
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_0);
			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_0);
			HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_0);
		}

		/*================== 900ms Routine ================== */
		if (time_states & flag_900ms){
			time_states &= ~flag_900ms; // Clear flag.
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_1);
			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_1);
			HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_1);
		}

		/*================== 800ms Routine ================== */
		if (time_states & flag_800ms){
			time_states &= ~flag_800ms; // Clear flag.
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2);
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_2);
			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_2);
		}

		/*================== 700ms Routine ================== */
		if (time_states & flag_700ms){
			time_states &= ~flag_700ms; // Clear flag.
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_3);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_3);
			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_3);
		}

		/*================== 600ms Routine ================== */
		if (time_states & flag_600ms){
			time_states &= ~flag_600ms; // Clear flag.
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_4);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4);
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_4);
			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_4);
		}

		/*================== 500ms Routine ================== */
		if (time_states & flag_500ms){
			time_states &= ~flag_500ms; // Clear flag.
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_5);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_5);
			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_5);
		}

		/*================== 400ms Routine ================== */
		if (time_states & flag_400ms){
			time_states &= ~flag_400ms; // Clear flag.
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_6);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_6);
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_6);
			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_6);
		}

		/*================== 300ms Routine ================== */
		if (time_states & flag_300ms){
			time_states &= ~flag_300ms; // Clear flag.
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_7);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_7);
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_7);
			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_7);
		}

		/*================== 200ms Routine ================== */
		if (time_states & flag_200ms){
			time_states &= ~flag_200ms; // Clear flag.
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_8);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_8);
			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_8);
		}

		/*================== 100ms Routine ================== */
		if (time_states & flag_100ms){
			time_states &= ~flag_100ms; // Clear flag.
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_9);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_9);
		}

		/*================== 50ms Routine ================== */
		if (time_states & flag_50ms){
			time_states &= ~flag_50ms; // Clear flag.
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_10);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);
			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_10);
		}

		/*================== 40ms Routine ================== */
		if (time_states & flag_40ms){
			time_states &= ~flag_40ms; // Clear flag.
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_11);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_11);
			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_11);
		}

		/*================== 30ms Routine ================== */
		if (time_states & flag_30ms){
			time_states &= ~flag_30ms; // Clear flag.
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_12);
			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_12);
		}

		/*================== 20ms Routine ================== */
		if (time_states & flag_20ms){
			time_states &= ~flag_20ms; // Clear flag.
			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_13);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_13);
			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
		}

		/*================== 10ms Routine ================== */
		if (time_states & flag_10ms){
			time_states &= ~flag_10ms; // Clear flag.
			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_14);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14);
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);
			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14);
		}

		/*================== 1ms Routine ================== */
		if (time_states & flag_1ms){
			time_states &= ~flag_1ms; // Clear flag.
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_15);
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_15);
			HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_15);
		}



  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
