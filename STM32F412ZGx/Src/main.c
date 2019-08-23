
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "fatfs.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"
#include "fsmc.h"

/* USER CODE BEGIN Includes */
#include <stdarg.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Private define  ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern ApplicationTypeDef Appli_state;
extern volatile unsigned char time_states;

//---------------
// FPGA variables
//---------------
const uint8_t fpga_key[FPGA_KEY_SIZE] = {'A','E','W','I','N','1','6','8'};

uint8_t fpga_spi_switch = FPGA_SPI_SWITCH_OFF;
uint8_t fpga_spi_mode = FLASH_NONE;
uint8_t fpga_key_readback[FPGA_KEY_SIZE];
uint8_t fpga_info[FPGA_INFO_SIZE];
uint8_t fpga_busy_status[FPGA_BUSY_STATUS_SIZE];
uint8_t fpga_timer_switch = FPGA_TIMER_DISABLE;

//---------------
// USB variables
//---------------
uint8_t usb_read_flag = 0;    // flag to mark whether or not USB disk has been read

// information read from aewin_file.txt
uint8_t usb_cmd_code = USB_CMD_NONE;
uint8_t usb_cmd_flash_num = FLASH_NONE;
uint8_t usb_cmd_ima_filename[IMA_FILENAME_LEN_LIMIT];
uint8_t usb_cmd_re_read_flash = RE_READ_FLASH_OFF;

uint8_t usb_err_code = USB_ERR_NONE;

uint8_t backup_flag[2] = {RE_READ_FLASH_OFF, RE_READ_FLASH_OFF};

//---------------
// USB timeout variables
//---------------
extern volatile unsigned char timeout_counter;
extern unsigned char timeout_counter_switch;

//---------------
// RTC variables
//---------------
RTC_TimeTypeDef RTC_Time;
RTC_DateTypeDef RTC_Date;
uint8_t RTC_bin_format[FPGA_TIMER_SIZE] = {0};

//---------------
// test variables
//---------------
unsigned char err_record = 0;
// I2C, FSMC, SPI, uart dont need to be list

#if AEWIN_DBUG
char dbg_buff[PRINT_BUFF];
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_USB_HOST_Init();
  MX_FATFS_Init();
  MX_FSMC_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	aewin_dbg("=============================\r\n");
	aewin_dbg("          R666 test          \r\n");
	aewin_dbg("=============================\r\n");

	// print booting date and time
	HAL_RTC_GetTime(&hrtc, &RTC_Time, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc, &RTC_Date, RTC_FORMAT_BCD);
	aewin_dbg("RTC : 20%02x.%02x.%02x - %02x:%02x:%02x\r\n", RTC_Date.Year, RTC_Date.Month, RTC_Date.Date, RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);
	aewin_dbg("-----------------------------\r\n");

	// print MCU firmware version
	aewin_dbg("MCU version: %d.%d.%d\r\n", VER_MAJOR, VER_MINOR, VER_PATCH);
	aewin_dbg("-----------------------------\r\n");

	//aewin_dbg("Unlock FPGA.\r\n");
	//--------------------------------
	// Unlock FPGA key
	i2c2_fpga_write(FPGA_KEY_BASE_ADDR, FPGA_KEY_SIZE, (uint8_t*)fpga_key);

	// Read FPGA key
	i2c2_fpga_read(FPGA_KEY_BASE_ADDR, FPGA_KEY_SIZE, (uint8_t*)fpga_key_readback);

	aewin_dbg("------ FPGA information -----\r\n");
	// Read FPGA information - version and time
	i2c2_fpga_read(FPGA_INFO_BASE_ADDR, FPGA_INFO_SIZE, (uint8_t*)fpga_info);
	aewin_dbg("FPGA version    : %d.%d.%d\r\n", fpga_info[0], fpga_info[1], fpga_info[2]);
	aewin_dbg("FPGA build date : 20%02d.%02d.%02d\r\n", fpga_info[5], fpga_info[4], fpga_info[3]);

	// Read FPGA Busy byte and status1 byte
	i2c2_fpga_read(FPGA_BUSY_STATUS_BASE_ADDR, FPGA_BUSY_STATUS_SIZE, (uint8_t*)fpga_busy_status);
	aewin_dbg("FPGA busy bit   : %x\r\n", fpga_busy_status[4]);
	aewin_dbg("FPGA busy status: %x %x %x\r\n", fpga_busy_status[7], fpga_busy_status[6], fpga_busy_status[5]);
	aewin_dbg("-----------------------------\r\n");

	// check FPGA information. If all byte are abnormal, output error code
	if((fpga_info[5] == FPGA_DEFULT_RETURN_VALUE) && (fpga_info[6] == FPGA_DEFULT_RETURN_VALUE) && (fpga_info[7] == FPGA_DEFULT_RETURN_VALUE))
	{
		// user can not distinguish this is I2C2 failed or FPGA unlock failed.
		usb_err_code = USB_FPGA_RW_FAILED;
		aewin_dbg("Access FPGA failed.\r\n");

		// [test] unlock failed, does not need to execute other test
		usb_read_flag = 1;
		err_record |= (1<<MASK_ERR_I2C);
	}

	//--------------------------------
	// set RTC time to FPGA
	// enable FPGA timer
	fpga_timer_switch = FPGA_TIMER_ENABLE;
	i2c2_fpga_write(FPGA_TIMER_SWITCH_ADDR, FPGA_TIMER_SWITCH_SIZE, &(fpga_timer_switch));
	//aewin_dbg("Enable FPGA timer.\r\n");

	// write MCU RTC time to FPGA
	RTC_bin_format[0] = bcd2bin_byte(RTC_Time.Seconds);
	RTC_bin_format[1] = bcd2bin_byte(RTC_Time.Minutes);
	RTC_bin_format[2] = bcd2bin_byte(RTC_Time.Hours);
	RTC_bin_format[3] = bcd2bin_byte(RTC_Date.Date);
	RTC_bin_format[4] = bcd2bin_byte(RTC_Date.Month);
	RTC_bin_format[5] = bcd2bin_byte(RTC_Date.Year);
	i2c2_fpga_write(FPGA_TIMER_BASE_ADDR, FPGA_TIMER_SIZE, (uint8_t*)RTC_bin_format);
	//aewin_dbg("Set RTC time to FPGA timer : 20%02d.%02d.%02d - %02d:%02d:%02d\r\n", \
			RTC_bin_format[5],RTC_bin_format[4], RTC_bin_format[3], RTC_bin_format[2], RTC_bin_format[1], RTC_bin_format[0]);

	// disable FPGA timer
	fpga_timer_switch = FPGA_TIMER_DISABLE;
	i2c2_fpga_write(FPGA_TIMER_SWITCH_ADDR, FPGA_TIMER_SWITCH_SIZE, &(fpga_timer_switch));
	//aewin_dbg("Disable FPGA timer.\r\n");
	//--------------------------------

  /*Start the TIM Base generation in interrupt mode ####################*/
    HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    //aewin_dbg("===== Enter while loop =====\r\n");
  while (1)
  {

  /* USER CODE END WHILE */
    MX_USB_HOST_Process();

  /* USER CODE BEGIN 3 */
    // set PG6 high
    //HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);

    //----------------------------------------------------------
	/*================== 1s Routine ================== */
	if (time_states & flag_1s){
		time_states &= ~flag_1s; // Clear flag.

		if(Appli_state == APPLICATION_READY)
		{
			// check FPGA busy status
			// Read FPGA Busy byte and status1 byte
			i2c2_fpga_read(FPGA_BUSY_STATUS_BASE_ADDR, FPGA_BUSY_STATUS_SIZE, (uint8_t*)fpga_busy_status);
		}

		// print RTC time
//		HAL_RTC_GetTime(&hrtc, &RTC_Time, RTC_FORMAT_BCD);
//		HAL_RTC_GetDate(&hrtc, &RTC_Date, RTC_FORMAT_BCD);
//		aewin_dbg("RTC : 20%02x.%02x.%02x - %02x:%02x:%02x\r\n", \
//				RTC_Date.Year, RTC_Date.Month, RTC_Date.Date, RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);
//		aewin_dbg("----------------------------\r\n");
	}

	/*================== 500ms Routine ================== */
	if (time_states & flag_500ms){
		time_states &= ~flag_500ms; // Clear flag.
	}

	/*================== 500ms Routine ================== */
	if (time_states & flag_100ms){
		time_states &= ~flag_100ms; // Clear flag.

		// [debug]
		HAL_GPIO_TogglePin(FM_MCU_HBLED_GPIO_Port, FM_MCU_HBLED_Pin);
	}
    //-----------------------------------------------------------------------------------------------

	// Check USB status and do file operations only if aewin_file.txt in USB disk haven't been read
    if(usb_read_flag == 0)
    {
    	if(Appli_state == APPLICATION_READY)
		{
    		//[remove]
    		// reset timeout_counter
    		timeout_counter = 0;
    		timeout_counter_switch = 0;

    		// check if FPGA is not busy - verify fpga_busy_status value(Busy and TPCM)
			if( (fpga_busy_status[0] == 0) && (fpga_busy_status[1] == 0) )
			{
				// print RTC time
				aewin_dbg("-----------------------------\r\n");
				aewin_dbg("-----   Start Testing   -----\r\n");
				aewin_dbg("-----------------------------\r\n");
				HAL_RTC_GetTime(&hrtc, &RTC_Time, RTC_FORMAT_BCD);
				HAL_RTC_GetDate(&hrtc, &RTC_Date, RTC_FORMAT_BCD);
				aewin_dbg("RTC : 20%02x.%02x.%02x - %02x:%02x:%02x\r\n", \
						RTC_Date.Year, RTC_Date.Month, RTC_Date.Date, RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);
				aewin_dbg("-----------------------------\r\n");

				usb_read_flag = 1;

				// read aewin_file.txt to check user command code
				//USB_MSC_File_Operations(USB_EXE_READ_CMD);
				//fpga_spi_mode = usb_cmd_flash_num;

				// [test]
				// 1. read log, output to uart and save to usb
				aewin_dbg("1. Read log from FPGA.\r\n");
				USB_MSC_File_Operations(USB_CMD_READ_LOG);
				aewin_dbg("-----------------------------\r\n");

				// 2. read 1 page from each flash
				aewin_dbg("2. Read 512 byte data from each flash.\r\n");

				for(fpga_spi_mode = 0; fpga_spi_mode < 4; fpga_spi_mode++)
				{
					// select target flash
					i2c2_fpga_write(FPGA_SPI_MODE_ADDR, FPGA_SPI_MODE_SIZE, &(fpga_spi_mode));
					aewin_dbg("\r\nRead data from flash %d.\r\n", fpga_spi_mode);

					// enable FPGA SPI
					fpga_spi_switch = FPGA_SPI_SWITCH_ON;
					i2c2_fpga_write(FPGA_SPI_SWITCH_ADDR, FPGA_SPI_SWITCH_SIZE, &(fpga_spi_switch));
					//aewin_dbg("Enable FPGA SPI.\r\n");

					// read flash
					usb_cmd_flash_num = fpga_spi_mode;
					USB_MSC_File_Operations(USB_CMD_READ_FLASH);

					// disable FPGA SPI
					fpga_spi_switch = FPGA_SPI_SWITCH_OFF;
					i2c2_fpga_write(FPGA_SPI_SWITCH_ADDR, FPGA_SPI_SWITCH_SIZE, &(fpga_spi_switch));
					//aewin_dbg("Disable FPGA SPI.\r\n");
				}

				// de-select target flash
				fpga_spi_mode = FLASH_NONE;
				i2c2_fpga_write(FPGA_SPI_MODE_ADDR, FPGA_SPI_MODE_SIZE, &(fpga_spi_mode));
				//aewin_dbg("Select flash back to 0.\r\n", fpga_spi_mode);


				// [debug] - specify user command code
				//usb_cmd_code = USB_CMD_READ_LOG;

//				switch(usb_cmd_code)
//				{
//					case USB_CMD_NONE:
//						//none
//						break;
//
//					case USB_CMD_READ_LOG:
//						aewin_dbg("Get command: Read log from FPGA.\r\n");
//						USB_MSC_File_Operations(USB_CMD_READ_LOG);
//
//						// read flash data to USB disk
//						if(usb_cmd_re_read_flash == RE_READ_FLASH_ON)
//						{
//							if(backup_flag[BACKUP_FLAG_BIOS] == RE_READ_FLASH_ON)
//							{
//								// select target flash
//								fpga_spi_mode = FLASH_BIOS_BACKUP;
//								i2c2_fpga_write(FPGA_SPI_MODE_ADDR, FPGA_SPI_MODE_SIZE, &(fpga_spi_mode));
//								aewin_dbg("Select flash %d for reading.\r\n", fpga_spi_mode);
//
//								// enable FPGA SPI
//								fpga_spi_switch = FPGA_SPI_SWITCH_ON;
//								i2c2_fpga_write(FPGA_SPI_SWITCH_ADDR, FPGA_SPI_SWITCH_SIZE, &(fpga_spi_switch));
//								aewin_dbg("Enable FPGA SPI.\r\n");
//
//								// read flash
//								usb_cmd_flash_num = FLASH_BIOS_BACKUP;
//								USB_MSC_File_Operations(USB_CMD_READ_FLASH);
//							}
//							if(backup_flag[BACKUP_FLAG_BMC] == RE_READ_FLASH_ON)
//							{
//								// select target flash
//								fpga_spi_mode = FLASH_BMC_BACKUP;
//								i2c2_fpga_write(FPGA_SPI_MODE_ADDR, FPGA_SPI_MODE_SIZE, &(fpga_spi_mode));
//								aewin_dbg("Select flash %d for reading.\r\n", fpga_spi_mode);
//
//								// enable FPGA SPI
//								fpga_spi_switch = FPGA_SPI_SWITCH_ON;
//								i2c2_fpga_write(FPGA_SPI_SWITCH_ADDR, FPGA_SPI_SWITCH_SIZE, &(fpga_spi_switch));
//								aewin_dbg("Enable FPGA SPI.\r\n");
//
//								// read flash
//								usb_cmd_flash_num = FLASH_BMC_BACKUP;
//								USB_MSC_File_Operations(USB_CMD_READ_FLASH);
//							}
//
//							// disable FPGA SPI
//							fpga_spi_switch = FPGA_SPI_SWITCH_OFF;
//							i2c2_fpga_write(FPGA_SPI_SWITCH_ADDR, FPGA_SPI_SWITCH_SIZE, &(fpga_spi_switch));
//							aewin_dbg("Disable FPGA SPI.\r\n");
//
//							// de-select target flash
//							fpga_spi_mode = FLASH_NONE;
//							i2c2_fpga_write(FPGA_SPI_MODE_ADDR, FPGA_SPI_MODE_SIZE, &(fpga_spi_mode));
//							aewin_dbg("Select flash back to 0.\r\n", fpga_spi_mode);
//						}
//
//						break;
//
//					case USB_CMD_UPDATE_IMA:
//						aewin_dbg("Get command: Update flash.\r\n");
//						aewin_dbg("ima file name: %s\r\n", usb_cmd_ima_filename);
//
//						// check flash number
//						if(fpga_spi_mode > FLASH_NUM)
//						{
//							// flash number is out of range, stopping execute user command
//							aewin_dbg("Flash number %d is not exist, please try number 1~4.\r\n", fpga_spi_mode);
//							usb_err_code = USB_ERR_FLASH_NOT_EXIST;
//							break;
//						}
//
//						// check ima file name
//						if(usb_cmd_ima_filename[FILE_PATH_HEAD_LEN + 1] == '\0')
//						{
//							aewin_dbg("The .ima file name not exist.\r\n");
//							usb_err_code = USB_ERR_IMA_NOT_EXIST;
//							break;
//						}
//
//						// enable FPGA SPI
//						fpga_spi_switch = FPGA_SPI_SWITCH_ON;
//						i2c2_fpga_write(FPGA_SPI_SWITCH_ADDR, FPGA_SPI_SWITCH_SIZE, &(fpga_spi_switch));
//						aewin_dbg("Enable FPGA SPI.\r\n");
//
//						// select target flash
//						i2c2_fpga_write(FPGA_SPI_MODE_ADDR, FPGA_SPI_MODE_SIZE, &(fpga_spi_mode));
//						aewin_dbg("Select flash %d for update.\r\n", fpga_spi_mode);
//
//						// update flash - write .ima file to flash
//						USB_MSC_File_Operations(USB_CMD_UPDATE_IMA);
//
//						// disable FPGA SPI
//						fpga_spi_switch = FPGA_SPI_SWITCH_OFF;
//						i2c2_fpga_write(FPGA_SPI_SWITCH_ADDR, FPGA_SPI_SWITCH_SIZE, &(fpga_spi_switch));
//						aewin_dbg("Disable FPGA SPI.\r\n");
//
//						// de-select target flash
//						fpga_spi_mode = FLASH_NONE;
//						i2c2_fpga_write(FPGA_SPI_MODE_ADDR, FPGA_SPI_MODE_SIZE, &(fpga_spi_mode));
//						aewin_dbg("Select flash back to 0.\r\n", fpga_spi_mode);
//
//						break;
//
//					case USB_CMD_READ_FLASH:
//						aewin_dbg("Get command: Read flash data.\r\n");
//
//						// check flash number
//						if(fpga_spi_mode > FLASH_NUM)
//						{
//							// flash number is out of range, stopping execute user command
//							aewin_dbg("Flash number %d is out of range, please try number 1~4.\r\n", fpga_spi_mode);
//							usb_err_code = USB_ERR_FLASH_NOT_EXIST;
//							break;
//						}
//
//						// select target flash
//						i2c2_fpga_write(FPGA_SPI_MODE_ADDR, FPGA_SPI_MODE_SIZE, &(fpga_spi_mode));
//						aewin_dbg("Select flash %d for reading.\r\n", fpga_spi_mode);
//
//						// enable FPGA SPI
//						fpga_spi_switch = FPGA_SPI_SWITCH_ON;
//						i2c2_fpga_write(FPGA_SPI_SWITCH_ADDR, FPGA_SPI_SWITCH_SIZE, &(fpga_spi_switch));
//						aewin_dbg("Enable FPGA SPI.\r\n");
//
//						// read flash
//						USB_MSC_File_Operations(USB_CMD_READ_FLASH);
//
//						// disable FPGA SPI
//						fpga_spi_switch = FPGA_SPI_SWITCH_OFF;
//						i2c2_fpga_write(FPGA_SPI_SWITCH_ADDR, FPGA_SPI_SWITCH_SIZE, &(fpga_spi_switch));
//						aewin_dbg("Disable FPGA SPI.\r\n");
//
//						// de-select target flash
//						fpga_spi_mode = FLASH_NONE;
//						i2c2_fpga_write(FPGA_SPI_MODE_ADDR, FPGA_SPI_MODE_SIZE, &(fpga_spi_mode));
//						aewin_dbg("Select flash back to 0.\r\n", fpga_spi_mode);
//
//						break;
//
//					//------------------------------
//					// below are test command
//					//------------------------------
//
//					case USB_CMD_TEST_RW:
//						aewin_dbg("Get command: Test read/writ function.\r\n");
//						USB_MSC_File_Operations(USB_CMD_TEST_RW);
//
//						break;
//
//					default:
//						usb_err_code = USB_ERR_CMD_NOT_EXIST;
//						aewin_dbg("!! This command is not exist!\r\n");
//
//						break;
//				}
			}

			// output error code
			if(usb_err_code != USB_ERR_NONE)
			{
				USB_MSC_File_Operations(USB_EXE_ERROR_REPORT);
				// [note] - print error massage
			}
		}
    	else
    	{
    		//[Note]
			// add start timer
			if(timeout_counter_switch == 1)
			{
				// add timeout handler, usb re-init and init
				//aewin_dbg("USB timeout_counter: %d! \r\n", timeout_counter);
				if(timeout_counter > USB_TIMEOUT_LIMIT)
				{
					aewin_dbg("USB timeout! software reset.\r\n");
					//HAL_TIM_Base_Stop_IT(&htim3);
					//MX_USB_HOST_Init();
					//MX_FATFS_Init();
					NVIC_SystemReset();
					//USBH_LL_DeInit(&hUsbHostFS);
					//USBH_LL_Init(&hUsbHostFS);
					//USBH_ReEnumerate(&hUsbHostFS);
					//HAL_TIM_Base_Start_IT(&htim3);
					//aewin_dbg("Reset USB_HOST finished.\r\n");

					// reset timeout counter
					timeout_counter_switch = 0;
					timeout_counter = 0;
				}
			}
    	}
    }
    else if(usb_read_flag == 1)
    {
    	aewin_dbg("===== test result =====\r\n");

    	if(err_record & (1<<MASK_ERR_I2C) != 1)
    	{
    		aewin_dbg("I2C  test ...... failed\r\n");
    	}
    	else
    	{
    		aewin_dbg("I2C  test ........ pass\r\n");
    	}

    	// don't know what means FSMC fail/pass
    	if(err_record & (1<<MASK_ERR_FSMC) != 1)
		{
			aewin_dbg("FSMC test ...... failed\r\n");
		}
		else
		{
			aewin_dbg("FSMC test ........ pass\r\n");
		}

    	if(err_record & (1<<MASK_ERR_SPI) != 1)
		{
			aewin_dbg("SPI  test ...... failed\r\n");
		}
		else
		{
			aewin_dbg("SPI  test ........ pass\r\n");
		}

    	if(err_record & (1<<MASK_ERR_USB) != 1)
		{
			aewin_dbg("USB  test ...... failed\r\n");
		}
		else
		{
			aewin_dbg("USB  test ........ pass\r\n");
		}

    	aewin_dbg("=======================\r\n");
    	usb_read_flag = 2;
    }
    else if(Appli_state == APPLICATION_DISCONNECT)
	{
		// when USB disconnect, clear read flag
    	//if(usb_read_flag == 1)
		{
			usb_read_flag = 0;

			// reset timeout counter
			timeout_counter_switch = 0;
			timeout_counter = 0;

			// reset user command
			usb_cmd_code = USB_CMD_NONE;
			usb_cmd_flash_num = FLASH_NONE;
			memset(usb_cmd_ima_filename , '\0', sizeof(usb_cmd_ima_filename));
			usb_cmd_re_read_flash = RE_READ_FLASH_OFF;
			usb_err_code = USB_ERR_NONE;
			backup_flag[0] = RE_READ_FLASH_OFF;
			backup_flag[1] = RE_READ_FLASH_OFF;
		}
	}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
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

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 72;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 12;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 3;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLI2SQ;
  PeriphClkInitStruct.PLLI2SSelection = RCC_PLLI2SCLKSOURCE_PLLSRC;
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
void aewin_dbg(char *fmt,...){
	int i = 0;
	static va_list arg_ptr;
	va_start (arg_ptr, fmt);
	vsnprintf(dbg_buff, PRINT_BUFF, fmt, arg_ptr);
	while(i < (PRINT_BUFF - 1) && dbg_buff[i]){
		//if (HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&dbg_buff[i], 1) == HAL_OK){
		if (HAL_UART_Transmit(&huart2, (uint8_t*)&dbg_buff[i], 1, 5000) == HAL_OK){
			i++;
		}
	}
	va_end(arg_ptr);
}

void i2c2_fpga_write(char base_addr, char data_len, char *pData)
{
	char local_index;
	char write_buffer[2];

	for(local_index = 0; local_index < data_len; local_index++)
	{
		write_buffer[0] = base_addr + local_index;
		write_buffer[1] = pData[local_index];
		if(HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)I2C2_FPGA_ADDR, (uint8_t*)write_buffer, 2, 10000) != HAL_OK)
		{
			usb_err_code = USB_FPGA_RW_FAILED;
			err_record |= (1 << MASK_ERR_I2C);
			return;
		}
	}
}

void i2c2_fpga_read(char base_addr, char data_len, char *pData)
{
	char local_index;
	char fpga_read_addr;
	char read_buffer;

	for(local_index = 0; local_index < data_len; local_index++)
	{
		fpga_read_addr = base_addr + local_index;
		if(HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)I2C2_FPGA_ADDR, &(fpga_read_addr), 1, 10000) != HAL_OK)
		{
			usb_err_code = USB_FPGA_RW_FAILED;
			err_record |= (1 << MASK_ERR_I2C);
			return;
		}

		if(HAL_I2C_Master_Receive(&hi2c2, (uint16_t)I2C2_FPGA_ADDR, &(read_buffer), 1, 10000) != HAL_OK)
		{
			usb_err_code = USB_FPGA_RW_FAILED;
			err_record |= (1 << MASK_ERR_I2C);
			return;
		}
		pData[local_index] = read_buffer;
	}

}

unsigned char bcd2bin_byte(unsigned char bcd_num)
{
	return ((bcd_num & 0xF0)>>4)*10 + (bcd_num & 0x0F);
}
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
