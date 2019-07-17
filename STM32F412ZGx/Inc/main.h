/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define FM_MCU_HBLED_Pin GPIO_PIN_8
#define FM_MCU_HBLED_GPIO_Port GPIOC

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
// --------------------------------
// Firmware version
// --------------------------------
#define VER_MAJOR 0
#define VER_MINOR 0
#define VER_PATCH 0


// --------------------------------
// USB Information
// --------------------------------
typedef enum{
  USB_CMD_NONE         = 0x00, /*!< Specifies the command for none. */
  USB_CMD_READ_LOG     = 0x01, /*!< Specifies the command for read log and save to USB disk. */
  USB_CMD_UPDATE_IMA   = 0x02, /*!< Specifies the command for update flash with .ima file in USB disk. */
  USB_CMD_READ_FLASH,          /*!< Specifies the command for read flash. */
  USB_CMD_ERASE_FLASH,         /*!< Specifies the command for erase flash. */
  USB_CMD_TEST_RW,             /*!< Specifies the command for function test, write and read a .txt file. */
  USB_EXE_READ_CMD     = 0xE1, /*!< Specifies the operation for read user command. */
  USB_EXE_ERROR_REPORT         /*!< Specifies the operation for generate a error report. */
}eUSB_Cmds;

// --------------------------------
// Error Code
// --------------------------------
typedef enum{
  USB_ERR_NONE           = 0x00, /*!< No error happened. */
  USB_ERR_FILE_RW_FAILED = 0x01, /*!< Read/Write/Open file failed. */
  USB_ERR_FILE_WRONG_FORMAT,     /*!< aewin_file.txt is in wrong format. */
  USB_ERR_CMD_NOT_EXIST,         /*!< User command is not exist. */
  USB_FPGA_RW_FAILED,            /*!< Read/write FPGA failed. */
  USB_ERR_IMA_NOT_EXIST,         /*!< .ima file is not in the USB disk. */
  USB_ERR_FLASH_NOT_EXIST,       /*!< Specifies flash number is not exist. */
  USB_ERR_FLASH_UPDATE_FAILED,   /*!< Flash update failed. */
  USB_ERR_LOG_RW_FAILED          /*!< Read/Write log failed. */
}eUSB_ErrorCodes;

// --------------------------------
// IMA File Information
// --------------------------------
#define IMA_FILE_SIZE          32*1024*1024  // 32MB
#define IMA_FILENAME_LEN_LIMIT 30
#define IMA_FILE_PATH_HEAD_LEN 2             // 0:

#define IMA_FILE_NONE          0             // user didn't give ima file name
#define IMA_FILE_TAG           0x5A

// --------------------------------
// I2C Information
// --------------------------------
#define I2C2_FPGA_ADDR 0xB0                  //FPGA device address

// --------------------------------
// FPGA Information
// --------------------------------
#define FPGA_KEY_SIZE         8
#define FPGA_INFO_SIZE        6
#define FPGA_SPI_SWITCH_SIZE  1
#define FPGA_SPI_MODE_SIZE    1
#define FPGA_BUSY_STATUS_SIZE 4

#define FPGA_KEY_BASE_ADDR         0x00
#define FPGA_INFO_BASE_ADDR        0x08
#define FPGA_SPI_SWITCH_ADDR       0x10
#define FPGA_SPI_MODE_ADDR         0x11
#define FPGA_BUSY_STATUS_BASE_ADDR 0x14

#define FPGA_DEFULT_RETURN_VALUE   0xFF     // if FPGA unlock failed, all command will get the same return value

// fpga - SPI EN
#define FPGA_SPI_SWITCH_ON   1
#define FPGA_SPI_SWITCH_OFF  0

#define FPGA_DATA_BYTE_SIZE  2    // read 16bit at 1 time
#define FPGA_DATA_NUM        32
#define FPGA_LOG_NUM         30
#define FPGA_DATA_BYTE_NUM   256  // byte
#define FPGA_DATA_SKIP_NUM   8    // first 2 data is not log
#define FPGA_LOG_BYTE_NUM    (FPGA_DATA_BYTE_NUM - FPGA_DATA_SKIP_NUM * FPGA_DATA_BYTE_SIZE)

// fpga - log
typedef enum{
  FPGA_LOG_BIOS    = 0, /*!< BIOS Number in FPGA log*/
  FPGA_LOG_BMC     = 1, /*!< BMC Number in FPGA log*/
  FPGA_LOG_YEAR,        /*!< Year Number in FPGA log*/
  FPGA_LOG_MONTH,       /*!< Month Number in FPGA log*/
  FPGA_LOG_DAY,         /*!< Day Number in FPGA log*/
  FPGA_LOG_HOUR,        /*!< Hour Number in FPGA log*/
  FPGA_LOG_MINUTE,      /*!< Minute Number in FPGA log*/
  FPGA_LOG_STATE,       /*!< State Number in FPGA log*/
  FPGA_LOG_SIZE         /*!< Size of 1 log in FPGA*/
}eFPGA_LOG_NUMs;

#define FPGA_LOG_EMPTY      0xff
#define FPGA_LOG_STATE_DONE 0x01
#define FPGA_LOG_OK         0x01
#define FPGA_LOG_RE         0x02



#define FPGA_FSMC_BASE_ADDR  0x60000000
#define FPGA_FSMC_LOG_ADDR   (FPGA_FSMC_BASE_ADDR + FPGA_DATA_SKIP_NUM*4)


// --------------------------------
// FLASH Information
// --------------------------------
#define FLASH_NUM            4
#define FLASH_NONE           0 // user didn't give flash number
#define FLASH_BIOS           1 //wait fpga define
#define FLASH_BMC            2 //wait fpga define
#define FLASH_BIO_BACKUP     3 //wait fpga define
#define FLASH_BMC_BACKUP     4 //wait fpga define

// --------------------------------
// SPI Information
// --------------------------------
#define SPI_READ_BUFFER_SIZE  256           // advice 256byte, test 1024 byte
#define SPI_WRITE_BUFFER_SIZE 256           // advice 256byte, test 2048 byte
#define SPI_READ_LOOP_LIMIT   IMA_FILE_SIZE/SPI_READ_BUFFER_SIZE
//#define SPI_WRITE_LOOP_LIMIT  IMA_FILE_SIZE/SPI_WRITE_BUFFER_SIZE

// --------------------------------
// Flash commands - MT25QL512ABB
// --------------------------------
#define FLASH_CMD_READ               0x13
#define FLASH_CMD_READ_STATUS        0x05
#define FLASH_CMD_WRITE_ENABLE       0x06
#define FLASH_CMD_WRITE_DISABLE      0x04
#define FLASH_CMD_CLEAR_FLAG         0x50
#define FLASH_CMD_WRITE_STATUS       0x01
#define FLASH_CMD_BULK_ERASE         0xC7
#define FLASH_CMD_PAGE_PROGRAM       0x12

#define FLASH_ROW_DATA_LIMIT         16
#define FLASH_DATA_BYTE              2
#define FLASH_SPACE_BYTE             1
//#define FLASH_NEWLINE_BYTE           1
//#define FLASH_RETURN_BYTE            1
#define FLASH_WRITE_ROW_NUM          1    // [user config]1,2,4,(8,16)



/** @defgroup Timer_Flag_Enumeration_definition Timer Flags definition
  * @brief  Timer Flags definition
  * @{
  */
typedef enum{
  flag_idle   = 0x01, /*!< Specifies the timer "idle" states. This bit-masking is used to set/clear time_states when timer is idle. */
  flag_1ms    = 0x02, /*!< Specifies the timer "1ms" states. This bit-masking is used to set/clear time_states in every 1ms. */
  flag_100ms  = 0x04, /*!< Specifies the timer "10ms" states. This bit-masking is used to set/clear time_states in every 100ms. */
  flag_500ms  = 0x08, /*!< Specifies the timer "100ms" states. This bit-masking is used to set/clear time_states in every 500ms. */
  flag_1s     = 0x10  /*!< Specifies the timer "1s" states. This bit-masking is used to set/clear time_states in every 1s. */
}eTimer_Flags;


 /** @defgroup AEWIN_DEBUG Aewin Debug Configuration
   * @{
   */
#define AEWIN_DBUG			(1)
#define PRINT_BUFF			(128)

void USB_MSC_File_Operations(unsigned char command_type);
void MSC_File_Operations(void);
void flash_check_status_reg(char target_value);
void flash_write_enable(void);
void flash_bulk_erase(void);
void flash_clear_flag_status_reg(void);
void flash_write_status_reg(char write_value);
void flash_unlock(void);
void flash_erase(void);

void aewin_dbg(char *fmt,...);
void i2c2_fpga_write(char base_addr, char data_len, char *pData);
void i2c2_fpga_read(char base_addr, char data_len, char *pData);
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
