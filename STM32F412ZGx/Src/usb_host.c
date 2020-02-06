/**
  ******************************************************************************
  * @file            : usb_host.c
  * @version         : v1.0_Cube
  * @brief           : This file implements the USB Host
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2020 STMicroelectronics International N.V. 
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

#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_msc.h"

/* USER CODE BEGIN Includes */
#include "fatfs.h"
#include "main.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
unsigned char debug_mount = 0;

extern volatile unsigned char timeout_counter;
extern unsigned char timeout_counter_switch;

//[debug]
//extern uint8_t flag_test_write;
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Host core handle declaration */
USBH_HandleTypeDef hUsbHostFS;
ApplicationTypeDef Appli_state = APPLICATION_IDLE;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
 * user callback declaration
 */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Init USB host library, add supported class and start the library
  * @retval None
  */
void MX_USB_HOST_Init(void)
{
  /* USER CODE BEGIN USB_HOST_Init_PreTreatment */
  
  /* USER CODE END USB_HOST_Init_PreTreatment */
  
  /* Init host Library, add supported class and start the library. */
  USBH_Init(&hUsbHostFS, USBH_UserProcess, HOST_FS);

  USBH_RegisterClass(&hUsbHostFS, USBH_MSC_CLASS);

  USBH_Start(&hUsbHostFS);

  /* USER CODE BEGIN USB_HOST_Init_PostTreatment */
  
  /* USER CODE END USB_HOST_Init_PostTreatment */
}

/*
 * Background task
 */
void MX_USB_HOST_Process(void)
{
  /* USB Host Background task */
  USBH_Process(&hUsbHostFS);
}
/*
 * user callback definition
 */
static void USBH_UserProcess  (USBH_HandleTypeDef *phost, uint8_t id)
{
  /* USER CODE BEGIN CALL_BACK_1 */

  switch(id)
  {
  case HOST_USER_SELECT_CONFIGURATION:
	  //[debug]
	  aewin_dbg("USBH_UserProcess - HOST_USER_SELECT_CONFIGURATION\r\n");
	break;

  case HOST_USER_DISCONNECTION:
	Appli_state = APPLICATION_DISCONNECT;

	// [debug]
	//flag_test_write = 0;
	aewin_dbg("USBH_UserProcess - HOST_USER_DISCONNECTION\r\n");

	/*
	if(f_mount(NULL, "", 1) != FR_OK)
	{
	  //LCD_ErrLog("ERROR : Cannot DeInitialize FatFs! \n");
		aewin_dbg("ERROR : Cannot DeInitialize FatFs!\r\n");
	}
	if (FATFS_UnLinkDriver(USBHPath) != 0)
	{
	  //LCD_ErrLog("ERROR : Cannot UnLink USB FatFS Driver! \n");
		aewin_dbg("ERROR : Cannot UnLink USB FatFS Driver!\r\n");
	}
	*/

	if (FATFS_UnLinkDriver(USBHPath) == 0)
	{
	  if(f_mount(NULL, "", 0) != FR_OK)
	  {
		  aewin_dbg("ERROR : Cannot DeInitialize FatFs! \r\n");
	  }
	}
	break;

  case HOST_USER_CLASS_ACTIVE:
	Appli_state = APPLICATION_READY;

	//[debug]
	aewin_dbg("USBH_UserProcess - HOST_USER_CLASS_ACTIVE\r\n");
	break;

  case HOST_USER_CONNECTION:
	Appli_state = APPLICATION_START;

	//[debug]
	aewin_dbg("USBH_UserProcess - HOST_USER_CONNECTION\r\n");

	/*
	if(FATFS_GetAttachedDriversNbr() == 0)
	{
		FATFS_LinkDriver(&USBH_Driver, USBHPath);
	}

	if (f_mount(&USBHFatFS, "", 0) != FR_OK)
	{
		debug_mount = 1;
		//[debug]
		aewin_dbg("mount failed\r\n");
	}
	else
	{
		debug_mount = 2;
		//[debug]
		aewin_dbg("mount OK\r\n");
	}
	*/

	if (FATFS_LinkDriver(&USBH_Driver, USBHPath) == 0)
	{
	  if (f_mount(&USBHFatFS, "", 0) != FR_OK)
	  {
		  aewin_dbg("ERROR : Cannot Initialize FatFs! \r\n");
	  }
	}

	//[Note]
	// add start timer
	if(timeout_counter_switch == 0)
	{
		timeout_counter = 0;
		timeout_counter_switch = 1;
	}
	else
	{
		// add timeout handler, usb re-init and init
		aewin_dbg("USB timeout_counter: %d! \r\n", timeout_counter);
		if(timeout_counter > USB_TIMEOUT_LIMIT)
		{
			aewin_dbg("USB timeout! Reset USB_HOST.\r\n");
			MX_USB_HOST_Init();

			timeout_counter_switch = 0;
			timeout_counter = 0;
		}
	}

	break;

  default:
	break;
  }


  /*
  switch(id)
  {
  case HOST_USER_SELECT_CONFIGURATION:
  break;

  case HOST_USER_DISCONNECTION:
  Appli_state = APPLICATION_DISCONNECT;
  if(f_mount(NULL, "", 0) != FR_OK)
  {
	//LCD_ErrLog("ERROR : Cannot DeInitialize FatFs! \n");
  }
  break;

  case HOST_USER_CLASS_ACTIVE:
  Appli_state = APPLICATION_READY;
  break;

  case HOST_USER_CONNECTION:
  Appli_state = APPLICATION_START;
  if(f_mount(&USBHFatFS, "", 0) != FR_OK)
  {
	//LCD_ErrLog("ERROR : Cannot Initialize FatFs! \n");
	  debug_mount = 1;
  }
  else
  {
	  debug_mount = 2;
  }
  break;

  default:
  break;
  }
  */

  /* USER CODE END CALL_BACK_1 */
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
