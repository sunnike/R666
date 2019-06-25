/**
  ******************************************************************************
  * @file    USB_Host/MSC_Standalone/Src/file_operations.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    17-February-2017
  * @brief   Write/read file on the disk.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright � 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
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


#include "ff.h"
#include "spi.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
FATFS USBH_fatfs;
FIL MyFile;
FRESULT res;
uint32_t bytesWritten;
uint8_t rtext[SPI_WRITE_BUFFER_SIZE];
uint8_t wtext[] = "USB Host Library : Mass Storage Example";
FRESULT res_write, res_read;
FIL WriteFile, ReadFile;

uint8_t usb_wtext_error_msg[] = "error code:";
uint8_t usb_rtext_file_cmd[] = "command:";
uint8_t usb_rtext_file_flash_num[] = "flash_number:";
uint8_t usb_rtext_file_ima_name[] = "ima_file";
uint8_t usb_rtext_buffer[sizeof(usb_rtext_file_ima_name)+IMA_FILENAME_LEN_LIMIT];

uint32_t loop_index;
uint8_t blank_counter;

// ---------------
// flash commands
//----------------
uint8_t flash_cmd_read[] = {FLASH_CMD_READ, 0x00, 0x00, 0x00, 0x00};             // fast read data: {command, ADD1, ADD2, ADD3, Dummy}
uint8_t flash_cmd_program[] = {FLASH_CMD_PAGE_PROGRAM, 0x00, 0x00, 0x00, 0x00};
uint8_t flash_cmd_write_status[] = {FLASH_CMD_WRITE_STATUS, 0x00};
uint8_t flash_cmd_read_status[] = {FLASH_CMD_READ_STATUS};
uint8_t flash_cmd_write_enable[] = {FLASH_CMD_WRITE_ENABLE};
uint8_t flash_cmd_clear_flag[] = {FLASH_CMD_CLEAR_FLAG};
uint8_t flash_cmd_bulk_erase[] = {FLASH_CMD_BULK_ERASE};

volatile uint8_t flash_data_read[SPI_READ_BUFFER_SIZE];
volatile uint8_t flash_data_read_byte[1];

uint32_t flash_program_address = 0;

//---------------
// USB variables
//---------------
uint8_t usb_file_name[IMA_FILE_PATH_HEAD_LEN + IMA_FILENAME_LEN_LIMIT] = "0:USBHost_RWtest.txt";
uint8_t usb_aewin_file_name[IMA_FILE_PATH_HEAD_LEN + IMA_FILENAME_LEN_LIMIT] = "0:aewin_file.txt";

uint8_t usb_ima_file_path[IMA_FILE_PATH_HEAD_LEN + IMA_FILENAME_LEN_LIMIT] = "0:";

extern uint8_t usb_err_code;
extern uint8_t usb_cmd_code;
extern uint8_t usb_cmd_flash_num;
extern uint8_t usb_cmd_ima_filename[IMA_FILENAME_LEN_LIMIT];

//debug
uint8_t flag_RWfailed = 0;
uint8_t flash_test_data[SPI_READ_BUFFER_SIZE];

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Files operations: Read/Write and compare
  * @param  None
  * @retval None
  */
void MSC_File_Operations(void)
{

  	uint16_t bytesread;

	//LCD_UsrLog("INFO : FatFs Initialized \n");

	if(f_open(&MyFile, "0:USBHost.txt",FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
	{
	  //LCD_ErrLog("Cannot Open 'USBHost.txt' file \n");
	}
	else
	{
	  //LCD_UsrLog("INFO : 'USBHost.txt' opened for write  \n");
	  res= f_write (&MyFile, wtext, sizeof(wtext), (void *)&bytesWritten);
	  f_close(&MyFile);

	  if((bytesWritten == 0) || (res != FR_OK)) /*EOF or Error*/
	  {
		//LCD_ErrLog("Cannot Write on the  'USBHost.txt' file \n");
	  }
	  else
	  {
		if(f_open(&MyFile, "0:USBHost.txt", FA_READ) != FR_OK)
		{
		  //LCD_ErrLog("Cannot Open 'USBHost.txt' file for read.\n");
		}
		else
		{
		  //LCD_UsrLog("INFO : Text written on the 'USBHost.txt' file \n");

		  res = f_read(&MyFile, rtext, sizeof(rtext), (void *)&bytesread);

		  if((bytesread == 0) || (res != FR_OK)) /*EOF or Error*/
		  {
			//LCD_ErrLog("Cannot Read from the  'USBHost.txt' file \n");
		  }
		  else
		  {
			//LCD_UsrLog("Read Text : \n");
			//LCD_DbgLog((char *)rtext);
			//LCD_DbgLog("\n");
		  }
		  f_close(&MyFile);
		}
		/* Compare read data with the expected data */
		if((bytesread == bytesWritten))
		{
		  //LCD_UsrLog("INFO : FatFs data compare SUCCES");
		  //LCD_UsrLog("\n");
		}
		else
		{
		  //LCD_ErrLog("FatFs data compare ERROR");
		  //LCD_ErrLog("\n");
		}
	  }
	}
	//----------------------------------------------------------------

	// read flash MX25L12835F

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

	if(HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_cmd_read, sizeof(flash_cmd_read), 5000) == HAL_OK)
	{
		//for(loop_index = 0; loop_index < SPI_READ_LOOP_LIMIT; loop_index++)
		for(loop_index = 0; loop_index < 5; loop_index++) //test
		{
			if(HAL_SPI_Receive(&hspi1, (uint8_t*)flash_data_read, sizeof(flash_data_read), 5000) != HAL_OK)
			{
				break;
			}
		}
	}

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	//----------------------------------------------------------------

	//================================================================
	// Erase flash MX25L12835F
	//================================================================
	//--------------------------------
	// Unlock flash
	//--------------------------------
	// READ STATUS REGISTER - wait 0x00
	flash_check_status_reg(0x00);

	// WRITE ENABLE
	flash_write_enable();

	// CLEAR FLAG STATUS REGISTER
	flash_clear_flag_status_reg();

	// FLASH_CMD_WRITE_STATUS
	flash_write_status_reg(0x00);

	// READ STATUS REGISTER - wait 0x00
	flash_check_status_reg(0x00);

	//--------------------------------
	// Erase flash
	//--------------------------------
	// WRITE ENABLE
	flash_write_enable();

	// READ STATUS REGISTER - wait 0x02
	flash_check_status_reg(0x02);

	// READ STATUS REGISTER - wait 0x02
	flash_check_status_reg(0x02);

	// WRITE ENABLE
	flash_write_enable();

	// BULK ERASE
	flash_bulk_erase();

	// READ STATUS REGISTER - wait 0x00
	flash_check_status_reg(0x00);

	//================================================================
	// Read back to check flash data - 0xFF
	//================================================================
	//--------------------------------
	// Read flash
	//--------------------------------
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	if(HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_cmd_read, sizeof(flash_cmd_read), 5000) == HAL_OK)
	{
		//for(loop_index = 0; loop_index < SPI_READ_LOOP_LIMIT; loop_index++)
		for(loop_index = 0; loop_index < 5; loop_index++) //test
		{
			if(HAL_SPI_Receive(&hspi1, (uint8_t*)flash_data_read, sizeof(flash_data_read), 5000) != HAL_OK)
			{
				break;
			}
		}
	}
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	//----------------------------------------------------------------

	//================================================================
	// Program flash MX25L12835F
	//================================================================
	//--------------------------------
	// Unlock flash
	//--------------------------------
	// READ STATUS REGISTER - wait 0x00
	flash_check_status_reg(0x00);

	// WRITE ENABLE
	flash_write_enable();

	// CLEAR FLAG STATUS REGISTER
	flash_clear_flag_status_reg();

	// FLASH_CMD_WRITE_STATUS
	flash_write_status_reg(0x00);

	// READ STATUS REGISTER - wait 0x00
	flash_check_status_reg(0x00);

	//--------------------------------
	// WRITE flash
	//--------------------------------
	// WRITE ENABLE
	flash_write_enable();

	// generate test data
	for(loop_index = 0; loop_index < 256; loop_index++)
	{
		flash_test_data[loop_index] = loop_index;
	}
	flash_program_address = 0x00000000;
	flash_cmd_program[1] = 0x00;
	flash_cmd_program[2] = 0x00;
	flash_cmd_program[3] = 0x00;
	flash_cmd_program[4] = 0x00;

	// read .ima file from USB disk

	if(f_open(&ReadFile, "0:1911-R4-3.2.3-5D2F.ima", FA_READ) != FR_OK)
	{
		flag_RWfailed = 1;
	}
	else
	{
		do
		{
			res_read = f_read(&ReadFile, rtext, sizeof(rtext), (void *)&bytesread);
			if((bytesread == 0) || (res_read != FR_OK)) /*EOF or Error*/
			{
				flag_RWfailed = 1;
			}
			else
			{
				// program 1-page data to flash
				// WRITE ENABLE
				flash_write_enable();

				// program flash
				HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
				if(HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_cmd_program, sizeof(flash_cmd_program), 5000) == HAL_OK)
				{
					HAL_SPI_Transmit(&hspi1, (uint8_t*)rtext, sizeof(rtext), 5000);

					// 1 page is 256 byte => 0x100
					// address need to add 0x100 each time
					flash_program_address = flash_program_address + SPI_WRITE_BUFFER_SIZE;
					flash_cmd_program[1] = (flash_program_address & 0xFF000000) >> (6*4);
					flash_cmd_program[2] = (flash_program_address & 0x00FF0000) >> (4*4);
					flash_cmd_program[3] = (flash_program_address & 0x0000FF00) >> (2*4);
					flash_cmd_program[4] = (flash_program_address & 0x000000FF) >> (0*4);
				}
				HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

				memset(rtext, '\0', sizeof(rtext));

				// READ STATUS REGISTER - wait 0x00
				flash_check_status_reg(0x00);
			}
		}while(bytesread != 0);

		f_close(&ReadFile);
	}

	//--------------------------------
	// Read flash
	//--------------------------------
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	if(HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_cmd_read, sizeof(flash_cmd_read), 5000) == HAL_OK)
	{
		//for(loop_index = 0; loop_index < SPI_READ_LOOP_LIMIT; loop_index++)
		for(loop_index = 0; loop_index < 5; loop_index++) //test
		{
			if(HAL_SPI_Receive(&hspi1, (uint8_t*)flash_data_read, sizeof(flash_data_read), 5000) != HAL_OK)
			{
				break;
			}
		}
	}
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	//----------------------------------------------------------------



	// program flash

	for(loop_index = 0; loop_index < 3; loop_index++)
	{
		// WRITE ENABLE
		flash_write_enable();

		// program flash
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		if(HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_cmd_program, sizeof(flash_cmd_program), 5000) == HAL_OK)
		{
			HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_test_data, sizeof(flash_test_data), 5000);

			// 1 page is 256 byte => 0x100
			// address need to add 0x100 each time
			flash_program_address = flash_program_address + SPI_WRITE_BUFFER_SIZE;
			flash_cmd_program[1] = (flash_program_address & 0xFF000000) >> (6*4);
			flash_cmd_program[2] = (flash_program_address & 0x00FF0000) >> (4*4);
			flash_cmd_program[3] = (flash_program_address & 0x0000FF00) >> (2*4);
			flash_cmd_program[4] = (flash_program_address & 0x000000FF) >> (0*4);
		}
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

		// READ STATUS REGISTER - wait 0x00
		flash_check_status_reg(0x00);
	}


	/*
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	if(HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_program, sizeof(flash_program), 5000) == HAL_OK)
	{
		HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_test_data, sizeof(flash_test_data), 5000);
		// 1 page is 256 byte => 0x100
		// address need to add 0x100 each time
		//flash_program_address = flash_program_address + SPI_WRITE_BUFFER_SIZE;
		//flash_program[1] = (flash_program_address & 0x11000000) >> 6;
		//flash_program[2] = (flash_program_address & 0x00110000) >> 4;
		//flash_program[3] = (flash_program_address & 0x00001100) >> 2;
		//flash_program[4] = (flash_program_address & 0x00000011) >> 0;

	}
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);


	// READ STATUS REGISTER - wait 0x00
	flash_check_status_reg(0x00);
	*/

	//----------------------------------------------------------------




	// try to read .ima or .bin file
	if(f_open(&ReadFile, "0:1911-R4-3.2.3-5D2F.ima", FA_READ) != FR_OK)
	{
		flag_RWfailed = 1;
	}
	else
	{
		// open a new file to store read file
		if(f_open(&WriteFile, "0:read_output.ima",FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
		{
			flag_RWfailed = 1;
		}
		else
		{
			do
			{
				res_read = f_read(&ReadFile, rtext, sizeof(rtext), (void *)&bytesread);
				if((bytesread == 0) || (res_read != FR_OK)) /*EOF or Error*/
				{
					flag_RWfailed = 1;
				}
				else
				{
					// read success
					/*
					blank_counter = 0;
					for(loop_index = 0; loop_index < sizeof(rtext); loop_index++)
					{
						if(rtext[sizeof(rtext)-loop_index-1] == '\0')
						{
							blank_counter++;
						}
						else
						{
							break;
						}
					}

					if(blank_counter == 1)
					{
						blank_counter = 0;
					}
					*/

#if 0
					res_write= f_write (&WriteFile, rtext, sizeof(rtext) - blank_counter, (void *)&bytesWritten);
					if((bytesWritten == 0) || (res_write != FR_OK)) /*EOF or Error*/
					{
						// write failed
						flag_RWfailed = 1;
					}
#endif


					memset(rtext, '\0', sizeof(rtext));

				}
			}while(bytesread != 0);

			f_close(&ReadFile);
			f_close(&WriteFile);
		}



		#if 0

		res = f_read(&MyFile, rtext, sizeof(rtext), (void *)&bytesread);
		if((bytesread == 0) || (res != FR_OK)) /*EOF or Error*/
		{
			//LCD_ErrLog("Cannot Read from the  'USBHost.txt' file \n");
		}
		else
		{
			// read success
			// write read content to new file
			if(f_open(&MyFile, "0:read_output.hex",FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
			{
			  //LCD_ErrLog("Cannot Open 'USBHost.txt' file \n");
			}
			else
			{
				res= f_write (&MyFile, rtext, sizeof(rtext), (void *)&bytesWritten);
				f_close(&MyFile);

				if((bytesWritten == 0) || (res != FR_OK)) /*EOF or Error*/
				{
					// write failed
				}
			}

		}
		f_close(&MyFile);
		#endif
	}


}


/**
  * @brief  Files operations: Read/Write and compare
  * @param  None
  * @retval None
  */
void USB_MSC_File_Operations(unsigned char command_type)
{
	uint16_t bytesread;
	uint16_t page_data_index;

	switch(command_type)
	{
		case USB_CMD_READ_LOG:

			break;

		case USB_CMD_UPDATE_IMA:

			aewin_dbg("Start erasing flash.\r\n");
			flash_unlock();
			flash_erase();
			aewin_dbg("Erasing flash finished.\r\n");

			flash_unlock();
			//--------------------------------
			// WRITE flash
			//--------------------------------
			// WRITE ENABLE
			flash_write_enable();

			// point to address 0x00000000 for starting programming
			flash_program_address = 0x00000000;
			flash_cmd_program[1] = 0x00;
			flash_cmd_program[2] = 0x00;
			flash_cmd_program[3] = 0x00;
			flash_cmd_program[4] = 0x00;

			// read .ima file from USB disk
			aewin_dbg("Start programming flash.\r\n");
			if(f_open(&ReadFile, usb_ima_file_path, FA_READ) != FR_OK)
			{
				usb_err_code = USB_ERR_FILE_RW_FAILED;
			}
			else
			{
				do
				{
					// read 1-page data from .ima file
					res_read = f_read(&ReadFile, rtext, sizeof(rtext), (void *)&bytesread);
					if((bytesread == 0) || (res_read != FR_OK)) /*EOF or Error*/
					{
						usb_err_code = USB_ERR_FILE_RW_FAILED;
					}
					else
					{
						// program 1-page data to flash
						// WRITE ENABLE
						flash_write_enable();

						// program flash
						HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
						if(HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_cmd_program, sizeof(flash_cmd_program), 5000) == HAL_OK)
						{
							HAL_SPI_Transmit(&hspi1, (uint8_t*)rtext, sizeof(rtext), 5000);

							// 1 page is 256 byte => 0x100
							// address need to add 0x100 each time
							flash_program_address = flash_program_address + SPI_WRITE_BUFFER_SIZE;
							flash_cmd_program[1] = (flash_program_address & 0xFF000000) >> (6*4);
							flash_cmd_program[2] = (flash_program_address & 0x00FF0000) >> (4*4);
							flash_cmd_program[3] = (flash_program_address & 0x0000FF00) >> (2*4);
							flash_cmd_program[4] = (flash_program_address & 0x000000FF) >> (0*4);
						}
						HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

						memset(rtext, '\0', sizeof(rtext));

						// READ STATUS REGISTER - wait 0x00
						flash_check_status_reg(0x00);
					}
				}while(bytesread != 0);

				f_close(&ReadFile);
			}
			aewin_dbg("Programming finished!.\r\n");

			// [debug]
			// read flash 5 pages
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

			if(HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_cmd_read, sizeof(flash_cmd_read), 5000) == HAL_OK)
			{
				//for(loop_index = 0; loop_index < SPI_READ_LOOP_LIMIT; loop_index++)
				for(loop_index = 0; loop_index < 5; loop_index++) //test
				{
					if(HAL_SPI_Receive(&hspi1, (uint8_t*)flash_data_read, sizeof(flash_data_read), 5000) != HAL_OK)
					{
						break;
					}

					//print to uart2
					aewin_dbg("\r\nPage %d ", loop_index);
					for(page_data_index = 0; page_data_index < sizeof(flash_data_read); page_data_index++)
					{
						if(page_data_index % 16 == 0)
						{
							aewin_dbg("\r\n");
						}
						aewin_dbg("0x%02x ", flash_data_read[page_data_index]);
					}
				}
			}

			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

			break;

		case USB_EXE_READ_CMD:
			if(f_open(&MyFile, usb_aewin_file_name, FA_READ) != FR_OK)
			{
				usb_err_code = USB_ERR_FILE_RW_FAILED;
			}
			else
			{
				// check command
				f_gets(usb_rtext_buffer, sizeof(usb_rtext_buffer), &MyFile);
				usb_cmd_code = usb_rtext_buffer[sizeof(usb_rtext_file_cmd)-1] - '0';
				for(loop_index = 0; loop_index < (sizeof(usb_rtext_file_cmd)-1) ; loop_index++)
				{
					if(usb_rtext_buffer[loop_index] != usb_rtext_file_cmd[loop_index])
					{
						usb_cmd_code = USB_CMD_NONE;
						break;
					}
				}

				// check flash number
				f_gets(usb_rtext_buffer, sizeof(usb_rtext_buffer), &MyFile);
				usb_cmd_flash_num = usb_rtext_buffer[sizeof(usb_rtext_file_flash_num)-1] - '0';
				for(loop_index = 0; loop_index < (sizeof(usb_rtext_file_flash_num)-1) ; loop_index++)
				{
					if(usb_rtext_buffer[loop_index] != usb_rtext_file_flash_num[loop_index])
					{
						usb_cmd_flash_num = USB_CMD_NONE;
						break;
					}
				}

				// check ima file name
				f_gets(usb_rtext_buffer, sizeof(usb_rtext_buffer),&MyFile);

				for(loop_index = 0; loop_index < sizeof(usb_rtext_file_ima_name)-1 ; loop_index++)
				{
					if(usb_rtext_buffer[loop_index] != usb_rtext_file_ima_name[loop_index])
					{
						usb_cmd_code = USB_CMD_NONE;
						break;
					}
				}

				for(loop_index = 0; loop_index < IMA_FILENAME_LEN_LIMIT; loop_index++)
				{
					if(usb_rtext_buffer[ sizeof(usb_rtext_file_ima_name) + loop_index] == '\n')
					{
						break;
					}
					else
					{
						usb_cmd_ima_filename[loop_index] = usb_rtext_buffer[ sizeof(usb_rtext_file_ima_name) + loop_index];
					}
				}

				strcat(usb_ima_file_path, usb_cmd_ima_filename);

				f_gets(usb_rtext_buffer, sizeof(usb_rtext_buffer),&MyFile);
				f_close(&MyFile);
			}
			break;

		case USB_EXE_ERROR_REPORT:
			if(f_open(&WriteFile, "0:error_report.txt", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
			{
				usb_err_code = USB_ERR_FILE_RW_FAILED;
			}
			else
			{
				//usb_wtext_error_msg[sizeof("error code:")] = usb_err_code;
				res= f_write (&WriteFile, usb_wtext_error_msg, sizeof(usb_wtext_error_msg), (void *)&bytesWritten);
				res= f_write (&WriteFile, &usb_err_code, 1, (void *)&bytesWritten);
				//res= f_write (&WriteFile, "\r\n", sizeof("\r\n"), (void *)&bytesWritten);
				f_close(&WriteFile);
			}
			break;

		//------below are debug/test command-------

		case USB_CMD_READ_FLASH:
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

			if(HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_cmd_read, sizeof(flash_cmd_read), 5000) == HAL_OK)
			{
				//for(loop_index = 0; loop_index < SPI_READ_LOOP_LIMIT; loop_index++)
				for(loop_index = 0; loop_index < 5; loop_index++) //test
				{
					if(HAL_SPI_Receive(&hspi1, (uint8_t*)flash_data_read, sizeof(flash_data_read), 5000) != HAL_OK)
					{
						break;
					}
				}
			}

			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
			break;

		case USB_CMD_TEST_RW:
			//LCD_UsrLog("INFO : FatFs Initialized \n");
			if(f_open(&MyFile, usb_file_name,FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
			{
			  //LCD_ErrLog("Cannot Open 'USBHost.txt' file \n");
			}
			else
			{
			  //LCD_UsrLog("INFO : 'USBHost.txt' opened for write  \n");
			  res= f_write (&MyFile, wtext, sizeof(wtext), (void *)&bytesWritten);
			  f_close(&MyFile);

			  if((bytesWritten == 0) || (res != FR_OK)) /*EOF or Error*/
			  {
				//LCD_ErrLog("Cannot Write on the  'USBHost.txt' file \n");
			  }
			  else
			  {
				if(f_open(&MyFile, usb_file_name, FA_READ) != FR_OK)
				{
				  //LCD_ErrLog("Cannot Open 'USBHost.txt' file for read.\n");
				}
				else
				{
				  //LCD_UsrLog("INFO : Text written on the 'USBHost.txt' file \n");

				  res = f_read(&MyFile, rtext, sizeof(rtext), (void *)&bytesread);

				  if((bytesread == 0) || (res != FR_OK)) /*EOF or Error*/
				  {
					//LCD_ErrLog("Cannot Read from the  'USBHost.txt' file \n");
				  }
				  else
				  {
					//LCD_UsrLog("Read Text : \n");
					//LCD_DbgLog((char *)rtext);
					//LCD_DbgLog("\n");
				  }
				  f_close(&MyFile);
				}
				/* Compare read data with the expected data */
				if((bytesread == bytesWritten))
				{
				  //LCD_UsrLog("INFO : FatFs data compare SUCCES");
				  //LCD_UsrLog("\n");
				}
				else
				{
				  //LCD_ErrLog("FatFs data compare ERROR");
				  //LCD_ErrLog("\n");
				}
			  }
			}


			//--------------
			// try to read .txt file
			if(f_open(&ReadFile, usb_file_name, FA_READ) != FR_OK)
			{
				usb_err_code = USB_ERR_FILE_RW_FAILED;
			}
			else
			{
				// open a new file to store read file
				if(f_open(&WriteFile, "0:read_test_output.ima",FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
				{
					usb_err_code = USB_ERR_FILE_RW_FAILED;
				}
				else
				{
					res_read = f_read(&ReadFile, rtext, sizeof(rtext), (void *)&bytesread);
					if((bytesread == 0) || (res_read != FR_OK)) /*EOF or Error*/
					{
						usb_err_code = USB_ERR_FILE_RW_FAILED;
					}
					else
					{
						res= f_write (&WriteFile, rtext, sizeof(rtext), (void *)&bytesWritten);
						f_close(&MyFile);

						if((bytesWritten == 0) || (res != FR_OK)) /*EOF or Error*/
						{
							// write failed
						}

					}

					f_close(&ReadFile);
					f_close(&WriteFile);
				}
			}
			break;

		default:
			break;

	}
}

/**
  * @brief  Check status register of flash
  * @param  target_value: wait status register be the the same value with target_value
  * @retval None
  */
void flash_check_status_reg(char target_value)
{
	do
	{
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_cmd_read_status, sizeof(flash_cmd_read_status), 5000);
		HAL_SPI_Receive(&hspi1, (uint8_t*)flash_data_read_byte, sizeof(flash_data_read_byte), 5000);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	}while(flash_data_read_byte[0] != target_value);

}

/**
  * @brief  Make flash be write enable
  * @param  None
  * @retval None
  */
void flash_write_enable(void)
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_cmd_write_enable, sizeof(flash_cmd_write_enable), 5000);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

/**
  * @brief  Erase whole flash
  * @param  None
  * @retval None
  */
void flash_bulk_erase(void)
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_cmd_bulk_erase, sizeof(flash_cmd_bulk_erase), 5000);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

/**
  * @brief  clear flag status register of flash
  * @param  None
  * @retval None
  */
void flash_clear_flag_status_reg(void)
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_cmd_clear_flag, sizeof(flash_cmd_clear_flag), 5000);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

/**
  * @brief  write status register of flash
  * @param  None
  * @retval None
  */
void flash_write_status_reg(char write_value)
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_cmd_write_status, sizeof(flash_cmd_write_status), 5000);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

/**
  * @brief  unlock flash for writing
  * @param  None
  * @retval None
  */
void flash_unlock(void)
{
	// READ STATUS REGISTER - wait 0x00
	flash_check_status_reg(0x00);

	// WRITE ENABLE
	flash_write_enable();

	// CLEAR FLAG STATUS REGISTER
	flash_clear_flag_status_reg();

	// FLASH_CMD_WRITE_STATUS
	flash_write_status_reg(0x00);

	// READ STATUS REGISTER - wait 0x00
	flash_check_status_reg(0x00);
}

void flash_erase(void)
{
	//--------------------------------
	// Erase flash
	//--------------------------------
	// WRITE ENABLE
	flash_write_enable();

	// READ STATUS REGISTER - wait 0x02
	flash_check_status_reg(0x02);

	// READ STATUS REGISTER - wait 0x02
	flash_check_status_reg(0x02);

	// WRITE ENABLE
	flash_write_enable();

	// BULK ERASE
	flash_bulk_erase();

	// READ STATUS REGISTER - wait 0x00
	flash_check_status_reg(0x00);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
