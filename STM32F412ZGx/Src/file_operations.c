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
#include "rtc.h"
#include "fatfs.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint32_t loop_index;

// ---------------
// file read/write
//----------------
FATFS USBH_fatfs;
FIL MyFile;
FRESULT res;
FIL WriteFile, ReadFile;
FRESULT res_write, res_read;

uint16_t bytesread;
uint32_t bytesWritten;

uint8_t rtext[SPI_WRITE_BUFFER_SIZE];  // receive buffer of SPI, for reading flash data

//---------------
// USB variables
//---------------
const uint8_t usb_rtext_file_cmd[] = "command:";
const uint8_t usb_rtext_file_flash_num[] = "flash_number:";
const uint8_t usb_rtext_file_ima_name[] = "ima_file";
const uint8_t usb_rtext_file_re_read_flash[] = "re_read_flash:";
const uint8_t usb_wtext_error_msg[] = "error code:";

const uint8_t usb_aewin_file_name[] = "0:aewin_file.txt";
const uint8_t usb_flash_data_name[] = "0:flash_data";
const uint8_t usb_log_report_name[] = "0:log_report";
const uint8_t usb_ima_attchment_name[] = ".ima";
const uint8_t usb_txt_attchment_name[] = ".txt";
uint8_t usb_ima_file_path[FILE_PATH_HEAD_LEN + IMA_FILENAME_LEN_LIMIT] = "0:";
uint8_t usb_read_flash_filename[sizeof(usb_flash_data_name) + FLASH_DATA_NUM_LEN + FILE_BUILD_TIME_LEN \
								+ FILE_ATTACHMENT_NAME_LEN] = "0:flash_data_0_20190725000000.ima";
uint8_t usb_log_report_filename[sizeof(usb_log_report_name) + FILE_BUILD_TIME_LEN + FILE_ATTACHMENT_NAME_LEN] = "0:log_report_20190725000000.txt";

uint8_t usb_rtext_buffer[sizeof(usb_rtext_file_ima_name)+IMA_FILENAME_LEN_LIMIT];
uint8_t usb_write_str[FLASH_WRITE_ROW_NUM*( (FLASH_DATA_BYTE + FLASH_SPACE_BYTE) * FLASH_ROW_DATA_LIMIT + 1)];

extern uint8_t usb_err_code;
extern uint8_t usb_cmd_code;
extern uint8_t usb_cmd_flash_num;
extern uint8_t usb_cmd_ima_filename[IMA_FILENAME_LEN_LIMIT];
extern uint8_t usb_cmd_re_read_flash;
extern uint8_t backup_flag[2];

// ---------------
// flash variables
//----------------
// flash command
const uint8_t flash_cmd_read[] = {FLASH_CMD_READ, 0x00, 0x00, 0x00, 0x00};             // fast read data: {command, ADD1, ADD2, ADD3, Dummy}
const uint8_t flash_cmd_write_status[] = {FLASH_CMD_WRITE_STATUS, 0x00};
const uint8_t flash_cmd_read_status[] = {FLASH_CMD_READ_STATUS};
const uint8_t flash_cmd_write_enable[] = {FLASH_CMD_WRITE_ENABLE};
const uint8_t flash_cmd_clear_flag[] = {FLASH_CMD_CLEAR_FLAG};
const uint8_t flash_cmd_bulk_erase[] = {FLASH_CMD_BULK_ERASE};

uint8_t flash_cmd_program[] = {FLASH_CMD_PAGE_PROGRAM, 0x00, 0x00, 0x00, 0x00};
uint32_t flash_program_address = 0;

// flash read buffer
uint8_t flash_data_read[SPI_READ_BUFFER_SIZE];
volatile uint8_t flash_data_read_byte[1];

//---------------
// FPGA variables
//---------------
uint32_t *fpga_log_addr;           // point to FSMC address
uint16_t fpga_log_16bit;           // put the data which is read from fpga_log_addr
uint8_t fpga_log_str[14];          // the read data will be put at this string temporarily, and write into an opened file
uint8_t fpga_log_num;              // number of fpga log, range: 1 ~ 30
uint8_t fpga_log[FPGA_LOG_SIZE];   // one row of fpga log
uint8_t fpga_log_index;            // index of one row fpga log
uint8_t fpga_last_log = 0;         // record the last log of fpga log

extern uint8_t fpga_info[FPGA_INFO_SIZE];

//---------------
// RTC variables
//---------------
extern RTC_TimeTypeDef RTC_Time;
extern RTC_DateTypeDef RTC_Date;

//debug
const uint8_t wtext[] = "USB Host Library : Mass Storage Example";
const uint8_t usb_file_name[FILE_PATH_HEAD_LEN + IMA_FILENAME_LEN_LIMIT] = "0:USBHost_RWtest.txt";
uint8_t flag_RWfailed = 0;
uint8_t flash_test_data[SPI_READ_BUFFER_SIZE];

//test
extern char err_record;
extern char usb_read_flag;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Files operations: Read/Write and compare
  * @param  None
  * @retval None
  */
void MSC_File_Operations(void)
{

  	//uint16_t bytesread;

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
	uint16_t page_data_index;

	switch(command_type)
	{
		case USB_CMD_READ_LOG:

			// print FPGA information
			aewin_dbg("FPGA version: %d.%d.%d\r\n", fpga_info[0], fpga_info[1], fpga_info[2]);
			aewin_dbg("FPGA build date: 20%02d %02d %02d\r\n", fpga_info[5], fpga_info[4], fpga_info[3]);
			aewin_dbg("---------------------------\r\n");

			// print log to UART2
			fpga_log_addr = (uint32_t *)FPGA_FSMC_BASE_ADDR;
			fpga_log_16bit = *fpga_log_addr;
			HAL_RTC_GetTime(&hrtc, &RTC_Time, RTC_FORMAT_BCD);
			HAL_RTC_GetDate(&hrtc, &RTC_Date, RTC_FORMAT_BCD);
			aewin_dbg("Record Build Time: 20%02x.%02x.%02x %02x:%02x:%02x\r\n", RTC_Date.Year, RTC_Date.Month, RTC_Date.Date, RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);
			aewin_dbg("Last log: %d\r\n", (fpga_log_16bit&0xFF));
			aewin_dbg("  BIOS    BMC   Year  Month    Day   Hour Minute  State");
			for(loop_index = 0; loop_index < 128; loop_index++)
			{
				if(loop_index % 4 == 0)
				{
					aewin_dbg("\r\n");
				}

				fpga_log_16bit = *fpga_log_addr;
				aewin_dbg("  0x%02x   0x%02x ", (fpga_log_16bit&0xFF), ((fpga_log_16bit & 0xFF00)>>8));

				fpga_log_addr++;
			}
			aewin_dbg("\r\n");

			/*
			// save original log to USB disk
			if(f_open(&WriteFile, "0:log_report.txt", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
			{
				usb_err_code = USB_ERR_FILE_RW_FAILED;
			}
			else
			{
				fpga_log_addr = (uint32_t *)FPGA_FSMC_BASE_ADDR;

				fpga_log_16bit = *fpga_log_addr;
				sprintf(fpga_log_str,"%d",(fpga_log_16bit&0xFF));
				res= f_write(&WriteFile, "Last log: ", sizeof("Last log: "), (void *)&bytesWritten);
				res= f_write(&WriteFile, fpga_log_str, 2, (void *)&bytesWritten);
				res= f_write(&WriteFile, "\r\n", sizeof("\r\n")-1, (void *)&bytesWritten);

				res= f_write (&WriteFile, "No.   BIOS    BMC   Year  Month    Day   Hour Minute  State", sizeof("No.   BIOS    BMC   Year  Month    Day   Hour Minute  State"), (void *)&bytesWritten);

				fpga_log_addr = (uint32_t *)FPGA_FSMC_LOG_ADDR;
				fpga_log_num = 0;
				for(loop_index = 0; loop_index < (FPGA_LOG_BYTE_NUM/FPGA_DATA_BYTE_SIZE); loop_index++)
				{
					if(loop_index % 4 == 0)
					{
						fpga_log_num++;
						sprintf(fpga_log_str,"%3d ",fpga_log_num);
						res= f_write(&WriteFile, "\r\n", sizeof("\r\n")-1, (void *)&bytesWritten);
						res= f_write(&WriteFile, fpga_log_str, 4, (void *)&bytesWritten);
					}

					fpga_log_16bit = *fpga_log_addr;
					sprintf(fpga_log_str,"  0x%02x   0x%02x ",(fpga_log_16bit&0xFF), ((fpga_log_16bit & 0xFF00)>>8));
					res= f_write (&WriteFile, fpga_log_str, sizeof(fpga_log_str), (void *)&bytesWritten);

					fpga_log_addr++;
				}
				f_close(&WriteFile);
			}
			*/

			// save log to USB disk
			//usb_log_report_filename
			sprintf(usb_log_report_filename, "%s_20%02x%02x%02x%02x%02x%02x%s", usb_log_report_name, \
					RTC_Date.Year, RTC_Date.Month, RTC_Date.Date, RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds, usb_txt_attchment_name);
			aewin_dbg("Output file: %s\r\n", usb_log_report_filename);

			aewin_dbg("Start saving log.\r\n");
			if(f_open(&WriteFile, usb_log_report_filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
			{
				usb_err_code = USB_ERR_FILE_RW_FAILED;
				err_record |= (1 << MASK_ERR_USB);
				aewin_dbg("Open log_report file failed.\r\n");
			}
			else
			{
				// print FPGA information
				res= f_write(&WriteFile, "FPGA version   :", sizeof("FPGA version   :"), (void *)&bytesWritten);
				sprintf(fpga_log_str, "%2d.%2d.%2d\r\n", fpga_info[0], fpga_info[1], fpga_info[2]);
				res= f_write(&WriteFile, fpga_log_str, 10, (void *)&bytesWritten);
				res= f_write(&WriteFile, "FPGA build date:", sizeof("FPGA build date:"), (void *)&bytesWritten);
				sprintf(fpga_log_str, "20%02d.%02d.%02d\r\n", fpga_info[5], fpga_info[4], fpga_info[3]);
				res= f_write(&WriteFile, fpga_log_str, 12, (void *)&bytesWritten);

				res= f_write(&WriteFile, "---------------------------\r\n", sizeof("---------------------------\r\n")-1, (void *)&bytesWritten);

				// print record build time
				res= f_write(&WriteFile, "Record Build Time:", sizeof("Record Build Time:"), (void *)&bytesWritten);
				sprintf(fpga_log_str, "20%02x.%02x.%02x %02x:%02x:%02x", RTC_Date.Year, RTC_Date.Month, RTC_Date.Date);
				res= f_write(&WriteFile, fpga_log_str, 11, (void *)&bytesWritten);
				sprintf(fpga_log_str, "%02x:%02x:%02x\r\n", RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);
				res= f_write(&WriteFile, fpga_log_str, 10, (void *)&bytesWritten);

				// print last order index
				fpga_log_addr = (uint32_t *)FPGA_FSMC_BASE_ADDR;
				fpga_log_16bit = *fpga_log_addr;
				sprintf(fpga_log_str,"%d",(fpga_log_16bit&0xFF));
				res= f_write(&WriteFile, "Last log:", sizeof("Last log:"), (void *)&bytesWritten);
				res= f_write(&WriteFile, fpga_log_str, 2, (void *)&bytesWritten);
				res= f_write(&WriteFile, "\r\n", sizeof("\r\n")-1, (void *)&bytesWritten);

				if(fpga_log_16bit > 30)
				{
					err_record |= (1 << MASK_ERR_FSMC);
				}

				// record last log index
				fpga_last_log = (fpga_log_16bit&0xFF);

				// print table header
				res= f_write (&WriteFile, "No.   BIOS    BMC   Year  Month    Day   Hour Minute  State", \
						sizeof("No.   BIOS    BMC   Year  Month    Day   Hour Minute  State"), (void *)&bytesWritten);

				// print log
				fpga_log_addr = (uint32_t *)FPGA_FSMC_LOG_ADDR;
				fpga_log_num = 0;
				for(loop_index = 0; loop_index < (FPGA_LOG_BYTE_NUM/FPGA_DATA_BYTE_SIZE); loop_index++)
				{
					fpga_log_16bit = *fpga_log_addr;
					fpga_log[(loop_index % 4) * 2 + 0] = (fpga_log_16bit&0xFF);
					fpga_log[(loop_index % 4) * 2 + 1] = ((fpga_log_16bit & 0xFF00)>>8);

					if(loop_index % 4 == 0)
					{
						// wrap new line and print log number
						fpga_log_num++;
						sprintf(fpga_log_str,"%3d ",fpga_log_num);
						res= f_write(&WriteFile, "\r\n", sizeof("\r\n")-1, (void *)&bytesWritten);
						res= f_write(&WriteFile, fpga_log_str, 4, (void *)&bytesWritten);
					}
					else if(loop_index % 4 == 3)
					{
						// end of one row fpga log, check if this is the last log
						if( ((loop_index + 1) / 4) == fpga_last_log)
						{
							//if BKRE happened, set flag for mark which flash should be read to USB disk
							//backup_flag[0] = fpga_log[FPGA_LOG_BIOS];
							//backup_flag[1] = fpga_log[FPGA_LOG_BMC];
							if(fpga_log[FPGA_LOG_BIOS] == FPGA_LOG_BKRE)
							{
								backup_flag[BACKUP_FLAG_BIOS] = RE_READ_FLASH_ON;
							}
							if(fpga_log[FPGA_LOG_BMC] == FPGA_LOG_BKRE)
							{
								backup_flag[BACKUP_FLAG_BMC] = RE_READ_FLASH_ON;
							}
						}

						// print log, convert hex-log to text
						for(fpga_log_index = 0; fpga_log_index < FPGA_LOG_SIZE; fpga_log_index++)
						{
							switch(fpga_log_index)
							{
								case FPGA_LOG_BIOS:
								case FPGA_LOG_BMC:
									if(fpga_log[fpga_log_index] == FPGA_LOG_EMPTY)
									{
										sprintf(fpga_log_str,"  0x%02x ", fpga_log[fpga_log_index]);
									}
									else if(fpga_log[fpga_log_index] == FPGA_LOG_OK)
									{
										sprintf(fpga_log_str,"%6s ","OK");
									}
									else if(fpga_log[fpga_log_index] == FPGA_LOG_RE)
									{
										sprintf(fpga_log_str,"%6s ","RE");
									}
									else if(fpga_log[fpga_log_index] == FPGA_LOG_BKRE)
									{
										sprintf(fpga_log_str,"%6s ","BKRE");
									}
									else if(fpga_log[fpga_log_index] == FPGA_LOG_BKREF)
									{
										sprintf(fpga_log_str,"%6s ","BKREF");
									}
									else
									{
										sprintf(fpga_log_str,"%6d ", fpga_log[fpga_log_index]);
									}
									break;

								case FPGA_LOG_YEAR:
									if(fpga_log[fpga_log_index] == FPGA_LOG_EMPTY)
									{
										sprintf(fpga_log_str,"  0x%02x ", fpga_log[fpga_log_index]);
									}
									else
									{
										sprintf(fpga_log_str,"  20%02d ", fpga_log[fpga_log_index]);
									}
									break;

								case FPGA_LOG_MONTH:
								case FPGA_LOG_DAY:
								case FPGA_LOG_HOUR:
								case FPGA_LOG_MINUTE:
									if(fpga_log[fpga_log_index] == FPGA_LOG_EMPTY)
									{
										sprintf(fpga_log_str,"  0x%02x ", fpga_log[fpga_log_index]);
									}
									else
									{
										sprintf(fpga_log_str,"%6d ", fpga_log[fpga_log_index]);
									}
									break;

								case FPGA_LOG_STATE:
									if(fpga_log[fpga_log_index] == FPGA_LOG_EMPTY)
									{
										sprintf(fpga_log_str,"  0x%02x ", fpga_log[fpga_log_index]);
									}
									else if(fpga_log[fpga_log_index] == FPGA_LOG_STATE_DONE)
									{
										sprintf(fpga_log_str,"%6s ","DONE");
									}
									else
									{
										sprintf(fpga_log_str,"%6d ", fpga_log[fpga_log_index]);
									}
									break;
							}
							res= f_write (&WriteFile, fpga_log_str, 7, (void *)&bytesWritten);
						}
					}
					fpga_log_addr++;
				}

				if(f_close(&WriteFile) != FR_OK)
				{
					aewin_dbg("Close log_report file failed.\r\n");
				}
			}
			aewin_dbg("Saving log finished.\r\n");

			break;

		case USB_CMD_UPDATE_IMA:

			//--------------------------------
			// EREASE flash
			//--------------------------------
			aewin_dbg("Start erasing flash.\r\n");
			flash_unlock();
			flash_erase();
			aewin_dbg("Erasing flash finished.\r\n");

			//--------------------------------
			// WRITE flash
			//--------------------------------
			flash_unlock();
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
				err_record |= (1 << MASK_ERR_USB);

				// [note]
				// add usb_err_code - ima file not found
				aewin_dbg("Open .ima file failed.\r\n");
			}
			else
			{
				// open .ima file success, start programming flash until EOF or error occur
				do
				{
					// read 1-page data from .ima file
					res_read = f_read(&ReadFile, rtext, sizeof(rtext), (void *)&bytesread);
					if((bytesread == 0) || (res_read != FR_OK)) /*EOF or Error*/
					{
						aewin_dbg("Read .ima file failed.\r\n");
						usb_err_code = USB_ERR_FILE_RW_FAILED;
						break;
					}
					else
					{
						//--------------------------------
						// PROGRAM flash
						//--------------------------------
						flash_write_enable();

						// program flash - program 1-page data to flash
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
						else
						{
							// [test] SPI failed
							aewin_dbg("SPI transmit failed.\r\n");
							usb_read_flag = 1;
							err_record |= (1<<MASK_ERR_SPI);
						}
						HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

						// clear receive buffer
						memset(rtext, '\0', sizeof(rtext));

						// READ STATUS REGISTER - wait 0x00
						flash_check_status_reg(0x00);
					}
				}while(bytesread != 0);

				if(f_close(&ReadFile) != FR_OK)
				{
					err_record |= (1 << MASK_ERR_USB);
					aewin_dbg("f_close failed.\r\n");
				}
			}
			aewin_dbg("Programming finished!.\r\n");

			// [debug]
			// read and print flash 2 pages to UART2
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
			if(HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_cmd_read, sizeof(flash_cmd_read), 5000) == HAL_OK)
			{
				//for(loop_index = 0; loop_index < SPI_READ_LOOP_LIMIT; loop_index++)
				//for(loop_index = 0; loop_index < 2; loop_index++) //test
				{
					if(HAL_SPI_Receive(&hspi1, (uint8_t*)flash_data_read, sizeof(flash_data_read), 5000) != HAL_OK)
					{
						// [test] SPI failed
						aewin_dbg("SPI receive failed.\r\n");
						usb_read_flag = 1;
						err_record |= (1<<MASK_ERR_SPI);
						break;
					}

					//print flash data to uart2
					//aewin_dbg("\r\nPage %d ", loop_index);
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
			else
			{
				// [test] SPI failed
				aewin_dbg("SPI transmit failed.\r\n");
				usb_read_flag = 1;
				err_record |= (1<<MASK_ERR_SPI);
			}
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

			break;

		case USB_EXE_READ_CMD:

			if(f_open(&MyFile, usb_aewin_file_name, FA_READ) != FR_OK)
			{
				usb_err_code = USB_ERR_FILE_RW_FAILED;
				err_record |= (1 << MASK_ERR_USB);
				aewin_dbg("f_open failed.\r\n");
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
						usb_err_code = USB_ERR_FILE_WRONG_FORMAT;
						break;
					}
				}
				aewin_dbg("Get USB command code: %d\r\n", usb_cmd_code);

				// check flash number
				f_gets(usb_rtext_buffer, sizeof(usb_rtext_buffer), &MyFile);
				usb_cmd_flash_num = usb_rtext_buffer[sizeof(usb_rtext_file_flash_num)-1] - '0';
				for(loop_index = 0; loop_index < (sizeof(usb_rtext_file_flash_num)-1) ; loop_index++)
				{
					if(usb_rtext_buffer[loop_index] != usb_rtext_file_flash_num[loop_index])
					{
						usb_cmd_flash_num = FLASH_NONE;
						usb_err_code = USB_ERR_FILE_WRONG_FORMAT;
						break;
					}
				}
				aewin_dbg("Get USB flash number: %d\r\n", usb_cmd_flash_num);

				// check ima file name format
				f_gets(usb_rtext_buffer, sizeof(usb_rtext_buffer),&MyFile);
				usb_cmd_ima_filename[0] = IMA_FILE_TAG;
				for(loop_index = 0; loop_index < sizeof(usb_rtext_file_ima_name)-1 ; loop_index++)
				{
					if(usb_rtext_buffer[loop_index] != usb_rtext_file_ima_name[loop_index])
					{
						usb_cmd_ima_filename[0] = IMA_FILE_NONE;
						usb_err_code = USB_ERR_FILE_WRONG_FORMAT;
						break;
					}
				}

				// get .ima file name
				memset(usb_ima_file_path , '\0', sizeof(usb_ima_file_path ));
				usb_ima_file_path[0] = '0';
				usb_ima_file_path[1] = ':';
				if(usb_cmd_ima_filename[0] != IMA_FILE_NONE)
				{
					for(loop_index = 0; loop_index < IMA_FILENAME_LEN_LIMIT; loop_index++)
					{
						if(usb_rtext_buffer[ sizeof(usb_rtext_file_ima_name) + loop_index] == '\n')
						{
							// end of .ima file name
							break;
						}
						else
						{
							usb_cmd_ima_filename[loop_index] = usb_rtext_buffer[sizeof(usb_rtext_file_ima_name) + loop_index];
						}
					}
					strcat(usb_ima_file_path, usb_cmd_ima_filename);
					aewin_dbg("Get USB .ima file name: %s\r\n", usb_cmd_ima_filename);
				}

				// check RE read flash data setting
				// usb_rtext_file_re_read_flash
				f_gets(usb_rtext_buffer, sizeof(usb_rtext_buffer), &MyFile);
				usb_cmd_re_read_flash = usb_rtext_buffer[sizeof(usb_rtext_file_re_read_flash)-1] - '0';
				for(loop_index = 0; loop_index < (sizeof(usb_rtext_file_re_read_flash)-1) ; loop_index++)
				{
					if(usb_rtext_buffer[loop_index] != usb_rtext_file_re_read_flash[loop_index])
					{
						usb_cmd_re_read_flash = RE_READ_FLASH_OFF;
						usb_err_code = USB_ERR_FILE_WRONG_FORMAT;
						break;
					}
				}
				aewin_dbg("Get RE read flash setting: %d\r\n", usb_cmd_re_read_flash);

				if(f_close(&MyFile) != FR_OK)
				{
					err_record |= (1<<MASK_ERR_USB);
					aewin_dbg("f_close failed.\r\n");
				}

				aewin_dbg("++++++++++++++++++++++++++++\r\n");
			}
			break;

		case USB_EXE_ERROR_REPORT:
			if(f_open(&WriteFile, "0:error_report.txt", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
			{
				aewin_dbg("Create error_report.txt failed.\r\n");
				err_record |= (1<<MASK_ERR_USB);
				usb_err_code = USB_ERR_FILE_RW_FAILED;
			}
			else
			{
				sprintf(fpga_log_str,"%d",usb_err_code);
				res= f_write (&WriteFile, usb_wtext_error_msg, sizeof(usb_wtext_error_msg), (void *)&bytesWritten);
				res= f_write(&WriteFile, fpga_log_str, 2, (void *)&bytesWritten);
				if(f_close(&WriteFile) != FR_OK)
				{
					aewin_dbg("f_close failed.\r\n");
				}
			}
			break;

		case USB_CMD_READ_FLASH:
			// setting file name of output data according to the flash number and RTC
			sprintf(usb_read_flash_filename, "%s_%d_20%02x%02x%02x%02x%02x%02x%s", usb_flash_data_name, usb_cmd_flash_num, \
					RTC_Date.Year, RTC_Date.Month, RTC_Date.Date, RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds, usb_ima_attchment_name);
			aewin_dbg("Output file: %s\r\n", usb_read_flash_filename);

			if(f_open(&WriteFile, usb_read_flash_filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
			{
				aewin_dbg("Create flash_data.txt failed.\r\n");
				err_record |= (1<<MASK_ERR_USB);
				usb_err_code = USB_ERR_FILE_RW_FAILED;
			}
			else
			{
				aewin_dbg("Start reading flash data.\r\n");
				HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
				if(HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_cmd_read, sizeof(flash_cmd_read), 5000) == HAL_OK)
				{
					// [test] comment bellow because of factory test
					//for(loop_index = 0; loop_index < SPI_READ_LOOP_LIMIT; loop_index++)
					{
						if(HAL_SPI_Receive(&hspi1, (uint8_t*)flash_data_read, sizeof(flash_data_read), 5000) != HAL_OK)
						{
							aewin_dbg("Receive flash data failed.\r\n");
							// [test] SPI failed
							aewin_dbg("SPI receive failed.\r\n");
							usb_read_flag = 1;
							err_record |= (1<<MASK_ERR_SPI);

							break;
						}

						// print page number of flash
						//res= f_write (&WriteFile, "\r\nPage", sizeof("\r\nPage"), (void *)&bytesWritten);
						//res= f_write (&WriteFile, &loop_index, 1, (void *)&bytesWritten);

						// write flash data to a .txt file
						/*
						for(page_data_index = 0; page_data_index < sizeof(flash_data_read); page_data_index++)
						{
							sprintf(flash_data_str,"%02x ",flash_data_read[page_data_index]);
							strcat(usb_write_str, flash_data_str);

							if(page_data_index % 16 == 15)
							{
								strcat(usb_write_str, "\r\n");
								res= f_write(&WriteFile, usb_write_str, sizeof(usb_write_str), (void *)&bytesWritten);
								memset(usb_write_str, '\0', sizeof(usb_write_str));
								//res= f_write(&WriteFile, "\r\n", sizeof("\r\n")-1, (void *)&bytesWritten);
							}
							//sprintf(flash_data_str,"%02x ",flash_data_read[page_data_index]);
							//res= f_write (&WriteFile, flash_data_str, sizeof(flash_data_str), (void *)&bytesWritten);
						}
						*/

						// write flash data to a .txt file - no strcat
						/*
						sprintf_offset = 0;
						for(page_data_index = 0; page_data_index < sizeof(flash_data_read); page_data_index++)
						{
							sprintf_offset += sprintf(usb_write_str + sprintf_offset, "%02X ", flash_data_read[page_data_index]);

							if(page_data_index % FLASH_ROW_DATA_LIMIT == (FLASH_ROW_DATA_LIMIT - 1))
							{
								usb_write_str[sprintf_offset - 1] = '\r';
								usb_write_str[sprintf_offset] = '\n';
								sprintf_offset = sprintf_offset + 1;
							}

							if(page_data_index % (FLASH_WRITE_ROW_NUM * FLASH_ROW_DATA_LIMIT) == (FLASH_WRITE_ROW_NUM * FLASH_ROW_DATA_LIMIT - 1))
							{
								res= f_write(&WriteFile, usb_write_str, sizeof(usb_write_str), (void *)&bytesWritten);
								sprintf_offset = 0;
								memset(usb_write_str, '\0', sizeof(usb_write_str));
							}
						}
						*/

						// write flash data to a .ima file
						res= f_write(&WriteFile, flash_data_read, sizeof(flash_data_read), (void *)&bytesWritten);

					}
				}
				else
				{
					// [test] SPI failed
					aewin_dbg("SPI transmit failed.\r\n");
					usb_read_flag = 1;
					err_record |= (1<<MASK_ERR_SPI);
				}
				HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

				if(f_close(&WriteFile) != FR_OK)
				{
					aewin_dbg("f_close failed.\r\n");
				}
				aewin_dbg("Reading flash data finished.\r\n");
			}

			break;

		//------below are debug/test command-------

		case USB_CMD_TEST_RW:

			//LCD_UsrLog("INFO : FatFs Initialized \n");
			if(f_open(&MyFile, usb_file_name,FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
			{
			  //LCD_ErrLog("Cannot Open 'USBHost.txt' file \n");
				aewin_dbg("Cannot Open '%s' file.\r\n", usb_file_name);
			}
			else
			{
			  //LCD_UsrLog("INFO : 'USBHost.txt' opened for write  \n");
				//aewin_dbg("'%s' opened for write.\r\n", usb_file_name);

			  res= f_write (&MyFile, wtext, sizeof(wtext), (void *)&bytesWritten);
			  f_close(&MyFile);

			  if((bytesWritten == 0) || (res != FR_OK)) /*EOF or Error*/
			  {
				//LCD_ErrLog("Cannot Write on the  'USBHost.txt' file \n");
				  aewin_dbg("Cannot Write on the '%s' file.\r\n", usb_file_name);
			  }
			  else
			  {
				if(f_open(&MyFile, usb_file_name, FA_READ) != FR_OK)
				{
				  //LCD_ErrLog("Cannot Open 'USBHost.txt' file for read.\n");
					aewin_dbg("Cannot Open '%s' file.\r\n", usb_file_name);
				}
				else
				{
				  //LCD_UsrLog("INFO : Text written on the 'USBHost.txt' file \n");

				  res = f_read(&MyFile, rtext, sizeof(rtext), (void *)&bytesread);

				  if((bytesread == 0) || (res != FR_OK)) /*EOF or Error*/
				  {
					//LCD_ErrLog("Cannot Read from the  'USBHost.txt' file \n");
					  aewin_dbg("Cannot Read from the '%s' file.\r\n", usb_file_name);
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
				aewin_dbg("Cannot open the '%s' file for read.\r\n", usb_file_name);
			}
			else
			{
				// open a new file to store read file
				if(f_open(&WriteFile, "0:read_test_output.txt",FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
				{
					usb_err_code = USB_ERR_FILE_RW_FAILED;
					aewin_dbg("Cannot open the '%s' file for write.\r\n", usb_file_name);
				}
				else
				{
					res_read = f_read(&ReadFile, rtext, sizeof(rtext), (void *)&bytesread);
					if((bytesread == 0) || (res_read != FR_OK)) /*EOF or Error*/
					{
						usb_err_code = USB_ERR_FILE_RW_FAILED;
						aewin_dbg("Cannot Read from the '%s' file.\r\n", usb_file_name);
					}
					else
					{
						res= f_write (&WriteFile, rtext, sizeof(rtext), (void *)&bytesWritten);
						f_close(&MyFile);

						if((bytesWritten == 0) || (res != FR_OK)) /*EOF or Error*/
						{
							// write failed
							aewin_dbg("Cannot write output file.\r\n");
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
