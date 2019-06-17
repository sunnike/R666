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

uint32_t loop_index;
uint8_t blank_counter;

// fast read data: {command, ADD1, ADD2, ADD3, Dummy}
uint8_t flash_command_read[] = {FLASH_CMD_READ, 0x00, 0x00, 0x00, 0x00};
uint8_t flash_program[] = {FLASH_CMD_PAGE_PROGRAM, 0x00, 0x00, 0x00, 0x00};
uint8_t flash_command_write_status[] = {FLASH_CMD_WRITE_STATUS, 0x00};
uint8_t flash_cmd_read_status[] = {FLASH_CMD_READ_STATUS};
uint8_t flash_cmd_write_enable[] = {FLASH_CMD_WRITE_ENABLE};
uint8_t flash_cmd_clear_flag[] = {FLASH_CMD_CLEAR_FLAG};
uint8_t flash_cmd_bulk_erase[] = {FLASH_CMD_BULK_ERASE};
uint8_t flash_cmd_read_device_id [] = {0x9F};

volatile uint8_t flash_data_read[SPI_READ_BUFFER_SIZE];
volatile uint8_t flash_data_read_byte[1];

uint32_t flash_program_address = 0;

//debug
uint8_t flag_RWfailed = 0;
uint8_t flag_debug_spi = 0;
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

	if(HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_command_read, sizeof(flash_command_read), 5000) == HAL_OK)
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


	// read device ID
	/*
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

	if(HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_cmd_read_device_id, sizeof(flash_cmd_read_device_id), 5000) == HAL_OK)
	{
		for(loop_index = 0; loop_index < 1; loop_index++)
		{
			if(HAL_SPI_Receive(&hspi1, (uint8_t*)flash_data_read, 2, 5000) != HAL_OK)
			{
				break;
			}
		}
	}

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	*/
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
	if(HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_command_read, sizeof(flash_command_read), 5000) == HAL_OK)
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
	flash_program[1] = 0x00;
	flash_program[2] = 0x00;
	flash_program[3] = 0x00;
	flash_program[4] = 0x00;

	// read .ima file from USB disk

	// not finish yet
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
				if(HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_program, sizeof(flash_program), 5000) == HAL_OK)
				{
					HAL_SPI_Transmit(&hspi1, (uint8_t*)rtext, sizeof(rtext), 5000);

					// 1 page is 256 byte => 0x100
					// address need to add 0x100 each time
					flash_program_address = flash_program_address + SPI_WRITE_BUFFER_SIZE;
					flash_program[1] = (flash_program_address & 0xFF000000) >> (6*4);
					flash_program[2] = (flash_program_address & 0x00FF0000) >> (4*4);
					flash_program[3] = (flash_program_address & 0x0000FF00) >> (2*4);
					flash_program[4] = (flash_program_address & 0x000000FF) >> (0*4);
				}
				HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

				memset(rtext, '\0', sizeof(rtext));

				// READ STATUS REGISTER - wait 0x00
				flash_check_status_reg(0x00);
			}
		}while(bytesread != 0);

		f_close(&ReadFile);
	}



	// program flash

	for(loop_index = 0; loop_index < 3; loop_index++)
	{
		// WRITE ENABLE
		flash_write_enable();

		// program flash
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		if(HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_program, sizeof(flash_program), 5000) == HAL_OK)
		{
			HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_test_data, sizeof(flash_test_data), 5000);

			// 1 page is 256 byte => 0x100
			// address need to add 0x100 each time
			flash_program_address = flash_program_address + SPI_WRITE_BUFFER_SIZE;
			flash_program[1] = (flash_program_address & 0xFF000000) >> (6*4);
			flash_program[2] = (flash_program_address & 0x00FF0000) >> (4*4);
			flash_program[3] = (flash_program_address & 0x0000FF00) >> (2*4);
			flash_program[4] = (flash_program_address & 0x000000FF) >> (0*4);
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
	//--------------------------------
	// Read flash
	//--------------------------------
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	if(HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_command_read, sizeof(flash_command_read), 5000) == HAL_OK)
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


					switch(HAL_SPI_Transmit(&hspi1, (uint8_t*)rtext, sizeof(rtext), 5000))
					{
						case HAL_OK:
					      /* Communication is completed ___________________________________________ */
							flag_debug_spi = 0;
					      break;

					    case HAL_TIMEOUT:
					      /* A Timeout Occur ______________________________________________________*/
					      /* Call Timeout Handler */
					    	flag_debug_spi = 3;
					      break;

					      /* An Error Occur ______________________________________________________ */
					    case HAL_ERROR:
					      /* Call Timeout Handler */
					    	flag_debug_spi = 1;
					      break;

					    default:
					    	flag_debug_spi = 9;
					      break;
					}

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
	HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_command_write_status, sizeof(flash_command_write_status), 5000);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
