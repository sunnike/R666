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
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
FATFS USBH_fatfs;
FIL MyFile;
FRESULT res;
uint32_t bytesWritten;
uint8_t rtext[512];
uint8_t wtext[] = "USB Host Library : Mass Storage Example";
FRESULT res_write, res_read;
FIL WriteFile, ReadFile;

uint16_t loop_index;
uint8_t blank_counter;

//debug
uint8_t flag_RWfailed = 0;

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

					res_write= f_write (&WriteFile, rtext, sizeof(rtext) - blank_counter, (void *)&bytesWritten);
					if((bytesWritten == 0) || (res_write != FR_OK)) /*EOF or Error*/
					{
						// write failed
						flag_RWfailed = 1;
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





  
#if 0
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
#endif

}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
