/*
 * c
 *
 *  Created on: Nov 3, 2022
 *      Author: Angus McLennan
 */

#include "data_logging.h"

/**
  * @brief Logs data to file
  * @param data_logging_handle* data_handle Struct containing configurations
  * @param char* filename Filename to log data to
  * @param char* data Data to be logged to file
  * @param size_t len Size of data string
  * @retval None
  */
void log_to_file(data_logging_handle* data_handle, char* filename, char* data, size_t len) {
    //Open file for writing 
    if(f_open(&SDFile, filename, FA_OPEN_APPEND | FA_WRITE) != FR_OK) {
        Error_Handler();
    }
    else {
        //Write to the text file
        uint8_t byteswritten;
        FRESULT res = f_write(&SDFile, data, len, (void *)&byteswritten);
        if((byteswritten == 0) || (res != FR_OK)) {
            Error_Handler();
        }
        else {
            f_close(&SDFile);
        }
    }
}

/**
  * @brief Logs data to file
  * @param data_logging_handle* data_handle Struct containing configurations
  * @retval None
  */
void data_logging_init(data_logging_handle* data_handle) {
    if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) != FR_OK) {
		Error_Handler();
	}
	else {
		if(f_mkfs((TCHAR const*)SDPath, FM_ANY, 0, data_handle->rtext, sizeof(data_handle->rtext)) != FR_OK) {
			Error_Handler();
	    }
	}
	f_mount(&SDFatFS, (TCHAR const*)NULL, 0);
}
