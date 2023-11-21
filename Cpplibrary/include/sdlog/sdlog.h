/*
 * sdlog.h
 *
 *  Created on: 2020.07.26
 *      Author: JackyPan
 */
#pragma once
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDLOG_H
#define __SDLOG_H

#include "common.h"

class SDLog{
private:
	typedef enum{
		LOG_CAT = 0,
		LOG_DATA,
		LOG_END,
	}Log_Type;

	void Log_To_File(Log_Type log_type);

public:
	SDLog(void);

	typedef enum{
		Logger_Idle = 0,
		Logger_Open,
		Logger_Close,
		Logger_Record,
		Logger_Gnss_Write,
		Logger_Gnss_Read
	}Logger_Status;
	Logger_Status m_Logger_Status;

	void Logger_Update(void);
	void Logger_Enable(void);
	void Logger_Disable(void);
	void Logger_Write_Gnss(void);
	void Logger_Read_Gnss(void);
};

#endif /* __SDLOG_H */
