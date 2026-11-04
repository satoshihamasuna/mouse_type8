/*
 * log_data.h
 *
 *  Created on: 2023/06/20
 *      Author: sato1
 */

#ifndef CPP_INC_LOG_DATA_H_
#define CPP_INC_LOG_DATA_H_

#include "Component/Inc/typedef.h"
#include "Component/Inc/half_float.h"
#include "Component/Inc/singleton.h"
#include "Module/Inc/log_buffer.h"

#define LOG_DATA_SIZE 400
#define LOG_DATA_NUM  55
#define LOG_DATA_PRIOD 1

#define LOG_MAGIC 0xA55A
#define LOG_MAGIC_END 0xFFFF

class LogData:public Singleton<LogData>
{

	public:
	    volatile t_bool log_enable = False;
		const int data_size = LOG_DATA_SIZE;
		const int data_num  = LOG_DATA_NUM;
		uint8_t mode = 0;
		void indicate_data();
		void indicate_data_binary();
		void reset_log();
		void init_log()
		{
			log_enable = False;
			reset_log();
		}

		void set_logmode(uint8_t _mode)
		{
			switch(_mode)
			{
			case 0:
			case 1:
			case 2:
				 mode = _mode;	break;
			default :
				 mode = 0; 		break;
			}
		}
		void logging();
	private:
		LogBuffer<LOG_DATA_SIZE, LOG_DATA_NUM> buffer;
		uint16_t log_tick = 0;
};

typedef struct __attribute__((packed))
{
	uint16_t magic;
	uint16_t index;
    half_float data[LOG_DATA_NUM];
} log_frame_t;

#endif /* CPP_INC_LOG_DATA_H_ */
