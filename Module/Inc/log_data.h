/*
 * log_data.h
 *
 *  Created on: 2023/06/20
 *      Author: sato1
 */

#ifndef CPP_INC_LOG_DATA_H_
#define CPP_INC_LOG_DATA_H_

#include "typedef.h"
#include "../../Component/Inc/half_float.h"
#include "../../Component/Inc/singleton.h"

#define LOG_DATA_SIZE 1000
#define LOG_DATA_NUM  40
#define LOG_DATA_PRIOD 3

class LogData:public Singleton<LogData>
{

	public:
	    t_bool log_enable = False;
		const int data_size = LOG_DATA_SIZE;
		const int data_num  = LOG_DATA_NUM;
		int data_count = 0;
		uint8_t mode = 0;
		half_float data[LOG_DATA_NUM][LOG_DATA_SIZE];
		void indicate_data();
		void init_log()
		{
			for(int i = 0; i < data_num ;i++)
			{
				for(int j = 0;j < data_size;j++)
				{
					data[i][j] = float_to_half(0.0f);
				}
			}
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
};




#endif /* CPP_INC_LOG_DATA_H_ */
