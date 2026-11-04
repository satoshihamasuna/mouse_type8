#ifndef TEST_EXPLORE_LOG_DATA_H_
#define TEST_EXPLORE_LOG_DATA_H_
#include "Component/Inc/typedef.h"
class LogData
{
public:
	t_bool log_enable = False;
	void reset_log() {}
	static LogData &getInstance() { static LogData log; return log; }
};
#endif
