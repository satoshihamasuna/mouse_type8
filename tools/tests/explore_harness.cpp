#include "System/Inc/explore.h"
#include "System/Inc/demo_util.h"
#include "Module/Inc/remote_explore.h"
#include "Module/Inc/log_stream.h"
#include "Module/Inc/log_data.h"
#include "Subsys/Inc/search_turn_diagnostics.h"
#include <cassert>
#include <cstring>

uint32_t test_primask, tick;
int test_sensor = 3000, test_motion_calls, test_motion_ends;
bool test_stop_during_motion, stream = true, low_battery;
bool stop_in_wait;
int shortest_case(const char *);
uint32_t HAL_GetTick() { return tick; }
void HAL_Delay(uint32_t ms)
{
	tick += ms;
	if(stop_in_wait && tick >= 20) RemoteExplore::request_stop();
}
extern "C" float Battery_GetAverageVoltage() { return low_battery ? 6.0f : 7.5f; }
extern "C" int Battery_IsVoltageError(float v) { return v < 7.0f; }
bool LogStream::active(uint32_t) { return stream; }
t_position DemoUtil::run_search(Search *, search_type_t, t_position, t_position goal, int, wall_class *, Motion *)
{
	++test_motion_calls;
	assert(test_sensor >= 2000);
	assert(LogData::getInstance().log_enable == True);
	SearchTurnGuard::Decision decision;
	decision.reason = test_motion_calls == 1 ? SearchTurnGuard::Normal : SearchTurnGuard::FrontDifference;
	SearchTurnDiagnostics::instance().record(tick, 1, 2, 0, decision);
	if(test_stop_during_motion) RemoteExplore::request_stop();
	return goal;
}

int main(int argc, char **argv)
{
	assert(argc == 2);
	if(strncmp(argv[1], "shortest_", 9) == 0) return shortest_case(argv[1] + 9);
	if(strcmp(argv[1], "show") == 0)
	{
		char *args[] = {const_cast<char *>("explore"), const_cast<char *>("show")};
		assert(shell_explore(2, args) == 0);
		assert(test_motion_calls == 0);
		return 0;
	}
	char command[] = "explore", start[] = "start", gx[] = "7", gy[] = "7", size[] = "2";
	char plain[] = "plain", route[] = "roundtrip", seconds[] = "360";
	char *args[] = {command, start, gx, gy, size, plain, route, seconds};
	RemoteExplore::arm();
	if(strcmp(argv[1], "stop_wait") == 0) { test_sensor = 0; stop_in_wait = true; }
	if(strcmp(argv[1], "stop_motion") == 0) test_stop_during_motion = true;
	if(strcmp(argv[1], "no_stream") == 0) stream = false;
	if(strcmp(argv[1], "low_battery") == 0) low_battery = true;
	if(strcmp(argv[1], "bad_goal") == 0) args[4] = const_cast<char *>("2147483647");
	const int result = shell_explore(8, args);
	if(strcmp(argv[1], "complete") == 0)
	{
		assert(result == 0 && test_motion_calls == 2 && test_motion_ends == 1);
	}
	else if(strcmp(argv[1], "stop_wait") == 0)
		assert(result == -1 && test_motion_calls == 0 && test_motion_ends == 1);
	else if(strcmp(argv[1], "stop_motion") == 0)
		assert(result == -1 && test_motion_calls == 1 && test_motion_ends == 1);
	else
		assert(result == -1 && test_motion_calls == 0 && test_motion_ends == 0);
	assert(!RemoteExplore::running());
	assert(LogData::getInstance().log_enable == False);
	assert((explore_completed_maze() != nullptr) == (result == 0));
	return 0;
}
