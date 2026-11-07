#include "System/Inc/shortest_run.h"
#include "System/Inc/explore.h"
#include "Subsys/Inc/make_path.h"
#include "Module/Inc/log_data.h"
#include "Module/Inc/remote_explore.h"
#include <cassert>
#include <cstring>

extern uint32_t tick;
extern bool stream, low_battery, stop_in_wait;
bool test_no_path;
bool test_shortest_running;
int test_plan_calls, shortest_calls, flash_reads;
bool cancel_plan, lose_link, motion_failure, cancel_motion;
t_run_config_type expected_type = RUN_CONFIG_PLAIN;
wall_class *expected_wall;

void read_save_data(wall_class *) { ++flash_reads; }
void test_shortest_plan()
{
	if(cancel_plan) RemoteExplore::request_stop();
	if(lose_link) stream = false;
}
void test_shortest_run(t_run_config_type type, bool (*abort_requested)(), wall_class *wall, Motion *motion)
{
	assert(type == expected_type && wall != nullptr);
	if(expected_wall) assert(wall == expected_wall);
	assert(abort_requested && !abort_requested());
	assert(LogData::getInstance().log_enable == True);
	++shortest_calls;
	test_shortest_running = true;
	motion->ir.stats = {100, 60, 4, 2, 40, 3, 12, 13, 2.5f};
	if(cancel_motion) RemoteExplore::request_stop();
	if(motion_failure) motion->status = error;
}

int shortest_case(const char *scenario)
{
	if(strcmp(scenario, "show") == 0)
	{
		char *args[] = {const_cast<char *>("shortest"), const_cast<char *>("show")};
		assert(shell_shortest(2, args) == 0 && shortest_calls == 0 && flash_reads == 0);
		return 0;
	}
	// Complete a real exploration command first: its map must be reusable without Flash writes.
	if(strcmp(scenario, "missing_maze") != 0)
	{
		char *args[] = {const_cast<char *>("explore"), const_cast<char *>("start"), const_cast<char *>("7"),
			const_cast<char *>("7"), const_cast<char *>("2"), const_cast<char *>("plain"),
			const_cast<char *>("roundtrip"), const_cast<char *>("360")};
		RemoteExplore::arm();
		assert(shell_explore(8, args) == 0);
		assert(explore_completed_maze() != nullptr);
	}
	expected_wall = explore_completed_maze();
	wall_class *completed_wall = expected_wall;
	char *args[] = {const_cast<char *>("shortest"), const_cast<char *>("start"), const_cast<char *>("7"),
		const_cast<char *>("7"), const_cast<char *>("2"), const_cast<char *>("plain700"),
		const_cast<char *>("ram"), const_cast<char *>("120")};
	if(strcmp(scenario, "suction") == 0) { args[5] = const_cast<char *>("suction1200_v0"); expected_type = RUN_CONFIG_SUCTION; }
	if(strcmp(scenario, "acc") == 0) { args[5] = const_cast<char *>("acc1600_v1"); expected_type = RUN_CONFIG_VARIABLE_TURN; }
	if(strcmp(scenario, "plain500") == 0) args[5] = const_cast<char *>("plain500");
	if(strcmp(scenario, "flash") == 0) { args[6] = const_cast<char *>("flash"); expected_wall = nullptr; }
	if(strcmp(scenario, "bad_goal") == 0) args[4] = const_cast<char *>("2147483647");
	if(strcmp(scenario, "bad_number") == 0) args[2] = const_cast<char *>("7garbage");
	if(strcmp(scenario, "bad_profile") == 0) args[5] = const_cast<char *>("unknown");
	if(strcmp(scenario, "no_stream") == 0) stream = false;
	if(strcmp(scenario, "low_battery") == 0) low_battery = true;
	if(strcmp(scenario, "no_path") == 0) test_no_path = true;
	if(strcmp(scenario, "stop_plan") == 0) cancel_plan = true;
	if(strcmp(scenario, "link_timeout") == 0) lose_link = true;
	if(strcmp(scenario, "stop_wait") == 0) { test_sensor = 0; stop_in_wait = true; }
	if(strcmp(scenario, "time_limit") == 0) { test_sensor = 0; args[7] = const_cast<char *>("1"); }
	if(strcmp(scenario, "motion_error") == 0) motion_failure = true;
	if(strcmp(scenario, "stop_motion") == 0) cancel_motion = true;
	if(strcmp(scenario, "no_arm") != 0) RemoteExplore::arm();
	const int result = shell_shortest(8, args);
	const bool success = strcmp(scenario, "complete") == 0 || strcmp(scenario, "suction") == 0 ||
		strcmp(scenario, "acc") == 0 || strcmp(scenario, "flash") == 0 || strcmp(scenario, "plain500") == 0;
	assert(result == (success ? 0 : -1));
	assert(shortest_calls == (success || motion_failure || cancel_motion ? 1 : 0));
	assert(flash_reads == (strcmp(scenario, "flash") == 0 ? 1 : 0));
	assert(!RemoteExplore::running() && LogData::getInstance().log_enable == False);
	assert(explore_completed_maze() == completed_wall);
	return 0;
}
