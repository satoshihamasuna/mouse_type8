#include <cassert>
#include <cstdint>
#include <cstdio>
#include <vector>

enum t_bool { False, True };
enum Direction { North, Dir_None };
enum Status { complete, error };
struct t_position { int x = 0, y = 0; Direction dir = North; };
constexpr int MAZE_GOAL_X = 7, MAZE_GOAL_Y = 7, MAZE_GOAL_SIZE = 1;
static uint32_t primask;
uint32_t __get_PRIMASK() { return primask; }
void __disable_irq() { primask = 1; }
void __set_PRIMASK(uint32_t value) { primask = value; }
namespace WallPositionCorrection { enum class Mode { Legacy, Shadow, Control }; }
namespace TurnPose {
enum class Mode { Legacy, Shadow, Control };
struct Tracker {
	Mode selected = Mode::Shadow;
	Mode mode() const { return selected; }
	void set_mode(Mode value) { assert(primask == 1); selected = value; }
};
}
struct IrSensTask {
	enum class WallEkfMode { Legacy, Shadow, Control };
	WallEkfMode wall_mode = WallEkfMode::Shadow;
	WallPositionCorrection::Mode edge_mode = WallPositionCorrection::Mode::Legacy;
	WallEkfMode WallEkfMode_Get() const { return wall_mode; }
	WallPositionCorrection::Mode WallEdgeMode_Get() const { return edge_mode; }
	void WallEkfMode_Set(WallEkfMode value) {
		assert(primask == 1); wall_mode = value;
#if defined(MOUSE_A)
		// Aでは直進EKFの変更により特徴点モードも変化する。
		edge_mode = value == WallEkfMode::Control ? WallPositionCorrection::Mode::Control
			: value == WallEkfMode::Shadow ? WallPositionCorrection::Mode::Shadow : WallPositionCorrection::Mode::Legacy;
#endif
	}
	void WallEdgeMode_Set(WallPositionCorrection::Mode value) { assert(primask == 1); edge_mode = value; }
	bool HandStartDetected(int threshold) { assert(threshold == 2000); return true; }
};
struct SensorHandWait { void set(bool) {} };
struct Motion {
	Status status = complete;
	Status motion_exeStatus_get() const { return status; }
	void Motion_end() {}
};
struct CtrlTask_type8 : Motion {
	IrSensTask ir;
	static CtrlTask_type8 &getInstance() { static CtrlTask_type8 value; return value; }
	IrSensTask *return_irObj() { return &ir; }
};
struct Interrupt {
	TurnPose::Tracker tracker;
	static Interrupt &getInstance() { static Interrupt value; return value; }
	uint32_t return_time_count() { return 0; }
	TurnPose::Tracker &turn_pose() { return tracker; }
};
static bool expect_ekf, fail_run;
static int item, frame, errors, writes, time_calls, history_calls, wall_ram, history_ram;
static uint8_t led;
static uint32_t expected_mask;
static std::vector<int> actions;
static void check_modes()
{
	const auto &ir = CtrlTask_type8::getInstance().ir;
	assert(primask == expected_mask);
	assert(ir.WallEkfMode_Get() == (expect_ekf ? IrSensTask::WallEkfMode::Control : IrSensTask::WallEkfMode::Legacy));
	assert(ir.WallEdgeMode_Get() == (expect_ekf ? WallPositionCorrection::Mode::Control : WallPositionCorrection::Mode::Legacy));
	assert(Interrupt::getInstance().turn_pose().mode() == (expect_ekf ? TurnPose::Mode::Control : TurnPose::Mode::Legacy));
}
void Battery_LimiterVoltage() { check_modes(); }
void HAL_Delay(uint32_t ms) { assert(ms == 50); check_modes(); }
uint8_t Return_LED_Status() { return led; }
void Indicate_LED(uint8_t value) { led = value; }
struct t_encoder {};
t_encoder Encoder_GetProperty_Left() { return {}; }
t_encoder Encoder_GetProperty_Right() { return {}; }
struct History {
	void history_init() {}
	int get_history_cnt() const { return 0; }
	void history_delete(int) { assert(false); }
};
struct wall_class {
	History wall_history;
	explicit wall_class(IrSensTask *) {}
	void init_maze() {}
};
void write_wall_WorkRam(wall_class *) { ++wall_ram; }
void write_history_WorkRam(History *) { ++history_ram; }
void write_history_flash(History *) { assert(false); }
void write_save_data(wall_class *) { check_modes(); ++writes; }
struct Search {
	void set_search_limit_time(int ms) { assert(ms == 360000 || ms == 123000); }
	void search_param_init() {}
	void reset_search_time() {}
	void set_virtual_wall_protected_area(t_position start, t_position goal, int size) {
		assert(start.x == 0 && start.y == 0 && start.dir == North);
		assert(goal.x == MAZE_GOAL_X && goal.y == MAZE_GOAL_Y && size == MAZE_GOAL_SIZE);
	}
};
namespace Mode {
void Demo();
void Demo2();
void indicate_error() { check_modes(); ++errors; }
void Wall_Histry_Check(wall_class *) { check_modes(); ++history_calls; }
int32_t Seach_Time_Select() { check_modes(); ++time_calls; return 123000; }
uint8_t Select(uint8_t, uint8_t maximum, t_encoder) {
	if (maximum == 1) { assert(++frame <= 4); return frame % 2 == 0 ? 1 : 0; }
	assert(maximum == 15);
	return frame >= 3 ? 15 : item;
}
}
struct RunConfig { int id; };
static const RunConfig run_config_uniform1000{0}, run_config_plain500{1}, run_config_acc1600_v1{2},
	run_config_suction1400_v1_600{3}, run_config_suction1400_v2{4},
	run_config_suction1600_v1{5}, run_config_suction1600_v2{6},
	run_config_acc1600_v2{7}, run_config_acc1600_v3{8};
struct Dijkstra {
	RunConfig selected = run_config_uniform1000;
	explicit Dijkstra(wall_class *) {}
	void run_config_set(const RunConfig *value) { selected = *value; }
	t_bool check_DijkstraPath(t_position, Direction, t_position, int) {
		check_modes(); actions.push_back(100 + selected.id); return fail_run ? False : True;
	}
	void run(const RunConfig *config, Motion *motion) {
		check_modes(); actions.push_back(100 + config->id); motion->status = fail_run ? error : complete;
	}
	void run_Dijkstra(t_position, Direction, t_position, int, const RunConfig *config, Motion *motion) { run(config, motion); }
	void run_Dijkstra_suction(t_position, Direction, t_position, int, const RunConfig *config, Motion *motion) { run(config, motion); }
	void run_Dijkstra_suction_acc(t_position, Direction, t_position, int, const RunConfig *config, Motion *motion) { run(config, motion); }
};
#include "demo_util.inc"
namespace DemoUtil {
t_position run_search(Search *, search_type_t type, t_position, t_position goal, int, wall_class *, Motion *motion) {
	check_modes(); actions.push_back(type); motion->status = fail_run ? error : complete; return goal;
}
}
#include "demo_menu.inc"
#include "demo2_entry.inc"

int main()
{
	for (bool ekf : {true, false}) for (bool failure : {false, true}) {
	for (uint32_t mask : {0U, 1U}) for (int selected = 0; selected < 16; ++selected) {
		expect_ekf = ekf; fail_run = failure; expected_mask = primask = mask;
		item = selected; frame = errors = writes = time_calls = history_calls = wall_ram = history_ram = 0;
		led = 0x10; actions.clear();
		auto &motion = CtrlTask_type8::getInstance();
		motion.status = complete;
		motion.ir.wall_mode = IrSensTask::WallEkfMode::Shadow;
		motion.ir.edge_mode = WallPositionCorrection::Mode::Legacy;
		Interrupt::getInstance().tracker.selected = TurnPose::Mode::Shadow;
		if (ekf) Mode::Demo(); else Mode::Demo2();
		assert(primask == mask);
		assert(motion.ir.wall_mode == IrSensTask::WallEkfMode::Shadow);
		assert(motion.ir.edge_mode == WallPositionCorrection::Mode::Legacy);
		assert(Interrupt::getInstance().tracker.selected == TurnPose::Mode::Shadow);
		assert(wall_ram == 1 && history_ram == 1);
		std::vector<int> expected;
		if (item < 4) {
			const int outward[] = {DemoUtil::ACCELERATED_GOAL_SEARCH_FIRST_PRIORITY, DemoUtil::GOAL_SEARCH_FIRST_PRIORITY,
				DemoUtil::ACCELERATED_GOAL_SEARCH_SECOND_PRIORITY, DemoUtil::ACCELERATED_GOAL_SEARCH_SECOND_PRIORITY};
			const int returning[] = {DemoUtil::ACCELERATED_PRUNED_FULL_EXPLORATION, DemoUtil::GOAL_SEARCH_FIRST_PRIORITY,
				DemoUtil::ACCELERATED_FULL_EXPLORATION, DemoUtil::ACCELERATED_GOAL_SEARCH_SECOND_PRIORITY};
			expected.push_back(outward[item]); if (!failure) expected.push_back(returning[item]);
			assert(writes == (failure ? 0 : 2));
		} else if (item == 4 || item == 5 || (item >= 8 && item <= 14)) {
			expected.push_back(100 + (item == 4 ? run_config_uniform1000.id : item == 5 ? run_config_plain500.id : item - 6));
			assert(writes == 0);
		}
		assert(actions == expected);
		assert(errors == (failure && !expected.empty() ? 1 : 0));
		assert(time_calls == (item == 6 ? 1 : 0));
		assert(history_calls == (item == 7 ? 1 : 0));
	}
	}
	std::puts("demo/demo2: 128 menu, mode, error and IRQ restoration paths passed");
}
