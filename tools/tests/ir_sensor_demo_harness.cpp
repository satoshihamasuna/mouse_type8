#include <cassert>
#include <cstdint>
#include <cstdio>
#include "Peripheral/Inc/ir_sensor_activity.h"

extern "C" void Sensor_Initialize(void);
extern "C" int test_ir_leds_on(void);
extern "C" void (*test_irq_restored_hook)(void);

struct SensorValue { int16_t value = 0; };
class IrSensTask {
public:
	SensorValue sen_l, sen_r;
	int16_t IrSensor_Avg();
	bool HandStartDetected(int16_t threshold);
	void ResetWallCornerDetection() {}
	void ResetStraightWallControl() {}
};
#include "hand_start_detected.inc"

enum t_bool { False, True };
enum t_exeStatus { complete, execute, error };
enum t_run_pattern { No_run };
enum t_runControl { NOP_STATE };
struct Field { void init() {} void set(float) {} };
struct Controller { void I_param_reset() {} };
struct Vehicle {
	float V_r = 0, V_l = 0;
	int motor_out_r = 0, motor_out_l = 0;
	Field turn_slip_k;
	struct { Controller speed_ctrl, omega_ctrl; } Vehicle_controller;
	void ego_initialize() {}
	void ideal_initialize() {}
	void suctionStop() {}
};
static Vehicle vehicle;
static IrSensTask ir;
struct Vehicle_type8 { static Vehicle &getInstance() { return vehicle; } };

class Motion {
	t_bool is_motion_enable = False;
public:
	Vehicle *vehicle = &::vehicle;
	IrSensTask *ir_sens = &ir;
	struct { void clear() {} } search_turn_heading;
	struct {
		Field velo, max_velo, end_velo, accel, deccel, end_length, length_accel, length_deccel;
		Field rad_accel, rad_deccel, rad_max_velo, end_radian, radian_accel, radian_deccel;
		Field turn_r_min, turn_state, turn_time_ms;
	} motion_plan;
	t_exeStatus status = complete;
	int error_cnt = 0;
#include "motion_activity_setters.inc"
	t_bool motion_is_enable_get() { return is_motion_enable; }
	t_exeStatus motion_exeStatus_get() { return status; }
	void motion_exeStatus_set(t_exeStatus value) { status = value; }
	void motion_pattern_set(t_run_pattern) {}
	void motion_control_set(t_runControl) {}
	void run_time_ms_reset() {}
	void run_time_limit_ms_reset() {}
	void error_counter_reset() { error_cnt = 0; }
	void Motion_start();
	void Motion_end();
	void Motion_error_handling();
};

class CtrlTask_type8 : public Motion {
public:
	static CtrlTask_type8 &getInstance() { static CtrlTask_type8 value; return value; }
	IrSensTask *return_irObj() { return &ir; }
	void motion_prev_control() {}
	void motion_control() {}
	void motion_post_control() {
		if (error_cnt >= 1000) motion_exeStatus_set(error);
		if (motion_exeStatus_get() == error) Motion_error_handling();
	}
};

static uint32_t now_ms;
static bool remote_stop;
static bool menu_active, menu_delay_started, action_done, selecting_exit, cancelled;
static int selected_case, menu_enable, menu_frames, waiting_frames, action_calls;
static bool request_cancel;
static bool inject_irq_at_enable;
namespace RemoteExplore { bool should_stop(uint32_t) { return remote_stop; } }
uint32_t HAL_GetTick() { return now_ms; }
void Motor_SetDuty_Left(int value) { assert(value == 0); }
void Motor_SetDuty_Right(int value) { assert(value == 0); }
void Motor_Stop() {}
void HAL_Delay(uint32_t ms);
void Motor_Initialize()
{
	HAL_Delay(200); // Real motor initialization also waits with the control IRQ running.
	if (inject_irq_at_enable) {
		inject_irq_at_enable = false;
		test_irq_restored_hook = [] { CtrlTask_type8::getInstance().motion_post_control(); };
	}
}
class Interrupt {
public:
	static Interrupt &getInstance() { static Interrupt value; return value; }
	uint32_t return_time_count() { return now_ms; }
	void main();
};
#include "ir_activity_interrupt.inc"
#include "motion_activity_lifecycle.inc"

static void tick()
{
	++now_ms;
	Sensor_AdcComplete();
	Interrupt::getInstance().main();
}

void HAL_Delay(uint32_t ms)
{
	if (menu_active) {
		// Every operation/countdown must consume its hand wait before blocking.
		assert(!test_ir_leds_on());
		menu_delay_started = true;
		if (ms == 500) action_done = true;
	}
	for (uint32_t i = 0; i < ms; ++i) tick();
}

static uint8_t indicator;
uint8_t Return_LED_Status() { return indicator; }
void Indicate_LED(uint8_t value) { indicator = value; }
struct t_encoder {};
t_encoder Encoder_GetProperty_Left() { return {}; }
t_encoder Encoder_GetProperty_Right() { return {}; }
t_bool Button_Status()
{
	return waiting_frames >= 4 && !menu_delay_started ? True : False;
}
void Battery_LimiterVoltage()
{
	ir.sen_l.value = ir.sen_r.value = request_cancel && !cancelled ? 0 : 3000;
	if (menu_enable) {
		++waiting_frames;
		assert(test_ir_leds_on()); // Also while a required button is still released.
	} else {
		waiting_frames = 0;
		assert(!test_ir_leds_on());
	}
	tick(); // Normal ADC/control IRQ, including a latched prior motion error.
}
namespace Mode {
uint8_t Select(uint8_t, uint8_t maximum, t_encoder)
{
	if (maximum == 1) {
		assert(++menu_frames < 100); // Turn a broken hand wait into a bounded failure.
		if (action_done && !selecting_exit) {
			selecting_exit = true;
			menu_enable = 0;
		} else if (menu_frames == 1) {
			menu_enable = 0;
		} else if (request_cancel && !cancelled && waiting_frames >= 3) {
			cancelled = true;
			menu_enable = 0;
		} else {
			menu_enable = 1;
			if (waiting_frames == 0) menu_delay_started = false;
		}
		return static_cast<uint8_t>(menu_enable);
	}
	assert(maximum == 15);
	return static_cast<uint8_t>(selecting_exit ? 15 : selected_case);
}
}

static void record_action()
{
	assert(!test_ir_leds_on());
	++action_calls;
}
struct History { void history_indicate() { record_action(); } };
struct wall_class {
	History wall_history;
	void init_maze() { record_action(); }
	void history2wall_append() { record_action(); }
	void indicate_wall() { record_action(); }
};
void read_save_data(wall_class *) { record_action(); }
void read_wall_WorkRam(wall_class *) { record_action(); }
void read_history_WorkRam(History *) { record_action(); }
void write_wall_flash(wall_class *) { record_action(); }
#define ENABLE (0x01 << 4)
#include "wall_history_menu.inc"
#include "search_time_menu.inc"

int main()
{
	Sensor_Initialize();
	auto &motion = CtrlTask_type8::getInstance();
	motion.Motion_start();
	assert(test_ir_leds_on() && motion.motion_is_enable_get() == True);
	motion.error_cnt = 1000;
	motion.motion_exeStatus_set(error);
	tick();
	assert(!test_ir_leds_on() && motion.motion_is_enable_get() == False);
	ir.sen_l.value = ir.sen_r.value = 3000;
	assert(!ir.HandStartDetected(2000));
	tick(); assert(!ir.HandStartDetected(2000));
	tick(); assert(ir.HandStartDetected(2000));
	assert(motion.motion_exeStatus_get() == error); // Waiting never re-enables motors/clears the error.
	assert(motion.motion_is_enable_get() == False && !test_ir_leds_on());
	motion.Motion_start();
	assert(motion.error_cnt == 0 && motion.motion_exeStatus_get() == complete);
	assert(motion.motion_is_enable_get() == True && Sensor_IrSamplesReady());
	motion.Motion_end();
	motion.motion_exeStatus_set(error);
	inject_irq_at_enable = true; // IRQ immediately after enabling must not see the old error.
	motion.Motion_start();
	assert(motion.motion_is_enable_get() == True && motion.motion_exeStatus_get() == complete);
	motion.Motion_end();
	{
		SensorHandWait waiting(true);
		tick(); tick(); assert(Sensor_IrSamplesReady());
		motion.Motion_end(); // Explicit foreground stop cancels even an idle hand wait.
		assert(!test_ir_leds_on());
	}
	{
		SensorHandWait waiting(true);
		tick(); tick(); assert(Sensor_IrSamplesReady());
		remote_stop = true;
		tick(); assert(!test_ir_leds_on()); // Real Interrupt::main remote-stop branch.
		remote_stop = false;
	}
	// Run both real submenus, every case, with and without a preceding cancellation.
	for (int menu = 0; menu != 2; ++menu) {
	for (int cancel = 0; cancel != 2; ++cancel) {
		for (int item = 0; item != 16; ++item) {
			Sensor_SetHandWaitActive(0U);
			menu_active = true;
			menu_delay_started = action_done = selecting_exit = cancelled = false;
			request_cancel = cancel != 0;
			selected_case = item;
			menu_enable = menu_frames = waiting_frames = action_calls = 0;
			wall_class wall;
			if (menu == 0) Mode::Wall_Histry_Check(&wall);
			else {
				int32_t selected = Mode::Seach_Time_Select();
				assert(selected >= 0 && selected <= 6*60*1000);
			}
			menu_active = false;
			assert(!test_ir_leds_on() && motion.motion_is_enable_get() == False);
			const bool has_action = menu == 0 && (item <= 2 || item == 7 || item == 8 || (item >= 12 && item <= 14));
			assert(action_calls == (has_action ? 1 : 0));
			if (cancel) assert(cancelled);
		}
	}
	}
	std::puts("IR activity: error retry, enable IRQ race, explicit/remote stop and 64 submenu paths passed");
}
