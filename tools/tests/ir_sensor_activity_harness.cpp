#include <cassert>
#include "Peripheral/Inc/ir_sensor_activity.h"
extern "C" void test_ir_sensor_driver(void);

struct SensorValue { int16_t value; };
class IrSensTask {
public:
	SensorValue sen_l{}, sen_r{};
	int16_t IrSensor_Avg();
	bool HandStartDetected(int16_t threshold);
};
#include "hand_start_detected.inc"

enum t_bool { False, True };
class Motion {
	t_bool is_motion_enable = False;
public:
#include "motion_activity_setters.inc"
};

int main()
{
	test_ir_sensor_driver();
	{
		SensorHandWait waiting(true);
		Sensor_AdcComplete(); Sensor_AdcComplete();
		assert(Sensor_IrSamplesReady());
	}
	assert(!Sensor_IrSamplesReady());
	IrSensTask ir;
	ir.sen_l.value = ir.sen_r.value = 3000;  // stale value from previous wait
	assert(ir.IrSensor_Avg() == 0);
	{
		SensorHandWait waiting(true);
		assert(ir.IrSensor_Avg() == 0);  // blocking shell/explore wait
		Sensor_AdcComplete(); assert(ir.IrSensor_Avg() == 0);
		Sensor_AdcComplete(); assert(ir.IrSensor_Avg() == 3000);
	}
	assert(ir.IrSensor_Avg() == 0);
	assert(!ir.HandStartDetected(2000));
	Sensor_AdcComplete(); assert(!ir.HandStartDetected(2000));
	Sensor_AdcComplete();
	ir.sen_l.value = ir.sen_r.value = 2000;
	assert(!ir.HandStartDetected(2000));  // preserve strict > comparison
	ir.sen_l.value = ir.sen_r.value = 2501;
	assert(ir.HandStartDetected(2500));
	assert(!Sensor_IrSamplesReady());  // LEDs off during the pre-run countdown
	assert(!ir.HandStartDetected(2500));  // never accept the stale trigger
	Sensor_SetHandWaitActive(0U);
	Motion motion;
	motion.motion_enable_set();
	Sensor_AdcComplete(); Sensor_AdcComplete();
	assert(Sensor_IrSamplesReady());
	{
		SensorHandWait waiting(true);
	}  // releasing a wait must not darken an active run
	assert(Sensor_IrSamplesReady());
	motion.motion_disable_set();
	assert(!Sensor_IrSamplesReady());
}
