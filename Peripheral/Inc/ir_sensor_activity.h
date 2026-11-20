#ifndef INC_IR_SENSOR_ACTIVITY_H_
#define INC_IR_SENSOR_ACTIVITY_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 同じ停止状態の再通知では、新しく開始した手かざし待機を解除しない。
// 待機の明示的な中断はSensor_SetHandWaitActive(0)で行う。
void Sensor_SetMotionActive(uint8_t active);
void Sensor_SetHandWaitActive(uint8_t active);
// 停止中の前景測定だけが取得できる発光要求。走行・待機中は取得しない。
uint8_t Sensor_BeginMeasurement(void);
void Sensor_EndMeasurement(void);
void Sensor_AdcComplete(void);
uint8_t Sensor_IrSamplesReady(void);

#ifdef __cplusplus
}

// Foreground wait ownership: cancellation and early returns release the LEDs.
class SensorHandWait {
public:
	explicit SensorHandWait(bool active = false) { set(active); }
	~SensorHandWait() { set(false); }
	void set(bool active) { Sensor_SetHandWaitActive(active ? 1U : 0U); }
	SensorHandWait(const SensorHandWait &) = delete;
	SensorHandWait &operator=(const SensorHandWait &) = delete;
};
#endif

#endif
