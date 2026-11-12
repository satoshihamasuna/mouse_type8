#include <assert.h>
#include <stdlib.h>
#include "ir_sensor_test_hal.h"
GPIO_TypeDef test_gpio;
ADC_HandleTypeDef hadc1;
TestHandle hlptim1, handle_GPDMA1_Channel0, handle_GPDMA1_Channel1;
TestHandle Node_GPDMA1_Channel0, Node_GPDMA1_Channel1;
TestHandle List_GPDMA1_Channel0, List_GPDMA1_Channel1;
uint32_t test_irq_mask, test_adc_starts, test_adc_stops, test_timer_starts;
void (*test_irq_restored_hook)(void);
void Error_Handler(void) { abort(); }

// Include the real DMA/ADC driver; only the MCU register boundary is replaced.
#include "Peripheral/Src/ir_sensor.c"

static void assert_off(void)
{
	for (int i = 0; i < NUM_ADC; ++i) assert(led_on_pattern[i] == 0U);
	assert(test_gpio.BSRR == (SENSOR_ALL_PATTERN << 16));
	assert(!Sensor_IrSamplesReady());
}

int test_ir_leds_on(void) { return leds_enabled != 0U; }

void test_ir_sensor_driver(void)
{
	Sensor_Initialize();
	assert_off();
	assert(test_adc_starts == 1U && test_timer_starts == 2U);
	adc_value[LED_FL_ON] = 2500; adc_value[LED_FL_OFF] = 100;
	adc_value[8] = 100; adc_value[9] = 200;
	assert(Sensor_GetValue(sensor_fl) == 0 && Sensor_GetBatteryValue() == 150);
	Sensor_SetHandWaitActive(1U);
	assert(led_on_pattern[LED_FR_ON] == IR1_Pin);
	assert(led_on_pattern[LED_SR_ON] == IR2_Pin);
	assert(led_on_pattern[LED_SL_ON] == IR3_Pin);
	assert(led_on_pattern[LED_FL_ON] == IR4_Pin);
	assert(!Sensor_IrSamplesReady() && Sensor_GetValue(sensor_fl) == 0);
	Sensor_AdcComplete(); assert(!Sensor_IrSamplesReady());
	Sensor_AdcComplete(); assert(Sensor_IrSamplesReady());
	assert(Sensor_GetValue(sensor_fl) == 2400);
	Sensor_SetMotionActive(1U);  // seamless hand-wait -> running
	Sensor_SetHandWaitActive(0U);
	assert(Sensor_IrSamplesReady());
	test_gpio.BSRR = SENSOR_ALL_PATTERN;  // simulate a DMA turn-on write
	Sensor_SetMotionActive(0U); assert_off();
	assert(Sensor_GetValue(sensor_fl) == 0 && Sensor_GetBatteryValue() == 150);
	assert(test_adc_stops == 0U);  // idle LEDs must not stop battery sampling
	Sensor_SetHandWaitActive(1U);
	Sensor_AdcComplete(); assert(!Sensor_IrSamplesReady());
	Sensor_AdcComplete(); assert(Sensor_IrSamplesReady());
	Sensor_SetMotionActive(0U);
	assert(Sensor_IrSamplesReady());  // repeated idle/error updates preserve a newly armed wait
	Sensor_SetHandWaitActive(0U); assert_off();  // explicit wait cancellation
	test_irq_mask = 1U;
	Sensor_SetHandWaitActive(1U); assert(test_irq_mask == 1U);
	Sensor_SetHandWaitActive(0U); assert(test_irq_mask == 1U);
	test_irq_mask = 0U;
	Sensor_SetMotionActive(1U); assert(test_irq_mask == 0U);
	Sensor_StopADC(); assert_off(); assert(test_adc_stops == 1U);
	Sensor_SetHandWaitActive(1U);  // stopping ADC also cancels an idle hand wait
	Sensor_StopADC(); assert_off(); assert(test_adc_stops == 2U);
}
