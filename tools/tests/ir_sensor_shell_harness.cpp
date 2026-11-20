#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
extern "C" {
#include "ir_sensor_test_hal.h"
#include "Peripheral/Inc/ir_sensor.h"
int test_ir_leds_on(void);
void test_ir_sensor_values(int base);
}
#include "Peripheral/Inc/ir_sensor_activity.h"

enum t_bool { False, True };
enum t_exeStatus { complete, execute, error };
struct t_sensor { int16_t value = 0; float distance = 0; };
struct IrSensTask { t_sensor sen_fl, sen_fr, sen_l, sen_r; };
struct IrSensTask_type8 {
    static IrSensTask &getInstance() { static IrSensTask ir; return ir; }
};
struct CtrlTask_type8 {
    bool enabled = false;
    t_exeStatus status = complete;
    static CtrlTask_type8 &getInstance() { static CtrlTask_type8 ctrl; return ctrl; }
    t_bool is_control_enable() { return enabled ? True : False; }
    t_exeStatus motion_exeStatus_get() { return status; }
};
static uint32_t ticks;
static int base_value;
static bool adc_available, stop_during_sample;
static std::string output;

static void publish()
{
    auto &ir = IrSensTask_type8::getInstance();
    t_sensor *sensors[] = {&ir.sen_fl, &ir.sen_fr, &ir.sen_l, &ir.sen_r};
    const t_sensor_dir dirs[] = {sensor_fl, sensor_fr, sensor_sl, sensor_sr};
    for (int i = 0; i < 4; ++i) {
        sensors[i]->value = Sensor_GetValue(dirs[i]);
        sensors[i]->distance = sensors[i]->value / 10.0f;
    }
}
static uint32_t HAL_GetTick() { return ticks; }
static void HAL_Delay(uint32_t ms)
{
    assert(test_irq_mask == 0U); // No waiting with the control IRQ masked.
    if (ms > 1) {
        base_value += 100;
        if (stop_during_sample) { Sensor_StopADC(); adc_available = false; }
    }
    for (uint32_t i = 0; i < ms; ++i) {
        ++ticks;
        if (adc_available) {
            test_ir_sensor_values(base_value);
            Sensor_AdcComplete();
            publish();
        }
    }
}
static void __enable_irq() { __set_PRIMASK(0U); }
static int ntlibc_atoi(const char *s) { return std::atoi(s); }
static int ntlibc_strcmp(const char *a, const char *b) { return std::strcmp(a,b); }
static float read_gyro_x_axis() { return 0; }
static float read_gyro_y_axis() { return 0; }
static float read_gyro_z_axis() { return 0; }
static float read_accel_x_axis() { return 0; }
static float read_accel_y_axis() { return 0; }
static float read_accel_z_axis() { return 0; }
static int capture_printf(const char *format, ...)
{
    assert(test_irq_mask == 0U); // UART work stays outside the IRQ mask.
    char buffer[2048];
    va_list args; va_start(args,format);
    const int count = std::vsnprintf(buffer,sizeof(buffer),format,args);
    va_end(args);
    assert(count >= 0 && count < static_cast<int>(sizeof(buffer)));
    const std::string line(buffer);
    if (line.find("IRSENS index:") == 0 || line.find("PHOTO index:") == 0)
        assert(Sensor_IrSamplesReady() && test_ir_leds_on());
    if (line.find("IRSENS_AVG") == 0 || line.find("IRSENS_END") == 0 || line.find("PHOTO_END") == 0)
        assert(!test_ir_leds_on());
    output += line;
    return count;
}
#define printf capture_printf
#include "ir_shell_commands.inc"
#undef printf

static void reset()
{
    CtrlTask_type8::getInstance().enabled = false;
    CtrlTask_type8::getInstance().status = complete;
    test_irq_mask = 0;
    Sensor_Initialize();
    ticks = 0; base_value = 100;
    adc_available = true; stop_during_sample = false;
    output.clear();
    assert(!test_ir_leds_on());
}
static int irsens(const char *count, const char *interval = "20", const char *name = "irsens")
{
    const char *args[] = {name,count,interval};
    return usrcmd_irsens(interval ? 3 : 2,const_cast<char **>(args));
}
static int interface(const char *kind, const char *count = "2")
{
    const char *args[] = {"interface",kind,count,"20"};
    return usrcmd_interface(4,const_cast<char **>(args));
}
int main()
{
    reset();
    assert(irsens("3") == 0);
    assert(ticks == 42U); // Two ADC completions precede the first of three samples.
    assert(output.find("IRSENS index:0 value_fl:100 value_fr:110 value_sl:120 value_sr:130") != std::string::npos);
    assert(output.find("IRSENS_AVG count:3 value_fl:200.000 value_fr:210.000 value_sl:220.000 value_sr:230.000") != std::string::npos);
    assert(output.find("IRSENS_END count:3") != std::string::npos && !test_ir_leds_on());
    assert(!CtrlTask_type8::getInstance().enabled && CtrlTask_type8::getInstance().status == complete);
    assert(test_adc_stops == 0U); // Battery acquisition remains running.

    reset(); assert(irsens("1",nullptr,"ir_sensor") == 0 && ticks == 2U);
    reset(); assert(irsens("0") == -1 && ticks == 0U && !test_ir_leds_on());
    reset(); assert(irsens("1001") == -1 && ticks == 0U && !test_ir_leds_on());
    reset(); assert(irsens("1","1001") == -1 && !test_ir_leds_on());

    reset(); adc_available = false;
    assert(irsens("2") == -1 && ticks == 50U && !test_ir_leds_on());
    assert(output == "IRSENS_ERROR adc_timeout\r\n");
    reset(); ticks = UINT32_MAX - 20U; adc_available = false;
    assert(irsens("2") == -1 && !test_ir_leds_on()); // Tick wrap does not defeat timeout.

    reset(); stop_during_sample = true;
    assert(irsens("2") == -1 && !test_ir_leds_on());
    assert(output.find("IRSENS_ERROR adc_not_ready") != std::string::npos);
    assert(output.find("IRSENS_AVG") == std::string::npos && output.find("IRSENS_END") == std::string::npos);

    for (int state = 0; state < 3; ++state) {
        reset();
        if (state == 0) CtrlTask_type8::getInstance().enabled = true;
        if (state == 1) CtrlTask_type8::getInstance().status = execute;
        if (state == 2) Sensor_SetHandWaitActive(1U);
        assert(irsens("2") == -1 && output == "IRSENS_ERROR busy\r\n" && ticks == 0U);
        assert(test_ir_leds_on() == (state == 2));
        assert(CtrlTask_type8::getInstance().enabled == (state == 0));
        assert(CtrlTask_type8::getInstance().status == (state == 1 ? execute : complete));
        Sensor_SetHandWaitActive(0U);
    }
    for (const char *kind : {"ir","photo"}) {
        reset(); assert(interface(kind) == 0 && !test_ir_leds_on());
        assert(output.find("raw_fr_on:160 raw_fr_off:50") != std::string::npos);
        assert(output.find("PHOTO_END") != std::string::npos);
        reset(); adc_available = false;
        assert(interface(kind) == -1 && output == "PHOTO_ERROR adc_timeout\r\n" && !test_ir_leds_on());
        reset(); CtrlTask_type8::getInstance().enabled = true;
        assert(interface(kind) == -1 && output == "PHOTO_ERROR busy\r\n" && !test_ir_leds_on());
        reset(); stop_during_sample = true;
        assert(interface(kind) == -1 && !test_ir_leds_on());
        assert(output.find("PHOTO_ERROR adc_not_ready") != std::string::npos && output.find("PHOTO_END") == std::string::npos);
    }
    reset(); assert(interface("imu") == 0 && !test_ir_leds_on());
    reset(); assert(interface("ir","0") == -1 && ticks == 0U && !test_ir_leds_on());
    std::puts("IR shell: lit fresh samples, means, aliases, busy refusal and error cleanup passed");
}
