/*
 * log_data.h
 *
 *  Created on: 2023/06/20
 *      Author: sato1
 */

#ifndef CPP_INC_LOG_DATA_H_
#define CPP_INC_LOG_DATA_H_

#include "Component/Inc/typedef.h"
#include "Component/Inc/half_float.h"
#include "Component/Inc/singleton.h"
#include "Module/Inc/log_buffer.h"

// L8S5 / saved magic A55D: 49 binary16 values, 4 signed ADC words,
// packed IR range states and motion pattern. Total size stays 55 words.
// Former ideal.turn_x/y slots now carry heading/sigma [rad], NaN if invalid.
// LogBuffer copies words verbatim; the integer trailer is never binary16.
namespace LogLayout {
constexpr uint16_t HALF_FIELDS = 49;
constexpr uint16_t ADC_FIELDS = 4;
constexpr uint16_t FIELD_COUNT = HALF_FIELDS + ADC_FIELDS + 2;
constexpr uint16_t RANGE_INDEX = HALF_FIELDS + ADC_FIELDS;
constexpr uint16_t MOTION_INDEX = RANGE_INDEX + 1;

inline uint16_t pack_ranges(uint8_t left, uint8_t right, uint8_t front_left, uint8_t front_right, uint8_t pillar_guard_mask = 0)
{
    return (left & 7U) | ((right & 7U) << 3) | ((front_left & 7U) << 6) | ((front_right & 7U) << 9)
        | ((pillar_guard_mask & 3U) << 12);
}

inline void set_ir_fields(half_float *words, const int16_t *adc, uint16_t ranges, uint16_t motion)
{
    for (uint16_t i = 0; i < ADC_FIELDS; ++i) words[HALF_FIELDS + i].bitPattern = static_cast<uint16_t>(adc[i]);
    words[RANGE_INDEX].bitPattern = ranges;
    words[MOTION_INDEX].bitPattern = motion;
}
}


#define LOG_DATA_SIZE 400
#define LOG_DATA_NUM  LogLayout::FIELD_COUNT
#define LOG_DATA_PRIOD 1

#define LOG_MAGIC 0xA55D
#define LOG_MAGIC_END 0xFFFF

class LogData:public Singleton<LogData>
{

	public:
	    volatile t_bool log_enable = False;
		const int data_size = LOG_DATA_SIZE;
		const int data_num  = LOG_DATA_NUM;
		uint8_t mode = 0;
		void indicate_data();
		void indicate_data_binary();
		void reset_log(uint16_t period_ms = LOG_DATA_PRIOD);
		uint16_t period_ms() const { return saved_period_ms; }
		void init_log()
		{
			log_enable = False;
			reset_log();
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
	private:
		LogBuffer<LOG_DATA_SIZE, LOG_DATA_NUM> buffer;
		uint16_t log_tick = 0;
		uint16_t saved_period_ms = LOG_DATA_PRIOD;
};

typedef struct __attribute__((packed))
{
	uint16_t magic;
	uint16_t index;
    half_float data[LogLayout::HALF_FIELDS];
    int16_t adc[LogLayout::ADC_FIELDS]; // sen_l/r/fl/fr.value: 距離変換前の点灯/消灯ADC差分
    uint16_t ir_range_states; // L/R/FL/FR各3 bit。bit12/13は直進の左/右Open起点保護。
    uint16_t motion_pattern;
} log_frame_t;

static_assert(sizeof(log_frame_t) == 4 + LOG_DATA_NUM * 2, "Saved log layout must match L8S5 words");

#endif /* CPP_INC_LOG_DATA_H_ */
