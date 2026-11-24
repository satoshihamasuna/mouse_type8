/*
 * log_data.cpp
 *
 *  Created on: 2023/06/20
 *      Author: sato1
 */



#include "Module/Inc/log_data.h"
#include "Module/Inc/log_stream.h"
#include <math.h>
#include "Module/Inc/interrupt_timing.h"
#include "Peripheral/Inc/communicate.h"
#include "stdio.h"

#include "Peripheral/Inc/peripheral.h"

#include "Task/Inc/sensing_task.h"
#include "Task/Inc/ctrl_task.h"

#include "Component/Inc/controller.h"
#include "Component/Inc/half_float.h"
/*
void LogData::indicate_data()
{
	if(mode == 0)
	{
		const char *labels[] = {
		    "cnt",
		    "ideal.velo", "ego.velo",
		    "ideal.rad_velo", "ego.rad_velo",
		    "ideal.length", "ego.length",
		    "ideal.radian", "ego.radian",

		    "V_r", "V_l",
		    "sp_feedback", "sp_feedforward",
		    "om_feedback", "om_feedforward",

		    "sen_l.avg_distance", "sen_r.avg_distance",
		    "sen_l.distance", "sen_r.distance",

		    "ego.x_point", "ideal.x_point",
		    "ego.turn_x", "ideal.turn_x",
		    "ego.turn_y", "ideal.turn_y",
		    "ego.turn_slip_theta", "ideal.turn_slip_theta",
		    "ego.horizon_accel.get", "ideal.horizon_accel.get",
		    "ego.horizon_velo.get", "ideal.horizon_velo.get",

		    "Encoder_GetProperty_Right().sp_pulse",
		    "Encoder_GetProperty_Left().sp_pulse",

			"Battery",

		    "ideal.accel",
		    "ego.accel",

			"speed_ctrl.get_I_peration",
			"omega_ctrl.get_I_peration",
			"test","test","test",

		};

		for (size_t i = 0; i < sizeof(labels) / sizeof(labels[0]); ++i)  {
		    printf("%s,", labels[i]);
		}

		printf("\n");
	}

	for(int i = 0; i< 1000;i++)
	{
		printf("%d,",i);
		HAL_Delay(2);
		printf("%4.4lf,%4.4lf,%4.4lf,%4.4lf,",
				half_to_float(data[0][i]),half_to_float(data[1][i]),
				half_to_float(data[2][i]),half_to_float(data[3][i]));
		HAL_Delay(2);
		printf("%4.4lf,%4.4lf,%4.4lf,%4.4lf,",
				half_to_float(data[4][i]),half_to_float(data[5][i]),
				half_to_float(data[6][i]),half_to_float(data[7][i]));
		HAL_Delay(2);
		printf("%4.4lf,%4.4lf,%4.4lf,%4.4lf,",
				half_to_float(data[8][i]),half_to_float(data[9][i]),
				half_to_float(data[10][i]),half_to_float(data[11][i]));
		HAL_Delay(2);
		printf("%4.4lf,%4.4lf,%4.4lf,%4.4lf,",
				half_to_float(data[12][i]),half_to_float(data[13][i]),
				half_to_float(data[14][i]),half_to_float(data[15][i]));
		HAL_Delay(2);
		printf("%4.4lf,%4.4lf,%4.4lf,%4.4lf,",
				half_to_float(data[16][i]),half_to_float(data[17][i]),
				half_to_float(data[18][i]),half_to_float(data[19][i]));
		HAL_Delay(2);
		printf("%4.4lf,%4.4lf,%4.4lf,%4.4lf,",
				half_to_float(data[20][i]),half_to_float(data[21][i]),
				half_to_float(data[22][i]),half_to_float(data[23][i]));
		HAL_Delay(2);
		printf("%4.4lf,%4.4lf,%4.4lf,%4.4lf,",
				half_to_float(data[24][i]),half_to_float(data[25][i]),
				half_to_float(data[26][i]),half_to_float(data[27][i]));
		HAL_Delay(2);
		printf("%4.4lf,%4.4lf,%4.4lf,%4.4lf,",
				half_to_float(data[28][i]),half_to_float(data[29][i]),
				half_to_float(data[30][i]),half_to_float(data[31][i]));
		HAL_Delay(2);
		printf("%4.4lf,%4.4lf,%4.4lf,%4.4lf,",
				half_to_float(data[32][i]),half_to_float(data[33][i]),
				half_to_float(data[34][i]),half_to_float(data[35][i]));
		HAL_Delay(2);
		printf("%4.4lf,%4.4lf,%4.4lf,%4.4lf",
				half_to_float(data[36][i]),half_to_float(data[37][i]),
				half_to_float(data[38][i]),half_to_float(data[39][i]));
		HAL_Delay(2);

		printf("\n");
		HAL_Delay(2);
	}


}
*/


    static const char *labels[(LOG_DATA_NUM+1)] = {
        "cnt",

        // 速度・姿勢
        "ideal.velo",
        "ego.velo",
        "ideal.accel",
        "ego.accel",
        "ideal.rad_velo",
        "ego.rad_velo",
        "ideal.radian",
        "ego.radian",

        // 位置・滑り
        "ideal.length",
        "ego.length",
        "ideal.x_point",
        "ego.x_point",
        "estimate.heading_rad",
        "ego.turn_x",
        "estimate.sigma_heading_rad",
        "ego.turn_y",
        "ideal.turn_slip_theta",
        "ego.turn_slip_theta",
        "ideal.horizon_accel.get",
        "ego.horizon_accel.get",
        "ideal.horizon_velo.get",
        "ego.horizon_velo.get",

        // モーター・制御出力
        "Encoder_GetProperty_Right().sp_pulse",
        "Encoder_GetProperty_Left().sp_pulse",
        "V_r",
        "V_l",
        "sp_feedback",
        "sp_feedforward",
        "om_feedback",
        "om_feedforward",
        "speed_ctrl.get_I_peration",
        "speed_ctrl.get_D_peration",
        "omega_ctrl.get_I_peration",
        "omega_ctrl.get_D_peration",
        "Battery",

        // 割り込み時間
        "irq.total_us",
        "irq.logging_us",
        "irq.max_us",
        "irq.period_us",

        // 壁距離・壁制御
        "sen_l.distance",
        "sen_r.distance",
        "sen_fl.distance",
        "sen_fr.distance",
        "sen_l.avg_distance",
        "sen_r.avg_distance",
        "sen_l.error",
        "sen_r.error",
        "control_ir",
        "control_ir_dot",

        // ADC差分・範囲状態・走行パターン（整数）
        "sen_l.value",
        "sen_r.value",
        "sen_fl.value",
        "sen_fr.value",
        "ir.range_states",
        "motion.pattern",
    };

void LogData::reset_log(uint16_t period_ms)
{
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	buffer.reset();
	log_tick = 0;
	saved_period_ms = (period_ms >= 1 && period_ms <= 100) ? period_ms : LOG_DATA_PRIOD;
	InterruptTiming::reset();
	__set_PRIMASK(primask);
}

void LogData::indicate_data()
{


    const int COLS = LOG_DATA_NUM;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const t_bool resume_logging = log_enable;
    log_enable = False;
    const uint16_t count = buffer.size();
    __set_PRIMASK(primask);
    half_float sample[LOG_DATA_NUM];
    printf("START\r\n");
    /* ==== ヘッダ ==== */
    if(mode == 0)
    {
    	printf("HEADER,");
        for (int i = 0; i < COLS + 1; i++) {
            printf("%s", labels[i]);
            if(i < COLS) printf(",");
        }
        printf("\r\n");
    }

    /* ==== データ ==== */
    for(uint16_t i = 0; i < count; i++)
    {
        buffer.copy(i, sample);
        printf("LOG,%u,", static_cast<unsigned>(i * saved_period_ms));

        for(int j = 0; j < LOG_DATA_NUM; j++) {
            if(j < LogLayout::HALF_FIELDS) printf("%4.5lf", half_to_float(sample[j]));
            else if(j < LogLayout::RANGE_INDEX) printf("%d", static_cast<int16_t>(sample[j].bitPattern));
            else printf("%u", static_cast<unsigned>(sample[j].bitPattern));
            if(j < LOG_DATA_NUM-1) printf(",");
        }

        printf("\r\n");
    }

    /* ==== 終端 ==== */
    printf("END\r\n");
    log_enable = resume_logging;
}

void LogData::indicate_data_binary()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const t_bool resume_logging = log_enable;
    log_enable = False;
    const uint16_t count = buffer.size();
    __set_PRIMASK(primask);
    half_float sample[LOG_DATA_NUM];
    printf("START\r\n");

    if(mode == 0)
    {
        printf("HEADER,");
        for(int i = 0; i < LOG_DATA_NUM+1; i++) {
            printf("%s", labels[i]);
            if(i < LOG_DATA_NUM) printf(",");
        }
        printf("\r\n");
    }

    printf("BINARY\r\n");

    log_frame_t frame = {};

    for(uint16_t i = 0; i < count; i++)
    {
        buffer.copy(i, sample);
    	frame.magic = LOG_MAGIC;
        frame.index = i * saved_period_ms;

        // Same 16-bit words as the ring buffer; ADC counts keep every bit.
        memcpy(reinterpret_cast<uint8_t *>(&frame) + 4, sample, sizeof(sample));



        Communicate_TxPushBuffer(
            (uint8_t*)&frame,
            sizeof(frame)
        );
        /*
        HAL_UART_Transmit(&huart1,
            (uint8_t*)&frame,
            sizeof(frame),
            HAL_MAX_DELAY);*/
    }

    // ★ バイナリ終了マーカ（ASCIIに戻る合図）
    frame.magic = LOG_MAGIC_END;
    Communicate_TxPushBuffer((uint8_t*)&frame, sizeof(frame));

    printf("END\r\n");
    log_enable = resume_logging;
}

void LogData::logging()
{
	static_assert(LOG_DATA_NUM == LogStream::FIELD_COUNT, "Stream field count must match the log");
	const uint32_t now_ms = HAL_GetTick();
	const bool stream_due = LogStream::sample_due(now_ms);
	if(log_enable == True || stream_due)
	{
		half_float sample[LOG_DATA_NUM];
		const InterruptTiming::Sample timing = InterruptTiming::sample();

		// 速度・姿勢
		sample[0] = float_to_half(Vehicle_type8::getInstance().ideal.velo.get());
		sample[1] = float_to_half(Vehicle_type8::getInstance().ego.velo.get());
		sample[2] = float_to_half(Vehicle_type8::getInstance().ideal.accel.get());
		sample[3] = float_to_half(Vehicle_type8::getInstance().ego.accel.get());
		sample[4] = float_to_half(Vehicle_type8::getInstance().ideal.rad_velo.get());
		sample[5] = float_to_half(Vehicle_type8::getInstance().ego.rad_velo.get());
		sample[6] = float_to_half(Vehicle_type8::getInstance().ideal.radian.get());
		sample[7] = float_to_half(Vehicle_type8::getInstance().ego.radian.get());

		// 位置・滑り
		const auto turn_position = Vehicle_type8::getInstance().pose.log_position();
		const auto heading = CtrlTask_type8::getInstance().HeadingForLog();
		sample[8] = float_to_half(Vehicle_type8::getInstance().ideal.length.get());
		sample[9] = float_to_half(Vehicle_type8::getInstance().ego.length.get());
		sample[10] = float_to_half(Vehicle_type8::getInstance().ideal.x_point.get());
		sample[11] = float_to_half(Vehicle_type8::getInstance().ego.x_point.get());
		// 空き2枠を採用姿勢と標準偏差 [rad] に使用。無効値は両方NaN。
		sample[12] = float_to_half(heading.usable() ? heading.heading_rad : NAN);
		sample[13] = float_to_half(turn_position.y_mm);
		sample[14] = float_to_half(heading.usable() ? sqrtf(heading.variance_rad2) : NAN);
		sample[15] = float_to_half(turn_position.x_mm);
		sample[16] = float_to_half(Vehicle_type8::getInstance().ideal.turn_slip_theta.get());
		sample[17] = float_to_half(Vehicle_type8::getInstance().ego.turn_slip_theta.get());
		sample[18] = float_to_half(Vehicle_type8::getInstance().ideal.horizon_accel.get());
		sample[19] = float_to_half(Vehicle_type8::getInstance().ego.horizon_accel.get());
		sample[20] = float_to_half(Vehicle_type8::getInstance().ideal.horizon_velo.get());
		sample[21] = float_to_half(Vehicle_type8::getInstance().ego.horizon_velo.get());

		// モーター・制御出力
		sample[22] = float_to_half(Encoder_GetProperty_Right().sp_pulse);
		sample[23] = float_to_half(Encoder_GetProperty_Left().sp_pulse);
		sample[24] = float_to_half(Vehicle_type8::getInstance().V_r);
		sample[25] = float_to_half(Vehicle_type8::getInstance().V_l);
		sample[26] = float_to_half(Vehicle_type8::getInstance().sp_feedback.get());
		sample[27] = float_to_half(Vehicle_type8::getInstance().sp_feedforward.get());
		sample[28] = float_to_half(Vehicle_type8::getInstance().om_feedback.get());
		sample[29] = float_to_half(Vehicle_type8::getInstance().om_feedforward.get());
		sample[30] = float_to_half(Vehicle_type8::getInstance().Vehicle_controller.speed_ctrl.get_I_peration());
		sample[31] = float_to_half(Vehicle_type8::getInstance().Vehicle_controller.speed_ctrl.get_D_peration());
		sample[32] = float_to_half(Vehicle_type8::getInstance().Vehicle_controller.omega_ctrl.get_I_peration());
		sample[33] = float_to_half(Vehicle_type8::getInstance().Vehicle_controller.omega_ctrl.get_D_peration());
		sample[34] = float_to_half(Vehicle_type8::getInstance().battery.get());

		// 割り込み時間
		sample[35] = float_to_half(timing.total_us);
		sample[36] = float_to_half(timing.logging_us);
		sample[37] = float_to_half(timing.max_us);
		sample[38] = float_to_half(timing.period_us);

		// 壁距離・壁制御
		sample[39] = float_to_half(IrSensTask_type8::getInstance().sen_l.distance);
		sample[40] = float_to_half(IrSensTask_type8::getInstance().sen_r.distance);
		sample[41] = float_to_half(IrSensTask_type8::getInstance().sen_fl.distance);
		sample[42] = float_to_half(IrSensTask_type8::getInstance().sen_fr.distance);
		sample[43] = float_to_half(IrSensTask_type8::getInstance().sen_l.avg_distance);
		sample[44] = float_to_half(IrSensTask_type8::getInstance().sen_r.avg_distance);
		sample[45] = float_to_half(IrSensTask_type8::getInstance().sen_l.error);
		sample[46] = float_to_half(IrSensTask_type8::getInstance().sen_r.error);
		sample[47] = float_to_half(IrSensTask_type8::getInstance().control_ir.get());
		sample[48] = float_to_half(IrSensTask_type8::getInstance().control_ir_dot.get());

		const auto &ir = IrSensTask_type8::getInstance();
		const int16_t adc[] = {ir.sen_l.value, ir.sen_r.value, ir.sen_fl.value, ir.sen_fr.value};
		LogLayout::set_ir_fields(sample, adc, LogLayout::pack_ranges(
			static_cast<uint8_t>(ir.sen_l.range_state), static_cast<uint8_t>(ir.sen_r.range_state),
			static_cast<uint8_t>(ir.sen_fl.range_state), static_cast<uint8_t>(ir.sen_fr.range_state),ir.straight_pillar_guard_mask),
			static_cast<uint16_t>(CtrlTask_type8::getInstance().motion_pattern_get()));

		if(log_enable == True)
		{
			if(log_tick == 0) buffer.push(sample);
			if (++log_tick >= saved_period_ms) log_tick = 0;
		}
		if(stream_due) LogStream::enqueue_sample(sample, now_ms);
	}
}
