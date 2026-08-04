/*
 * log_data.cpp
 *
 *  Created on: 2023/06/20
 *      Author: sato1
 */



#include "Module/Inc/log_data.h"
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

		"control_ir",
		"control_ir_dot",

        "sen_l.error", "sen_r.error",
        "sen_fl.distance", "sen_fr.distance",

        "speed_ctrl.get_D_peration",
        "omega_ctrl.get_D_peration",

        "speed_ctrl.Kp",
        "speed_ctrl.Ki",
        "speed_ctrl.Kd",
        "omega_ctrl.Kp",
        "omega_ctrl.Ki",
        "omega_ctrl.Kd",

    };

void LogData::indicate_data()
{


    const int COLS = LOG_DATA_NUM;

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
    for(int i = 0; i < 1000; i++)
    {
        printf("LOG,%d,", i);

        for(int j = 0; j < LOG_DATA_NUM; j++) {
            printf("%4.5lf", half_to_float(data[j][i]));
            if(j < LOG_DATA_NUM-1) printf(",");
        }

        printf("\r\n");
    }

    /* ==== 終端 ==== */
    printf("END\r\n");
}

void LogData::indicate_data_binary()
{
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

    log_frame_t frame;

    for(int i = 0; i < 1000; i++)
    {
    	frame.magic = LOG_MAGIC;
    	frame.index = i;

        for(int j = 0; j < LOG_DATA_NUM; j++) {
            frame.data[j] = data[j][i];   // ★ half のまま
        }



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
}

void LogData::logging()
{
	if(log_enable == True)
	{
		data[0][(data_count/LOG_DATA_PRIOD)%data_size]  =  float_to_half(Vehicle_type8::getInstance().ideal.velo.get());
		data[1][(data_count/LOG_DATA_PRIOD)%data_size]  =  float_to_half(Vehicle_type8::getInstance().ego.velo.get());
		data[2][(data_count/LOG_DATA_PRIOD)%data_size]  =  float_to_half(Vehicle_type8::getInstance().ideal.rad_velo.get());
		data[3][(data_count/LOG_DATA_PRIOD)%data_size]  =  float_to_half(Vehicle_type8::getInstance().ego.rad_velo.get());
		data[4][(data_count/LOG_DATA_PRIOD)%data_size]  =  float_to_half(Vehicle_type8::getInstance().ideal.length.get());
		data[5][(data_count/LOG_DATA_PRIOD)%data_size]  =  float_to_half(Vehicle_type8::getInstance().ego.length.get());
		data[6][(data_count/LOG_DATA_PRIOD)%data_size]  =  float_to_half(Vehicle_type8::getInstance().ideal.radian.get());
		data[7][(data_count/LOG_DATA_PRIOD)%data_size]  =  float_to_half(Vehicle_type8::getInstance().ego.radian.get());

		data[8][(data_count/LOG_DATA_PRIOD)%data_size]  =  float_to_half(Vehicle_type8::getInstance().V_r);
		data[9][(data_count/LOG_DATA_PRIOD)%data_size]  =  float_to_half(Vehicle_type8::getInstance().V_l);
		data[10][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().sp_feedback.get());
		data[11][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().sp_feedforward.get());
		data[12][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().om_feedback.get());
		data[13][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().om_feedforward.get());

		data[14][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(IrSensTask_type8::getInstance().sen_l.avg_distance);
		data[15][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(IrSensTask_type8::getInstance().sen_r.avg_distance);
		data[16][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(IrSensTask_type8::getInstance().sen_l.distance);
		data[17][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(IrSensTask_type8::getInstance().sen_r.distance);

		data[18][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().ego.x_point.get());
		data[19][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().ideal.x_point.get());
		data[20][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().ego.turn_x.get());
		data[21][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().ideal.turn_x.get());
		data[22][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().ego.turn_y.get());
		data[23][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().ideal.turn_y.get());
		data[24][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().ego.turn_slip_theta.get());
		data[25][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().ideal.turn_slip_theta.get());
		data[26][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().ego.horizon_accel.get());
		data[27][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().ideal.horizon_accel.get());
		data[28][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().ego.horizon_velo.get());
		data[29][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().ideal.horizon_velo.get());

		data[30][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Encoder_GetProperty_Right().sp_pulse);
		data[31][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Encoder_GetProperty_Left().sp_pulse);
		data[32][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().battery.get());
		data[33][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().ideal.accel.get());
		data[34][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().ego.accel.get());


		data[35][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().Vehicle_controller.speed_ctrl.get_I_peration());
		data[36][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().Vehicle_controller.omega_ctrl.get_I_peration());

		data[37][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(IrSensTask_type8::getInstance().control_ir.get());
		data[38][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(IrSensTask_type8::getInstance().control_ir_dot.get());

		data[39][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(IrSensTask_type8::getInstance().sen_l.error);
		data[40][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(IrSensTask_type8::getInstance().sen_r.error);
		data[41][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(IrSensTask_type8::getInstance().sen_fl.distance);
		data[42][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(IrSensTask_type8::getInstance().sen_fr.distance);

		data[43][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().Vehicle_controller.speed_ctrl.get_D_peration());
		data[44][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().Vehicle_controller.omega_ctrl.get_D_peration());

		data[45][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().Vehicle_controller.speed_ctrl.Kp);
		data[46][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().Vehicle_controller.speed_ctrl.Ki);
		data[47][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().Vehicle_controller.speed_ctrl.Kd);
		data[48][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().Vehicle_controller.omega_ctrl.Kp);
		data[49][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().Vehicle_controller.omega_ctrl.Ki);
		data[50][(data_count/LOG_DATA_PRIOD)%data_size] =  float_to_half(Vehicle_type8::getInstance().Vehicle_controller.omega_ctrl.Kd);

		data_count++;
		if(data_count >= data_size*LOG_DATA_PRIOD) data_count = (data_size*LOG_DATA_PRIOD) - 1;
	}
}
