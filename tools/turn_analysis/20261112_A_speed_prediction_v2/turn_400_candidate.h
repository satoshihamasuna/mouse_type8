// 機体A / 400 mm/s / モデル予測候補。実機未確認・未採用。
// 生成元: tools/analysis/turn_speed_prediction.py。測定済みRAM PID/FFを保持。
// Paramsやmode配列へは未接続。既存の300/500設定との混用を避ける。
#ifndef MOUSE_A_PREDICTED_TURN_400_H_
#define MOUSE_A_PREDICTED_TURN_400_H_
#include "Params/typedef_run_param.h"

const static t_pid_gain predicted_long_r90_400_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_long_r90_400_om = {0.1f,0.008f,0.0f};
const static t_ff_gain predicted_long_r90_400_ff = {0.870046f,0.114741f,0.094599f,0.002077f,0.000836f,0.00093f,0.085591f,1.569e-06f,0.0f,0.0f};
const static t_turn_param_table predicted_long_r90_400_table = {0.4f,-37.5f,44.117f,34.894f,-90.0f,Turn_R,0.0f};
const static t_param predicted_long_r90_400 = {&predicted_long_r90_400_table,&predicted_long_r90_400_sp,&predicted_long_r90_400_om,&predicted_long_r90_400_ff};

const static t_pid_gain predicted_long_l90_400_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_long_l90_400_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_long_l90_400_ff = {0.870046f,0.114741f,0.094599f,0.003051f,0.000741f,0.000965f,0.085591f,1.971e-06f,0.0f,0.0f};
const static t_turn_param_table predicted_long_l90_400_table = {0.4f,37.5f,41.831f,33.869f,90.0f,Turn_L,-0.5f};
const static t_param predicted_long_l90_400 = {&predicted_long_l90_400_table,&predicted_long_l90_400_sp,&predicted_long_l90_400_om,&predicted_long_l90_400_ff};

const static t_pid_gain predicted_long_r180_400_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_long_r180_400_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_long_r180_400_ff = {0.870046f,0.114741f,0.094599f,0.002126f,0.000531f,0.001443f,0.085591f,0.0f,0.0f,0.0f};
const static t_turn_param_table predicted_long_r180_400_table = {0.4f,-45.0f,23.581f,16.347f,-180.0f,Turn_R,0.0f};
const static t_param predicted_long_r180_400 = {&predicted_long_r180_400_table,&predicted_long_r180_400_sp,&predicted_long_r180_400_om,&predicted_long_r180_400_ff};

const static t_pid_gain predicted_long_l180_400_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_long_l180_400_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_long_l180_400_ff = {0.870046f,0.114741f,0.094599f,0.002432f,0.000538f,0.001405f,0.085591f,0.0f,0.0f,0.0f};
const static t_turn_param_table predicted_long_l180_400_table = {0.4f,44.7f,23.591f,16.336f,180.0f,Turn_L,0.0f};
const static t_param predicted_long_l180_400 = {&predicted_long_l180_400_table,&predicted_long_l180_400_sp,&predicted_long_l180_400_om,&predicted_long_l180_400_ff};

const static t_pid_gain predicted_in_r45_400_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_in_r45_400_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_in_r45_400_ff = {0.870046f,0.114741f,0.094599f,0.000849f,0.000794f,0.000776f,0.085591f,3.679e-06f,0.0f,0.0f};
const static t_turn_param_table predicted_in_r45_400_table = {0.4f,-50.0f,15.178f,28.022f,-45.0f,Turn_R,0.5f};
const static t_param predicted_in_r45_400 = {&predicted_in_r45_400_table,&predicted_in_r45_400_sp,&predicted_in_r45_400_om,&predicted_in_r45_400_ff};

const static t_pid_gain predicted_in_l45_400_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_in_l45_400_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_in_l45_400_ff = {0.870046f,0.114741f,0.094599f,0.001129f,0.000934f,0.000928f,0.085591f,3.962e-06f,0.0f,0.0f};
const static t_turn_param_table predicted_in_l45_400_table = {0.4f,50.0f,19.946f,25.055f,45.0f,Turn_L,0.0f};
const static t_param predicted_in_l45_400 = {&predicted_in_l45_400_table,&predicted_in_l45_400_sp,&predicted_in_l45_400_om,&predicted_in_l45_400_ff};

const static t_pid_gain predicted_out_r45_400_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_out_r45_400_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_out_r45_400_ff = {0.870046f,0.114741f,0.094599f,0.000849f,0.000794f,0.000776f,0.085591f,3.679e-06f,0.0f,0.0f};
const static t_turn_param_table predicted_out_r45_400_table = {0.4f,-50.0f,37.57f,8.837f,-45.0f,Turn_R,0.5f};
const static t_param predicted_out_r45_400 = {&predicted_out_r45_400_table,&predicted_out_r45_400_sp,&predicted_out_r45_400_om,&predicted_out_r45_400_ff};

const static t_pid_gain predicted_out_l45_400_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_out_l45_400_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_out_l45_400_ff = {0.870046f,0.114741f,0.094599f,0.001129f,0.000934f,0.000928f,0.085591f,3.962e-06f,0.0f,0.0f};
const static t_turn_param_table predicted_out_l45_400_table = {0.4f,50.0f,34.279f,11.429f,45.0f,Turn_L,-0.5f};
const static t_param predicted_out_l45_400 = {&predicted_out_l45_400_table,&predicted_out_l45_400_sp,&predicted_out_l45_400_om,&predicted_out_l45_400_ff};

const static t_pid_gain predicted_in_r135_400_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_in_r135_400_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_in_r135_400_ff = {0.870046f,0.114741f,0.094599f,0.00319f,0.001071f,0.000987f,0.085591f,3.89e-07f,0.0f,0.0f};
const static t_turn_param_table predicted_in_r135_400_table = {0.4f,-30.0f,49.185f,37.485f,-135.0f,Turn_R,0.5f};
const static t_param predicted_in_r135_400 = {&predicted_in_r135_400_table,&predicted_in_r135_400_sp,&predicted_in_r135_400_om,&predicted_in_r135_400_ff};

const static t_pid_gain predicted_in_l135_400_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_in_l135_400_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_in_l135_400_ff = {0.870046f,0.114741f,0.094599f,0.003156f,0.001066f,0.000982f,0.085591f,1.78e-07f,0.0f,0.0f};
const static t_turn_param_table predicted_in_l135_400_table = {0.4f,30.0f,48.845f,35.993f,135.0f,Turn_L,-0.5f};
const static t_param predicted_in_l135_400 = {&predicted_in_l135_400_table,&predicted_in_l135_400_sp,&predicted_in_l135_400_om,&predicted_in_l135_400_ff};

const static t_pid_gain predicted_out_r135_400_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_out_r135_400_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_out_r135_400_ff = {0.870046f,0.114741f,0.094599f,0.00319f,0.001071f,0.000987f,0.085591f,3.89e-07f,0.0f,0.0f};
const static t_turn_param_table predicted_out_r135_400_table = {0.4f,-30.0f,41.925f,45.214f,-135.0f,Turn_R,0.5f};
const static t_param predicted_out_r135_400 = {&predicted_out_r135_400_table,&predicted_out_r135_400_sp,&predicted_out_r135_400_om,&predicted_out_r135_400_ff};

const static t_pid_gain predicted_out_l135_400_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_out_l135_400_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_out_l135_400_ff = {0.870046f,0.114741f,0.094599f,0.003156f,0.001066f,0.000982f,0.085591f,1.78e-07f,0.0f,0.0f};
const static t_turn_param_table predicted_out_l135_400_table = {0.4f,30.0f,43.452f,45.635f,135.0f,Turn_L,-0.5f};
const static t_param predicted_out_l135_400 = {&predicted_out_l135_400_table,&predicted_out_l135_400_sp,&predicted_out_l135_400_om,&predicted_out_l135_400_ff};

const static t_pid_gain predicted_r_v90_400_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_r_v90_400_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_r_v90_400_ff = {0.870046f,0.114741f,0.094599f,0.001678f,0.000675f,0.001023f,0.085591f,7.58e-07f,0.0f,0.0f};
const static t_turn_param_table predicted_r_v90_400_table = {0.4f,-41.0f,11.563f,2.739f,-90.0f,Turn_R,0.5f};
const static t_param predicted_r_v90_400 = {&predicted_r_v90_400_table,&predicted_r_v90_400_sp,&predicted_r_v90_400_om,&predicted_r_v90_400_ff};

const static t_pid_gain predicted_l_v90_400_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_l_v90_400_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_l_v90_400_ff = {0.870046f,0.114741f,0.094599f,0.002037f,0.00065f,0.00104f,0.085591f,1.038e-06f,0.0f,0.0f};
const static t_turn_param_table predicted_l_v90_400_table = {0.4f,41.0f,10.015f,4.442f,90.0f,Turn_L,-0.5f};
const static t_param predicted_l_v90_400 = {&predicted_l_v90_400_table,&predicted_l_v90_400_sp,&predicted_l_v90_400_om,&predicted_l_v90_400_ff};

#endif // MOUSE_A_PREDICTED_TURN_400_H_
