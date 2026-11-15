// 機体A / 600 mm/s / モデル予測候補。実機未確認・未採用。
// 生成元: tools/analysis/turn_speed_prediction.py。測定済みRAM PID/FFを保持。
// Paramsやmode配列へは未接続。既存の300/500設定との混用を避ける。
#ifndef MOUSE_A_PREDICTED_TURN_600_H_
#define MOUSE_A_PREDICTED_TURN_600_H_
#include "Params/typedef_run_param.h"

const static t_pid_gain predicted_long_r90_600_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_long_r90_600_om = {0.1f,0.008f,0.0f};
const static t_ff_gain predicted_long_r90_600_ff = {0.870046f,0.114741f,0.094599f,0.002077f,0.000836f,0.00093f,0.085591f,1.569e-06f,0.0f,0.0f};
const static t_turn_param_table predicted_long_r90_600_table = {0.6f,-37.5f,43.795f,35.184f,-90.0f,Turn_R,0.0f};
const static t_param predicted_long_r90_600 = {&predicted_long_r90_600_table,&predicted_long_r90_600_sp,&predicted_long_r90_600_om,&predicted_long_r90_600_ff};

const static t_pid_gain predicted_long_l90_600_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_long_l90_600_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_long_l90_600_ff = {0.870046f,0.114741f,0.094599f,0.003051f,0.000741f,0.000965f,0.085591f,1.971e-06f,0.0f,0.0f};
const static t_turn_param_table predicted_long_l90_600_table = {0.6f,37.5f,42.163f,33.538f,90.0f,Turn_L,-0.5f};
const static t_param predicted_long_l90_600 = {&predicted_long_l90_600_table,&predicted_long_l90_600_sp,&predicted_long_l90_600_om,&predicted_long_l90_600_ff};

const static t_pid_gain predicted_long_r180_600_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_long_r180_600_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_long_r180_600_ff = {0.870046f,0.114741f,0.094599f,0.002126f,0.000531f,0.001443f,0.085591f,0.0f,0.0f,0.0f};
const static t_turn_param_table predicted_long_r180_600_table = {0.6f,-45.0f,23.679f,16.252f,-180.0f,Turn_R,0.0f};
const static t_param predicted_long_r180_600 = {&predicted_long_r180_600_table,&predicted_long_r180_600_sp,&predicted_long_r180_600_om,&predicted_long_r180_600_ff};

const static t_pid_gain predicted_long_l180_600_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_long_l180_600_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_long_l180_600_ff = {0.870046f,0.114741f,0.094599f,0.002432f,0.000538f,0.001405f,0.085591f,0.0f,0.0f,0.0f};
const static t_turn_param_table predicted_long_l180_600_table = {0.6f,44.7f,23.652f,16.279f,180.0f,Turn_L,0.0f};
const static t_param predicted_long_l180_600 = {&predicted_long_l180_600_table,&predicted_long_l180_600_sp,&predicted_long_l180_600_om,&predicted_long_l180_600_ff};

const static t_pid_gain predicted_in_r45_600_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_in_r45_600_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_in_r45_600_ff = {0.870046f,0.114741f,0.094599f,0.000849f,0.000794f,0.000776f,0.085591f,3.679e-06f,0.0f,0.0f};
const static t_turn_param_table predicted_in_r45_600_table = {0.6f,-50.0f,15.421f,27.778f,-45.0f,Turn_R,0.5f};
const static t_param predicted_in_r45_600 = {&predicted_in_r45_600_table,&predicted_in_r45_600_sp,&predicted_in_r45_600_om,&predicted_in_r45_600_ff};

const static t_pid_gain predicted_in_l45_600_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_in_l45_600_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_in_l45_600_ff = {0.870046f,0.114741f,0.094599f,0.001129f,0.000934f,0.000928f,0.085591f,3.962e-06f,0.0f,0.0f};
const static t_turn_param_table predicted_in_l45_600_table = {0.6f,50.0f,20.055f,24.945f,45.0f,Turn_L,0.0f};
const static t_param predicted_in_l45_600 = {&predicted_in_l45_600_table,&predicted_in_l45_600_sp,&predicted_in_l45_600_om,&predicted_in_l45_600_ff};

const static t_pid_gain predicted_out_r45_600_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_out_r45_600_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_out_r45_600_ff = {0.870046f,0.114741f,0.094599f,0.000849f,0.000794f,0.000776f,0.085591f,3.679e-06f,0.0f,0.0f};
const static t_turn_param_table predicted_out_r45_600_table = {0.6f,-50.0f,36.727f,9.663f,-45.0f,Turn_R,0.5f};
const static t_param predicted_out_r45_600 = {&predicted_out_r45_600_table,&predicted_out_r45_600_sp,&predicted_out_r45_600_om,&predicted_out_r45_600_ff};

const static t_pid_gain predicted_out_l45_600_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_out_l45_600_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_out_l45_600_ff = {0.870046f,0.114741f,0.094599f,0.001129f,0.000934f,0.000928f,0.085591f,3.962e-06f,0.0f,0.0f};
const static t_turn_param_table predicted_out_l45_600_table = {0.6f,50.0f,33.393f,12.292f,45.0f,Turn_L,-0.5f};
const static t_param predicted_out_l45_600 = {&predicted_out_l45_600_table,&predicted_out_l45_600_sp,&predicted_out_l45_600_om,&predicted_out_l45_600_ff};

const static t_pid_gain predicted_in_r135_600_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_in_r135_600_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_in_r135_600_ff = {0.870046f,0.114741f,0.094599f,0.00319f,0.001071f,0.000987f,0.085591f,3.89e-07f,0.0f,0.0f};
const static t_turn_param_table predicted_in_r135_600_table = {0.6f,-30.0f,45.917f,39.284f,-135.0f,Turn_R,0.5f};
const static t_param predicted_in_r135_600 = {&predicted_in_r135_600_table,&predicted_in_r135_600_sp,&predicted_in_r135_600_om,&predicted_in_r135_600_ff};

const static t_pid_gain predicted_in_l135_600_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_in_l135_600_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_in_l135_600_ff = {0.870046f,0.114741f,0.094599f,0.003156f,0.001066f,0.000982f,0.085591f,1.78e-07f,0.0f,0.0f};
const static t_turn_param_table predicted_in_l135_600_table = {0.6f,30.0f,47.336f,37.103f,135.0f,Turn_L,-0.5f};
const static t_param predicted_in_l135_600 = {&predicted_in_l135_600_table,&predicted_in_l135_600_sp,&predicted_in_l135_600_om,&predicted_in_l135_600_ff};

const static t_pid_gain predicted_out_r135_600_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_out_r135_600_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_out_r135_600_ff = {0.870046f,0.114741f,0.094599f,0.00319f,0.001071f,0.000987f,0.085591f,3.89e-07f,0.0f,0.0f};
const static t_turn_param_table predicted_out_r135_600_table = {0.6f,-30.0f,39.079f,47.013f,-135.0f,Turn_R,0.5f};
const static t_param predicted_out_r135_600 = {&predicted_out_r135_600_table,&predicted_out_r135_600_sp,&predicted_out_r135_600_om,&predicted_out_r135_600_ff};

const static t_pid_gain predicted_out_l135_600_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_out_l135_600_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_out_l135_600_ff = {0.870046f,0.114741f,0.094599f,0.003156f,0.001066f,0.000982f,0.085591f,1.78e-07f,0.0f,0.0f};
const static t_turn_param_table predicted_out_l135_600_table = {0.6f,30.0f,42.151f,46.596f,135.0f,Turn_L,-0.5f};
const static t_param predicted_out_l135_600 = {&predicted_out_l135_600_table,&predicted_out_l135_600_sp,&predicted_out_l135_600_om,&predicted_out_l135_600_ff};

const static t_pid_gain predicted_r_v90_600_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_r_v90_600_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_r_v90_600_ff = {0.870046f,0.114741f,0.094599f,0.001678f,0.000675f,0.001023f,0.085591f,7.58e-07f,0.0f,0.0f};
const static t_turn_param_table predicted_r_v90_600_table = {0.6f,-41.0f,9.354f,4.658f,-90.0f,Turn_R,0.5f};
const static t_param predicted_r_v90_600 = {&predicted_r_v90_600_table,&predicted_r_v90_600_sp,&predicted_r_v90_600_om,&predicted_r_v90_600_ff};

const static t_pid_gain predicted_l_v90_600_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_l_v90_600_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_l_v90_600_ff = {0.870046f,0.114741f,0.094599f,0.002037f,0.00065f,0.00104f,0.085591f,1.038e-06f,0.0f,0.0f};
const static t_turn_param_table predicted_l_v90_600_table = {0.6f,41.0f,8.377f,5.917f,90.0f,Turn_L,-0.5f};
const static t_param predicted_l_v90_600 = {&predicted_l_v90_600_table,&predicted_l_v90_600_sp,&predicted_l_v90_600_om,&predicted_l_v90_600_ff};

#endif // MOUSE_A_PREDICTED_TURN_600_H_
