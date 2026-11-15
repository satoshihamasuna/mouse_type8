// 機体A / 300 mm/s / モデル予測候補。実機未確認・未採用。
// 生成元: tools/analysis/turn_speed_prediction.py。測定済みRAM PID/FFを保持。
// Paramsやmode配列へは未接続。既存の300/500設定との混用を避ける。
#ifndef MOUSE_A_PREDICTED_TURN_300_H_
#define MOUSE_A_PREDICTED_TURN_300_H_
#include "Params/typedef_run_param.h"

const static t_pid_gain predicted_long_r90_300_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_long_r90_300_om = {0.1f,0.008f,0.0f};
const static t_ff_gain predicted_long_r90_300_ff = {0.870046f,0.114741f,0.094599f,0.002077f,0.000836f,0.00093f,0.085591f,1.569e-06f,0.0f,0.0f};
const static t_turn_param_table predicted_long_r90_300_table = {0.3f,-37.5f,44.148f,34.867f,-90.0f,Turn_R,0.0f};
const static t_param predicted_long_r90_300 = {&predicted_long_r90_300_table,&predicted_long_r90_300_sp,&predicted_long_r90_300_om,&predicted_long_r90_300_ff};

const static t_pid_gain predicted_long_l90_300_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_long_l90_300_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_long_l90_300_ff = {0.870046f,0.114741f,0.094599f,0.003051f,0.000741f,0.000965f,0.085591f,1.971e-06f,0.0f,0.0f};
const static t_turn_param_table predicted_long_l90_300_table = {0.3f,37.5f,41.656f,34.046f,90.0f,Turn_L,-0.5f};
const static t_param predicted_long_l90_300 = {&predicted_long_l90_300_table,&predicted_long_l90_300_sp,&predicted_long_l90_300_om,&predicted_long_l90_300_ff};

const static t_pid_gain predicted_long_r180_300_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_long_r180_300_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_long_r180_300_ff = {0.870046f,0.114741f,0.094599f,0.002126f,0.000531f,0.001443f,0.085591f,0.0f,0.0f,0.0f};
const static t_turn_param_table predicted_long_r180_300_table = {0.3f,-45.001f,23.532f,16.395f,-180.0f,Turn_R,0.0f};
const static t_param predicted_long_r180_300 = {&predicted_long_r180_300_table,&predicted_long_r180_300_sp,&predicted_long_r180_300_om,&predicted_long_r180_300_ff};

const static t_pid_gain predicted_long_l180_300_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_long_l180_300_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_long_l180_300_ff = {0.870046f,0.114741f,0.094599f,0.002432f,0.000538f,0.001405f,0.085591f,0.0f,0.0f,0.0f};
const static t_turn_param_table predicted_long_l180_300_table = {0.3f,44.7f,23.535f,16.391f,180.0f,Turn_L,0.0f};
const static t_param predicted_long_l180_300 = {&predicted_long_l180_300_table,&predicted_long_l180_300_sp,&predicted_long_l180_300_om,&predicted_long_l180_300_ff};

const static t_pid_gain predicted_in_r45_300_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_in_r45_300_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_in_r45_300_ff = {0.870046f,0.114741f,0.094599f,0.000849f,0.000794f,0.000776f,0.085591f,3.679e-06f,0.0f,0.0f};
const static t_turn_param_table predicted_in_r45_300_table = {0.3f,-50.0f,15.057f,28.145f,-45.0f,Turn_R,0.5f};
const static t_param predicted_in_r45_300 = {&predicted_in_r45_300_table,&predicted_in_r45_300_sp,&predicted_in_r45_300_om,&predicted_in_r45_300_ff};

const static t_pid_gain predicted_in_l45_300_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_in_l45_300_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_in_l45_300_ff = {0.870046f,0.114741f,0.094599f,0.001129f,0.000934f,0.000928f,0.085591f,3.962e-06f,0.0f,0.0f};
const static t_turn_param_table predicted_in_l45_300_table = {0.3f,50.0f,19.892f,25.109f,45.0f,Turn_L,0.0f};
const static t_param predicted_in_l45_300 = {&predicted_in_l45_300_table,&predicted_in_l45_300_sp,&predicted_in_l45_300_om,&predicted_in_l45_300_ff};

const static t_pid_gain predicted_out_r45_300_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_out_r45_300_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_out_r45_300_ff = {0.870046f,0.114741f,0.094599f,0.000849f,0.000794f,0.000776f,0.085591f,3.679e-06f,0.0f,0.0f};
const static t_turn_param_table predicted_out_r45_300_table = {0.3f,-50.0f,37.835f,8.575f,-45.0f,Turn_R,0.5f};
const static t_param predicted_out_r45_300 = {&predicted_out_r45_300_table,&predicted_out_r45_300_sp,&predicted_out_r45_300_om,&predicted_out_r45_300_ff};

const static t_pid_gain predicted_out_l45_300_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_out_l45_300_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_out_l45_300_ff = {0.870046f,0.114741f,0.094599f,0.001129f,0.000934f,0.000928f,0.085591f,3.962e-06f,0.0f,0.0f};
const static t_turn_param_table predicted_out_l45_300_table = {0.3f,50.0f,34.529f,11.182f,45.0f,Turn_L,-0.5f};
const static t_param predicted_out_l45_300 = {&predicted_out_l45_300_table,&predicted_out_l45_300_sp,&predicted_out_l45_300_om,&predicted_out_l45_300_ff};

const static t_pid_gain predicted_in_r135_300_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_in_r135_300_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_in_r135_300_ff = {0.870046f,0.114741f,0.094599f,0.00319f,0.001071f,0.000987f,0.085591f,3.89e-07f,0.0f,0.0f};
const static t_turn_param_table predicted_in_r135_300_table = {0.3f,-30.0f,50.142f,36.768f,-135.0f,Turn_R,0.5f};
const static t_param predicted_in_r135_300 = {&predicted_in_r135_300_table,&predicted_in_r135_300_sp,&predicted_in_r135_300_om,&predicted_in_r135_300_ff};

const static t_pid_gain predicted_in_l135_300_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_in_l135_300_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_in_l135_300_ff = {0.870046f,0.114741f,0.094599f,0.003156f,0.001066f,0.000982f,0.085591f,1.78e-07f,0.0f,0.0f};
const static t_turn_param_table predicted_in_l135_300_table = {0.3f,30.0f,49.295f,35.607f,135.0f,Turn_L,-0.5f};
const static t_param predicted_in_l135_300 = {&predicted_in_l135_300_table,&predicted_in_l135_300_sp,&predicted_in_l135_300_om,&predicted_in_l135_300_ff};

const static t_pid_gain predicted_out_r135_300_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_out_r135_300_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_out_r135_300_ff = {0.870046f,0.114741f,0.094599f,0.00319f,0.001071f,0.000987f,0.085591f,3.89e-07f,0.0f,0.0f};
const static t_turn_param_table predicted_out_r135_300_table = {0.3f,-30.0f,42.805f,44.502f,-135.0f,Turn_R,0.5f};
const static t_param predicted_out_r135_300 = {&predicted_out_r135_300_table,&predicted_out_r135_300_sp,&predicted_out_r135_300_om,&predicted_out_r135_300_ff};

const static t_pid_gain predicted_out_l135_300_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_out_l135_300_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_out_l135_300_ff = {0.870046f,0.114741f,0.094599f,0.003156f,0.001066f,0.000982f,0.085591f,1.78e-07f,0.0f,0.0f};
const static t_turn_param_table predicted_out_l135_300_table = {0.3f,30.0f,43.826f,45.314f,135.0f,Turn_L,-0.5f};
const static t_param predicted_out_l135_300 = {&predicted_out_l135_300_table,&predicted_out_l135_300_sp,&predicted_out_l135_300_om,&predicted_out_l135_300_ff};

const static t_pid_gain predicted_r_v90_300_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_r_v90_300_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_r_v90_300_ff = {0.870046f,0.114741f,0.094599f,0.001678f,0.000675f,0.001023f,0.085591f,7.58e-07f,0.0f,0.0f};
const static t_turn_param_table predicted_r_v90_300_table = {0.3f,-41.0f,12.257f,2.092f,-90.0f,Turn_R,0.5f};
const static t_param predicted_r_v90_300 = {&predicted_r_v90_300_table,&predicted_r_v90_300_sp,&predicted_r_v90_300_om,&predicted_r_v90_300_ff};

const static t_pid_gain predicted_l_v90_300_sp = {2.0f,0.016f,0.0f};
const static t_pid_gain predicted_l_v90_300_om = {0.1f,0.01f,0.0f};
const static t_ff_gain predicted_l_v90_300_ff = {0.870046f,0.114741f,0.094599f,0.002037f,0.00065f,0.00104f,0.085591f,1.038e-06f,0.0f,0.0f};
const static t_turn_param_table predicted_l_v90_300_table = {0.3f,41.0f,10.53f,3.954f,90.0f,Turn_L,-0.5f};
const static t_param predicted_l_v90_300 = {&predicted_l_v90_300_table,&predicted_l_v90_300_sp,&predicted_l_v90_300_om,&predicted_l_v90_300_ff};

#endif // MOUSE_A_PREDICTED_TURN_300_H_
