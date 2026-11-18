/* 機体Bの探索ターン設定。速度[m/s]、長さ[mm]、角度[deg]。 */
#ifndef PARAM_B_TURN_SEARCH_H_
#define PARAM_B_TURN_SEARCH_H_

// 値の定義は同じディレクトリのturn_search.cppにあります。

#include "Params/typedef_run_param.h"

extern const t_pid_gain sp_gain_search_turn;//{1.0,0.04,0.05};;//{2.0,0.04};
extern const t_pid_gain om_gain_search_turn;//{0.50f, 0.0005f, 0.001f};
extern const t_turn_param_table slalom_L90_table_320;
extern const t_turn_param_table slalom_R90_table_320;
extern const t_param param_L90_search_320;
extern const t_param param_R90_search_320;

extern const t_turn_param_table slalom_L90_table_300;
extern const t_turn_param_table slalom_R90_table_300;
extern const t_param param_L90_search_300;
extern const t_param param_R90_search_300;

extern const t_turn_param_table slalom_L90_table_280;
extern const t_turn_param_table slalom_R90_table_280;
extern const t_param param_L90_search_280;
extern const t_param param_R90_search_280;

#endif /* PARAM_B_TURN_SEARCH_H_ */
