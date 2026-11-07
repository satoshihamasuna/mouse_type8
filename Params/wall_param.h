#ifndef PARAMS_WALL_PARAM_H_
#define PARAMS_WALL_PARAM_H_

#include "Params/mouse_config.h"

#if defined(MOUSE_A)
	#include "Params/wall_param_A.h"
#elif defined(MOUSE_B)
	#include "Params/wall_param_B.h"
#else
	#error "MOUSE_A または MOUSE_B を選択してください。"
#endif

#endif /* PARAMS_WALL_PARAM_H_ */
