/*
 * run_pram.h
 *
 *  Created on: 2023/06/21
 *      Author: sato1
 */

#ifndef CPP_INC_RUN_PARAM_H_
#define CPP_INC_RUN_PARAM_H_

#include "Params/mouse_config.h"

#if defined(MOUSE_A)
	#include "Params/run_param_A.h"
#elif defined(MOUSE_B)
	#include "Params/run_param_B.h"
#else
    #error "MOUSEA または MOUSEB が定義されていません。mouse_select.h を確認してください。"
#endif

#endif /* CPP_INC_RUN_PARAM_H_ */
