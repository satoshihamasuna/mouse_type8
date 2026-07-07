/*
 * mode.h
 *
 *  Created on: Jul 19, 2025
 *      Author: sato1
 */

#ifndef INC_MODE_H_
#define INC_MODE_H_

#include "peripheral.h"
#include "wall_class.h"
#include "typedef_run_param.h"

namespace Mode
{
	void Demo();
	void Demo2();
	void Debug(const  t_straight_param *st_param,const t_param *const *turn_mode,int suction);
	void Debug2(const  t_straight_param *st_param,const t_param *const *turn_mode,int suction);
	void Debug3();
	void Interface_Check();
	void Wall_Histry_Check(wall_class *wall_data);
	int32_t Seach_Time_Select();
	uint8_t Select(uint8_t _param,uint8_t max,t_encoder enc);
	void Select_Mode();
	void LED_Toggle(uint8_t led_num,uint32_t period_time);
	void indicate_error();

}


#endif /* INC_MODE_H_ */
