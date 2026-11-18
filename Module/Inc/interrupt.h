/*
 * interrupt.h
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */

#ifndef CPP_INC_INTERRUPT_H_
#define CPP_INC_INTERRUPT_H_

#include "Component/Inc/singleton.h"
#include "Component/Inc/math_utils.h"
#include "Peripheral/Inc/peripheral.h"
#include "Core/Inc/tim.h"
#include "Task/Inc/turn_pose_ekf.h"

void Interrupt_Initialize();

class Interrupt:public Singleton<Interrupt>
{
	private:
		uint32_t time_count;
		uint32_t acc_time_cnt;
		float acc_buff[ACC_BUFF_SIZE];
		float acc_sum;

		float x_acc_buff[ACC_BUFF_SIZE];
		float x_acc_sum;

		float z_acc_buff[ACC_BUFF_SIZE];
		float z_acc_sum;
		TurnPose::Tracker turn_pose_;
		TurnPose::Input turn_pose_input_;

	public:
		void setTask();
		void preprocess();
		void main();
		void postprocess();
		TurnPose::Tracker &turn_pose() { return turn_pose_; }
		uint32_t return_time_count()
		{
			return time_count;
		}
};


#endif /* CPP_INC_INTERRUPT_H_ */
