/*
 * sensing_task.h
 *
 *  Created on: 2023/06/13
 *      Author: sato1
 */

#ifndef CPP_INC_SENSING_TASK_H_
#define CPP_INC_SENSING_TASK_H_

#include "Module/Inc/vehicle.h"
#include "Component/Inc/singleton.h"
#include "Component/Inc/typedef.h"
#include "Component/Inc/maze_typedef.h"
#include "Task/Inc/run_typedef.h"
#include "Task/Inc/side_wall_stability.h"
#include "Task/Inc/straight_wall_edge.h"
#include "Task/Inc/diagonal_wall_edge.h"
#include "Task/Inc/zirconia_wall_control.h"
#include "Task/Inc/straight_wall_ekf.h"
#include "Task/Inc/search_turn_guard.h"
#include "Task/Inc/diagonal_wall_control.h"
#include "Peripheral/Inc/ir_sensor_activity.h"

#define STRAIGHT_REF		(45.0)
#define DIAGONAL_REF		(diagonal_wall_param.reference_m * 1000.0f)

#define SIDE_R_THRESHOLD		(65.0)
#define SIDE_L_THRESHOLD		(65.0)
#define SIDE_THRESHOLD			(65.0)
#define CORNER_R_THRESHOLD		(60.0)
#define CORNER_L_THRESHOLD		(60.0)

//#define SIDECONTROL_FR_THRESHOLD		(80.0)
//#define SIDECONTROL_FL_THRESHOLD		(80.0)

#if defined(MOUSE_A)
	#define SIDECONTROL_FR_THRESHOLD		(85.0)
	#define SIDECONTROL_FL_THRESHOLD		(85.0)
	#define FRONT_THRESHOLD		(110.0)

#elif defined(MOUSE_B)
	#define SIDECONTROL_FR_THRESHOLD		(80.0)
	#define SIDECONTROL_FL_THRESHOLD		(80.0)
	#define FRONT_THRESHOLD		(105.0)

#else
    #error "MOUSEA または MOUSEB が定義されていません。mouse_select.h を確認してください。"
#endif



#define SIDE_CORNER_THRESHOLD (68.0)

typedef struct{
	int16_t value;
	t_bool prev_is_wall;
	t_bool is_wall;
	t_bool is_control;
	float distance;
	float control_th;
	uint16_t control_cnt;
	float error;
	float distance_log[20];
	float distance_sum;
	float control_distance;
	float avg_distance;
	float diff;
	float prev_diff;
}t_sensor;


typedef enum
{
	STRAIGHT_IRSENS = 0,
	DIAGONAL_IRSENS = 1,
	TURN_IRSENS		= 2,
}t_irsens_motion;

class IrSensTask
{
	private:
		// Foreground motion initializers and the control IRQ share this state.
		// Host harnesses have no IRQs; nested guards retain the original mask.
		class StateLock {
#if defined(__arm__) || defined(__thumb__)
			uint32_t saved_mask;
		public:
			StateLock() : saved_mask(__get_PRIMASK()) { __disable_irq(); }
			~StateLock() { __set_PRIMASK(saved_mask); }
#else
		public:
			StateLock() {}
			~StateLock() {}
#endif
		};
		float Sensor_CalcDistance(t_sensor_dir dir,int16_t value);
		float IrSensor_adc2voltage(int16_t value);
		float	 wall_ref = STRAIGHT_REF;
		t_bool	 isEnableIrSens = False;
		t_irsens_motion irsens_motion = STRAIGHT_IRSENS;
		int ir_log_cnt;
		SideWallStability right_wall_stability, left_wall_stability;
		StraightWallEdge right_wall_edge, left_wall_edge;
		DiagonalWallEdge right_diagonal_edge, left_diagonal_edge;
		bool straight_edge_rotation_valid = true;
		bool diagonal_edge_motion_valid = true;
		bool right_straight_corner_seen = false, left_straight_corner_seen = false;
		bool right_diagonal_corner_seen = false, left_diagonal_corner_seen = false;
		ZirconiaWallControl straight_wall_control;
		ZirconiaWallControl ekf_wall_control;
		StraightWallEkf straight_wall_filter;
		DiagonalWallControl diagonal_wall_control;
		bool diagonal_segment_active = false;
		float left_edge_guard_m = 0.0f, right_edge_guard_m = 0.0f;
		float pending_corner_lateral_m = 0.0f;
		bool corner_pending = false;
		ZirconiaWallAverage straight_left_average, straight_right_average;
		float straight_left_distance_mm = 80.0f, straight_right_distance_mm = 80.0f;
		float wall_forward_step_mm = 0.0f;
		float straight_heading_rad = 0.0f;
		float straight_heading_variance_rad2 = wall_ekf_param.initial_heading_std_rad * wall_ekf_param.initial_heading_std_rad;
		bool straight_heading_valid = false, wall_sample_updated = false;
		bool straight_prime_pending = false;
		bool straight_heading_skip_once = false;
		SearchTurnGuard search_turn_guard;
		float search_turn_gyro_radps = 0.0f;
		bool search_turn_guard_enabled = false;

	public:
		enum class WallEkfMode : uint8_t { Legacy, Shadow, Control };
	private:
		volatile WallEkfMode wall_ekf_mode = WallEkfMode::Shadow;
	public:

		param_element control_ir;
		param_element control_ir_dot;
		ZirconiaWallControl::Output straight_wall_output;
		ZirconiaWallControl::Output straight_wall_ekf_control_output;
		StraightWallEkf::Estimate straight_wall_estimate;
		bool straight_wall_ekf_active = false;
		bool straight_wall_ekf_used = false;
		DiagonalWallControl::Output diagonal_wall_output;
		struct DiagonalWallStats {
			uint32_t cycles = 0, used = 0, updates = 0, segments = 0;
			uint32_t fallback = 0, front_blocked = 0;
			uint32_t edges_right = 0, edges_left = 0;
			float edge_delay_max_mm = 0.0f;
		};
	private:
		DiagonalWallStats diagonal_wall_stats;
	public:
		void ResetDiagonalWallDiagnostics() { const StateLock lock; diagonal_wall_stats = DiagonalWallStats{}; diagonal_segment_active = false; }
		DiagonalWallStats DiagonalWallDiagnostics() const { const StateLock lock; return diagonal_wall_stats; }
		void ResetDiagonalWallControl() { const StateLock lock; diagonal_wall_control.reset(); diagonal_wall_output = DiagonalWallControl::Output{}; diagonal_segment_active = false; }
		uint8_t straight_wall_candidate_mask = 0;
		WallEkfMode WallEkfMode_Get() const { return wall_ekf_mode; }
		void WallEkfMode_Set(WallEkfMode mode) { const StateLock lock; wall_ekf_mode = mode; ResetStraightWallControl(); }
		// True means queued/applied; EKF innovation rejection occurs after prediction.
		bool CorrectStraightWallCorner(Vehicle *vehicle, float left_positive_mm);
		void ResetStraightWallControl(bool preserve_pose = false)
		{
			const StateLock lock;
			if (preserve_pose) return; // Same passage: retain phase, heading and estimate together.
			ResetDiagonalWallControl();
			search_turn_guard.clear();
			straight_wall_control.reset();
			ekf_wall_control.reset();
			straight_wall_filter.clear();
			corner_pending = false;
			left_edge_guard_m = right_edge_guard_m = 0.0f;
			straight_wall_ekf_active = false;
			straight_wall_ekf_used = false;
			straight_wall_candidate_mask = 0;
			straight_wall_ekf_control_output = ZirconiaWallControl::Output{};
			straight_wall_output = ZirconiaWallControl::Output{};
			straight_heading_rad = 0.0f;
			straight_heading_variance_rad2 = wall_ekf_param.initial_heading_std_rad * wall_ekf_param.initial_heading_std_rad;
			straight_heading_valid = false;
			straight_prime_pending = false;
			straight_heading_skip_once = false;
		}
		void PrimeStraightWallControl()
		{
			const StateLock lock;
			ResetStraightWallControl();
			straight_prime_pending = true;
		}
		void PrimeSearchTurnExit(float heading_rad, bool valid, float heading_variance_rad2 = -1.0f)
		{
			const StateLock lock;
			ResetStraightWallControl();
			// The final ten rotating samples do not describe the new passage.
			straight_left_average = ZirconiaWallAverage{};
			straight_right_average = ZirconiaWallAverage{};
			search_turn_guard.clear();
			straight_prime_pending = true;
			straight_heading_rad = valid ? heading_rad : 0.0f;
			straight_heading_valid = valid;
			if (valid && isfinite(heading_variance_rad2) && heading_variance_rad2 > 0.0f)
				straight_heading_variance_rad2 = heading_variance_rad2;
			// The handoff already contains this IRQ's measured yaw.
			straight_heading_skip_once = true;
		}
		bool CanContinueStraightWallPose() const
		{
			const StateLock lock;
			return straight_heading_valid;
		}
		float StraightHeadingForTurn() const
		{
			const StateLock lock;
			return straight_wall_ekf_used && CanUseStraightWallEkfPose()
				? straight_wall_filter.estimate().heading_rad : straight_heading_rad;
		}
		bool CanUseStraightWallEkfPose() const
		{
			// A small covariance alone cannot validate yaw inferred from a changing
			// range after a turn. Retain the independent interval gyro as a check.
			return straight_heading_valid && straight_wall_filter.heading_trusted()
				&& isfinite(straight_heading_rad)
				&& fabsf(straight_wall_filter.estimate().heading_rad - straight_heading_rad)
					<= wall_ekf_param.heading_disagreement_limit_rad;
		}
		float StraightHeadingVarianceForTurn() const
		{
			const StateLock lock;
			return straight_wall_ekf_used && CanUseStraightWallEkfPose()
				? straight_wall_filter.estimate().variance_heading_rad2 : straight_heading_variance_rad2;
		}
		SearchTurnGuard::Decision SearchTurnDecision(bool right_turn, bool front_wall) const;
		t_sensor sen_fr,sen_fl,sen_r,sen_l;
		t_bool 	 wall_correction;
		t_bool 	 r_wall_corner,		l_wall_corner;
		uint16_t r_corner_time,		l_corner_time;
		float r_corner_delay_mm = 0.0f, l_corner_delay_mm = 0.0f;
		param_element 	 r_corner_length,	l_corner_length;
		void ResetWallCornerDetection()
		{
			const StateLock lock;
			right_wall_edge.reset(); left_wall_edge.reset();
			right_diagonal_edge.reset(); left_diagonal_edge.reset();
			r_wall_corner = l_wall_corner = False;
			r_corner_delay_mm = l_corner_delay_mm = 0.0f;
			r_corner_time = l_corner_time = 65535U;
			right_straight_corner_seen = left_straight_corner_seen = false;
			right_diagonal_corner_seen = left_diagonal_corner_seen = false;
		}
		float StraightCornerDelay() const
		{
			return r_wall_corner == True ? r_corner_delay_mm
				: l_wall_corner == True ? l_corner_delay_mm : 0.0f;
		}
		void RecordStraightCorner(bool right, float current_length_mm)
		{
			if (right) { r_corner_length.set(current_length_mm - r_corner_delay_mm); right_straight_corner_seen = true; }
			else { l_corner_length.set(current_length_mm - l_corner_delay_mm); left_straight_corner_seen = true; }
		}
		bool StraightCornerPairValid() const { return right_straight_corner_seen && left_straight_corner_seen; }
		void RecordDiagonalCorner(bool right, float current_length_mm)
		{
			if (right) { r_corner_length.set(current_length_mm - r_corner_delay_mm); right_diagonal_corner_seen = true; }
			else { l_corner_length.set(current_length_mm - l_corner_delay_mm); left_diagonal_corner_seen = true; }
		}
		bool DiagonalCornerLateralCorrection(float &lateral_mm)
		{
			if (!right_diagonal_corner_seen || !left_diagonal_corner_seen || r_wall_corner == l_wall_corner) return false;
			float gap = r_wall_corner == True ? r_corner_length.get() - l_corner_length.get()
				: l_corner_length.get() - r_corner_length.get();
			if (gap >= DIAG_SECTION * 2.0f && gap < DIAG_SECTION * 3.5f) gap /= 3.0f;
			if (!isfinite(gap) || fabsf(gap - DIAG_SECTION) >= 20.0f) return false;
			lateral_mm = (r_wall_corner == True ? -0.5f : 0.5f) * (gap - DIAG_SECTION);
			return true;
		}
		t_wall_state conv_Sensin2Wall(t_sensor_dir sens_dir);
		virtual 		void IrSensorSet(Vehicle *vehicle, float forward_step_mm);
		void IrSensMotion_Set(t_irsens_motion motion)
		{
			const StateLock lock;
			if (motion != irsens_motion)
			{
				ResetWallCornerDetection();
				const bool prime = straight_prime_pending && motion == STRAIGHT_IRSENS;
				ResetStraightWallControl();
				straight_prime_pending = prime;
			}
			irsens_motion = motion;
		}

		void IrSensorReferenceSet(float ref_value);
		void IrSensorDistanceSet(Vehicle *vehicle);
		void IrSensorWallSet(float forward_step_mm);
		void SetWallControl_RadVelo(Vehicle *vehicle,float delta_tms);
		inline void EnableIrSens()		{isEnableIrSens = True;}
		inline void DisableIrSens()			{isEnableIrSens = False; ResetStraightWallControl();}
		void EnableIrSensStraight()		{	EnableIrSens();		IrSensMotion_Set(STRAIGHT_IRSENS);	IrSensorReferenceSet(STRAIGHT_REF);	}
		void EnableIrSensDiagonal()		{	EnableIrSens();		IrSensMotion_Set(DIAGONAL_IRSENS);	IrSensorReferenceSet(DIAGONAL_REF);	}
		float IrSensorMaxValueFromLog(t_sensor_dir dir);
		int16_t IrSensor_Avg();
		bool HandStartDetected(int16_t threshold);
		t_bool Division_Wall_Correction()
		{
			t_bool flag = False;
			if(r_wall_corner == True)
			{
				if(wall_correction == False)
				{
					wall_correction = True;
					flag = True;
					Indicate_LED(0x10|Return_LED_Status());
				}
				//Indicate_LED(0x10|Return_LED_Status());
			}
			if(l_wall_corner == True)
			{
				if(wall_correction == False)
				{
					wall_correction = True;
					flag = True;
					Indicate_LED(0x20|Return_LED_Status());
				}
				//Indicate_LED(0x20|Return_LED_Status());
			}


			return flag;
		}
		void Division_Wall_Correction_Reset()
		{
			const StateLock lock;
			Indicate_LED(0x00);
			//r_check = l_check =
			wall_correction = False;
			right_straight_corner_seen = left_straight_corner_seen = false;
			right_diagonal_corner_seen = left_diagonal_corner_seen = false;
		}


};

class IrSensTask_type8: public IrSensTask,public Singleton<IrSensTask_type8>
{
	public:
		void IrSensorSet(Vehicle *vehicle, float forward_step_mm) override
		{
			sen_fl.value =  Sensor_GetValue(sensor_fl);
			sen_fr.value =  Sensor_GetValue(sensor_fr);
			sen_l.value  =  Sensor_GetValue(sensor_sl);
			sen_r.value  =  Sensor_GetValue(sensor_sr);
			IrSensorDistanceSet(vehicle);
			IrSensorWallSet(forward_step_mm);
		}

};

#endif /* CPP_INC_SENSING_TASK_H_ */
