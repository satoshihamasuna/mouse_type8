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
#include "Task/Inc/wall_detection.h"
#include "Task/Inc/zirconia_wall_control.h"
#include "Task/Inc/straight_wall_ekf.h"
#include "Task/Inc/wall_position_correction.h"
#include "Task/Inc/ir_range_state.h"
#include "Task/Inc/wall_diagnostics.h"
#include "Task/Inc/motion_heading.h"
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
	IrRangeState range_state = IrRangeState::Unknown;
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
		float Sensor_CalcDistance(t_sensor_dir dir,int16_t value, IrRangeState *state = nullptr);
		float IrSensor_adc2voltage(int16_t value);
		float	 wall_ref = STRAIGHT_REF;
		t_bool	 isEnableIrSens = False;
		t_irsens_motion irsens_motion = STRAIGHT_IRSENS;
		int ir_log_cnt;
		SideWallStability right_wall_stability, left_wall_stability;
		StraightWallEdge right_wall_edge, left_wall_edge;
		volatile WallPositionCorrection::Mode wall_edge_mode = WallPositionCorrection::Mode::Legacy;
		DiagonalWallEdge right_diagonal_edge, left_diagonal_edge;
		StraightPillarEdge straight_pillars[2];
		DiagonalPillarEdge diagonal_pillars[2];
		SpatialPillarPeak::Event pillar_events[2];
		PillarDiagnostics::Sample pillar_minimum[2];
		uint32_t pillar_sample_ms = 0;
		uint8_t pillar_axis = 0;
		PillarDiagnostics::Sample PillarSample(Vehicle *vehicle);
		bool straight_edge_rotation_valid = true;
		bool diagonal_edge_motion_valid = true;
		bool right_straight_corner_seen = false, left_straight_corner_seen = false;
		bool right_diagonal_corner_seen = false, left_diagonal_corner_seen = false;
		ZirconiaWallControl straight_wall_control;
		ZirconiaWallControl ekf_wall_control;
		StraightWallEkf straight_wall_filter;
		PoseFrames::Pose straight_world_origin;
		bool straight_world_bound=false;
		struct PendingFeature {
			bool pending = false;
			PillarDiagnostics::Sample sample;
			float reference_mm = 0.0f, delay_mm[2] = {};
			uint32_t generation[2] = {};
			uint8_t mask = 0, motion = 0;
			WallPositionCorrection::Phase phase = WallPositionCorrection::Phase::SearchStraight;
			uint8_t expected_side = 0;
			int pillar_index = -1;
		};
		PendingFeature pending_wall_feature, pending_pillar_feature[2];
		uint32_t longitudinal_generation = 0;
		WallPositionCorrection::FeatureContext straight_run_frame;
		bool straight_run_consumed = false;
		float straight_run_consumed_wall_mm = 0.0f;
		bool FeatureConsumed(float reference_mm, bool pillar, WallPositionCorrection::Phase phase) const {
			const float wall_mm = pillar ? reference_mm-wall_feature_ekf_param.pillar_reference_mm+wall_edge_trial_search_reference_mm : reference_mm;
			if (phase == WallPositionCorrection::Phase::StraightEntry && straight_run_frame.valid && straight_run_consumed
				&& fabsf(wall_mm-straight_run_consumed_wall_mm)<SECTION*0.5f) return true;
			if (phase != WallPositionCorrection::Phase::StraightRun) return wall_correction == True;
			return straight_run_consumed && wall_mm <= straight_run_consumed_wall_mm + SECTION*0.5f;
		}
		void ConsumeFeature(float reference_mm, bool pillar, WallPositionCorrection::Phase phase) {
			if (phase != WallPositionCorrection::Phase::StraightRun) return;
			straight_run_consumed = true;
			straight_run_consumed_wall_mm = pillar ? reference_mm-wall_feature_ekf_param.pillar_reference_mm+wall_edge_trial_search_reference_mm : reference_mm;
		}
		DiagonalWallControl diagonal_wall_control;
		DiagonalFeatureShadow diagonal_feature_shadow;
		DiagonalPose::Filter diagonal_pose;
		DiagonalPose::Steering diagonal_pose_steering;
		DiagonalPose::Route diagonal_pose_route;
		PoseFrames::Pose diagonal_world_origin;
		bool diagonal_world_bound=false;
		DiagonalPose::Pose diagonal_pose_peak[2];
		bool diagonal_pose_peak_valid[2]={};
		uint8_t diagonal_feature_wall_recorded = 0;
		float diagonal_feature_seed_heading = 0;
		bool diagonal_feature_seed_pending = false;
		MotionHeading::Prior turn_heading_prior, turn_heading_posterior, diagonal_heading_prior;
		bool turn_heading_connection = false;
		uint16_t diagonal_heading_ticks = 0;
		float diagonal_heading_travel_mm = 0;
		bool diagonal_segment_active = false;
		float left_edge_guard_m = 0.0f, right_edge_guard_m = 0.0f;
		float pending_corner_lateral_m = 0.0f;
		bool corner_pending = false;
		ZirconiaWallAverage straight_left_average, straight_right_average;
		float straight_left_distance_mm = 80.0f, straight_right_distance_mm = 80.0f;
		IrRangeState straight_left_range = IrRangeState::Unknown, straight_right_range = IrRangeState::Unknown;
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
		uint32_t wall_edge_generation = 0, r_corner_generation = 0, l_corner_generation = 0;
		WallPositionCorrection::Mode WallEdgeMode_Get() const { return wall_edge_mode; }
		void WallEdgeMode_Set(WallPositionCorrection::Mode mode) {
			const StateLock lock;
			if (mode != wall_edge_mode) {
				wall_edge_mode = mode; ResetWallCornerDetection();
				turn_heading_prior=turn_heading_posterior=diagonal_heading_prior=MotionHeading::Prior{};
				turn_heading_connection=false;
			}
		}

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
			uint32_t pose_control_cycles = 0;
			uint32_t pose_handoffs = 0;
			float pose_omega_max_radps = 0.0f;
			float edge_delay_max_mm = 0.0f;
		};
	private:
		DiagonalWallStats diagonal_wall_stats;
	public:
		void ResetDiagonalWallDiagnostics() { const StateLock lock; diagonal_wall_stats = DiagonalWallStats{}; diagonal_segment_active = false; }
		DiagonalWallStats DiagonalWallDiagnostics() const { const StateLock lock; return diagonal_wall_stats; }
		void ResetDiagonalWallControl() { const StateLock lock; diagonal_wall_control.reset(); diagonal_wall_output = DiagonalWallControl::Output{}; diagonal_pose_steering.reset(); diagonal_segment_active = false; diagonal_heading_prior=MotionHeading::Prior{}; }
		bool DiagonalPoseControlReady(DiagonalPose::Pose &pose) const {
			return irsens_motion==DIAGONAL_IRSENS && wall_ekf_mode==WallEkfMode::Control
				&& wall_edge_mode==WallPositionCorrection::Mode::Control
				&& diagonal_pose.control_pose(pose);
		}
		bool DiagonalPoseControlsCorner() const {
			DiagonalPose::Pose pose;
			return diagonal_pose_steering.engaged() || DiagonalPoseControlReady(pose);
		}
		bool DiagonalHeadingForTurn(float &heading_rad, float &variance_rad2) const {
			const StateLock lock;
			if (irsens_motion!=DIAGONAL_IRSENS || wall_ekf_mode!=WallEkfMode::Control
				|| wall_edge_mode!=WallPositionCorrection::Mode::Control) return false;
			if (diagonal_pose_param.calibrated && diagonal_pose.heading_for_turn(heading_rad,variance_rad2)) return true;
			// An inherited yaw can coast through a short passage without claiming
			// a pillar was seen. Uncalibrated proximity ranges never supply yaw.
			if (!diagonal_heading_prior.usable() || diagonal_heading_ticks>=220U
				|| diagonal_heading_travel_mm>=1.5f*DIAG_SECTION
				|| fabsf(diagonal_heading_prior.heading_rad)>0.13962634f
				|| diagonal_heading_prior.variance_rad2>wall_ekf_param.heading_std_limit_rad*wall_ekf_param.heading_std_limit_rad) return false;
			heading_rad=diagonal_heading_prior.heading_rad; variance_rad2=diagonal_heading_prior.variance_rad2;
			return true;
		}
		void BeginTurnHeading(const MotionHeading::Prior &prior) {
			const StateLock lock;
			turn_heading_prior=prior; turn_heading_posterior=MotionHeading::Prior{};
			turn_heading_connection=wall_ekf_mode==WallEkfMode::Control && prior.usable();
			if (!turn_heading_connection) turn_heading_prior=MotionHeading::Prior{};
		}
		MotionHeading::Prior TakeTurnHeadingPrior() {
			const StateLock lock; const auto prior=turn_heading_prior;
			turn_heading_prior=MotionHeading::Prior{}; return prior;
		}
		void UpdateTurnHeadingPosterior(const MotionHeading::Prior &posterior) {
			const StateLock lock;
			turn_heading_posterior=turn_heading_connection ? posterior : MotionHeading::Prior{};
		}
		MotionHeading::Prior TurnHeadingPosterior() const {
			const StateLock lock; return turn_heading_posterior;
		}
		uint8_t straight_wall_candidate_mask = 0;
		uint8_t straight_pillar_guard_mask = 0; // L=1, R=2: Open-origin, excluded from continuous-wall control.
		WallEkfMode WallEkfMode_Get() const { return wall_ekf_mode; }
		void WallEkfMode_Set(WallEkfMode mode) {
			const StateLock lock; wall_ekf_mode = mode; ResetStraightWallControl();
			// AのEKF選択は特徴点の観測モードにも反映する。Bの既存選択は保持。
			if (wall_feature_ekf_param.enabled) WallEdgeMode_Set(mode == WallEkfMode::Control
				? WallPositionCorrection::Mode::Control : mode == WallEkfMode::Shadow
				? WallPositionCorrection::Mode::Shadow : WallPositionCorrection::Mode::Legacy);
		}
		bool FeatureEkfEnabled() const {
			return wall_feature_ekf_param.enabled && wall_ekf_mode != WallEkfMode::Legacy
				&& wall_edge_mode != WallPositionCorrection::Mode::Legacy;
		}
		float StraightNodeFeatureReference(float legacy_reference_mm) const {
			// 最短のC_posはセル中心。探索のセル入口線基準を半区画だけ移す。
			return FeatureEkfEnabled() ? wall_edge_trial_search_reference_mm-SECTION*0.5f : legacy_reference_mm;
		}
		void BeginStraightRunFeatures(float wall_reference_mm) {
			const StateLock lock;
			if (straight_run_frame.valid || !isfinite(wall_reference_mm)) return;
			straight_run_frame.valid=true; straight_run_frame.phase=WallPositionCorrection::Phase::StraightRun;
			straight_run_frame.wall_reference_mm=wall_reference_mm; straight_run_frame.period_mm=SECTION;
			straight_run_consumed=false;
		}
		WallPositionCorrection::FeatureContext StraightRunFeatureContext() const { const StateLock lock; return straight_run_frame; }
		void BeginDiagonalExitFeatures(float end_mm) { const StateLock lock; diagonal_feature_shadow.begin_exit(end_mm); }
		void StageDiagonalPoseRoute(const DiagonalPose::Route &route, bool across_turn=false);
		void ClearDiagonalPoseRoute() { const StateLock lock; diagonal_pose_route.count=0; diagonal_pose.stop(); diagonal_world_bound=false; diagonal_heading_prior=MotionHeading::Prior{}; }
		void BeginDiagonalPoseRoute(Vehicle *vehicle, float length_mm, float heading_rad);
		void PrimeDiagonalFeatureHeading(float heading_rad) {
			const StateLock lock; diagonal_feature_seed_heading=heading_rad; diagonal_feature_seed_pending=isfinite(heading_rad);
		}
		void PrimeDiagonalPoseHeading(const MotionHeading::Prior &prior) {
			const StateLock lock;
			diagonal_heading_prior=wall_ekf_mode==WallEkfMode::Control && prior.usable() ? prior : MotionHeading::Prior{};
			diagonal_heading_ticks=0; diagonal_heading_travel_mm=0;
		}
		void RebaseDiagonalFeatureCandidates(float delta_mm, float reset_heading_rad=0, float reset_left_mm=0) {
			const StateLock lock;
			if (irsens_motion!=DIAGONAL_IRSENS) return;
			diagonal_feature_shadow.rebase(delta_mm,reset_heading_rad,reset_left_mm);
			for (unsigned i=0;i<2;++i) if (diagonal_pillars[i].pending()) {
				pillar_minimum[i].length_mm+=delta_mm; pillar_minimum[i].rebase_mm+=delta_mm;
			}
		}
		void RecordDiagonalWallFeatures(Vehicle *vehicle, uint8_t motion, WallPositionCorrection::Phase phase, uint8_t expected_side=0);
		void QueueWallFeature(Vehicle *vehicle, float reference_mm, uint8_t motion,
			WallPositionCorrection::Phase phase = WallPositionCorrection::Phase::SearchStraight, uint8_t expected_side = 0);
		void AssimilateFeatures(Vehicle *vehicle);
		WallPositionCorrection::Result ObserveFeature(Vehicle *vehicle, float reference_mm, float delay_mm, float variance_m2,
			bool pillar = false, WallPositionCorrection::Phase phase = WallPositionCorrection::Phase::SearchStraight);
		void ShiftStraightFeatureMeasurements(float delta_mm) {
			const StateLock lock;
			if (irsens_motion != STRAIGHT_IRSENS || !isfinite(delta_mm)) return;
			// EKF注入は観測済み座標の補正。地図の参照位置・消費済み境界は動かさない。
			for (unsigned i=0;i<2;++i) if (straight_pillars[i].pending() || pillar_events[i].detected) {
				pillar_minimum[i].length_mm+=delta_mm; pillar_minimum[i].rebase_mm+=delta_mm;
			}
			for (auto &p:pending_pillar_feature) if (p.pending) {
				p.sample.length_mm+=delta_mm;
				PillarDiagnostics::instance().rebase_pending(p.pillar_index,delta_mm,false);
			}
			if (pending_wall_feature.pending) pending_wall_feature.sample.length_mm+=delta_mm;
			if (right_straight_corner_seen) r_corner_length.set(r_corner_length.get()+delta_mm);
			if (left_straight_corner_seen) l_corner_length.set(l_corner_length.get()+delta_mm);
		}
		void RebaseStraightFeatureCandidates(float delta_mm) {
			const StateLock lock;
			if (irsens_motion != STRAIGHT_IRSENS || !isfinite(delta_mm)) return;
			if (straight_run_frame.valid) straight_run_frame.wall_reference_mm += delta_mm;
			if (straight_run_consumed) straight_run_consumed_wall_mm += delta_mm;
			for (unsigned i=0;i<2;++i) if (straight_pillars[i].pending() || pillar_events[i].detected) {
				pillar_minimum[i].length_mm += delta_mm; pillar_minimum[i].rebase_mm += delta_mm;
			}
			// 確定が同一IRQの原点変更に先行した場合も、観測と参照を同じだけ移す。
			for (auto &p:pending_pillar_feature) if (p.pending && p.phase==WallPositionCorrection::Phase::StraightRun) {
				p.sample.length_mm+=delta_mm; p.reference_mm+=delta_mm;
				PillarDiagnostics::instance().rebase_pending(p.pillar_index,delta_mm);
			}
			if (pending_wall_feature.pending && pending_wall_feature.phase==WallPositionCorrection::Phase::StraightRun) {
				pending_wall_feature.sample.length_mm+=delta_mm; pending_wall_feature.reference_mm+=delta_mm;
			}
		}
		// True means queued/applied; EKF innovation rejection occurs after prediction.
		bool CorrectStraightWallCorner(Vehicle *vehicle, float left_positive_mm);
		void ResetStraightWallControl(bool preserve_pose = false, bool preserve_diagonal_bridge = false)
		{
			const StateLock lock;
			if (preserve_pose) return; // Same passage: retain phase, heading and estimate together.
			straight_world_bound=false;
			turn_heading_prior=turn_heading_posterior=diagonal_heading_prior=MotionHeading::Prior{};
			turn_heading_connection=false;
			straight_run_frame=WallPositionCorrection::FeatureContext{}; straight_run_consumed=false;
			ResetDiagonalWallControl();
			diagonal_feature_shadow.reset(); diagonal_feature_wall_recorded=0;
			if (!preserve_diagonal_bridge || !diagonal_pose.bridging()) { diagonal_pose.stop(); diagonal_world_bound=false; }
			diagonal_pose_steering.reset();
			diagonal_feature_seed_pending=false;
			search_turn_guard.clear();
			straight_wall_control.reset();
			ekf_wall_control.reset();
			straight_wall_filter.clear();
			pending_wall_feature = PendingFeature{};
			for (auto &pending : pending_pillar_feature) pending = PendingFeature{};
			corner_pending = false;
			left_edge_guard_m = right_edge_guard_m = 0.0f;
			straight_wall_ekf_active = false;
			straight_wall_ekf_used = false;
			straight_wall_candidate_mask = 0;
			straight_pillar_guard_mask = 0;
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
			ResetStraightWallControl(false, true);
			straight_prime_pending = true;
		}
		void PrimeStraightTurnExit(float heading_rad, bool valid, float heading_variance_rad2 = -1.0f,
			bool preserve_diagonal_bridge = false)
		{
			const StateLock lock;
			ResetStraightWallControl(false, preserve_diagonal_bridge);
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
		void PrimeSearchTurnExit(float heading_rad, bool valid, float heading_variance_rad2 = -1.0f)
		{
			PrimeStraightTurnExit(heading_rad, valid, heading_variance_rad2);
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
		void ResetWallCornerDetection(bool preserve_diagonal_bridge = false)
		{
			const StateLock lock;
			straight_run_frame=WallPositionCorrection::FeatureContext{}; straight_run_consumed=false;
			diagonal_feature_shadow.reset(); diagonal_feature_wall_recorded=0;
			if (!preserve_diagonal_bridge || !diagonal_pose.bridging()) { diagonal_pose.stop(); diagonal_world_bound=false; }
			diagonal_pose_peak_valid[0]=diagonal_pose_peak_valid[1]=false;
			diagonal_pose_steering.reset();
			diagonal_feature_seed_pending=false;
			++wall_edge_generation;
			pending_wall_feature = PendingFeature{};
			for (auto &pending : pending_pillar_feature) pending = PendingFeature{};
			wall_correction = False;
			right_wall_edge.reset(); left_wall_edge.reset();
			right_diagonal_edge.reset(); left_diagonal_edge.reset();
			for (unsigned i=0; i<2; ++i) {
				straight_pillars[i].reset(); diagonal_pillars[i].reset();
				pillar_events[i] = SpatialPillarPeak::Event{}; pillar_minimum[i] = PillarDiagnostics::Sample{};
			}
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
		virtual 		void IrSensorSet(Vehicle *vehicle, float forward_step_mm, uint32_t sample_ms = 0);
		void UpdatePillarCandidates(Vehicle *vehicle, float forward_step_mm, uint32_t sample_ms);
		void RecordPillarCandidates(Vehicle *vehicle, uint8_t motion, uint8_t control, uint8_t turn, bool search_frame,
			const WallPositionCorrection::FeatureContext &context = WallPositionCorrection::FeatureContext{});
		void IrSensMotion_Set(t_irsens_motion motion)
		{
			const StateLock lock;
			if (motion != irsens_motion)
			{
				const auto turn_prior=turn_heading_prior;
				const bool keep_turn=turn_heading_connection && motion==TURN_IRSENS;
				const auto diagonal_prior=diagonal_heading_prior;
				const bool prime_diagonal=diagonal_feature_seed_pending && motion==DIAGONAL_IRSENS;
				const float diagonal_seed=diagonal_feature_seed_heading;
				const bool keep_bridge=motion==STRAIGHT_IRSENS || motion==DIAGONAL_IRSENS || motion==TURN_IRSENS;
				ResetWallCornerDetection(keep_bridge);
				const bool prime = straight_prime_pending && motion == STRAIGHT_IRSENS;
				ResetStraightWallControl(false, keep_bridge);
				straight_prime_pending = prime;
				if (keep_turn) BeginTurnHeading(turn_prior);
				if (prime_diagonal) diagonal_feature_shadow.prime_heading(diagonal_seed);
				if (motion==DIAGONAL_IRSENS && diagonal_prior.usable()) {
					PrimeDiagonalPoseHeading(diagonal_prior);
					diagonal_wall_control.prime_heading(diagonal_prior.heading_rad,diagonal_prior.variance_rad2);
				}
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
				}
				//Indicate_LED(0x10|Return_LED_Status());
			}
			if(l_wall_corner == True)
			{
				if(wall_correction == False)
				{
					wall_correction = True;
					flag = True;
				}
				//Indicate_LED(0x20|Return_LED_Status());
			}


			return flag;
		}
		void Division_Wall_Correction_Reset(bool continue_axis_features = false)
		{
			const StateLock lock;
			const uint32_t previous_generation = wall_edge_generation;
			// A new correction frame keeps same-axis approach history, but not
			// an old event or a crossing already pending in the previous frame.
			++wall_edge_generation;
			if (continue_axis_features && irsens_motion == STRAIGHT_IRSENS) {
				for (unsigned i=0;i<2;++i) if (straight_pillars[i].pending() && pillar_minimum[i].generation == previous_generation)
					pillar_minimum[i].generation = wall_edge_generation;
				for (auto &p:pending_pillar_feature) if (p.pending && p.phase==WallPositionCorrection::Phase::StraightRun)
					for (unsigned i=0;i<2;++i) if ((p.mask&(1U<<i)) && p.generation[i]==previous_generation) {
						p.generation[i]=wall_edge_generation; p.sample.generation=wall_edge_generation;
						PillarDiagnostics::instance().continue_pending(p.pillar_index,wall_edge_generation);
					}
				if (pending_wall_feature.pending && pending_wall_feature.phase==WallPositionCorrection::Phase::StraightRun)
					for (unsigned i=0;i<2;++i) if ((pending_wall_feature.mask&(1U<<i)) && pending_wall_feature.generation[i]==previous_generation) {
						pending_wall_feature.generation[i]=wall_edge_generation; pending_wall_feature.sample.generation=wall_edge_generation;
					}
			}
			if (continue_axis_features && irsens_motion == DIAGONAL_IRSENS)
				for (unsigned i=0;i<2;++i) if (diagonal_pillars[i].pending() && pillar_minimum[i].generation==previous_generation)
					pillar_minimum[i].generation=wall_edge_generation;
			right_wall_edge.discard_pending(); left_wall_edge.discard_pending();
			right_diagonal_edge.discard_pending(); left_diagonal_edge.discard_pending();
			r_wall_corner = l_wall_corner = False;
			r_corner_delay_mm = l_corner_delay_mm = 0.0f;
			r_corner_time = l_corner_time = 65535U;
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
		void IrSensorSet(Vehicle *vehicle, float forward_step_mm, uint32_t sample_ms = 0) override
		{
			sen_fl.value =  Sensor_GetValue(sensor_fl);
			sen_fr.value =  Sensor_GetValue(sensor_fr);
			sen_l.value  =  Sensor_GetValue(sensor_sl);
			sen_r.value  =  Sensor_GetValue(sensor_sr);
			IrSensorDistanceSet(vehicle);
			IrSensorWallSet(forward_step_mm);
			UpdatePillarCandidates(vehicle, forward_step_mm, sample_ms);
		}

};

#endif /* CPP_INC_SENSING_TASK_H_ */
