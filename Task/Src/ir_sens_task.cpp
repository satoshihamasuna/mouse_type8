/*
 * ir_sens_task.h
 *
 *  Created on: 2024/03/21
 *      Author: sato1
 */


/*
 * sensing_task.cpp
 *
 *  Created on: 2023/06/14
 *      Author: sato1
 */


#include "Task/Inc/sensing_task.h"
#include "Params/sens_table.h"
#include "Component/Inc/typedef.h"
#include "Component/Inc/maze_typedef.h"
#include "Peripheral/Inc/peripheral.h"
#include <math.h>
#include "Task/Inc/wall_diagnostics.h"
/*
#ifdef MOUSE_A
  #include "Params/sens_table_A.h"
#elif defined(MOUSE_B)
  #include "Params/sens_table_B.h"
#else
  #error "No SENSOR_TYPE defined. Please define SENSOR_TYPE_A or SENSOR_TYPE_B."
#endif
*/
t_wall_state IrSensTask::conv_Sensin2Wall(t_sensor_dir sens_dir)
{
	switch(sens_dir){
		case sensor_fl:
			return ((sen_fl.is_wall)?WALL:NOWALL);
		case sensor_fr:
			return ((sen_fr.is_wall)?WALL:NOWALL);
		case sensor_sl:
			return ((sen_l.is_wall)?WALL:NOWALL);
		case sensor_sr:
			return ((sen_r.is_wall)?WALL:NOWALL);
		default :
			return NOWALL;
	}
}

void IrSensTask::IrSensorSet(Vehicle *vehicle, float forward_step_mm, uint32_t sample_ms)
{
	sen_fl.value =  100;//Sensor_GetValue(sensor_fl);
	sen_fr.value =  100;//Sensor_GetValue(sensor_fr);
	sen_l.value  =  100;//Sensor_GetValue(sensor_sl);
	sen_r.value  =  100;//Sensor_GetValue(sensor_sr);
	IrSensorDistanceSet(vehicle);
	IrSensorWallSet(forward_step_mm);
	UpdatePillarCandidates(vehicle, forward_step_mm, sample_ms);
}

PillarDiagnostics::Sample IrSensTask::PillarSample(Vehicle *vehicle)
{
	PillarDiagnostics::Sample out;
	out.ms = pillar_sample_ms; out.generation = wall_edge_generation;
	out.source_generation = wall_edge_generation;
	out.length_mm = vehicle->ego.length.get(); out.left_mm = vehicle->ego.x_point.get();
	out.heading_rad = vehicle->ego.radian.get(); out.gyro_radps = vehicle->ego.rad_velo.get();
	out.model_left_mm=diagonal_feature_shadow.lateral(out.left_mm);
	out.model_heading_rad=diagonal_feature_shadow.heading(out.heading_rad);
	out.control_left_mm = control_ir.get(); out.control_heading_rad = control_ir_dot.get();
	const t_sensor *sensors[] = {&sen_l, &sen_r, &sen_fl, &sen_fr};
	for (unsigned i=0; i<4; ++i) {
		out.adc[i] = sensors[i]->value; out.distance[i] = sensors[i]->distance; out.range[i] = sensors[i]->range_state;
	}
	return out;
}

void IrSensTask::StageDiagonalPoseRoute(const DiagonalPose::Route &route, bool across_turn)
{
	const StateLock lock;
	if (diagonal_pose.bridging() && (!route.count || !diagonal_pose_route.same_frame(route)))
		diagonal_pose.stop();
	if (across_turn && !diagonal_pose.bridging() && diagonal_pose_param.calibrated
		&& WallEdgeMode_Get()!=WallPositionCorrection::Mode::Legacy)
		diagonal_pose.begin_bridge(route);
	diagonal_pose_route=route;
}

void IrSensTask::BeginDiagonalPoseRoute(Vehicle *vehicle, float length_mm, float heading_rad)
{
	const StateLock lock;
	if (!diagonal_pose_route.count || WallEdgeMode_Get()==WallPositionCorrection::Mode::Legacy) return;
	if (diagonal_pose.finish_bridge(length_mm,diagonal_pose_route))
		++diagonal_wall_stats.pose_handoffs;
	else diagonal_pose.start(length_mm,diagonal_heading_prior.usable()?diagonal_heading_prior.heading_rad:diagonal_feature_shadow.heading(heading_rad),
		diagonal_pose_route,diagonal_heading_prior.usable()?diagonal_heading_prior.variance_rad2:-1.0f);
	const auto local=diagonal_pose.estimate_pose();
	diagonal_world_origin=vehicle->pose.passage_origin({local.s,local.e,local.heading});
	diagonal_world_bound=diagonal_pose.active() && vehicle->pose.snapshot().active;
	diagonal_pose_route.count=0;
	diagonal_pose_peak_valid[0]=diagonal_pose_peak_valid[1]=false;
}

void IrSensTask::UpdatePillarCandidates(Vehicle *vehicle, float forward_step_mm, uint32_t sample_ms)
{
	pillar_sample_ms = sample_ms;
	diagonal_feature_wall_recorded=0;
	for (auto &event : pillar_events) event = SpatialPillarPeak::Event{};
	if (irsens_motion==DIAGONAL_IRSENS && diagonal_heading_prior.usable()) {
		diagonal_heading_prior.predict(vehicle->ego.rad_velo.get(),0.001f,wall_ekf_param.heading_walk_rad_sqrt_s);
		if (diagonal_heading_ticks<220U) ++diagonal_heading_ticks;
		if (!isfinite(forward_step_mm) || forward_step_mm<0 || forward_step_mm>6) diagonal_heading_prior=MotionHeading::Prior{};
		else diagonal_heading_travel_mm+=forward_step_mm;
	}
	if (irsens_motion==DIAGONAL_IRSENS || diagonal_pose.bridging())
		diagonal_pose.advance(forward_step_mm,vehicle->ego.rad_velo.get()*0.001f,irsens_motion==TURN_IRSENS);
	if (WallEdgeMode_Get() == WallPositionCorrection::Mode::Legacy || irsens_motion == TURN_IRSENS) {
		return;
	}
	pillar_axis = irsens_motion == DIAGONAL_IRSENS ? 1U : 0U;
	if (pillar_axis) diagonal_feature_shadow.advance(forward_step_mm,
		diagonal_feature_shadow.heading(vehicle->ego.radian.get()),diagonal_feature_shadow_param);
	const bool motion_valid = vehicle->ideal.velo.get() > 0 && isfinite(vehicle->ego.rad_velo.get())
		&& fabsf(vehicle->ego.rad_velo.get()) <= 2.0f;
	uint16_t context_flags = 0;
	if (sen_fl.range_state == IrRangeState::InRange && sen_fr.range_state == IrRangeState::InRange
		&& sen_fl.distance <= 66 && sen_fr.distance <= 66) context_flags |= PillarDiagnostics::FrontPairClose;
	if (sen_fl.range_state == IrRangeState::Invalid || sen_fr.range_state == IrRangeState::Invalid) context_flags |= PillarDiagnostics::InvalidFront;
	const t_sensor *sensors[] = {&sen_l, &sen_r};
	for (unsigned i=0;i<2;++i) {
		const auto &sensor = *sensors[i];
		const bool valid = sensor.range_state != IrRangeState::Invalid;
		if (pillar_axis) {
			pillar_events[i] = diagonal_pillars[i].update(sensor.distance,sensor.value,forward_step_mm,valid,motion_valid,context_flags);
		} else {
			pillar_events[i] = straight_pillars[i].update(sensor.distance,sensor.value,forward_step_mm,valid,motion_valid,context_flags,
				ir_range_is_metric(sensor.range_state) && sensor.distance>IR_SIDE_MIN_MM);
		}
		if (pillar_events[i].peak_updated) {
			pillar_minimum[i] = PillarSample(vehicle);
			diagonal_pose_peak_valid[i]=pillar_axis && diagonal_pose.active();
			if (diagonal_pose_peak_valid[i]) diagonal_pose_peak[i]=diagonal_pose.odometry();
		}
	}
}

void IrSensTask::RecordPillarCandidates(Vehicle *vehicle, uint8_t motion, uint8_t control, uint8_t turn, bool search_frame,
	const WallPositionCorrection::FeatureContext &context)
{
	auto frame=context;
	if (search_frame) {
		frame.valid=true; frame.phase=WallPositionCorrection::Phase::SearchStraight;
		frame.wall_reference_mm=wall_edge_trial_search_reference_mm; frame.expected_side=0;
	}
	for (unsigned i=0;i<2;++i) if (pillar_events[i].detected) {
		const auto confirmation = PillarSample(vehicle);
		const float reference=WallPositionCorrection::feature_reference(frame,confirmation.length_mm-pillar_events[i].delay_mm,true);
		// Indication does not depend on the diagnostic buffer being enabled/full.
		if (!PillarDiagnostics::quality_flags(pillar_events[i],pillar_minimum[i],confirmation,pillar_axis,i == 0 ? 1U : 2U))
			Indicate_LED_Pulse(i == 0 ? 0x20U : 0x10U,50U);
		const int index = PillarDiagnostics::instance().record(pillar_events[i],pillar_minimum[i],confirmation,pillar_axis,
			i == 0 ? 1U : 2U,motion,control,turn,WallEdgeMode_Get(),frame.valid,reference,frame.phase);
		if (pillar_axis && frame.valid && WallPositionCorrection::diagonal_phase(frame.phase)) {
			diagonal_feature_shadow.set_phase(frame.phase);
			DiagonalFeatureShadow::Input input;
			input.phase=frame.phase; input.side=i+1U; input.expected_side=frame.expected_side;
			input.generation=confirmation.generation; input.event_generation=pillar_minimum[i].generation;
			input.quality_flags=PillarDiagnostics::quality_flags(pillar_events[i],pillar_minimum[i],confirmation,1,i+1U);
			if (PillarDiagnostics::diagonal_front_pair_blocked(pillar_minimum[i]) || PillarDiagnostics::diagonal_front_pair_blocked(confirmation))
				input.quality_flags|=PillarDiagnostics::FrontPairClose;
			input.before_mm=confirmation.length_mm; input.delay_mm=pillar_events[i].delay_mm;
			input.left_mm=pillar_minimum[i].model_left_mm; input.heading_rad=pillar_minimum[i].model_heading_rad;
			input.gyro_radps=confirmation.gyro_radps;
			const auto decision=diagonal_feature_shadow.observe(input,diagonal_feature_shadow_param);
			PillarDiagnostics::instance().diagonal_shadow_result(index,decision);
			if (diagonal_pose_peak_valid[i] && input.generation==input.event_generation) {
				const auto &peak=pillar_events[i];
				const auto &minimum=pillar_minimum[i];
				const bool saturated=minimum.range[i]==IrRangeState::NearLimit || minimum.adc[i]>=4000
					|| minimum.distance[i]<=25.5f;
				const auto pose=diagonal_pose.observe(i,diagonal_pose_peak[i],peak.sample_delay_mm-peak.delay_mm,
					peak.width_mm,saturated,minimum.gyro_radps,diagonal_pose_param);
				PillarDiagnostics::instance().diagonal_pose_result(index,pose);
				DiagonalPose::Pose accepted_pose;
				if (diagonal_world_bound && pose.status==DiagonalPose::Updated
					&& DiagonalPoseControlReady(accepted_pose))
					vehicle->pose.adopt_world(PoseFrames::compose(diagonal_world_origin,
						{accepted_pose.s,accepted_pose.e,accepted_pose.heading}));
			}
			diagonal_pose_peak_valid[i]=false;
		}
		if (frame.valid && !pillar_axis && FeatureEkfEnabled()
			&& !PillarDiagnostics::quality_flags(pillar_events[i],pillar_minimum[i],confirmation,0,i==0?1U:2U)) {
			auto &pending = pending_pillar_feature[i]; pending = PendingFeature{};
			pending.pending = true; pending.sample = confirmation; pending.pillar_index = index;
			pending.reference_mm = reference; pending.phase=frame.phase; pending.expected_side=frame.expected_side;
			pending.mask = i==0 ? 1U : 2U; pending.motion = motion;
			pending.delay_mm[i] = pillar_events[i].delay_mm; pending.generation[i] = confirmation.generation;
		}
		pillar_events[i].detected = false;
	}
}

void IrSensTask::RecordDiagonalWallFeatures(Vehicle *vehicle, uint8_t motion, WallPositionCorrection::Phase phase, uint8_t expected_side)
{
	if (irsens_motion!=DIAGONAL_IRSENS || WallEdgeMode_Get()==WallPositionCorrection::Mode::Legacy
		|| !WallPositionCorrection::diagonal_phase(phase)) return;
	diagonal_feature_shadow.set_phase(phase);
	const auto sample=PillarSample(vehicle);
	const uint8_t mask=(l_wall_corner==True?1U:0U)|(r_wall_corner==True?2U:0U);
	for (unsigned i=0;i<2;++i) if ((mask&(1U<<i)) && !(diagonal_feature_wall_recorded&(1U<<i))) {
		diagonal_feature_wall_recorded|=1U<<i;
		DiagonalFeatureShadow::Input input;
		input.phase=phase; input.kind=DiagonalFeatureShadow::DifferenceEdge; input.side=i+1U; input.expected_side=expected_side;
		input.generation=wall_edge_generation; input.event_generation=i==0?l_corner_generation:r_corner_generation;
		input.before_mm=sample.length_mm; input.delay_mm=i==0?l_corner_delay_mm:r_corner_delay_mm;
		input.left_mm=sample.model_left_mm; input.heading_rad=sample.model_heading_rad; input.gyro_radps=sample.gyro_radps;
		if (PillarDiagnostics::diagonal_front_pair_blocked(sample)) input.quality_flags|=PillarDiagnostics::FrontPairClose;
		if (!isfinite(sample.gyro_radps) || fabsf(sample.gyro_radps)>2) input.quality_flags|=PillarDiagnostics::Rotating;
		if (sample.range[2]==IrRangeState::Invalid || sample.range[3]==IrRangeState::Invalid) input.quality_flags|=PillarDiagnostics::InvalidFront;
		if (sample.range[i]==IrRangeState::NearLimit) input.quality_flags|=PillarDiagnostics::SideNearLimit;
		const auto decision=diagonal_feature_shadow.observe(input,diagonal_feature_shadow_param);
		WallDiagnostics::instance().record(sample.ms,motion,phase,i+1U,decision.position,sample.adc,sample.range,
			WallEdgeMode_Get(),wall_edge_generation,input.event_generation,decision.model);
	}
}

void IrSensTask::QueueWallFeature(Vehicle *vehicle, float reference_mm, uint8_t motion,
	WallPositionCorrection::Phase phase, uint8_t expected_side)
{
	auto &p=pending_wall_feature; p=PendingFeature{}; p.pending=true;
	p.sample=PillarSample(vehicle); p.reference_mm=reference_mm; p.motion=motion;
	p.phase=phase; p.expected_side=expected_side;
	p.mask=(l_wall_corner==True?1U:0U)|(r_wall_corner==True?2U:0U);
	p.delay_mm[0]=l_corner_delay_mm; p.delay_mm[1]=r_corner_delay_mm;
	p.generation[0]=l_corner_generation; p.generation[1]=r_corner_generation;
	if (phase==WallPositionCorrection::Phase::StraightRun) {
		const float delay = p.mask&1U ? p.delay_mm[0] : p.delay_mm[1];
		p.reference_mm=WallPositionCorrection::feature_reference(straight_run_frame,p.sample.length_mm-delay,false);
	}
}

WallPositionCorrection::Result IrSensTask::ObserveFeature(Vehicle *vehicle, float reference_mm, float delay_mm, float variance_m2,
	bool pillar, WallPositionCorrection::Phase phase)
{
	using namespace WallPositionCorrection;
	auto result=evaluate(vehicle->ego.length.get(),reference_mm,delay_mm,phase,
		pillar ? straight_pillar_param.maximum_after_peak_mm : -1.0f,pillar);
	result.ekf=true;
	if (!result.accepted()) return result;
	if (!straight_wall_filter.observe_longitudinal(result.innovation_mm/1000.0f,variance_m2)) {
		suppress(result,Reason::Invalid);
		result.proposed_reason=Reason::Invalid; result.proposed_delta_mm=0.0f; result.proposed_after_mm=result.before_mm;
	} else {
		const auto &estimate=straight_wall_filter.estimate();
		const float desired=estimate.longitudinal_error_m*1000.0f;
		const auto &policy=wall_position_param[static_cast<unsigned>(phase)];
		const float limit=fminf(policy.maximum_step_mm,fabsf(result.innovation_mm)*fminf(policy.blend,wall_feature_ekf_param.maximum_gain));
		result.delta_mm=fmaxf(-limit,fminf(limit,desired)); result.after_mm=result.before_mm+result.delta_mm;
		result.reason=fabsf(desired)>limit+1.0e-6f ? Reason::Limited : Reason::Accepted;
		result.proposed_reason=result.reason; result.proposed_delta_mm=result.delta_mm; result.proposed_after_mm=result.after_mm;
		if (WallEkfMode_Get()==WallEkfMode::Control && WallEdgeMode_Get()==Mode::Control) {
			vehicle->ego.length.set(result.after_mm);
			vehicle->pose.correct_passage(result.delta_mm,0.0f);
			ShiftStraightFeatureMeasurements(result.delta_mm);
			straight_wall_filter.inject_longitudinal(result.delta_mm/1000.0f);
		} else suppress(result,Reason::Shadow);
		wall_correction=True;
	}
	const auto &estimate=straight_wall_filter.estimate();
	result.ekf_nis=estimate.longitudinal_nis; result.ekf_gain=estimate.longitudinal_gain;
	result.ekf_variance_mm2=estimate.variance_longitudinal_m2*1000000.0f;
	return result;
}

void IrSensTask::AssimilateFeatures(Vehicle *vehicle)
{
	using namespace WallPositionCorrection;
	// Same-axis lateral/yaw prior survives; longitudinal error belongs to this frame.
	if (longitudinal_generation!=wall_edge_generation) {
		straight_wall_filter.reset_longitudinal(); longitudinal_generation=wall_edge_generation;
	}
	if (pending_wall_feature.pending) {
		auto &p=pending_wall_feature; p.pending=false;
		auto decision=decide(p.sample.length_mm,p.reference_mm,p.delay_mm[1],p.delay_mm[0],p.mask,p.expected_side,
			wall_edge_generation,p.generation[1],p.generation[0],p.phase,Mode::Control);
		for (unsigned i=0;i<2;++i) if (p.mask & (1U<<i)) {
			auto &result=i==0?decision.left:decision.right;
			if (FeatureConsumed(p.reference_mm,false,p.phase) && result.accepted()) suppress(result,Reason::NotSelected);
			else if (decision.selected_side==i+1U && result.accepted()) {
				result=ObserveFeature(vehicle,p.reference_mm,p.delay_mm[i],wall_feature_ekf_param.wall_std_m*wall_feature_ekf_param.wall_std_m,false,p.phase);
				if (result.accepted() || result.reason==Reason::Shadow) ConsumeFeature(p.reference_mm,false,p.phase);
			}
			WallDiagnostics::instance().record(p.sample.ms,p.motion,p.phase,i+1U,result,
				p.sample.adc,p.sample.range,WallEdgeMode_Get(),wall_edge_generation,p.generation[i]);
		}
	}
	// Simultaneous pillar observations share the same boundary; choose one prior.
	unsigned order[2]={0,1};
	if (pending_pillar_feature[0].pending && pending_pillar_feature[1].pending) {
		const auto &l=pending_pillar_feature[0], &r=pending_pillar_feature[1];
		if (fabsf(r.reference_mm+r.delay_mm[1]-r.sample.length_mm)<fabsf(l.reference_mm+l.delay_mm[0]-l.sample.length_mm)) { order[0]=1; order[1]=0; }
	}
	for (auto i:order) if (pending_pillar_feature[i].pending) {
		auto &p=pending_pillar_feature[i]; p.pending=false;
		auto result=evaluate(p.sample.length_mm,p.reference_mm,p.delay_mm[i],p.phase,straight_pillar_param.maximum_after_peak_mm,true);
		if (p.generation[i]!=wall_edge_generation) suppress(result,Reason::StaleGeneration);
		else if (p.expected_side && p.expected_side!=i+1U) suppress(result,Reason::WrongSide);
		else if (FeatureConsumed(p.reference_mm,true,p.phase)) suppress(result,Reason::NotSelected);
		else if (!before_entry_origin(p.sample.length_mm-p.delay_mm[i],p.phase)) {
			result=ObserveFeature(vehicle,p.reference_mm,p.delay_mm[i],wall_feature_ekf_param.pillar_std_m*wall_feature_ekf_param.pillar_std_m,true,p.phase);
			if (result.accepted() || result.reason==Reason::Shadow) ConsumeFeature(p.reference_mm,true,p.phase);
			if (i==0) left_edge_guard_m=wall_ekf_param.edge_guard_m;
			else right_edge_guard_m=wall_ekf_param.edge_guard_m;
		}
		PillarDiagnostics::instance().position_result(p.pillar_index,result);
	}
}

float IrSensTask::IrSensor_adc2voltage(int16_t value)
{
	return (float)(value)/4096.0*3.3;
}


float IrSensTask::Sensor_CalcDistance(t_sensor_dir dir, int16_t value, IrRangeState *state)
{
    const int16_t *sens_table = nullptr;
    int array_length = 0;
    float d_start = 0.0f;
    float min_d = 0.0f, max_d = 0.0f;

    constexpr float d_step = 5.0f;

    switch (dir)
    {
        case sensor_fr:
            sens_table   = sens_fr_table;
            array_length = sizeof(sens_fr_table) / sizeof(int16_t);
            d_start = 40.0f;
            min_d   = 40.0f;
            max_d   = 125.0f;
            break;

        case sensor_fl:
            sens_table   = sens_fl_table;
            array_length = sizeof(sens_fl_table) / sizeof(int16_t);
            d_start = 40.0f;
            min_d   = 40.0f;
            max_d   = 125.0f;
            break;

        case sensor_sr:
            sens_table   = sens_sr_table;
            array_length = sizeof(sens_sr_table) / sizeof(int16_t);
            d_start = IR_SIDE_MIN_MM;
            min_d   = IR_SIDE_MIN_MM;
            max_d   = 80.0f;
            break;

        case sensor_sl:
            sens_table   = sens_sl_table;
            array_length = sizeof(sens_sl_table) / sizeof(int16_t);
            d_start = IR_SIDE_MIN_MM;
            min_d   = IR_SIDE_MIN_MM;
            max_d   = 80.0f;
            break;

        default:
            if (state) *state = IrRangeState::Invalid;
            return 0.0f;
    }

    if (state) *state = ir_range_state(value, sens_table, array_length);
    float distance = tableToDistanceStep(
        sens_table,
        array_length,
        d_start,
        d_step,
        value
    );

    return clampf(distance, min_d, max_d);
}

void IrSensTask::IrSensorDistanceSet(Vehicle *vehicle)
{
	search_turn_gyro_radps = vehicle->ego.rad_velo.get();
	search_turn_guard_enabled = isEnableIrSens == True && irsens_motion == STRAIGHT_IRSENS
		&& vehicle->ideal.velo.get() > 0.20f && isfinite(search_turn_gyro_radps)
		&& fabsf(search_turn_gyro_radps) <= 2.0f;
	straight_edge_rotation_valid = isfinite(vehicle->ego.rad_velo.get())
		&& fabsf(vehicle->ego.rad_velo.get()) <= wall_edge_param.maximum_yaw_rate_radps;
	diagonal_edge_motion_valid = vehicle->ideal.velo.get() > 0.0f && isfinite(vehicle->ego.rad_velo.get())
		&& fabsf(vehicle->ego.rad_velo.get()) <= diagonal_edge_param.maximum_yaw_rate_radps;
	// Keep straight-control ADC averages separate from diagnostic range history.
	straight_left_average.update(sen_l.value);
	straight_right_average.update(sen_r.value);
	straight_left_distance_mm = Sensor_CalcDistance(sensor_sl, straight_left_average.value(), &straight_left_range);
	straight_right_distance_mm = Sensor_CalcDistance(sensor_sr, straight_right_average.value(), &straight_right_range);
	wall_sample_updated = true;
	static int i = 0;
	i = i + 1;
	if(i == 20) i = 0;
	ir_log_cnt = i;
	sen_fl.distance_sum = sen_fl.distance_sum - sen_fl.distance_log[i%20];
	sen_fr.distance_sum = sen_fr.distance_sum - sen_fr.distance_log[i%20];
	sen_r.distance_sum = sen_r.distance_sum - sen_r.distance_log[i%20];
	sen_l.distance_sum = sen_l.distance_sum - sen_l.distance_log[i%20];

	sen_fl.distance = Sensor_CalcDistance(sensor_fl,sen_fl.value, &sen_fl.range_state);
	sen_fr.distance = Sensor_CalcDistance(sensor_fr,sen_fr.value, &sen_fr.range_state);
	sen_l.distance = Sensor_CalcDistance(sensor_sl,sen_l.value, &sen_l.range_state);
	sen_r.distance = Sensor_CalcDistance(sensor_sr,sen_r.value, &sen_r.range_state);
	WallDiagnostics::instance().sample(sen_l.range_state, sen_r.range_state, sen_fl.range_state, sen_fr.range_state,
		vehicle->ideal.velo.get() > 0.20f);

	sen_fl.distance_log[i%20] = sen_fl.distance;
	sen_fr.distance_log[i%20] = sen_fr.distance;
	sen_r.distance_log[i%20]  = sen_r.distance;
	sen_l.distance_log[i%20]  = sen_l.distance;

	sen_fl.distance_sum = sen_fl.distance_sum + sen_fl.distance_log[i%20];
	sen_fr.distance_sum = sen_fr.distance_sum + sen_fr.distance_log[i%20];
	sen_r.distance_sum = sen_r.distance_sum + sen_r.distance_log[i%20];
	sen_l.distance_sum = sen_l.distance_sum + sen_l.distance_log[i%20];


	sen_fl.avg_distance = (sen_fl.distance_sum/20.0);
	sen_fr.avg_distance = (sen_fr.distance_sum/20.0);
	sen_l.avg_distance = (sen_l.distance_sum/20.0);
	sen_r.avg_distance = (sen_r.distance_sum/20.0);

	sen_fl.prev_diff = sen_fl.diff;
	sen_fr.prev_diff = sen_fr.diff;
	sen_l.prev_diff = sen_l.diff;
	sen_r.prev_diff = sen_r.diff;

	sen_fl.diff = sen_fl.distance - sen_fl.avg_distance;
	sen_fr.diff = sen_fr.distance - sen_fr.avg_distance;
	sen_l.diff = sen_l.distance - sen_l.avg_distance;
	sen_r.diff = sen_r.distance - sen_r.avg_distance;

}

void IrSensTask::IrSensorWallSet(float forward_step_mm)
{
	wall_forward_step_mm = forward_step_mm;
	search_turn_guard.update(sen_fl.distance, sen_fr.distance, forward_step_mm, search_turn_guard_enabled);
	sen_fr.prev_is_wall 	= sen_fr.is_wall ;
	sen_fl.prev_is_wall 	= sen_fl.is_wall ;
	sen_r.prev_is_wall 	= sen_r.is_wall ;
	sen_l.prev_is_wall 	= sen_l.is_wall ;

	sen_fr.is_wall 	= (sen_fr.distance <= FRONT_THRESHOLD)? True:False;
	sen_fl.is_wall 	= (sen_fl.distance <= FRONT_THRESHOLD)? True:False;
	sen_r.is_wall 	= (sen_r.distance <= SIDE_THRESHOLD)? True:False;
	sen_l.is_wall  	= (sen_l.distance <= SIDE_THRESHOLD)? True:False;

	r_wall_corner = l_wall_corner = False;
	r_corner_delay_mm = l_corner_delay_mm = 0.0f;
	if (r_corner_time < 65535U) ++r_corner_time;
	if (l_corner_time < 65535U) ++l_corner_time;
	if (irsens_motion == STRAIGHT_IRSENS)
	{
		right_diagonal_edge.reset(); left_diagonal_edge.reset();
		const auto &parameter = WallPositionCorrection::edge_param(WallEdgeMode_Get());
		const bool classify_open_history = WallEdgeMode_Get() != WallPositionCorrection::Mode::Legacy;
		const auto right = right_wall_edge.update(sen_r.distance, forward_step_mm, sen_r.range_state != IrRangeState::Invalid, parameter, classify_open_history,
			ir_range_is_metric(sen_r.range_state) && sen_r.distance>IR_SIDE_MIN_MM,straight_edge_rotation_valid);
		const auto left = left_wall_edge.update(sen_l.distance, forward_step_mm, sen_l.range_state != IrRangeState::Invalid, parameter, classify_open_history,
			ir_range_is_metric(sen_l.range_state) && sen_l.distance>IR_SIDE_MIN_MM,straight_edge_rotation_valid);
		if (right.detected) {
			r_wall_corner = True; r_corner_time = right.age_ticks; r_corner_delay_mm = right.delay_mm;
			r_corner_generation = wall_edge_generation;
		}
		if (left.detected) {
			l_wall_corner = True; l_corner_time = left.age_ticks; l_corner_delay_mm = left.delay_mm;
			l_corner_generation = wall_edge_generation;
		}
	}
	else
	{
		right_wall_edge.reset(); left_wall_edge.reset();
		if (irsens_motion == DIAGONAL_IRSENS) {
			const auto right = right_diagonal_edge.update(sen_r.distance, forward_step_mm, diagonal_edge_motion_valid && sen_r.range_state != IrRangeState::Invalid);
			const auto left = left_diagonal_edge.update(sen_l.distance, forward_step_mm, diagonal_edge_motion_valid && sen_l.range_state != IrRangeState::Invalid);
			if (right.detected) {
				r_wall_corner = True; r_corner_time = right.age_ticks; r_corner_delay_mm = right.delay_mm;
				r_corner_generation = wall_edge_generation;
				++diagonal_wall_stats.edges_right;
				diagonal_wall_stats.edge_delay_max_mm = fmaxf(diagonal_wall_stats.edge_delay_max_mm, right.delay_mm);
			}
			if (left.detected) {
				l_wall_corner = True; l_corner_time = left.age_ticks; l_corner_delay_mm = left.delay_mm;
				l_corner_generation = wall_edge_generation;
				++diagonal_wall_stats.edges_left;
				diagonal_wall_stats.edge_delay_max_mm = fmaxf(diagonal_wall_stats.edge_delay_max_mm, left.delay_mm);
			}
		}
		else { right_diagonal_edge.reset(); left_diagonal_edge.reset(); }
	}

	// Confirmed Wall->Open and legacy diagonal crossings share the side LEDs.
	if (r_wall_corner == True) Indicate_LED_Pulse(0x10U,50U);
	if (l_wall_corner == True) Indicate_LED_Pulse(0x20U,50U);
	sen_fr.control_cnt = (sen_fr.is_wall == True) ? sen_fr.control_cnt + 1 : 0;
	sen_fl.control_cnt = (sen_fl.is_wall == True) ? sen_fl.control_cnt + 1 : 0;
	sen_fr.control_th = (sen_fr.control_cnt > 10) ? FRONT_THRESHOLD : 90.0;
	sen_fl.control_th = (sen_fl.control_cnt > 10) ? FRONT_THRESHOLD : 90.0;

	// Use encoder distance for stability; diff/prev_diff remain the temporal
	// signals used by wall-corner detection above.
	right_wall_stability.update(sen_r.distance, forward_step_mm,
		sen_r.is_wall == True && r_wall_corner == False
		&& sen_fr.distance > SIDECONTROL_FR_THRESHOLD);
	left_wall_stability.update(sen_l.distance, forward_step_mm,
		sen_l.is_wall == True && l_wall_corner == False
		&& sen_fl.distance > SIDECONTROL_FL_THRESHOLD);
	sen_r.control_distance = right_wall_stability.stable_distance_mm();
	sen_l.control_distance = left_wall_stability.stable_distance_mm();

	//need to update
	if(isEnableIrSens == True)
	{

		if(irsens_motion == STRAIGHT_IRSENS || irsens_motion == DIAGONAL_IRSENS)
		{
			sen_r.control_th = right_wall_stability.ready() ? SIDE_THRESHOLD : wall_ref;
			sen_l.control_th = left_wall_stability.ready() ? SIDE_THRESHOLD : wall_ref;

		}
		else
		{
			sen_r.control_th = wall_ref;
			sen_l.control_th = wall_ref;
		}

		sen_r.is_control 	= (sen_r.is_wall == True && sen_r.distance <= sen_r.control_th)? True:False;
		sen_l.is_control 	= (sen_l.is_wall == True && sen_l.distance <= sen_l.control_th)? True:False;

		if(irsens_motion == STRAIGHT_IRSENS || irsens_motion == DIAGONAL_IRSENS)
		{
			sen_r.is_control 	= (sen_fr.distance > SIDE_THRESHOLD+1.0)? sen_r.is_control:False;
			sen_l.is_control 	= (sen_fl.distance > SIDE_THRESHOLD+1.0)? sen_l.is_control:False;
		}
		else
		{
			sen_r.is_control 	= (sen_fr.distance <= SIDECONTROL_FR_THRESHOLD)? False:sen_r.is_control;
			sen_l.is_control 	= (sen_fl.distance <= SIDECONTROL_FL_THRESHOLD)? False:sen_l.is_control;

		}

		sen_r.error	= (sen_r.is_control == True) ? sen_r.distance - wall_ref : 0.0;
		sen_l.error	= (sen_l.is_control == True) ? sen_l.distance - wall_ref : 0.0;



		if(irsens_motion == STRAIGHT_IRSENS)
		{
			//sen_r.error	= sen_r.error;
			//sen_l.error	= sen_l.error;

		}
		if(irsens_motion == DIAGONAL_IRSENS)
		{
			sen_r.error	= (sen_r.error	<= 0.0f) ? sen_r.error : 0.0;
			sen_l.error	= (sen_l.error	<= 0.0f) ? sen_l.error : 0.0;
		}
	}
	else
	{

		sen_r.control_th = DIAGONAL_REF;
		sen_l.control_th = DIAGONAL_REF;

		sen_r.is_control 	= False;
		sen_l.is_control 	= False;

		sen_r.error	=  0.0;
		sen_l.error	=  0.0;
	}
}

void IrSensTask::IrSensorReferenceSet(float ref_value)
{
	 wall_ref = ref_value;
}

float IrSensTask::IrSensorMaxValueFromLog(t_sensor_dir dir)
{
	float value;
	switch(dir)
	{
		case sensor_fl:
			value = sen_fl.distance_log[0];
			for(int i = 0; i < 20; i++)
			{
				if(sen_fl.distance_log[i] >value)
				{
					value = sen_fl.distance_log[i];
				}
			}
			break;
		case sensor_fr:
			value = sen_fr.distance_log[0];
			for(int i = 0; i < 20; i++)
			{
				if(sen_fr.distance_log[i] >value)
				{
					value = sen_fr.distance_log[i];
				}
			}
			break;
		case sensor_sr:
			value = sen_r.distance_log[0];
			for(int i = 0; i < 20; i++)
			{
				if(sen_r.distance_log[i] >value)
				{
					value = sen_r.distance_log[i];
				}
			}
			break;
		case sensor_sl:
			value = sen_l.distance_log[0];
			for(int i = 0; i < 20; i++)
			{
				if(sen_l.distance_log[i] >value)
				{
					value = sen_l.distance_log[i];
				}
			}
			break;
	}
	return value;
}

void IrSensTask::SetWallControl_RadVelo(Vehicle *vehicle,float delta_tms)
{
	if (irsens_motion == STRAIGHT_IRSENS)
	{
		bool synchronize_world_lateral=false;
		ZirconiaWallControl::Input input;
		const bool moving_enabled = isEnableIrSens == True && vehicle->ideal.velo.get() > 0.20f;
		input.enabled = moving_enabled && straight_left_average.ready() && straight_right_average.ready();
		if (moving_enabled)
		{
			if (input.enabled && straight_prime_pending)
			{
				straight_wall_control.prime_tracking(straight_left_distance_mm / 1000.0f,
					straight_right_distance_mm / 1000.0f);
				straight_prime_pending = false;
			}
			// Gyro heading relative to this straight-control interval. Keeping it
			// separate avoids existing wall-edge/segment odometry resets erasing
			// the heading damping term. Positive yaw is a left turn.
			if (straight_heading_valid && !straight_heading_skip_once)
			{
				straight_heading_rad += vehicle->ego.rad_velo.get() * delta_tms / 1000.0f;
				straight_heading_variance_rad2 += wall_ekf_param.heading_walk_rad_sqrt_s
					* wall_ekf_param.heading_walk_rad_sqrt_s * delta_tms / 1000.0f;
			}
			straight_heading_skip_once = false;
			straight_heading_valid = true;
		}
		else
		{
			ResetStraightWallControl();
		}
		input.left_m = straight_left_distance_mm / 1000.0f;
		input.right_m = straight_right_distance_mm / 1000.0f;
		// A raw near-limit sample must retain repulsion while the ADC average
		// catches up. Neither that bound nor a raw invalid sample is a metric observation.
		if (sen_l.range_state == IrRangeState::NearLimit) input.left_m = sen_l.distance / 1000.0f;
		if (sen_r.range_state == IrRangeState::NearLimit) input.right_m = sen_r.distance / 1000.0f;
		input.left_signal = straight_left_average.value();
		input.right_signal = straight_right_average.value();
		input.travel_m = wall_sample_updated ? wall_forward_step_mm / 1000.0f : 0.0f;
		input.velocity_mps = vehicle->ego.velo.get();
		input.heading_rad = straight_heading_rad;
		// type8 x is positive left; zirconia's center error is positive right.
		input.estimated_error_m = -vehicle->ego.x_point.get() / 1000.0f;
		input.dt_s = delta_tms / 1000.0f;
		input.sample_updated = wall_sample_updated;
		if (input.enabled && !straight_world_bound) {
			straight_world_origin=vehicle->pose.passage_origin({0,vehicle->ego.x_point.get(),0});
			straight_world_bound=vehicle->pose.snapshot().active;
		}
		input.range_valid_mask = (ir_range_is_metric(straight_left_range) && ir_range_is_metric(sen_l.range_state) ? 1U : 0U)
			| (ir_range_is_metric(straight_right_range) && ir_range_is_metric(sen_r.range_state) ? 2U : 0U);
		if (!ir_range_can_see_wall(straight_left_range) || !ir_range_can_see_wall(sen_l.range_state)) input.permitted_mask &= ~1U;
		if (!ir_range_can_see_wall(straight_right_range) || !ir_range_can_see_wall(sen_r.range_state)) input.permitted_mask &= ~2U;
		straight_pillar_guard_mask = WallEdgeMode_Get() == WallPositionCorrection::Mode::Legacy ? 0U
			: (straight_pillars[0].open_origin()?1U:0U) | (straight_pillars[1].open_origin()?2U:0U);
		// A qualified Open-origin return is a feature, not a continuous wall.
		// Apply before BOTH fallback control and EKF wall assimilation, including ADC-average tails.
		input.permitted_mask &= static_cast<uint8_t>(~straight_pillar_guard_mask);
		input.range_valid_mask &= static_cast<uint8_t>(~straight_pillar_guard_mask);
		input.heading_only_gap = straight_pillar_guard_mask && straight_heading_valid;
		straight_wall_output = straight_wall_control.update(input);
		straight_wall_ekf_used = false;
		if (input.enabled && wall_ekf_mode != WallEkfMode::Legacy)
		{
			// Predict from signed travel for THIS IRQ, never from ego.length
			// differences: wall-edge corrections may rebase that progress value.
			const bool already_started = straight_wall_filter.estimate().initialized;
			if (!already_started)
				straight_wall_filter.start(vehicle->ego.x_point.get() / 1000.0f, straight_heading_rad,
					straight_heading_variance_rad2);
			else
				straight_wall_filter.predict(input.travel_m, vehicle->ego.rad_velo.get() * input.dt_s, input.dt_s);
			if (FeatureEkfEnabled()) AssimilateFeatures(vehicle);
			// Motion generation sees the current wall edge before this call.
			// Assimilate it only after predicting to that same sensor timestamp.
			if (corner_pending)
			{
				straight_wall_filter.observe_corner_lateral(pending_corner_lateral_m);
				corner_pending = false;
			}
			const float ds = fabsf(input.travel_m);
			left_edge_guard_m = fmaxf(0.0f, left_edge_guard_m - ds);
			right_edge_guard_m = fmaxf(0.0f, right_edge_guard_m - ds);
			if (input.sample_updated && (left_wall_edge.pending() || l_wall_corner == True)) left_edge_guard_m = wall_ekf_param.edge_guard_m;
			if (input.sample_updated && (right_wall_edge.pending() || r_wall_corner == True)) right_edge_guard_m = wall_ekf_param.edge_guard_m;
			// Legacy side-quality gates are evaluated before the current EKF wall
			// correction. A corner is a separate event, not a large lateral step.
			uint8_t candidates = straight_wall_output.trusted_mask;
			if (left_edge_guard_m > 0.0f) candidates &= ~1U;
			if (right_edge_guard_m > 0.0f) candidates &= ~2U;
			straight_wall_candidate_mask = candidates;
			const uint8_t accepted = straight_wall_filter.observe_walls(input.left_m, input.right_m,
				candidates, input.sample_updated);
			straight_wall_estimate = straight_wall_filter.estimate();
			straight_wall_ekf_active = straight_wall_estimate.initialized;
			ZirconiaWallControl::Input filtered = input;
			filtered.use_estimated_pose = true;
			filtered.prediction_trusted = CanUseStraightWallEkfPose();
			// Keep the concurrently running legacy output until the complete pose
			// is usable. Reset the EKF controller while gated out: an old saturated
			// yaw command must not survive loss of wall/pose trust.
			// Keep the existing close-wall, away-only response while a wall is
			// being confirmed. Only the tracking measurement needs EKF acceptance.
			filtered.permitted_mask = straight_wall_output.observed_mask;
			if (left_edge_guard_m > 0.0f) filtered.permitted_mask &= ~1U;
			if (right_edge_guard_m > 0.0f) filtered.permitted_mask &= ~2U;
			filtered.enabled = filtered.prediction_trusted
				&& (accepted != 0 || filtered.permitted_mask == 0);
			filtered.measurement_mask = straight_wall_filter.lateral_trusted() ? accepted : 0;
			filtered.heading_rad = filtered.prediction_trusted ? straight_wall_estimate.heading_rad : straight_heading_rad;
			filtered.estimated_error_m = -straight_wall_estimate.lateral_m;
			straight_wall_ekf_control_output = ekf_wall_control.update(filtered);
			// Lateral observation and yaw validity are separate. Fresh accepted
			// ranges can still update position while yaw control uses the gyro.
			if (wall_ekf_mode == WallEkfMode::Control && accepted && straight_wall_filter.lateral_trusted())
			{
				vehicle->ego.x_point.set(straight_wall_estimate.lateral_m * 1000.0f);
				synchronize_world_lateral=true;
			}
			if (wall_ekf_mode == WallEkfMode::Control && filtered.enabled
				&& (straight_wall_ekf_control_output.pose_control
					|| straight_wall_ekf_control_output.phase == ZirconiaWallControl::Estimated))
			{
				straight_wall_ekf_used = true;
				straight_wall_output = straight_wall_ekf_control_output;
				vehicle->ego.x_point.set(straight_wall_estimate.lateral_m * 1000.0f);
				synchronize_world_lateral=true;
			}
			else if (wall_ekf_mode == WallEkfMode::Control && filtered.permitted_mask == 0 && !input.heading_only_gap)
			{
				// No visible wall and no usable heading: do not centre on stale x.
				straight_wall_output = straight_wall_ekf_control_output;
			}
		}
		wall_sample_updated = false;
		const auto &output = straight_wall_output;
		sen_l.is_control = output.phase == ZirconiaWallControl::Tracking
			&& (output.trusted_mask & 1) ? True : False;
		sen_r.is_control = output.phase == ZirconiaWallControl::Tracking
			&& (output.trusted_mask & 2) ? True : False;
		sen_l.error = sen_l.is_control ? straight_left_distance_mm - STRAIGHT_REF : 0.0f;
		sen_r.error = sen_r.is_control ? straight_right_distance_mm - STRAIGHT_REF : 0.0f;
		if (output.sync_estimator && !straight_wall_ekf_used)
		{
			vehicle->ego.x_point.set(-output.raw_error_m * 1000.0f);
			synchronize_world_lateral=true;
		}
		if (straight_world_bound && synchronize_world_lateral) {
			// Preserve global progress, including any limited longitudinal update.
			// Replace the native lateral coordinate; adding its absolute value or
			// comparing it with resettable ego.x_point would count travel twice.
			auto &frames=vehicle->pose;
			auto local=PoseFrames::relative(straight_world_origin,frames.snapshot().world);
			local.y_mm=vehicle->ego.x_point.get();
			if (straight_wall_ekf_used && CanUseStraightWallEkfPose()) local.yaw_rad=straight_wall_estimate.heading_rad;
			frames.adopt_world(PoseFrames::compose(straight_world_origin,local));
		}
		control_ir.set(-output.error_m * 1000.0f);
		control_ir_dot.set(straight_wall_ekf_used
			? straight_wall_estimate.heading_rad : straight_heading_rad);
		// Straight trajectories have zero base yaw. Assign the external-loop
		// correction once; adding it to last cycle's target would integrate twice.
		vehicle->ideal.rad_velo.set(output.omega_radps);
		vehicle->ideal.rad_accel.set(output.alpha_radps2);
		return;
	}

	const float diagonal_travel_mm=wall_sample_updated ? wall_forward_step_mm : 0.0f;
	if (irsens_motion == DIAGONAL_IRSENS)
	{
		straight_pillar_guard_mask = 0;
		const bool enabled = isEnableIrSens == True && vehicle->ideal.velo.get() > 0.20f;
		if (!enabled) diagonal_heading_prior=MotionHeading::Prior{};
		if (enabled && !diagonal_segment_active) ++diagonal_wall_stats.segments;
		diagonal_segment_active = enabled;
		if (enabled) ++diagonal_wall_stats.cycles;
		if (wall_ekf_mode != WallEkfMode::Legacy)
		{
			DiagonalWallControl::Input input;
			input.enabled = enabled;
			input.fresh = wall_sample_updated;
			input.travel_m = wall_sample_updated ? wall_forward_step_mm / 1000.0f : 0.0f;
			input.dt_s = delta_tms / 1000.0f;
			input.gyro_radps = vehicle->ego.rad_velo.get();
			input.left_m = sen_l.distance / 1000.0f;
			input.right_m = sen_r.distance / 1000.0f;
			input.front_left_m = sen_fl.distance / 1000.0f;
			input.front_right_m = sen_fr.distance / 1000.0f;
			input.eligible_mask = (sen_l.is_wall == True && l_wall_corner == False && ir_range_is_metric(sen_l.range_state) ? 1U : 0U)
				| (sen_r.is_wall == True && r_wall_corner == False && ir_range_is_metric(sen_r.range_state) ? 2U : 0U);
			diagonal_wall_output = diagonal_wall_control.update(input);
			if (diagonal_wall_output.updated) ++diagonal_wall_stats.updates;
			if (diagonal_wall_output.front_blocked) ++diagonal_wall_stats.front_blocked;
			if (wall_ekf_mode == WallEkfMode::Control)
			{
				if (!enabled) {
					diagonal_pose_steering.reset();
					control_ir.init(); control_ir_dot.init();
					vehicle->ideal.rad_velo.set(0.0f);
					vehicle->ideal.rad_accel.set(0.0f);
					wall_sample_updated = false;
					return;
				}
			}
			else
			{
				diagonal_wall_output.used_ekf = false; // Shadow observes without applying.
			}
		}
		wall_sample_updated = false;
	}
	float ir_rad_acc_control = 0.0;
	const float k1 = 1.0;
	const float k2 = 19.0;
	control_ir.init();
	control_ir_dot.init();
	float ir_xposition = vehicle->ego.x_point.get();
	float deviation_rad = 0.0f;
	float gain = CLAMP(vehicle->ego.velo.get()/0.320f,1.0,20.0);

	//sensor_output = k1*ydiff/1000.0 + k2/1000.0*theta;
	if(isEnableIrSens == True && vehicle->ideal.velo.get() > 0.20)
	{
		if(sen_r.is_control == True && sen_l.is_control == True)
		{
			ir_xposition =  -(sen_l.error - sen_r.error)/2.0;
			if(irsens_motion == STRAIGHT_IRSENS)
			{
				vehicle->ego.x_point.set(ir_xposition);
			}

		}
		else if(sen_r.is_control == True || sen_l.is_control == True)
		{
			ir_xposition =  -(sen_l.error - sen_r.error);
			if(irsens_motion == STRAIGHT_IRSENS)
			{
				//vehicle->ego.x_point.set((ir_xposition+vehicle->ego.x_point.get())/2.0);
			}

		}
	}



	// Keep the existing diagonal trajectory law, its velocity gain and damping.
	// Sparse EKF observations replace only the current lateral input. Losing
	// them must not remove the odometry/IR response between pillars.
	if (irsens_motion == DIAGONAL_IRSENS && wall_ekf_mode == WallEkfMode::Control)
	{
		if (diagonal_wall_output.used_ekf) {
			ir_xposition = diagonal_wall_output.error_m * 1000.0f;
			++diagonal_wall_stats.used;
		}
		else ++diagonal_wall_stats.fallback;
	}

	if(irsens_motion == STRAIGHT_IRSENS){
		//deviation_rad = vehicle->ego.radian.get();
	}
	else if(irsens_motion == DIAGONAL_IRSENS)
	{
		deviation_rad =  vehicle->ideal.radian.get();
	}

	ir_rad_acc_control = ir_xposition + k2*deviation_rad*0.0;

	control_ir.set(ir_rad_acc_control );
	//s_dot 	= k1*vehicle->ideal.velo.get()*1000.0*(vehicle->ideal.radian.get())*1.0 + k2*vehicle->ideal.rad_velo.get();
	control_ir_dot.set( k1*vehicle->ideal.velo.get()*1000.0*(deviation_rad)*(1.0) + k2*vehicle->ideal.rad_velo.get());


	float target_rad_acc	= 	(-1.0)*(	300.0*gain*k1/k2*control_ir.get()
										+ 	60.0*gain*1.0/k2*control_ir_dot.get()
										+ 	k1/k2*(vehicle->ideal.accel.get()*1000.0*deviation_rad*0.0 + vehicle->ideal.velo.get()*vehicle->ideal.rad_velo.get()*1000.0));



	const float previous_omega=vehicle->ideal.rad_velo.get();
	float target_rad_velo=previous_omega+target_rad_acc*delta_tms/1000.0f;
	DiagonalPose::Pose pose;
	const bool pose_control=irsens_motion==DIAGONAL_IRSENS && DiagonalPoseControlReady(pose);
	if (irsens_motion==DIAGONAL_IRSENS && (pose_control || diagonal_pose_steering.engaged())) {
		const auto target=diagonal_pose_steering.update(pose_control ? &pose : nullptr,vehicle->ideal.velo.get(),
			diagonal_travel_mm,delta_tms/1000.0f,previous_omega,{target_rad_velo,target_rad_acc});
		target_rad_velo=target.omega_radps;
		target_rad_acc=target.alpha_radps2;
		if (pose_control && diagonal_pose_steering.blend()>0) {
			++diagonal_wall_stats.pose_control_cycles;
			diagonal_wall_stats.pose_omega_max_radps=fmaxf(diagonal_wall_stats.pose_omega_max_radps,fabsf(target_rad_velo));
			control_ir.set(ir_xposition+diagonal_pose_steering.blend()*(pose.e-ir_xposition));
		}
	}
	vehicle->ideal.rad_accel.set(target_rad_acc);
	vehicle->ideal.rad_velo.set(target_rad_velo);
	if (irsens_motion == DIAGONAL_IRSENS && wall_ekf_mode == WallEkfMode::Control)
	{
		diagonal_wall_output.error_m = ir_xposition / 1000.0f;
		diagonal_wall_output.omega_radps = target_rad_velo;
		diagonal_wall_output.alpha_radps2 = target_rad_acc;
		// Preserve the diagonal Control log unit [rad]. It is diagnostic only;
		// the trajectory derivative above stays in the shared control law.
		control_ir_dot.set(pose_control && diagonal_pose_steering.blend()>0 ? pose.heading : diagonal_wall_output.estimate.heading_rad);
	}

}

SearchTurnGuard::Decision IrSensTask::SearchTurnDecision(bool right_turn, bool front_wall) const
{
	SearchTurnGuard::Input input;
	SearchTurnGuard::History history;
	{
		const StateLock lock;
		input.right_turn = right_turn; input.front_wall = front_wall;
		input.left_mm = sen_l.distance; input.right_mm = sen_r.distance;
		input.front_left_mm = sen_fl.distance; input.front_right_mm = sen_fr.distance;
		const t_sensor *sensors[] = {&sen_l, &sen_r, &sen_fl, &sen_fr};
		for (unsigned i = 0; i < 4; ++i) { input.adc[i] = sensors[i]->value; input.range[i] = sensors[i]->range_state; }
		input.left_wall = sen_l.is_wall == True; input.right_wall = sen_r.is_wall == True;
		input.ekf_control = wall_ekf_mode == WallEkfMode::Control;
		input.ekf_pose_used = straight_wall_ekf_used;
		input.pose_trusted = straight_wall_ekf_used && CanUseStraightWallEkfPose()
			&& straight_wall_filter.estimate().accepted_mask != 0 && straight_wall_output.phase == ZirconiaWallControl::Tracking;
		const auto &pose = straight_wall_filter.estimate();
		input.lateral_mm = pose.lateral_m * 1000.0f; input.heading_rad = pose.heading_rad;
		input.lateral_variance_mm2 = pose.variance_lateral_m2 * 1000000.0f;
		input.heading_variance_rad2 = pose.variance_heading_rad2;
		input.gyro_radps = search_turn_gyro_radps;
		history = search_turn_guard.history();
	}
	return SearchTurnGuard::decide(input, history);
}

bool IrSensTask::CorrectStraightWallCorner(Vehicle *vehicle, float left_positive_mm)
{
	const StateLock lock;
	if (!isfinite(left_positive_mm)) return false;
	if (irsens_motion == STRAIGHT_IRSENS && wall_ekf_mode != WallEkfMode::Legacy
		&& straight_wall_filter.estimate().initialized)
	{
		if (wall_ekf_mode == WallEkfMode::Control && fabsf(left_positive_mm) > STRAIGHT_REF) return false;
		pending_corner_lateral_m = left_positive_mm / 1000.0f;
		corner_pending = true;
		if (wall_ekf_mode == WallEkfMode::Control)
			return true;
	}
	// Shadow/legacy modes preserve the existing correction exactly. If the
	// filter is not initialized there is no second estimate to correct yet.
	auto &frames=vehicle->pose;
	if (straight_world_bound) {
		auto local=PoseFrames::relative(straight_world_origin,frames.snapshot().world);
		local.y_mm=left_positive_mm; frames.adopt_world(PoseFrames::compose(straight_world_origin,local));
	} else frames.correct_passage(0.0f,left_positive_mm-vehicle->ego.x_point.get());
	vehicle->ego.x_point.set(left_positive_mm);
	return true;
}



int16_t IrSensTask::IrSensor_Avg()
{
	if (!Sensor_IrSamplesReady()) return 0;
	return (sen_l.value + sen_r.value)/2 ;
}

bool IrSensTask::HandStartDetected(int16_t threshold)
{
	Sensor_SetHandWaitActive(1U);
	if (!Sensor_IrSamplesReady() || IrSensor_Avg() <= threshold) return false;
	Sensor_SetHandWaitActive(0U);
	return true;
}


