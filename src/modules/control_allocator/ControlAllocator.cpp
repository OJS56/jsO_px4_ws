/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ControlAllocator.cpp
 *
 * Control allocator.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ControlAllocator.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;

ModuleBase::Descriptor ControlAllocator::desc{task_spawn, custom_command, print_usage};

ControlAllocator::ControlAllocator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_control_allocator_status_pub[0].advertise();
	_control_allocator_status_pub[1].advertise();

	_control_allocator_ftc_debug_pub.advertise();
	_actuator_motors_pub.advertise();
	_actuator_servos_pub.advertise();
	_actuator_servos_trim_pub.advertise();

	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_R%u_SLEW", i);
		_param_handles.slew_rate_motors[i] = param_find(buffer);
	}

	for (int i = 0; i < MAX_NUM_SERVOS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SV%u_SLEW", i);
		_param_handles.slew_rate_servos[i] = param_find(buffer);
	}

	parameters_updated();
}

ControlAllocator::~ControlAllocator()
{
	for (int i = 0; i < ActuatorEffectiveness::MAX_NUM_MATRICES; ++i) {
		delete _control_allocation[i];
	}

	delete _actuator_effectiveness;

	perf_free(_loop_perf);
}

bool
ControlAllocator::init()
{
	if (!_vehicle_torque_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

#ifndef ENABLE_LOCKSTEP_SCHEDULER // Backup schedule would interfere with lockstep
	ScheduleDelayed(50_ms);
#endif

	return true;
}

void
ControlAllocator::parameters_updated()
{
	_has_slew_rate = false;

	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		param_get(_param_handles.slew_rate_motors[i], &_params.slew_rate_motors[i]);
		_has_slew_rate |= _params.slew_rate_motors[i] > FLT_EPSILON;
	}

	for (int i = 0; i < MAX_NUM_SERVOS; ++i) {
		param_get(_param_handles.slew_rate_servos[i], &_params.slew_rate_servos[i]);
		_has_slew_rate |= _params.slew_rate_servos[i] > FLT_EPSILON;
	}

	// Allocation method & effectiveness source
	// Do this first: in case a new method is loaded, it will be configured below
	bool updated = update_effectiveness_source();
	update_allocation_method(updated); // must be called after update_effectiveness_source()

	if (_num_control_allocation == 0) {
		return;
	}

	for (int i = 0; i < _num_control_allocation; ++i) {
		_control_allocation[i]->updateParameters();
	}

	update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::CONFIGURATION_UPDATE);
}

void
ControlAllocator::update_allocation_method(bool force)
{
	AllocationMethod configured_method = (AllocationMethod)_param_ca_method.get();
   
	if (!_actuator_effectiveness) {
		PX4_ERR("_actuator_effectiveness null");
		return;
	}

	if (_allocation_method_id != configured_method || force) {

		ActuatorVector actuator_sp[ActuatorEffectiveness::MAX_NUM_MATRICES];

		// Cleanup first
		for (int i = 0; i < ActuatorEffectiveness::MAX_NUM_MATRICES; ++i) {
			// Save current state
			if (_control_allocation[i] != nullptr) {
				actuator_sp[i] = _control_allocation[i]->getActuatorSetpoint();
			}

			delete _control_allocation[i];
			_control_allocation[i] = nullptr;
		}

		_num_control_allocation = _actuator_effectiveness->numMatrices();

		AllocationMethod desired_methods[ActuatorEffectiveness::MAX_NUM_MATRICES];
		_actuator_effectiveness->getDesiredAllocationMethod(desired_methods);

		bool normalize_rpy[ActuatorEffectiveness::MAX_NUM_MATRICES];
		_actuator_effectiveness->getNormalizeRPY(normalize_rpy);

		for (int i = 0; i < _num_control_allocation; ++i) {
			AllocationMethod method = configured_method;

			if (configured_method == AllocationMethod::AUTO) {
				method = desired_methods[i];
			}

			switch (method) {
			case AllocationMethod::PSEUDO_INVERSE:
				_control_allocation[i] = new ControlAllocationPseudoInverse();
				break;

			case AllocationMethod::SEQUENTIAL_DESATURATION:
				_control_allocation[i] = new ControlAllocationSequentialDesaturation();
				break;

			default:
				PX4_ERR("Unknown allocation method");
				break;
			}

			if (_control_allocation[i] == nullptr) {
				PX4_ERR("alloc failed");
				_num_control_allocation = 0;

			} else {
				_control_allocation[i]->setNormalizeRPY(normalize_rpy[i]);
				_control_allocation[i]->setActuatorSetpoint(actuator_sp[i]);
			}
		}

		_allocation_method_id = configured_method;
	}
}

bool
ControlAllocator::update_effectiveness_source()
{
	const EffectivenessSource source = (EffectivenessSource)_param_ca_airframe.get();

	if (_effectiveness_source_id != source) {

		// try to instanciate new effectiveness source
		ActuatorEffectiveness *tmp = nullptr;

		switch (source) {
		case EffectivenessSource::NONE:
		case EffectivenessSource::MULTIROTOR:
			tmp = new ActuatorEffectivenessMultirotor(this);
			break;

		case EffectivenessSource::STANDARD_VTOL:
			tmp = new ActuatorEffectivenessStandardVTOL(this);
			break;

		case EffectivenessSource::TILTROTOR_VTOL:
			tmp = new ActuatorEffectivenessTiltrotorVTOL(this);
			break;

		case EffectivenessSource::TAILSITTER_VTOL:
			tmp = new ActuatorEffectivenessTailsitterVTOL(this);
			break;

		case EffectivenessSource::ROVER_ACKERMANN:
			tmp = new ActuatorEffectivenessRoverAckermann();
			break;

		case EffectivenessSource::ROVER_DIFFERENTIAL:
			// rover_differential_control does allocation and publishes directly to actuator_motors topic
			break;

		case EffectivenessSource::FIXED_WING:
			tmp = new ActuatorEffectivenessFixedWing(this);
			break;

		case EffectivenessSource::MOTORS_6DOF: // just a different UI from MULTIROTOR
			tmp = new ActuatorEffectivenessUUV(this);
			break;

		case EffectivenessSource::MULTIROTOR_WITH_TILT:
			tmp = new ActuatorEffectivenessMCTilt(this);
			break;

		case EffectivenessSource::CUSTOM:
			tmp = new ActuatorEffectivenessCustom(this);
			break;

		case EffectivenessSource::HELICOPTER_TAIL_ESC:
			tmp = new ActuatorEffectivenessHelicopter(this, ActuatorType::MOTORS);
			break;

		case EffectivenessSource::HELICOPTER_TAIL_SERVO:
			tmp = new ActuatorEffectivenessHelicopter(this, ActuatorType::SERVOS);
			break;

		case EffectivenessSource::HELICOPTER_COAXIAL:
			tmp = new ActuatorEffectivenessHelicopterCoaxial(this);
			break;

		case EffectivenessSource::SPACECRAFT_2D:
			tmp = new ActuatorEffectivenessSpacecraft(this);
			break;

		case EffectivenessSource::SPACECRAFT_3D:
			tmp = new ActuatorEffectivenessSpacecraft(this);
			break;

		default:
			PX4_ERR("Unknown airframe");
			break;
		}

		// Replace previous source with new one
		if (tmp == nullptr) {
			// It did not work, forget about it
			PX4_ERR("Actuator effectiveness init failed");
			_param_ca_airframe.set((int)_effectiveness_source_id);

		} else {
			// Swap effectiveness sources
			delete _actuator_effectiveness;
			_actuator_effectiveness = tmp;

			// Save source id
			_effectiveness_source_id = source;
		}

		return true;
	}

	return false;
}

void
ControlAllocator::Run()
{
	if (should_exit()) {
		_vehicle_torque_setpoint_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_loop_perf);

#ifndef ENABLE_LOCKSTEP_SCHEDULER // Backup schedule would interfere with lockstep
	// Push backup schedule
	ScheduleDelayed(50_ms);
#endif

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		if (_handled_motor_failure_bitmask == 0) {
			// We don't update the geometry after an actuator failure, as it could lead to unexpected results
			// (e.g. a user could add/remove motors, such that the bitmask isn't correct anymore)
			updateParams();
			parameters_updated();
		}
	}

	if (_num_control_allocation == 0 || _actuator_effectiveness == nullptr) {
		return;
	}

	{
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.update(&vehicle_status)) {
			const bool was_armed = _armed;

			_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
			_is_vtol = vehicle_status.is_vtol;

			if (_armed != was_armed) {
				_ftc_fault_trigger_active = false;
				_ftc_degraded_allocation_active = false;
				_ftc_output_fault_active = false;
				_ftc_mode = FtcMode::NORMAL;
				_ftc_fault_type = 0;
				_ftc_fault_timestamp = 0;
				_ftc_fault_motor_idx = -1;
				_ftc_current_loe = 1.f;
				_ftc_fault_nominal_command = 0.f;
				_ftc_fault_applied_command = 0.f;
				_ftc_fault_command_limit = 1.f;
				_ftc_residual_yaw_moment = 0.f;
				_reaction_wheel_torque_command = 0.f;
				_reaction_wheel_active = false;
				_reaction_wheel_latched_active = false;
			}

			ActuatorEffectiveness::FlightPhase flight_phase{ActuatorEffectiveness::FlightPhase::HOVER_FLIGHT};

			// Check if the current flight phase is HOVER or FIXED_WING
			if (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				flight_phase = ActuatorEffectiveness::FlightPhase::HOVER_FLIGHT;

			} else {
				flight_phase = ActuatorEffectiveness::FlightPhase::FORWARD_FLIGHT;
			}

			// Special cases for VTOL in transition
			if (vehicle_status.is_vtol && vehicle_status.in_transition_mode) {
				if (vehicle_status.in_transition_to_fw) {
					flight_phase = ActuatorEffectiveness::FlightPhase::TRANSITION_HF_TO_FF;

				} else {
					flight_phase = ActuatorEffectiveness::FlightPhase::TRANSITION_FF_TO_HF;
				}
			}

			// Forward to effectiveness source
			_actuator_effectiveness->setFlightPhase(flight_phase);
		}
	}

	{
		vehicle_control_mode_s vehicle_control_mode;

		if (_vehicle_control_mode_sub.update(&vehicle_control_mode)) {
			_publish_controls = vehicle_control_mode.flag_control_allocation_enabled;
		}
	}

	// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);

	update_ftc_state(now);

	bool do_update = false;
	vehicle_torque_setpoint_s vehicle_torque_setpoint;
	vehicle_thrust_setpoint_s vehicle_thrust_setpoint;

	// Run allocator on torque changes
	if (_vehicle_torque_setpoint_sub.update(&vehicle_torque_setpoint)) {
		_torque_sp = matrix::Vector3f(vehicle_torque_setpoint.xyz);

		do_update = true;
		_timestamp_sample = vehicle_torque_setpoint.timestamp_sample;

	}

	if (_vehicle_thrust_setpoint_sub.update(&vehicle_thrust_setpoint)) {
		_thrust_sp = matrix::Vector3f(vehicle_thrust_setpoint.xyz);
	}

	if (do_update) {
		_last_run = now;
		float pre_ftc_motor_controls[MAX_NUM_MOTORS] {};
		float post_ftc_motor_controls[MAX_NUM_MOTORS] {};
		float final_motor_controls[MAX_NUM_MOTORS] {};

		check_for_motor_failures();

		update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::NO_EXTERNAL_UPDATE);

		// Set control setpoint vector(s)
		matrix::Vector<float, NUM_AXES> c[ActuatorEffectiveness::MAX_NUM_MATRICES];
		c[0](0) = _torque_sp(0);
		c[0](1) = _torque_sp(1);
		c[0](2) = _torque_sp(2);
		c[0](3) = _thrust_sp(0);
		c[0](4) = _thrust_sp(1);
		c[0](5) = _thrust_sp(2);

		if (_num_control_allocation > 1) {
			if (_vehicle_torque_setpoint1_sub.copy(&vehicle_torque_setpoint)) {
				c[1](0) = vehicle_torque_setpoint.xyz[0];
				c[1](1) = vehicle_torque_setpoint.xyz[1];
				c[1](2) = vehicle_torque_setpoint.xyz[2];
			}

			if (_vehicle_thrust_setpoint1_sub.copy(&vehicle_thrust_setpoint)) {
				c[1](3) = vehicle_thrust_setpoint.xyz[0];
				c[1](4) = vehicle_thrust_setpoint.xyz[1];
				c[1](5) = vehicle_thrust_setpoint.xyz[2];
			}
		}

		for (int i = 0; i < _num_control_allocation; ++i) {

			_control_allocation[i]->setControlSetpoint(c[i]);

			// Do allocation
			_control_allocation[i]->allocate();
			_actuator_effectiveness->allocateAuxilaryControls(dt, i, _control_allocation[i]->_actuator_sp); //flaps and spoilers
			_actuator_effectiveness->updateSetpoint(c[i], i, _control_allocation[i]->_actuator_sp,
								_control_allocation[i]->getActuatorMin(), _control_allocation[i]->getActuatorMax());

			if (i == 0) {
				fill_motor_controls_from_allocation(pre_ftc_motor_controls);
				if (_ftc_degraded_allocation_active) {
					apply_active_ftc_allocation(i, c[i], now);
				}
				// The motors are always in allocation 0
				handle_stopped_motors(now);
				fill_motor_controls_from_allocation(post_ftc_motor_controls);
			}

			if (_has_slew_rate) {
				_control_allocation[i]->applySlewRateLimit(dt);
			}

			_control_allocation[i]->clipActuatorSetpoint();

			if (i == 0) {
				fill_motor_outputs_for_publish(final_motor_controls);
			}
		}

		publish_control_allocator_ftc_debug(now, pre_ftc_motor_controls, post_ftc_motor_controls, final_motor_controls);
	}

	update_reaction_wheel_setpoint(_reaction_wheel_torque_command, _ftc_residual_yaw_moment,
				       _reaction_wheel_latched_active, now);

	// Publish actuator setpoint and allocator status
	publish_actuator_controls();

	// Publish status at limited rate, as it's somewhat expensive and we use it for slower dynamics
	// (i.e. anti-integrator windup)
	if (now - _last_status_pub >= 5_ms) {
		publish_control_allocator_status(0);

		if (_num_control_allocation > 1) {
			publish_control_allocator_status(1);
		}

		_last_status_pub = now;
	}

	perf_end(_loop_perf);
}

void
ControlAllocator::fill_motor_controls_from_allocation(float controls[MAX_NUM_MOTORS]) const
{
	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		controls[i] = NAN;
	}

	int actuator_idx = 0;
	int actuator_idx_matrix[ActuatorEffectiveness::MAX_NUM_MATRICES] {};

	for (int motors_idx = 0; motors_idx < _num_actuators[0] && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
		const int selected_matrix = _control_allocation_selection_indexes[actuator_idx];
		const float actuator_sp = _control_allocation[selected_matrix]->getActuatorSetpoint()(actuator_idx_matrix[selected_matrix]);
		controls[motors_idx] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;

		++actuator_idx_matrix[selected_matrix];
		++actuator_idx;
	}
}

void
ControlAllocator::fill_motor_outputs_for_publish(float controls[MAX_NUM_MOTORS]) const
{
	fill_motor_controls_from_allocation(controls);
	apply_ftc_output_fault(controls);
}

void
ControlAllocator::overlay_reaction_wheel_control(float controls[MAX_NUM_MOTORS])
{
	const int motor_idx = math::constrain(_param_ca_rw_mot_idx.get() - 1, 0, MAX_NUM_MOTORS - 1);

	reaction_wheel_actuator_setpoint_s wheel_sp{};

	if (!_reaction_wheel_actuator_setpoint_sub.copy(&wheel_sp) || !wheel_sp.active) {
		controls[motor_idx] = 0.f;
		return;
	}

	controls[motor_idx] = math::constrain(wheel_sp.control, -1.f, 1.f);

	if (((_param_r_rev.get() & (1u << motor_idx)) == 0u) && !_reaction_wheel_reversible_warned) {
		PX4_WARN("Reaction wheel motor slot %d is not reversible in CA_R_REV", motor_idx + 1);
		_reaction_wheel_reversible_warned = true;
	}
}

void
ControlAllocator::fill_motor_saturation_from_allocation(int8_t saturation[MAX_NUM_MOTORS]) const
{
	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		saturation[i] = control_allocator_ftc_debug_s::ACTUATOR_SATURATION_OK;
	}

	int actuator_idx = 0;
	int actuator_idx_matrix[ActuatorEffectiveness::MAX_NUM_MATRICES] {};

	for (int motors_idx = 0; motors_idx < _num_actuators[0] && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
		const int selected_matrix = _control_allocation_selection_indexes[actuator_idx];
		const auto &allocator = _control_allocation[selected_matrix];
		const float actuator_sp = allocator->getActuatorSetpoint()(actuator_idx_matrix[selected_matrix]);
		const float actuator_min = allocator->getActuatorMin()(actuator_idx_matrix[selected_matrix]);
		const float actuator_max = allocator->getActuatorMax()(actuator_idx_matrix[selected_matrix]);

		if (actuator_sp > (actuator_max - FLT_EPSILON)) {
			saturation[motors_idx] = control_allocator_ftc_debug_s::ACTUATOR_SATURATION_UPPER;

		} else if (actuator_sp < (actuator_min + FLT_EPSILON)) {
			saturation[motors_idx] = control_allocator_ftc_debug_s::ACTUATOR_SATURATION_LOWER;
		}

		++actuator_idx_matrix[selected_matrix];
		++actuator_idx;
	}
}

void
ControlAllocator::publish_control_allocator_ftc_debug(const hrt_abstime now, const float pre_ftc[MAX_NUM_MOTORS],
		const float post_ftc[MAX_NUM_MOTORS], const float final[MAX_NUM_MOTORS])
{
	control_allocator_ftc_debug_s debug{};
	debug.timestamp = now;
	debug.timestamp_sample = _timestamp_sample;
	debug.ftc_active = _ftc_fault_trigger_active;
	debug.fault_trigger_active = _ftc_fault_trigger_active;
	debug.degraded_allocation_active = _ftc_degraded_allocation_active;
	debug.output_fault_active = _ftc_output_fault_active;
	debug.fault_motor_index = _ftc_fault_motor_idx >= 0 ? _ftc_fault_motor_idx + 1 : 0;
	debug.handled_motor_failure_mask = _handled_motor_failure_bitmask;
	debug.motor_stop_mask = _motor_stop_mask;
	debug.loe = _ftc_current_loe;
	debug.nominal_fault_command = _ftc_fault_nominal_command;
	debug.applied_fault_command = _ftc_fault_applied_command;
	debug.fault_command_limit = _ftc_fault_command_limit;
	debug.residual_yaw_moment = _ftc_residual_yaw_moment;

	for (int axis = 0; axis < 3; ++axis) {
		debug.torque_setpoint[axis] = _torque_sp(axis);
		debug.thrust_setpoint[axis] = _thrust_sp(axis);
	}

	fill_motor_saturation_from_allocation(debug.actuator_saturation);

	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		debug.pre_ftc_control[i] = pre_ftc[i];
		debug.post_ftc_control[i] = post_ftc[i];
		debug.final_control[i] = final[i];
	}

	_control_allocator_ftc_debug_pub.publish(debug);
}

void
ControlAllocator::update_ftc_state(hrt_abstime now)
{
	manual_control_setpoint_s manual_control_setpoint{};

	if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
		_manual_control_setpoint = manual_control_setpoint;
	}

	const FtcMode previous_mode = _ftc_mode;
	_ftc_mode = FtcMode::NORMAL;

	if (!_armed || _param_ca_ftc_en.get() == 0 || _param_ca_ftc_type.get() == 0) {
		_ftc_fault_trigger_active = false;
		_ftc_degraded_allocation_active = false;
		_ftc_output_fault_active = false;
		_reaction_wheel_active = false;
		_reaction_wheel_latched_active = false;
		return;
	}

	const FtcTriggerMode trigger_mode = static_cast<FtcTriggerMode>(_param_ca_ftc_trig_mode.get());
	const auto report_ftc_transition = [&](FtcMode mode, const char *source_label, double source_value, bool integer_value) {
		if (mode == FtcMode::NORMAL || previous_mode == mode) {
			return;
		}

		_ftc_fault_timestamp = now;
		const char *fault_type_str = (_param_ca_ftc_type.get() == 1) ? "LOE"
			: (_param_ca_ftc_type.get() == 2) ? "Saturation" : "unknown";
		const char *mode_str = (mode == FtcMode::FAULT_DEGRADED) ? "fault_degraded" : "fault_nominal";
		const int fault_motor = static_cast<int>(math::constrain(_param_ca_ftc_mot.get(), int32_t{1},
					int32_t{actuator_motors_s::NUM_CONTROLS}));
		const double loe = (double)math::constrain(_param_ca_ftc_loe.get(), 0.f, 1.f);

		if (integer_value) {
			PX4_WARN("FTC state %s: motor=%d type=%s lambda=%.2f %s=%.0f",
				 mode_str, fault_motor, fault_type_str, loe, source_label, source_value);

		} else {
			PX4_WARN("FTC state %s: motor=%d type=%s lambda=%.2f %s=%.2f",
				 mode_str, fault_motor, fault_type_str, loe, source_label, source_value);
		}

		// Keep the GCS message short enough for MAVLink STATUSTEXT.
		mavlink_log_warning(&_mavlink_log_pub, "FTC %s m%d %s lam=%.2f\t",
				    mode == FtcMode::FAULT_DEGRADED ? "degraded" : "nominal",
				    fault_motor, fault_type_str, loe);

		if (mode == FtcMode::FAULT_DEGRADED) {
			events::send(events::ID("control_allocator_ftc_degraded"),
				     events::Log::Warning, "FTC degraded");

		} else {
			events::send(events::ID("control_allocator_ftc_nominal"),
				     events::Log::Warning, "FTC nominal");
		}
	};

	switch (trigger_mode) {
	case FtcTriggerMode::PARAM: {
		switch (_param_ca_ftc_state.get()) {
		case 1:
			_ftc_mode = FtcMode::FAULT_DEGRADED;
			report_ftc_transition(_ftc_mode, "param_state", 1., true);
			break;

		case 2:
			_ftc_mode = FtcMode::FAULT_NOMINAL;
			report_ftc_transition(_ftc_mode, "param_state", 2., true);
			break;

		default:
			_ftc_mode = FtcMode::NORMAL;
			break;
		}

		break;
	}

	case FtcTriggerMode::AUX: {
		if (!_manual_control_setpoint.valid || _param_ca_ftc_trig_src.get() <= 0) {
			_ftc_fault_trigger_active = false;
			_ftc_degraded_allocation_active = false;
			_ftc_output_fault_active = false;
			_reaction_wheel_active = false;
			_reaction_wheel_latched_active = false;
			return;
		}

		const float aux_value = get_selected_ftc_aux_value();

		if (PX4_ISFINITE(aux_value)) {
			if (aux_value < -0.5f) {
				_ftc_mode = FtcMode::FAULT_DEGRADED;
				report_ftc_transition(_ftc_mode, "aux", (double)aux_value, false);

			} else if (aux_value > 0.5f) {
				_ftc_mode = FtcMode::FAULT_NOMINAL;
				report_ftc_transition(_ftc_mode, "aux", (double)aux_value, false);
			}
		}

		if (previous_mode != FtcMode::NORMAL && _ftc_mode == FtcMode::NORMAL) {
			mavlink_log_info(&_mavlink_log_pub, "no failure\t");
			events::send(events::ID("control_allocator_ftc_no_failure"),
				     events::Log::Info, "no failure");
		}

		break;
	}

	case FtcTriggerMode::BUTTONS: {
		if (!_manual_control_setpoint.valid) {
			_ftc_fault_trigger_active = false;
			_ftc_degraded_allocation_active = false;
			_ftc_output_fault_active = false;
			_reaction_wheel_active = false;
			_reaction_wheel_latched_active = false;
			return;
		}

		const bool degraded_pressed = is_ftc_button_pressed(_param_ca_ftc_btn_deg.get());
		const bool nominal_pressed = is_ftc_button_pressed(_param_ca_ftc_btn_nom.get());

		if (degraded_pressed != nominal_pressed) {
			if (degraded_pressed) {
				_ftc_mode = FtcMode::FAULT_DEGRADED;
				report_ftc_transition(_ftc_mode, "buttons", (double)_param_ca_ftc_btn_deg.get(), true);

			} else {
				_ftc_mode = FtcMode::FAULT_NOMINAL;
				report_ftc_transition(_ftc_mode, "buttons", (double)_param_ca_ftc_btn_nom.get(), true);
			}
		}

		break;
	}

	default:
		break;
	}

	_ftc_fault_trigger_active = _ftc_mode != FtcMode::NORMAL;
	_ftc_degraded_allocation_active = _ftc_mode == FtcMode::FAULT_DEGRADED;
	_ftc_output_fault_active = _ftc_fault_trigger_active;

	if (_ftc_fault_trigger_active) {
		_ftc_fault_type = _param_ca_ftc_type.get();
		_ftc_fault_motor_idx = math::constrain(_param_ca_ftc_mot.get() - 1, int32_t{0},
					      int32_t{actuator_motors_s::NUM_CONTROLS - 1});
		_ftc_current_loe = math::constrain(_param_ca_ftc_loe.get(), 0.f, 1.f);

	} else {
		_ftc_fault_type = 0;
		_ftc_fault_motor_idx = -1;
		_ftc_current_loe = 1.f;
		_reaction_wheel_active = false;
		_reaction_wheel_latched_active = false;
	}
}

float
ControlAllocator::get_selected_ftc_aux_value() const
{
	switch (_param_ca_ftc_trig_src.get()) {
	case 1:
		return _manual_control_setpoint.aux1;

	case 2:
		return _manual_control_setpoint.aux2;

	case 3:
		return _manual_control_setpoint.aux3;

	case 4:
		return _manual_control_setpoint.aux4;

	case 5:
		return _manual_control_setpoint.aux5;

	case 6:
		return _manual_control_setpoint.aux6;

	default:
		return NAN;
	}
}

bool
ControlAllocator::is_ftc_button_pressed(int button_number) const
{
	if (button_number <= 0 || button_number > 16) {
		return false;
	}

	return (_manual_control_setpoint.buttons & (1u << (button_number - 1))) != 0;
}

void
ControlAllocator::update_reaction_wheel_setpoint(float torque_command, float residual_yaw_moment, bool active, hrt_abstime now)
{
	reaction_wheel_setpoint_s reaction_wheel_setpoint{};
	reaction_wheel_setpoint.timestamp = now;
	reaction_wheel_setpoint.torque = torque_command;
	reaction_wheel_setpoint.residual_yaw_moment = residual_yaw_moment;
	reaction_wheel_setpoint.active = active;
	_reaction_wheel_setpoint_pub.publish(reaction_wheel_setpoint);
}

bool
ControlAllocator::get_motor_column_index(int motor_idx, int matrix_index, int &matrix_column) const
{
	int actuator_idx = 0;
	int actuator_idx_matrix[ActuatorEffectiveness::MAX_NUM_MATRICES] {};

	for (int motors_idx = 0; motors_idx < _num_actuators[(int)ActuatorType::MOTORS]
	     && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
		const int selected_matrix = _control_allocation_selection_indexes[actuator_idx];
		const int current_matrix_column = actuator_idx_matrix[selected_matrix];

		if (motors_idx == motor_idx && selected_matrix == matrix_index) {
			matrix_column = current_matrix_column;
			return true;
		}

		++actuator_idx_matrix[selected_matrix];
		++actuator_idx;
	}

	return false;
}

void
ControlAllocator::apply_active_ftc_allocation(int matrix_index, const matrix::Vector<float, NUM_AXES> &control_sp, hrt_abstime now)
{
	const bool was_reaction_wheel_active = _reaction_wheel_latched_active;
	_ftc_fault_nominal_command = 0.f;
	_ftc_fault_applied_command = 0.f;
	_ftc_fault_command_limit = 1.f;
	_ftc_residual_yaw_moment = 0.f;
	_reaction_wheel_torque_command = 0.f;
	_reaction_wheel_active = false;

	if (!_ftc_fault_trigger_active || _ftc_fault_motor_idx < 0 || matrix_index != 0) {
		_reaction_wheel_latched_active = false;
		return;
	}

	const int motor_count = _num_actuators[(int)ActuatorType::MOTORS];

	if (motor_count < 4) {
		_reaction_wheel_latched_active = false;
		return;
	}

	int fault_column = -1;

	if (!get_motor_column_index(_ftc_fault_motor_idx, matrix_index, fault_column)) {
		_reaction_wheel_latched_active = false;
		return;
	}

	ControlAllocationPseudoInverse *pseudo_inverse_allocation =
		static_cast<ControlAllocationPseudoInverse *>(_control_allocation[matrix_index]);

	if (pseudo_inverse_allocation == nullptr) {
		_reaction_wheel_latched_active = false;
		return;
	}

	const matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> &mix = pseudo_inverse_allocation->getMixMatrix();
	const ActuatorVector &actuator_min = _control_allocation[matrix_index]->getActuatorMin();
	const ActuatorVector &actuator_max = _control_allocation[matrix_index]->getActuatorMax();
	const ActuatorVector &actuator_trim = _control_allocation[matrix_index]->getActuatorTrim();
	const matrix::Vector<float, NUM_AXES> &control_trim = _control_allocation[matrix_index]->getControlTrim();
	ActuatorVector &actuator_sp = _control_allocation[matrix_index]->_actuator_sp;

	const float nominal_fault_command = actuator_sp(fault_column);
	const float fault_command_limit = actuator_max(fault_column) * _ftc_current_loe;
	const float applied_fault_command = fminf(nominal_fault_command, fault_command_limit);

	_ftc_fault_nominal_command = nominal_fault_command;
	_ftc_fault_applied_command = applied_fault_command;
	_ftc_fault_command_limit = fault_command_limit;

	matrix::SquareMatrix<float, 3> reduced_mix;
	matrix::Vector3f reduced_control_delta;
	int healthy_columns[3] {};
	int healthy_count = 0;
	int fault_motor_matrix_column = -1;

	for (int motor_idx = 0; motor_idx < motor_count && motor_idx < actuator_motors_s::NUM_CONTROLS; motor_idx++) {
		int motor_matrix_column = -1;

		if (!get_motor_column_index(motor_idx, matrix_index, motor_matrix_column)) {
			continue;
		}

		if (motor_idx == _ftc_fault_motor_idx) {
			fault_motor_matrix_column = motor_matrix_column;
			continue;
		}

		if (healthy_count < 3) {
			healthy_columns[healthy_count++] = motor_matrix_column;
		}
	}

	if (healthy_count != 3 || fault_motor_matrix_column < 0) {
		_reaction_wheel_latched_active = false;
		return;
	}

	// Keep the wheel controller latched for the whole FTC fault session once the
	// degraded allocation path is valid. A zero residual only means no additional
	// wheel acceleration is needed for this sample, not that wheel state should reset.
	_reaction_wheel_latched_active = true;
	_reaction_wheel_active = true;

	if (nominal_fault_command <= fault_command_limit + FLT_EPSILON) {
		return;
	}

	const int reduced_axes[3] {0, 1, 5};

	for (int axis = 0; axis < 3; axis++) {
		reduced_control_delta(axis) = control_sp(reduced_axes[axis]) - control_trim(reduced_axes[axis]);

		for (int motor = 0; motor < 3; motor++) {
			reduced_mix(motor, axis) = mix(healthy_columns[motor], reduced_axes[axis]);
		}
	}

	const matrix::Vector3f healthy_solution = reduced_mix * reduced_control_delta;

	for (int col = 0; col < 3; col++) {
		const float actuator_command = actuator_trim(healthy_columns[col]) + healthy_solution(col);
		actuator_sp(healthy_columns[col]) = math::constrain(actuator_command, actuator_min(healthy_columns[col]),
					 actuator_max(healthy_columns[col]));
	}

	actuator_sp(fault_motor_matrix_column) = math::constrain(applied_fault_command, actuator_min(fault_motor_matrix_column),
				 actuator_max(fault_motor_matrix_column));

	const matrix::Vector<float, NUM_AXES> allocated_control = _control_allocation[matrix_index]->getAllocatedControl();
	_ftc_residual_yaw_moment = control_sp(2) - allocated_control(2);
	// Publish the wheel/body-reaction-sign-corrected feedforward torque request.
	// Yaw-rate feedback is intentionally applied downstream in
	// reaction_wheel_torque_control, not in the allocator.
	_reaction_wheel_torque_command = -_ftc_residual_yaw_moment;
	_reaction_wheel_active = _reaction_wheel_latched_active;

	if (!was_reaction_wheel_active) {
		PX4_WARN("FTC reallocation active: motor=%d nominal=%.3f applied=%.3f limit=%.3f yaw_res=%.3f wheel=%.3f t=%.3fs",
			 _ftc_fault_motor_idx + 1, (double)_ftc_fault_nominal_command, (double)_ftc_fault_applied_command,
			 (double)_ftc_fault_command_limit, (double)_ftc_residual_yaw_moment,
			 (double)_reaction_wheel_torque_command, (double)(now / 1e6));
		mavlink_log_warning(&_mavlink_log_pub, "FTC realloc active m%d lim=%.2f\t",
				    _ftc_fault_motor_idx + 1, (double)_ftc_fault_command_limit);
	}
}

float
ControlAllocator::get_ftc_fault_output_limit() const
{
	if (_ftc_fault_motor_idx < 0 || _num_control_allocation == 0 || _control_allocation[0] == nullptr) {
		return NAN;
	}

	int fault_column = -1;

	if (!get_motor_column_index(_ftc_fault_motor_idx, 0, fault_column)) {
		return NAN;
	}

	const ActuatorVector &actuator_max = _control_allocation[0]->getActuatorMax();
	return actuator_max(fault_column) * _ftc_current_loe;
}

void
ControlAllocator::apply_ftc_output_fault(float controls[MAX_NUM_MOTORS]) const
{
	if (!_ftc_output_fault_active || _ftc_fault_motor_idx < 0 || _ftc_fault_motor_idx >= MAX_NUM_MOTORS) {
		return;
	}

	float &fault_control = controls[_ftc_fault_motor_idx];

	if (!PX4_ISFINITE(fault_control)) {
		return;
	}

	const float fault_limit = get_ftc_fault_output_limit();

	if (!PX4_ISFINITE(fault_limit)) {
		return;
	}

	fault_control = fminf(fault_control, fault_limit);
}

void
ControlAllocator::update_effectiveness_matrix_if_needed(EffectivenessUpdateReason reason)
{
	ActuatorEffectiveness::Configuration config{};

	if (reason == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE
	    && hrt_elapsed_time(&_last_effectiveness_update) < 100_ms) { // rate-limit updates
		return;
	}

	if (_actuator_effectiveness->getEffectivenessMatrix(config, reason)) {
		_last_effectiveness_update = hrt_absolute_time();

		memcpy(_control_allocation_selection_indexes, config.matrix_selection_indexes,
		       sizeof(_control_allocation_selection_indexes));

		// Get the minimum and maximum depending on type and configuration
		ActuatorEffectiveness::ActuatorVector minimum[ActuatorEffectiveness::MAX_NUM_MATRICES];
		ActuatorEffectiveness::ActuatorVector maximum[ActuatorEffectiveness::MAX_NUM_MATRICES];
		ActuatorEffectiveness::ActuatorVector slew_rate[ActuatorEffectiveness::MAX_NUM_MATRICES];
		int actuator_idx = 0;
		int actuator_idx_matrix[ActuatorEffectiveness::MAX_NUM_MATRICES] {};

		actuator_servos_trim_s trims{};
		static_assert(actuator_servos_trim_s::NUM_CONTROLS == actuator_servos_s::NUM_CONTROLS, "size mismatch");

		for (int actuator_type = 0; actuator_type < (int)ActuatorType::COUNT; ++actuator_type) {
			_num_actuators[actuator_type] = config.num_actuators[actuator_type];

			for (int actuator_type_idx = 0; actuator_type_idx < config.num_actuators[actuator_type]; ++actuator_type_idx) {
				if (actuator_idx >= NUM_ACTUATORS) {
					_num_actuators[actuator_type] = 0;
					PX4_ERR("Too many actuators");
					break;
				}

				int selected_matrix = _control_allocation_selection_indexes[actuator_idx];

				if ((ActuatorType)actuator_type == ActuatorType::MOTORS) {
					if (actuator_type_idx >= MAX_NUM_MOTORS) {
						PX4_ERR("Too many motors");
						_num_actuators[actuator_type] = 0;
						break;
					}

					if (_param_r_rev.get() & (1u << actuator_type_idx)) {
						minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;

					} else {
						minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = 0.f;
					}

					slew_rate[selected_matrix](actuator_idx_matrix[selected_matrix]) = _params.slew_rate_motors[actuator_type_idx];

				} else if ((ActuatorType)actuator_type == ActuatorType::SERVOS) {
					if (actuator_type_idx >= MAX_NUM_SERVOS) {
						PX4_ERR("Too many servos");
						_num_actuators[actuator_type] = 0;
						break;
					}

					minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;
					slew_rate[selected_matrix](actuator_idx_matrix[selected_matrix]) = _params.slew_rate_servos[actuator_type_idx];
					trims.trim[actuator_type_idx] = config.trim[selected_matrix](actuator_idx_matrix[selected_matrix]);

				} else {
					minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;
				}

				maximum[selected_matrix](actuator_idx_matrix[selected_matrix]) = 1.f;

				++actuator_idx_matrix[selected_matrix];
				++actuator_idx;
			}
		}

		// Handle failed actuators
		if (_handled_motor_failure_bitmask) {
			actuator_idx = 0;
			memset(&actuator_idx_matrix, 0, sizeof(actuator_idx_matrix));

			for (int motors_idx = 0; motors_idx < _num_actuators[0] && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
				int selected_matrix = _control_allocation_selection_indexes[actuator_idx];

				if (_handled_motor_failure_bitmask & (1 << motors_idx)) {
					ActuatorEffectiveness::EffectivenessMatrix &matrix = config.effectiveness_matrices[selected_matrix];

					for (int i = 0; i < NUM_AXES; i++) {
						matrix(i, actuator_idx_matrix[selected_matrix]) = 0.0f;
					}
				}

				++actuator_idx_matrix[selected_matrix];
				++actuator_idx;
			}
		}

		for (int i = 0; i < _num_control_allocation; ++i) {
			_control_allocation[i]->setActuatorMin(minimum[i]);
			_control_allocation[i]->setActuatorMax(maximum[i]);
			_control_allocation[i]->setSlewRateLimit(slew_rate[i]);

			// Set all the elements of a row to 0 if that row has weak authority.
			// That ensures that the algorithm doesn't try to control axes with only marginal control authority,
			// which in turn would degrade the control of the main axes that actually should and can be controlled.

			ActuatorEffectiveness::EffectivenessMatrix &matrix = config.effectiveness_matrices[i];

			for (int n = 0; n < NUM_AXES; n++) {
				bool all_entries_small = true;

				for (int m = 0; m < config.num_actuators_matrix[i]; m++) {
					if (fabsf(matrix(n, m)) > 0.05f) {
						all_entries_small = false;
					}
				}

				if (all_entries_small) {
					matrix.row(n) = 0.f;
				}
			}

			// Assign control effectiveness matrix
			int total_num_actuators = config.num_actuators_matrix[i];
			_control_allocation[i]->setEffectivenessMatrix(config.effectiveness_matrices[i], config.trim[i],
					config.linearization_point[i], total_num_actuators, reason == EffectivenessUpdateReason::CONFIGURATION_UPDATE);
		}

		trims.timestamp = hrt_absolute_time();
		_actuator_servos_trim_pub.publish(trims);
	}
}


void
ControlAllocator::handle_stopped_motors(const hrt_abstime now)
{
	const ActuatorBitmask stopped_motors_due_to_effectiveness = _actuator_effectiveness->getStoppedMotors();

	const ActuatorBitmask stopped_motors = stopped_motors_due_to_effectiveness
					       | _handled_motor_failure_bitmask
					       | _motor_stop_mask;

	// Handle stopped motors by setting NaN
	const unsigned int allocation_index = 0;  // Motors always in allocation 0
	_control_allocation[allocation_index]->applyNanToActuators(stopped_motors);

	// Apply ice shedding, which applies _only_ to stopped motors
	const bool any_stopped_motor_failed = 0 != (stopped_motors_due_to_effectiveness & (_handled_motor_failure_bitmask | _motor_stop_mask));
	const float ice_shedding_output = get_ice_shedding_output(now);

	if (ice_shedding_output > FLT_EPSILON && !any_stopped_motor_failed) {
		for (int motors_idx = 0; motors_idx < _num_actuators[allocation_index] && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
			if (stopped_motors & 1u << motors_idx) {
				_control_allocation[allocation_index]->_actuator_sp(motors_idx) = ice_shedding_output;
			}
		}
	}
}

void
ControlAllocator::publish_control_allocator_status(int matrix_index)
{
	control_allocator_status_s control_allocator_status{};
	control_allocator_status.timestamp = hrt_absolute_time();

	// TODO: disabled motors (?)

	// Allocated control
	const matrix::Vector<float, NUM_AXES> &allocated_control = _control_allocation[matrix_index]->getAllocatedControl();

	// Unallocated control
	const matrix::Vector<float, NUM_AXES> unallocated_control = _control_allocation[matrix_index]->getControlSetpoint() -
			allocated_control;
	control_allocator_status.unallocated_torque[0] = unallocated_control(0);
	control_allocator_status.unallocated_torque[1] = unallocated_control(1);
	control_allocator_status.unallocated_torque[2] = unallocated_control(2);
	control_allocator_status.unallocated_thrust[0] = unallocated_control(3);
	control_allocator_status.unallocated_thrust[1] = unallocated_control(4);
	control_allocator_status.unallocated_thrust[2] = unallocated_control(5);

	// override control_allocator_status in customized saturation logic for certain effectiveness types
	_actuator_effectiveness->getUnallocatedControl(matrix_index, control_allocator_status);

	// Allocation success flags
	control_allocator_status.torque_setpoint_achieved = (Vector3f(control_allocator_status.unallocated_torque[0],
			control_allocator_status.unallocated_torque[1],
			control_allocator_status.unallocated_torque[2]).norm_squared() < 1e-6f);
	control_allocator_status.thrust_setpoint_achieved = (Vector3f(control_allocator_status.unallocated_thrust[0],
			control_allocator_status.unallocated_thrust[1],
			control_allocator_status.unallocated_thrust[2]).norm_squared() < 1e-6f);

	// Actuator saturation
	const ActuatorVector &actuator_sp = _control_allocation[matrix_index]->getActuatorSetpoint();
	const ActuatorVector &actuator_min = _control_allocation[matrix_index]->getActuatorMin();
	const ActuatorVector &actuator_max = _control_allocation[matrix_index]->getActuatorMax();

	for (int i = 0; i < NUM_ACTUATORS; i++) {
		if (actuator_sp(i) > (actuator_max(i) - FLT_EPSILON)) {
			control_allocator_status.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_UPPER;

		} else if (actuator_sp(i) < (actuator_min(i) + FLT_EPSILON)) {
			control_allocator_status.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_LOWER;
		}
	}

	// Handled motor failures
	control_allocator_status.handled_motor_failure_mask = _handled_motor_failure_bitmask;
	control_allocator_status.motor_stop_mask = _motor_stop_mask;

	_control_allocator_status_pub[matrix_index].publish(control_allocator_status);
}

float
ControlAllocator::get_ice_shedding_output(hrt_abstime now)
{
	const float period_sec = _param_ice_shedding_period.get();

	const bool feature_disabled_by_param = period_sec <= FLT_EPSILON;
	const bool in_forward_flight = _actuator_effectiveness->getFlightPhase() == ActuatorEffectiveness::FlightPhase::FORWARD_FLIGHT;

	// If any stopped motor has failed, the feature will create much more
	// torque than in the nominal case, and becomes pointless anyway as we
	// cannot go back to multicopter
	const bool apply_shedding = _is_vtol && in_forward_flight;

	if (feature_disabled_by_param || !apply_shedding) {
		return 0.0f;

	} else {
		// Square wave output
		const float elapsed_in_period = fmodf(static_cast<float>(now) / 1_s, period_sec);
		const float ice_shedding_output = elapsed_in_period < ICE_SHEDDING_ON_SEC ? ICE_SHEDDING_OUTPUT : 0.0f;

		return ice_shedding_output;
	}
}

void
ControlAllocator::publish_actuator_controls()
{
	if (!_publish_controls) {
		return;
	}

	actuator_motors_s actuator_motors;
	actuator_motors.timestamp = hrt_absolute_time();
	actuator_motors.timestamp_sample = _timestamp_sample;

	actuator_servos_s actuator_servos;
	actuator_servos.timestamp = actuator_motors.timestamp;
	actuator_servos.timestamp_sample = _timestamp_sample;

	actuator_motors.reversible_flags = _param_r_rev.get();
	fill_motor_outputs_for_publish(actuator_motors.control);
	overlay_reaction_wheel_control(actuator_motors.control);

	_actuator_motors_pub.publish(actuator_motors);

	// servos
	if (_num_actuators[1] > 0) {
		int actuator_idx = _num_actuators[(int)ActuatorType::MOTORS];
		int actuator_idx_matrix[ActuatorEffectiveness::MAX_NUM_MATRICES] {};

		for (int motor_idx = 0; motor_idx < _num_actuators[(int)ActuatorType::MOTORS]
		     && motor_idx < actuator_motors_s::NUM_CONTROLS; motor_idx++) {
			const int selected_matrix = _control_allocation_selection_indexes[motor_idx];
			++actuator_idx_matrix[selected_matrix];
		}

		int servos_idx;

		for (servos_idx = 0; servos_idx < _num_actuators[1] && servos_idx < actuator_servos_s::NUM_CONTROLS; servos_idx++) {
			const int selected_matrix = _control_allocation_selection_indexes[actuator_idx];
			const float actuator_sp = _control_allocation[selected_matrix]->getActuatorSetpoint()(actuator_idx_matrix[selected_matrix]);
			actuator_servos.control[servos_idx] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;
			++actuator_idx_matrix[selected_matrix];
			++actuator_idx;
		}

		for (int i = servos_idx; i < actuator_servos_s::NUM_CONTROLS; i++) {
			actuator_servos.control[i] = NAN;
		}

		_actuator_servos_pub.publish(actuator_servos);
	}
}

void
ControlAllocator::check_for_motor_failures()
{
	failure_detector_status_s failure_detector_status;

	if ((FailureMode)_param_ca_failure_mode.get() > FailureMode::IGNORE
	    && _failure_detector_status_sub.update(&failure_detector_status)) {

		if (_motor_stop_mask != failure_detector_status.motor_stop_mask) {
			_motor_stop_mask = failure_detector_status.motor_stop_mask;
			PX4_WARN("Stopping motors (%d)", _motor_stop_mask);
		}

		if (failure_detector_status.fd_motor) {
			if (_handled_motor_failure_bitmask != failure_detector_status.motor_failure_mask) {
				// motor failure bitmask changed
				switch ((FailureMode)_param_ca_failure_mode.get()) {
				case FailureMode::REMOVE_FIRST_FAILING_MOTOR: {
						// Count number of failed motors
						const int num_motors_failed = math::countSetBits(failure_detector_status.motor_failure_mask);

						// Only handle if it is the first failure
						if (_handled_motor_failure_bitmask == 0 && num_motors_failed == 1) {
							_handled_motor_failure_bitmask = failure_detector_status.motor_failure_mask;
							PX4_WARN("Removing motor from allocation (0x%x)", _handled_motor_failure_bitmask);

							for (int i = 0; i < _num_control_allocation; ++i) {
								_control_allocation[i]->setHadActuatorFailure(true);
							}

							update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE);
						}
					}
					break;

				default:
					break;
				}

			}

		} else if (_handled_motor_failure_bitmask != 0) {
			// Clear bitmask completely
			PX4_INFO("Restoring all motors");
			_handled_motor_failure_bitmask = 0;

			for (int i = 0; i < _num_control_allocation; ++i) {
				_control_allocation[i]->setHadActuatorFailure(false);
			}

			update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE);
		}
	}
}

int ControlAllocator::task_spawn(int argc, char *argv[])
{
	ControlAllocator *instance = new ControlAllocator();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int ControlAllocator::print_status()
{
	PX4_INFO("Running");
	const char *fault_type_str = "none";

	switch (_ftc_fault_type) {
	case 1:
		fault_type_str = "LOE";
		break;

	case 2:
		fault_type_str = "Saturation";
		break;

	default:
		break;
	}

	const char *mode_str = "normal";

	switch (_ftc_mode) {
	case FtcMode::FAULT_DEGRADED:
		mode_str = "fault_degraded";
		break;

	case FtcMode::FAULT_NOMINAL:
		mode_str = "fault_nominal";
		break;

	case FtcMode::NORMAL:
	default:
		break;
	}

	const char *trigger_mode_str = "aux";

	switch (static_cast<FtcTriggerMode>(_param_ca_ftc_trig_mode.get())) {
	case FtcTriggerMode::PARAM:
		trigger_mode_str = "param";
		break;

	case FtcTriggerMode::BUTTONS:
		trigger_mode_str = "buttons";
		break;

	case FtcTriggerMode::AUX:
	default:
		break;
	}

	PX4_INFO("FTC: %s, trigger=%s, mode=%s, fault_time=%.3fs, motor=%d, type=%s, lambda=%.2f",
		 _ftc_fault_trigger_active ? "active" : "inactive",
		 trigger_mode_str,
		 mode_str,
		 (double)(_ftc_fault_timestamp / 1e6),
		 _ftc_fault_motor_idx >= 0 ? _ftc_fault_motor_idx + 1 : 0,
		 fault_type_str,
		 (double)_ftc_current_loe);

	PX4_INFO("FTC alloc: degraded=%s output_fault=%s nominal=%.3f applied=%.3f limit=%.3f yaw_res=%.3f wheel=%.3f wheel_active=%s",
		 _ftc_degraded_allocation_active ? "yes" : "no",
		 _ftc_output_fault_active ? "yes" : "no",
		 (double)_ftc_fault_nominal_command,
		 (double)_ftc_fault_applied_command,
		 (double)_ftc_fault_command_limit,
		 (double)_ftc_residual_yaw_moment,
		 (double)_reaction_wheel_torque_command,
		 _reaction_wheel_latched_active ? "yes" : "no");

	// Print current allocation method
	switch (_allocation_method_id) {
	case AllocationMethod::NONE:
		PX4_INFO("Method: None");
		break;

	case AllocationMethod::PSEUDO_INVERSE:
		PX4_INFO("Method: Pseudo-inverse");
		break;

	case AllocationMethod::SEQUENTIAL_DESATURATION:
		PX4_INFO("Method: Sequential desaturation");
		break;

	case AllocationMethod::AUTO:
		PX4_INFO("Method: Auto");
		break;
	}

	// Print current airframe
	if (_actuator_effectiveness != nullptr) {
		PX4_INFO("Effectiveness Source: %s", _actuator_effectiveness->name());
	}

	// Print current effectiveness matrix
	for (int i = 0; i < _num_control_allocation; ++i) {
		const ActuatorEffectiveness::EffectivenessMatrix &effectiveness = _control_allocation[i]->getEffectivenessMatrix();

		if (_num_control_allocation > 1) {
			PX4_INFO("Instance: %i", i);
		}

		PX4_INFO("  Effectiveness =");
		int num_configured = _control_allocation[i]->numConfiguredActuators();

		// print column numbering
		if (num_configured > 1) {
			printf("  ");

			for (int col = 0; col < num_configured; col++) {
				printf("|%2u      ", col);
			}

			printf("\n");
		}

		// Print effectiveness matrix with row labels
		const char *row_labels[] = {"Mx", "My", "Mz", "Fx", "Fy", "Fz"};

		for (int row = 0; row < 6; row++) {
			printf("%2s|", row_labels[row]);

			for (int col = 0; col < num_configured; col++) {
				double d = static_cast<double>(effectiveness(row, col));

				// avoid -0.0 for display
				if (fabs(d - 0.0) < 1e-9) {
					// print fixed width zero
					printf(" 0       ");

				} else if ((fabs(d) < 1e-4) || (fabs(d) >= 10.0)) {
					printf("% .1e ", d);

				} else {
					printf("% 6.5f ", d);
				}
			}

			printf("\n");
		}

		PX4_INFO("  minimum =");

		// print column numbering
		if (num_configured > 1) {
			printf("  ");

			for (int col = 0; col < num_configured; col++) {
				printf("|%2u      ", col);
			}

			printf("\n");
		}

		printf("  |");

		for (int col = 0; col < num_configured; col++) {
			double d = static_cast<double>(_control_allocation[i]->getActuatorMin()(col));

			// avoid -0.0 for display
			if (fabs(d - 0.0) < 1e-9) {
				// print fixed width zero
				printf(" 0       ");

			} else if ((fabs(d) < 1e-4) || (fabs(d) >= 10.0)) {
				printf("% .1e ", d);

			} else {
				printf("% 6.5f ", d);
			}
		}

		printf("\n");
		PX4_INFO("  maximum =");

		// print column numbering
		if (num_configured > 1) {
			printf("  ");

			for (int col = 0; col < num_configured; col++) {
				printf("|%2u      ", col);
			}

			printf("\n");
		}

		printf("  |");

		for (int col = 0; col < num_configured; col++) {
			double d = static_cast<double>(_control_allocation[i]->getActuatorMax()(col));

			// avoid -0.0 for display
			if (fabs(d - 0.0) < 1e-9) {
				// print fixed width zero
				printf(" 0       ");

			} else if ((fabs(d) < 1e-4) || (fabs(d) >= 10.0)) {
				printf("% .1e ", d);

			} else {
				printf("% 6.5f ", d);
			}
		}

		printf("\n");
		PX4_INFO("  Configured actuators: %i", num_configured);
	}

	if (_handled_motor_failure_bitmask) {
		PX4_INFO("Failed motors: %i (0x%x)", math::countSetBits(_handled_motor_failure_bitmask),
			 _handled_motor_failure_bitmask);
	}

	// Print perf
	perf_print_counter(_loop_perf);

	return 0;
}

int ControlAllocator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ControlAllocator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements control allocation. It takes torque and thrust setpoints
as inputs and outputs actuator setpoint messages.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("control_allocator", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * Control Allocator app start / stop handling function
 */
extern "C" __EXPORT int control_allocator_main(int argc, char *argv[]);

int control_allocator_main(int argc, char *argv[])
{
	return ModuleBase::main(ControlAllocator::desc, argc, argv);
}
