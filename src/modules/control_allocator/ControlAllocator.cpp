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

	if (!_esc_status_sub.registerCallback()) {
		PX4_WARN("esc_status callback registration failed, FTC INDI fast loop disabled");
	}

	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_WARN("angular velocity callback registration failed, FTC INDI gyro-rate loop disabled");
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
		_esc_status_sub.unregisterCallback();
		_vehicle_angular_velocity_sub.unregisterCallback();
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
				_ftc_indi_control_active = false;
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
				_ftc_indi_filter_initialized = false;
				_ftc_indi_latched_active = false;
				_ftc_dual_indi_active = false;
				_ftc_indi_force_error_int = 0.f;
				_control_setpoint_valid = false;
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
	update_ftc_motor_speed_feedback(now);

	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {
		_angular_rates = matrix::Vector3f{angular_velocity.xyz};
		_angular_accel = matrix::Vector3f{angular_velocity.xyz_derivative};
	}

	vehicle_acceleration_s vehicle_acceleration;

	if (_vehicle_acceleration_sub.update(&vehicle_acceleration)) {
		_vehicle_acceleration = matrix::Vector3f{vehicle_acceleration.xyz};
	}

	vehicle_attitude_s vehicle_attitude;

	if (_vehicle_attitude_sub.update(&vehicle_attitude)) {
		_vehicle_attitude_q = matrix::Quatf{vehicle_attitude.q};
		_vehicle_attitude_valid = _vehicle_attitude_q.isAllFinite();
	}

	vehicle_attitude_setpoint_s vehicle_attitude_setpoint;

	if (_vehicle_attitude_setpoint_sub.update(&vehicle_attitude_setpoint)) {
		_vehicle_attitude_setpoint_q = matrix::Quatf{vehicle_attitude_setpoint.q_d};
		_vehicle_attitude_setpoint_valid = _vehicle_attitude_setpoint_q.isAllFinite();
	}

	vehicle_local_position_s vehicle_local_position;

	if (_vehicle_local_position_sub.update(&vehicle_local_position)) {
		_local_position = matrix::Vector3f{vehicle_local_position.x, vehicle_local_position.y, vehicle_local_position.z};
		_local_velocity = matrix::Vector3f{vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz};
		_local_acceleration = matrix::Vector3f{vehicle_local_position.ax, vehicle_local_position.ay, vehicle_local_position.az};
		_local_position_valid = PX4_ISFINITE(vehicle_local_position.z) && PX4_ISFINITE(vehicle_local_position.vz);
	}

	vehicle_local_position_setpoint_s vehicle_local_position_setpoint;

		if (_vehicle_local_position_setpoint_sub.update(&vehicle_local_position_setpoint)) {
			_local_position_sp = matrix::Vector3f{vehicle_local_position_setpoint.x, vehicle_local_position_setpoint.y,
							      vehicle_local_position_setpoint.z};
			_local_velocity_sp = matrix::Vector3f{vehicle_local_position_setpoint.vx, vehicle_local_position_setpoint.vy,
							      vehicle_local_position_setpoint.vz};
			_local_acceleration_sp = matrix::Vector3f{vehicle_local_position_setpoint.acceleration};
			_local_position_sp_valid = PX4_ISFINITE(vehicle_local_position_setpoint.z);
		}

		vehicle_ftc_physical_setpoint_s vehicle_ftc_physical_setpoint;

		if (_vehicle_ftc_physical_setpoint_sub.update(&vehicle_ftc_physical_setpoint)) {
			_ftc_physical_fz_des_body = vehicle_ftc_physical_setpoint.fz_des_body;
			_ftc_physical_setpoint_timestamp = vehicle_ftc_physical_setpoint.timestamp;
		}

		vehicle_rates_setpoint_s vehicle_rates_setpoint_for_indi;

	const bool rates_sp_updated = _vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint_for_indi);

	if (rates_sp_updated) {
		const hrt_abstime rates_sp_timestamp = vehicle_rates_setpoint_for_indi.timestamp;
		const matrix::Vector3f rates_sp{
			vehicle_rates_setpoint_for_indi.roll,
			vehicle_rates_setpoint_for_indi.pitch,
			PX4_ISFINITE(vehicle_rates_setpoint_for_indi.yaw) ? vehicle_rates_setpoint_for_indi.yaw : _angular_rates(2)
		};

		if (_last_rates_sp_timestamp != 0 && rates_sp_timestamp > _last_rates_sp_timestamp) {
			const float rates_sp_dt = math::constrain((rates_sp_timestamp - _last_rates_sp_timestamp) * 1e-6f, 0.0002f, 0.02f);
			_rates_sp_dot = (rates_sp - _rates_sp) / rates_sp_dt;

		} else {
			_rates_sp_dot.zero();
		}

		_rates_sp = rates_sp;
		_last_rates_sp_timestamp = rates_sp_timestamp;
	} else {
		_rates_sp_dot.zero();
	}

	if (!_ftc_indi_control_active || !_param_ca_ftc_indi_en.get()) {
		_ftc_indi_filter_initialized = false;
		_ftc_indi_latched_active = false;
		_ftc_dual_indi_active = false;
		_ftc_dual_zdot_prev = _local_velocity(2);
		_ftc_dual_y2_dot_f = 0.f;
		_ftc_dual_y2_dot_prev = 0.f;
		_ftc_indi_force_error_int = 0.f;
	}

	bool do_update = false;
	vehicle_torque_setpoint_s vehicle_torque_setpoint;
	vehicle_thrust_setpoint_s vehicle_thrust_setpoint;
	const bool ftc_fast_update = _ftc_indi_control_active && _param_ca_ftc_indi_en.get() && _control_setpoint_valid;

	// Run allocator on torque changes
	if (_vehicle_torque_setpoint_sub.update(&vehicle_torque_setpoint)) {
		_torque_sp = matrix::Vector3f(vehicle_torque_setpoint.xyz);

		do_update = true;
		_control_setpoint_valid = true;
		_timestamp_sample = vehicle_torque_setpoint.timestamp_sample;

	}

	if (_vehicle_thrust_setpoint_sub.update(&vehicle_thrust_setpoint)) {
		_thrust_sp = matrix::Vector3f(vehicle_thrust_setpoint.xyz);
	}

	do_update = do_update || ftc_fast_update;

	if (!do_update) {
		perf_end(_loop_perf);
		return;
	}

	{
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
				if (_ftc_indi_control_active) {
					apply_active_ftc_allocation(i, c[i], now, dt);
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
	debug.indi_control_active = _ftc_indi_control_active;
	debug.degraded_allocation_active = false;
	debug.output_fault_active = _ftc_output_fault_active;
	debug.fault_motor_index = _ftc_fault_motor_idx >= 0 ? _ftc_fault_motor_idx + 1 : 0;
	const bool dual_mode_selected = _ftc_indi_control_active && _param_ca_ftc_dual_en.get();
	debug.ftc_mode = dual_mode_selected ? 2 : (_ftc_indi_control_active ? 1 : (_ftc_mode == FtcMode::FAULT_NOMINAL ? 3 : 0));
	debug.fault_motor_mask = get_ftc_fault_motor_mask();
	debug.remaining_motor_mask = get_ftc_remaining_motor_mask();
	debug.handled_motor_failure_mask = _handled_motor_failure_bitmask;
	debug.motor_stop_mask = _motor_stop_mask;
	debug.loe = _ftc_current_loe;
	debug.nominal_fault_command = _ftc_fault_nominal_command;
	debug.applied_fault_command = _ftc_fault_applied_command;
	debug.fault_command_limit = _ftc_fault_command_limit;
	debug.residual_yaw_moment = _ftc_residual_yaw_moment;
	const float dual_sl = (_param_ca_ftc_pair.get() == 0) ? -1.f : 1.f;
	debug.chi = dual_mode_selected ? dual_sl * math::radians(math::constrain(_param_ca_ftc_chi.get(), 1.f, 179.f)) :
		    _ftc_dual_debug_chi;
	debug.sl = dual_mode_selected ? dual_sl : _ftc_dual_debug_sl;
	debug.sn = _ftc_dual_debug_sn;
	debug.indi_success = _ftc_indi_success;
	debug.indi_fail_reason = _ftc_indi_fail_reason;
	debug.indi_fault_column = _ftc_indi_fault_column;
	debug.indi_healthy_count = _ftc_indi_healthy_count;
	debug.indi_matrix_invertible = _ftc_indi_matrix_invertible;
	debug.indi_collective_thrust = _ftc_indi_collective_thrust;
	debug.indi_effectiveness_det = _ftc_indi_effectiveness_det;
	debug.indi_cond_proxy = _ftc_indi_cond_proxy;
	debug.indi_scaled_det = _ftc_indi_scaled_det;
	debug.indi_fz_des = _ftc_indi_fz_des;
	debug.indi_fz_error_int = _ftc_indi_force_error_int;

	for (int i = 0; i < 4; ++i) {
		debug.indi_eff_mx[i] = _ftc_indi_eff_mx[i];
		debug.indi_eff_my[i] = _ftc_indi_eff_my[i];
		debug.indi_eff_fz[i] = _ftc_indi_eff_fz[i];
		debug.indi_ratio_mx_abs_fz[i] = _ftc_indi_ratio_mx_abs_fz[i];
		debug.indi_ratio_my_abs_fz[i] = _ftc_indi_ratio_my_abs_fz[i];
		debug.indi_omega2_f[i] = _ftc_indi_u_f[i];
		debug.indi_omega2_cmd[i] = _ftc_indi_omega2_cmd[i];
	}

	for (int i = 0; i < 9; ++i) {
		debug.indi_g_raw[i] = _ftc_indi_g_raw[i];
		debug.indi_g_scaled[i] = _ftc_indi_g_scaled[i];
	}

	for (int i = 0; i < 3; ++i) {
		debug.indi_row_scale[i] = _ftc_indi_row_scale[i];
		debug.indi_nu_in[i] = _ftc_indi_nu_in(i);
		debug.indi_y_dot_f[i] = _ftc_indi_y_dot_f(i);
		debug.indi_error[i] = _ftc_indi_error(i);
		debug.indi_delta_omega2[i] = _ftc_indi_delta_omega2(i);
	}

	for (int i = 0; i < 2; ++i) {
		debug.y[i] = _ftc_dual_debug_y(i);
		debug.nu[i] = _ftc_dual_debug_nu(i);
		debug.indi_u[i] = _ftc_dual_debug_u(i);
	}

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
		_ftc_indi_control_active = false;
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
		const char *mode_str = (mode == FtcMode::FAULT_INDI) ? "fault_indi" : "fault_nominal";
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
				    mode == FtcMode::FAULT_INDI ? "INDI" : "nominal",
				    fault_motor, fault_type_str, loe);

		if (mode == FtcMode::FAULT_INDI) {
			events::send(events::ID("control_allocator_ftc_indi"),
				     events::Log::Warning, "FTC INDI");

		} else {
			events::send(events::ID("control_allocator_ftc_nominal"),
				     events::Log::Warning, "FTC nominal");
		}
	};

	switch (trigger_mode) {
	case FtcTriggerMode::PARAM: {
		switch (_param_ca_ftc_state.get()) {
		case 1:
			_ftc_mode = FtcMode::FAULT_INDI;
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
			_ftc_indi_control_active = false;
			_ftc_output_fault_active = false;
			_reaction_wheel_active = false;
			_reaction_wheel_latched_active = false;
			return;
		}

		const float aux_value = get_selected_ftc_aux_value();

		if (PX4_ISFINITE(aux_value)) {
			if (aux_value < -0.5f) {
				_ftc_mode = FtcMode::FAULT_INDI;
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
			_ftc_indi_control_active = false;
			_ftc_output_fault_active = false;
			_reaction_wheel_active = false;
			_reaction_wheel_latched_active = false;
			return;
		}

		const bool degraded_pressed = is_ftc_button_pressed(_param_ca_ftc_btn_deg.get());
		const bool nominal_pressed = is_ftc_button_pressed(_param_ca_ftc_btn_nom.get());

		if (degraded_pressed != nominal_pressed) {
			if (degraded_pressed) {
				_ftc_mode = FtcMode::FAULT_INDI;
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
	_ftc_indi_control_active = _ftc_mode == FtcMode::FAULT_INDI;
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

void
ControlAllocator::update_ftc_motor_speed_feedback(hrt_abstime now)
{
	esc_status_s esc_status{};

	if (!_esc_status_sub.update(&esc_status)) {
		return;
	}

	const float omega_max = math::max(_param_ca_ftc_omax.get(), 1.f);
	const float esc_rpm_max = math::max(_param_ca_ftc_erpmax.get(), 1.f);
	const uint8_t esc_count = math::min(esc_status.esc_count, esc_status_s::CONNECTED_ESC_MAX);

	for (uint8_t esc_idx = 0; esc_idx < esc_count; esc_idx++) {
		const esc_report_s &esc = esc_status.esc[esc_idx];
		int motor_idx = esc_idx;

		if (math::isInRange(esc.actuator_function, esc_report_s::ACTUATOR_FUNCTION_MOTOR1,
				     esc_report_s::ACTUATOR_FUNCTION_MOTOR12)) {
			motor_idx = esc.actuator_function - esc_report_s::ACTUATOR_FUNCTION_MOTOR1;
		}

		if (motor_idx < 0 || motor_idx >= NUM_ACTUATORS) {
			continue;
		}

		const float speed_fraction = math::constrain(fabsf(static_cast<float>(esc.esc_rpm)) / esc_rpm_max, 0.f, 1.f);
		const float omega = speed_fraction * omega_max;
		_ftc_motor_omega2_feedback[motor_idx] = omega * omega;
		_ftc_motor_feedback_timestamp[motor_idx] = esc.timestamp != 0 ? esc.timestamp : now;
	}
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

bool
ControlAllocator::apply_active_ftc_indi_allocation(int matrix_index, const matrix::Vector<float, NUM_AXES> &control_sp,
		hrt_abstime now, float dt)
{
	_ftc_fault_nominal_command = 0.f;
	_ftc_fault_applied_command = 0.f;
	_ftc_fault_command_limit = 1.f;
	_ftc_residual_yaw_moment = 0.f;
	_reaction_wheel_torque_command = 0.f;
	_reaction_wheel_active = false;
	_reaction_wheel_latched_active = false;
	_ftc_indi_success = false;
	_ftc_indi_fail_reason = control_allocator_ftc_debug_s::INDI_FAIL_NONE;
	_ftc_indi_fault_column = -1;
	_ftc_indi_healthy_count = 0;
	_ftc_indi_matrix_invertible = false;
	_ftc_indi_collective_thrust = 0.f;
	_ftc_indi_effectiveness_det = 0.f;
	_ftc_indi_cond_proxy = 0.f;
	_ftc_indi_scaled_det = 0.f;
	_ftc_indi_nu_in.zero();
	_ftc_indi_error.zero();
	_ftc_indi_fz_des = 0.f;
	_ftc_indi_delta_omega2.zero();

	for (int i = 0; i < 4; ++i) {
		_ftc_indi_eff_mx[i] = 0.f;
		_ftc_indi_eff_my[i] = 0.f;
		_ftc_indi_eff_fz[i] = 0.f;
		_ftc_indi_ratio_mx_abs_fz[i] = 0.f;
		_ftc_indi_ratio_my_abs_fz[i] = 0.f;
		_ftc_indi_omega2_cmd[i] = 0.f;
	}

	for (int i = 0; i < 9; ++i) {
		_ftc_indi_g_raw[i] = 0.f;
		_ftc_indi_g_scaled[i] = 0.f;
	}

	for (int i = 0; i < 3; ++i) {
		_ftc_indi_row_scale[i] = 0.f;
	}

	const auto fail_single_indi = [&](uint8_t reason) {
		_ftc_indi_fail_reason = reason;
		_ftc_indi_success = false;
		return false;
	};

	if (!_ftc_fault_trigger_active || _ftc_fault_motor_idx < 0 || matrix_index != 0) {
		return fail_single_indi(control_allocator_ftc_debug_s::INDI_FAIL_NOT_TRIGGERED);
	}

	const int motor_count = _num_actuators[(int)ActuatorType::MOTORS];

	if (motor_count < 4) {
		return fail_single_indi(control_allocator_ftc_debug_s::INDI_FAIL_MOTOR_COUNT);
	}

	int fault_column = -1;

	if (!get_motor_column_index(_ftc_fault_motor_idx, matrix_index, fault_column)) {
		return fail_single_indi(control_allocator_ftc_debug_s::INDI_FAIL_FAULT_COLUMN);
	}

	_ftc_indi_fault_column = fault_column;

	const ActuatorEffectiveness::EffectivenessMatrix &effectiveness =
		_control_allocation[matrix_index]->getEffectivenessMatrix();
	const ActuatorVector &actuator_min = _control_allocation[matrix_index]->getActuatorMin();
	const ActuatorVector &actuator_max = _control_allocation[matrix_index]->getActuatorMax();
	ActuatorVector &actuator_sp = _control_allocation[matrix_index]->_actuator_sp;

	for (int motor_idx = 0; motor_idx < motor_count && motor_idx < 4; motor_idx++) {
		int motor_matrix_column = -1;

		if (get_motor_column_index(motor_idx, matrix_index, motor_matrix_column)) {
			const float thrust_z_effectiveness = effectiveness(5, motor_matrix_column);
			const float thrust_effectiveness_abs = fabsf(thrust_z_effectiveness);
			_ftc_indi_eff_mx[motor_idx] = effectiveness(0, motor_matrix_column);
			_ftc_indi_eff_my[motor_idx] = effectiveness(1, motor_matrix_column);
			_ftc_indi_eff_fz[motor_idx] = thrust_z_effectiveness;

			if (thrust_effectiveness_abs > FLT_EPSILON) {
				_ftc_indi_ratio_mx_abs_fz[motor_idx] = effectiveness(0, motor_matrix_column) / thrust_effectiveness_abs;
				_ftc_indi_ratio_my_abs_fz[motor_idx] = effectiveness(1, motor_matrix_column) / thrust_effectiveness_abs;
			}
		}
	}

	const float nominal_fault_command = actuator_sp(fault_column);
	const float fault_command_limit = actuator_max(fault_column) * _ftc_current_loe;
	const float applied_fault_command = math::constrain(fminf(nominal_fault_command, fault_command_limit),
					     actuator_min(fault_column), actuator_max(fault_column));

	_ftc_fault_nominal_command = nominal_fault_command;
	_ftc_fault_applied_command = applied_fault_command;
	_ftc_fault_command_limit = fault_command_limit;

	int healthy_columns[3] {};
	int healthy_count = 0;

	for (int motor_idx = 0; motor_idx < motor_count && motor_idx < actuator_motors_s::NUM_CONTROLS; motor_idx++) {
		int motor_matrix_column = -1;

		if (!get_motor_column_index(motor_idx, matrix_index, motor_matrix_column)) {
			continue;
		}

		if (motor_idx == _ftc_fault_motor_idx) {
			continue;
		}

		if (healthy_count < 3) {
			healthy_columns[healthy_count++] = motor_matrix_column;
		}
	}

	if (healthy_count != 3) {
		_ftc_indi_healthy_count = healthy_count;
		return fail_single_indi(control_allocator_ftc_debug_s::INDI_FAIL_HEALTHY_COUNT);
	}

	_ftc_indi_healthy_count = healthy_count;

	float thrust_effectiveness_sum_abs = 0.f;

	for (int motor_idx = 0; motor_idx < motor_count && motor_idx < actuator_motors_s::NUM_CONTROLS; motor_idx++) {
		int motor_matrix_column = -1;

		if (get_motor_column_index(motor_idx, matrix_index, motor_matrix_column)) {
			thrust_effectiveness_sum_abs += fabsf(effectiveness(5, motor_matrix_column));
		}
	}

	if (thrust_effectiveness_sum_abs < FLT_EPSILON) {
		return fail_single_indi(control_allocator_ftc_debug_s::INDI_FAIL_THRUST_EFFECTIVENESS);
	}

	const float hover_thrust = math::constrain(_param_mpc_thr_hover.get(), 0.05f, 0.9f);
	const float thrust_control_to_specific_force = 9.80665f / hover_thrust;
	const float rotor_force_coefficient = _param_ca_ftc_kf.get();
	const float omega_max = math::max(_param_ca_ftc_omax.get(), 1.f);
	const float max_omega2 = omega_max * omega_max;
	const float ixx = math::max(_param_ca_ftc_indi_ix.get(), 0.001f);
	const float iyy = math::max(_param_ca_ftc_indi_iy.get(), 0.001f);
	const float mass = math::max(_param_ca_ftc_indi_m.get(), 0.1f);
	const float collective_thrust_sp = math::max(-control_sp(5), 0.f);
	_ftc_indi_collective_thrust = collective_thrust_sp;

	if (rotor_force_coefficient <= FLT_EPSILON) {
		return fail_single_indi(control_allocator_ftc_debug_s::INDI_FAIL_ROTOR_COEFFICIENT);
	}

	if (collective_thrust_sp < 0.10f) {
		actuator_sp(fault_column) = applied_fault_command;
		_ftc_indi_filter_initialized = false;
		_ftc_indi_latched_active = false;
		_ftc_indi_force_error_int = 0.f;
		_ftc_indi_fail_reason = control_allocator_ftc_debug_s::INDI_FAIL_LOW_THRUST;
		return true;
	}

	const auto indi_effectiveness_column = [&](int motor_matrix_column) {
		Vector3f column{};
		const float thrust_z_effectiveness = effectiveness(5, motor_matrix_column);
		const float thrust_effectiveness_abs = fabsf(thrust_z_effectiveness);

		if (thrust_effectiveness_abs > FLT_EPSILON) {
			column(0) = effectiveness(0, motor_matrix_column) / thrust_effectiveness_abs * rotor_force_coefficient / ixx;
			column(1) = effectiveness(1, motor_matrix_column) / thrust_effectiveness_abs * rotor_force_coefficient / iyy;
			column(2) = (thrust_z_effectiveness > 0.f ? 1.f : -1.f) * rotor_force_coefficient / mass;
		}

		return column;
	};

	const bool was_indi_active = _ftc_indi_latched_active;
	_ftc_indi_latched_active = true;

	Vector3f y_dot_raw{_angular_accel(0), _angular_accel(1), _vehicle_acceleration(2)};

	if (!y_dot_raw.isAllFinite()) {
		y_dot_raw = Vector3f{0.f, 0.f, 0.f};
	}

	const float cutoff = math::max(_param_ca_ftc_indi_fc.get(), 1.f);
	const float rc = 1.f / (2.f * M_PI_F * cutoff);
	const float alpha = math::constrain(dt / (dt + rc), 0.f, 1.f);
	const auto motor_omega2_feedback = [&](int motor_column, float actuator_command) {
		const bool feedback_recent = _ftc_motor_feedback_timestamp[motor_column] != 0
					     && now - _ftc_motor_feedback_timestamp[motor_column] < 100_ms;

		if (feedback_recent && PX4_ISFINITE(_ftc_motor_omega2_feedback[motor_column])) {
			return math::constrain(_ftc_motor_omega2_feedback[motor_column], 0.f, max_omega2);
		}

		return math::sq(math::max(actuator_command, 0.f) * omega_max);
	};

	if (!_ftc_indi_filter_initialized) {
		_ftc_indi_y_dot_f = y_dot_raw;

		for (int i = 0; i < NUM_ACTUATORS; i++) {
			const float command = (i == fault_column) ? applied_fault_command : (PX4_ISFINITE(actuator_sp(i)) ? actuator_sp(i) : 0.f);
			const float omega2_command = motor_omega2_feedback(i, command);
			_ftc_indi_u_f[i] = omega2_command;
			_ftc_indi_input_raw[i] = omega2_command;
		}

		_ftc_indi_force_error_int = 0.f;
		_ftc_indi_filter_initialized = true;

	} else {
		_ftc_indi_y_dot_f += (y_dot_raw - _ftc_indi_y_dot_f) * alpha;

		for (int i = 0; i < NUM_ACTUATORS; i++) {
			_ftc_indi_u_f[i] += (_ftc_indi_input_raw[i] - _ftc_indi_u_f[i]) * alpha;
		}
	}

		// Prefer the physical body-z specific-force command computed by the position controller.
		// PX4's public thrust topics stay normalized; this side channel keeps the INDI allocator in
		// the paper's physical output space [p_dot, q_dot, f_z] without relying on hover-thrust scaling.
		const bool physical_fz_des_recent = _ftc_physical_setpoint_timestamp != 0
						    && now - _ftc_physical_setpoint_timestamp < 200_ms
						    && PX4_ISFINITE(_ftc_physical_fz_des_body);
		const float fz_des = physical_fz_des_recent ? _ftc_physical_fz_des_body :
				     control_sp(5) * thrust_control_to_specific_force;
	_ftc_indi_fz_des = fz_des;
	const float force_error = fz_des - _ftc_indi_y_dot_f(2);
	const float force_integrator_limit = math::constrain(_param_ca_ftc_indi_ilim.get(), 0.f, 20.f);
	_ftc_indi_force_error_int = math::constrain(_ftc_indi_force_error_int + force_error * dt,
				    -force_integrator_limit, force_integrator_limit);

	const float rates_sp_dot_limit = math::max(_param_ca_ftc_indi_sdl.get(), 0.f);
	Vector3f rates_sp_dot_limited{_rates_sp_dot(0), _rates_sp_dot(1), 0.f};

	if (rates_sp_dot_limit > FLT_EPSILON) {
		rates_sp_dot_limited(0) = math::constrain(rates_sp_dot_limited(0), -rates_sp_dot_limit, rates_sp_dot_limit);
		rates_sp_dot_limited(1) = math::constrain(rates_sp_dot_limited(1), -rates_sp_dot_limit, rates_sp_dot_limit);
	}

	Vector3f nu_in{
		rates_sp_dot_limited(0) + _param_ca_ftc_indi_k1.get() * (_rates_sp(0) - _angular_rates(0)),
		rates_sp_dot_limited(1) + _param_ca_ftc_indi_k2.get() * (_rates_sp(1) - _angular_rates(1)),
		fz_des + _param_ca_ftc_indi_k3.get() * _ftc_indi_force_error_int
	};
	_ftc_indi_nu_in = nu_in;

	SquareMatrix<float, 3> control_effectiveness;

	for (int motor = 0; motor < 3; motor++) {
		const Vector3f motor_effectiveness = indi_effectiveness_column(healthy_columns[motor]);

		for (int axis = 0; axis < 3; axis++) {
			control_effectiveness(axis, motor) = motor_effectiveness(axis);
			_ftc_indi_g_raw[axis * 3 + motor] = motor_effectiveness(axis);
		}
	}

	_ftc_indi_effectiveness_det =
		control_effectiveness(0, 0) * (control_effectiveness(1, 1) * control_effectiveness(2, 2) -
					       control_effectiveness(1, 2) * control_effectiveness(2, 1))
		- control_effectiveness(0, 1) * (control_effectiveness(1, 0) * control_effectiveness(2, 2) -
						 control_effectiveness(1, 2) * control_effectiveness(2, 0))
		+ control_effectiveness(0, 2) * (control_effectiveness(1, 0) * control_effectiveness(2, 1) -
						 control_effectiveness(1, 1) * control_effectiveness(2, 0));

	SquareMatrix<float, 3> scaled_control_effectiveness;

	for (int axis = 0; axis < 3; axis++) {
		float row_max_abs = 0.f;

		for (int motor = 0; motor < 3; motor++) {
			row_max_abs = math::max(row_max_abs, fabsf(control_effectiveness(axis, motor)));
		}

		if (row_max_abs < FLT_EPSILON) {
			_ftc_indi_latched_active = false;
			return fail_single_indi(control_allocator_ftc_debug_s::INDI_FAIL_SINGULAR_EFFECTIVENESS);
		}

		_ftc_indi_row_scale[axis] = 1.f / row_max_abs;

		for (int motor = 0; motor < 3; motor++) {
			scaled_control_effectiveness(axis, motor) = control_effectiveness(axis, motor) * _ftc_indi_row_scale[axis];
			_ftc_indi_g_scaled[axis * 3 + motor] = scaled_control_effectiveness(axis, motor);
		}
	}

	_ftc_indi_scaled_det =
		scaled_control_effectiveness(0, 0) * (scaled_control_effectiveness(1, 1) * scaled_control_effectiveness(2, 2) -
				scaled_control_effectiveness(1, 2) * scaled_control_effectiveness(2, 1))
		- scaled_control_effectiveness(0, 1) * (scaled_control_effectiveness(1, 0) * scaled_control_effectiveness(2, 2) -
				scaled_control_effectiveness(1, 2) * scaled_control_effectiveness(2, 0))
		+ scaled_control_effectiveness(0, 2) * (scaled_control_effectiveness(1, 0) * scaled_control_effectiveness(2, 1) -
				scaled_control_effectiveness(1, 1) * scaled_control_effectiveness(2, 0));

	Vector3f scaled_row_norms{};

	for (int axis = 0; axis < 3; axis++) {
		for (int motor = 0; motor < 3; motor++) {
			scaled_row_norms(axis) += scaled_control_effectiveness(axis, motor) * scaled_control_effectiveness(axis, motor);
		}

		scaled_row_norms(axis) = sqrtf(scaled_row_norms(axis));
	}

	const float min_scaled_row_norm = math::min(scaled_row_norms(0), math::min(scaled_row_norms(1), scaled_row_norms(2)));
	const float max_scaled_row_norm = math::max(scaled_row_norms(0), math::max(scaled_row_norms(1), scaled_row_norms(2)));
	_ftc_indi_cond_proxy = min_scaled_row_norm > FLT_EPSILON ? max_scaled_row_norm / min_scaled_row_norm : INFINITY;

	SquareMatrix<float, 3> scaled_control_effectiveness_inv;

	if (!matrix::inv(scaled_control_effectiveness, scaled_control_effectiveness_inv)) {
		_ftc_indi_latched_active = false;
		return fail_single_indi(control_allocator_ftc_debug_s::INDI_FAIL_SINGULAR_EFFECTIVENESS);
	}

	_ftc_indi_matrix_invertible = true;

	const Vector3f control_error = nu_in - _ftc_indi_y_dot_f;
	_ftc_indi_error = control_error;
	// Numerical row scaling only changes the units used by the solver:
	//   S * Ghat * delta_omega2 = S * (nu_in - y_dot_f)
	// It preserves the physical solution while avoiding matrix::inv()'s absolute determinant threshold.
	const Vector3f scaled_control_error{
		control_error(0) * _ftc_indi_row_scale[0],
		control_error(1) * _ftc_indi_row_scale[1],
		control_error(2) * _ftc_indi_row_scale[2]
	};
	const Vector3f unconstrained_delta_u = scaled_control_effectiveness_inv * scaled_control_error;
	const float delta_u_limit = math::max(_param_ca_ftc_indi_dul.get(), 0.f) * max_omega2 * math::max(dt, 0.0002f);
	Vector3f healthy_delta_u = unconstrained_delta_u;

	Vector3f delta_min{};
	Vector3f delta_max{};

	for (int col = 0; col < 3; col++) {
		const int motor_column = healthy_columns[col];
		const float min_omega2_command = math::sq(math::max(actuator_min(motor_column), 0.f) * omega_max);
		const float max_omega2_command = math::sq(math::max(actuator_max(motor_column), 0.f) * omega_max);
		delta_min(col) = min_omega2_command - _ftc_indi_u_f[motor_column];
		delta_max(col) = max_omega2_command - _ftc_indi_u_f[motor_column];

		if (delta_u_limit > FLT_EPSILON) {
			delta_min(col) = math::max(delta_min(col), -delta_u_limit);
			delta_max(col) = math::min(delta_max(col), delta_u_limit);
		}
	}

	const bool unconstrained_feasible = unconstrained_delta_u(0) >= delta_min(0) && unconstrained_delta_u(0) <= delta_max(0)
					     && unconstrained_delta_u(1) >= delta_min(1) && unconstrained_delta_u(1) <= delta_max(1)
					     && unconstrained_delta_u(2) >= delta_min(2) && unconstrained_delta_u(2) <= delta_max(2);

	if (!unconstrained_feasible) {
		float best_cost = FLT_MAX;
		Vector3f best_delta{};

		for (int active_set = 0; active_set < 27; active_set++) {
			int state[3] {};
			int state_code = active_set;
			Vector3f candidate{};
			Vector3f residual = scaled_control_error;
			int free_index[3] {};
			int free_count = 0;

			for (int col = 0; col < 3; col++) {
				state[col] = state_code % 3;
				state_code /= 3;

				if (state[col] == 0) {
					free_index[free_count++] = col;

				} else {
					candidate(col) = (state[col] == 1) ? delta_min(col) : delta_max(col);

					for (int axis = 0; axis < 3; axis++) {
						residual(axis) -= scaled_control_effectiveness(axis, col) * candidate(col);
					}
				}
			}

			bool valid = true;

			switch (free_count) {
			case 3:
				candidate = scaled_control_effectiveness_inv * scaled_control_error;
				break;

			case 2: {
					const int c0 = free_index[0];
					const int c1 = free_index[1];
					float a00 = 0.f;
					float a01 = 0.f;
					float a11 = 0.f;
					float b0 = 0.f;
					float b1 = 0.f;

					for (int axis = 0; axis < 3; axis++) {
						const float g0 = scaled_control_effectiveness(axis, c0);
						const float g1 = scaled_control_effectiveness(axis, c1);
						a00 += g0 * g0;
						a01 += g0 * g1;
						a11 += g1 * g1;
						b0 += g0 * residual(axis);
						b1 += g1 * residual(axis);
					}

					const float det = a00 * a11 - a01 * a01;

					if (fabsf(det) > FLT_EPSILON) {
						candidate(c0) = (b0 * a11 - b1 * a01) / det;
						candidate(c1) = (a00 * b1 - a01 * b0) / det;

					} else {
						valid = false;
					}
				}

				break;

			case 1: {
					const int c0 = free_index[0];
					float a00 = 0.f;
					float b0 = 0.f;

					for (int axis = 0; axis < 3; axis++) {
						const float g0 = scaled_control_effectiveness(axis, c0);
						a00 += g0 * g0;
						b0 += g0 * residual(axis);
					}

					if (a00 > FLT_EPSILON) {
						candidate(c0) = b0 / a00;

					} else {
						valid = false;
					}
				}

				break;

			case 0:
				break;

			default:
				valid = false;
				break;
			}

			for (int col = 0; col < 3; col++) {
				if (candidate(col) < delta_min(col) - 1e-5f || candidate(col) > delta_max(col) + 1e-5f) {
					valid = false;
					break;
				}
			}

			if (!valid) {
				continue;
			}

			Vector3f achieved{};

			for (int col = 0; col < 3; col++) {
				for (int axis = 0; axis < 3; axis++) {
					achieved(axis) += scaled_control_effectiveness(axis, col) * candidate(col);
				}
			}

			const Vector3f solve_error = achieved - scaled_control_error;
			const float cost = solve_error.norm_squared();

			if (cost < best_cost) {
				best_cost = cost;
				best_delta = candidate;
			}
		}

		if (best_cost < FLT_MAX) {
			healthy_delta_u = best_delta;
		}
	}

	_ftc_indi_delta_omega2 = healthy_delta_u;

	for (int col = 0; col < 3; col++) {
		const int motor_column = healthy_columns[col];
		const float delta_u = math::constrain(healthy_delta_u(col), delta_min(col), delta_max(col));

		const float omega2_command = _ftc_indi_u_f[motor_column] + delta_u;
		_ftc_indi_omega2_cmd[motor_column] = omega2_command;
		const float actuator_command = sqrtf(math::max(omega2_command, 0.f)) / omega_max;
		actuator_sp(motor_column) = math::constrain(actuator_command, actuator_min(motor_column), actuator_max(motor_column));
	}

	actuator_sp(fault_column) = applied_fault_command;
	_ftc_indi_omega2_cmd[fault_column] = motor_omega2_feedback(fault_column, applied_fault_command);

	for (int i = 0; i < NUM_ACTUATORS; i++) {
		const float command = PX4_ISFINITE(actuator_sp(i)) ? actuator_sp(i) : 0.f;
		_ftc_indi_input_raw[i] = motor_omega2_feedback(i, command);
	}

	float allocated_yaw_moment = 0.f;

	for (int i = 0; i < NUM_ACTUATORS; i++) {
		allocated_yaw_moment += effectiveness(2, i) * actuator_sp(i);
	}

	_ftc_residual_yaw_moment = control_sp(2) - allocated_yaw_moment;
	_ftc_indi_success = true;
	_ftc_indi_fail_reason = control_allocator_ftc_debug_s::INDI_FAIL_NONE;

	if (!was_indi_active) {
		PX4_WARN("FTC INDI active: motor=%d nominal=%.3f applied=%.3f limit=%.3f yaw_res=%.3f t=%.3fs",
			 _ftc_fault_motor_idx + 1, (double)_ftc_fault_nominal_command, (double)_ftc_fault_applied_command,
			 (double)_ftc_fault_command_limit, (double)_ftc_residual_yaw_moment, (double)(now / 1e6));
		mavlink_log_warning(&_mavlink_log_pub, "FTC INDI active m%d lim=%.2f\t",
				    _ftc_fault_motor_idx + 1, (double)_ftc_fault_command_limit);
	}

	return true;
}

void
ControlAllocator::apply_active_ftc_allocation(int matrix_index, const matrix::Vector<float, NUM_AXES> &control_sp,
		hrt_abstime now, float dt)
{
	if (_param_ca_ftc_dual_en.get()) {
		if (apply_active_ftc_dual_indi_allocation(matrix_index, control_sp, now, dt)) {
			return;
		}

		_ftc_fault_nominal_command = 0.f;
		_ftc_fault_applied_command = 0.f;
		_ftc_fault_command_limit = 0.f;
		_ftc_residual_yaw_moment = 0.f;
		_reaction_wheel_torque_command = 0.f;
		_reaction_wheel_active = false;
		_reaction_wheel_latched_active = false;
		_ftc_indi_latched_active = false;
		return;
	}

	if (_param_ca_ftc_indi_en.get() && apply_active_ftc_indi_allocation(matrix_index, control_sp, now, dt)) {
		return;
	}

	_ftc_fault_nominal_command = 0.f;
	_ftc_fault_applied_command = 0.f;
	_ftc_fault_command_limit = 1.f;
	_ftc_residual_yaw_moment = 0.f;
	_reaction_wheel_torque_command = 0.f;
	_reaction_wheel_active = false;
	_reaction_wheel_latched_active = false;
	_ftc_indi_latched_active = false;
	_ftc_dual_indi_active = false;
}

bool
ControlAllocator::get_dual_pair_motor_indices(int failed_motors[2], int remaining_motors[2]) const
{
	const int pair = math::constrain(_param_ca_ftc_pair.get(), int32_t{0}, int32_t{1});

	if (pair == 0) {
		failed_motors[0] = 0;
		failed_motors[1] = 1;
		remaining_motors[0] = 2;
		remaining_motors[1] = 3;

	} else {
		failed_motors[0] = 2;
		failed_motors[1] = 3;
		remaining_motors[0] = 0;
		remaining_motors[1] = 1;
	}

	return true;
}

uint16_t
ControlAllocator::get_ftc_fault_motor_mask() const
{
	if (!_ftc_fault_trigger_active) {
		return 0;
	}

	if (_param_ca_ftc_dual_en.get()) {
		int failed_motors[2] {};
		int remaining_motors[2] {};
		get_dual_pair_motor_indices(failed_motors, remaining_motors);
		return (1u << failed_motors[0]) | (1u << failed_motors[1]);
	}

	return _ftc_fault_motor_idx >= 0 ? (1u << _ftc_fault_motor_idx) : 0;
}

uint16_t
ControlAllocator::get_ftc_remaining_motor_mask() const
{
	if (!_ftc_fault_trigger_active || !_param_ca_ftc_dual_en.get()) {
		return 0;
	}

	int failed_motors[2] {};
	int remaining_motors[2] {};
	get_dual_pair_motor_indices(failed_motors, remaining_motors);
	return (1u << remaining_motors[0]) | (1u << remaining_motors[1]);
}

bool
ControlAllocator::apply_active_ftc_dual_indi_allocation(int matrix_index,
		const matrix::Vector<float, NUM_AXES> &control_sp, hrt_abstime now, float dt)
{
	const bool was_dual_indi_active = _ftc_dual_indi_active;
	_ftc_dual_indi_active = false;
	_reaction_wheel_torque_command = 0.f;
	_reaction_wheel_active = false;
	_reaction_wheel_latched_active = false;
	_ftc_dual_debug_y.zero();
	_ftc_dual_debug_nu.zero();
	_ftc_dual_debug_u.zero();
	_ftc_dual_debug_chi = 0.f;
	_ftc_dual_debug_sl = 0.f;
	_ftc_dual_debug_sn = 0.f;

	if (!_ftc_fault_trigger_active || matrix_index != 0 || _num_actuators[(int)ActuatorType::MOTORS] != 4
	    || !_vehicle_attitude_valid || !_vehicle_attitude_setpoint_valid || !_local_position_valid || !_local_position_sp_valid) {
		return false;
	}

	int failed_motors[2] {};
	int remaining_motors[2] {};
	get_dual_pair_motor_indices(failed_motors, remaining_motors);

	int failed_columns[2] {-1, -1};
	int remaining_columns[2] {-1, -1};

	for (int i = 0; i < 2; ++i) {
		if (!get_motor_column_index(failed_motors[i], matrix_index, failed_columns[i])
		    || !get_motor_column_index(remaining_motors[i], matrix_index, remaining_columns[i])) {
			return false;
		}
	}

	const ActuatorEffectiveness::EffectivenessMatrix &effectiveness =
		_control_allocation[matrix_index]->getEffectivenessMatrix();
	const ActuatorVector &actuator_min = _control_allocation[matrix_index]->getActuatorMin();
	const ActuatorVector &actuator_max = _control_allocation[matrix_index]->getActuatorMax();
	ActuatorVector &actuator_sp = _control_allocation[matrix_index]->_actuator_sp;

	_ftc_fault_nominal_command = 0.5f * (actuator_sp(failed_columns[0]) + actuator_sp(failed_columns[1]));
	_ftc_fault_command_limit = 0.f;
	_ftc_fault_applied_command = 0.f;

	for (int i = 0; i < 2; ++i) {
		actuator_sp(failed_columns[i]) = math::max(actuator_min(failed_columns[i]), 0.f);
	}

	const Dcmf R{_vehicle_attitude_q};
	const Dcmf R_des{_vehicle_attitude_setpoint_q};
	Vector3f n_des_inertial = R_des.col(2);
	n_des_inertial *= -1.f;

	const Vector3f h = R.transpose() * n_des_inertial;
	const float h1 = h(0);
	const float h2 = h(1);
	const float h3 = math::constrain(h(2), -1.f, -0.05f);
	const float sl = (_param_ca_ftc_pair.get() == 0) ? -1.f : 1.f;
	const float chi = sl * math::radians(math::constrain(_param_ca_ftc_chi.get(), 1.f, 179.f));
	const float c = cosf(chi);
	const float s = sinf(chi);
	const float y2 = h1 * c + h2 * s;
	const float y2_dot = c * (-h3 * _angular_rates(1) + h2 * _angular_rates(2))
			     + s * (h3 * _angular_rates(0) - h1 * _angular_rates(2));

	const float z_ref = PX4_ISFINITE(_local_position_sp(2)) ? _local_position_sp(2) : _local_position(2);
	const float vz_ref = PX4_ISFINITE(_local_velocity_sp(2)) ? _local_velocity_sp(2) : 0.f;
	const float zdd_ref = PX4_ISFINITE(_local_acceleration_sp(2)) ? _local_acceleration_sp(2) : 0.f;

	Vector2f nu{
		-_param_ca_ftc_z_p.get() * (_local_position(2) - z_ref) - _param_ca_ftc_z_d.get() * (_local_velocity(2) - vz_ref) + zdd_ref,
		-_param_ca_ftc_y2_p.get() * y2 - _param_ca_ftc_y2_d.get() * y2_dot
	};

	const float cutoff = math::max(_param_ca_ftc_indi_fc.get(), 1.f);
	const float rc = 1.f / (2.f * M_PI_F * cutoff);
	const float alpha = math::constrain(dt / (dt + rc), 0.f, 1.f);

	const float z_ddot_raw = PX4_ISFINITE(_local_acceleration(2)) ? _local_acceleration(2) :
				 ((was_dual_indi_active && PX4_ISFINITE(_ftc_dual_zdot_prev))
				  ? (_local_velocity(2) - _ftc_dual_zdot_prev) / math::max(dt, 0.0002f) : 0.f);
	const float y2_dot_f_raw = was_dual_indi_active ? _ftc_dual_y2_dot_f + (y2_dot - _ftc_dual_y2_dot_f) * alpha : y2_dot;
	const float y2_ddot_raw = was_dual_indi_active ? (y2_dot_f_raw - _ftc_dual_y2_dot_prev) / math::max(dt, 0.0002f) : 0.f;

	Vector2f y_ddot_raw{z_ddot_raw, y2_ddot_raw};

	if (!_ftc_indi_filter_initialized || !was_dual_indi_active) {
		_ftc_dual_y_ddot_f = y_ddot_raw;
		_ftc_dual_zdot_prev = _local_velocity(2);
		_ftc_dual_y2_dot_f = y2_dot;
		_ftc_dual_y2_dot_prev = y2_dot;

		for (int i = 0; i < 2; ++i) {
			_ftc_dual_u_f(i) = math::constrain(PX4_ISFINITE(actuator_sp(remaining_columns[i])) ? actuator_sp(remaining_columns[i]) : 0.f,
							   actuator_min(remaining_columns[i]), actuator_max(remaining_columns[i]));
		}

		_ftc_indi_filter_initialized = true;

	} else {
		_ftc_dual_y_ddot_f += (y_ddot_raw - _ftc_dual_y_ddot_f) * alpha;
		_ftc_dual_zdot_prev = _local_velocity(2);
		_ftc_dual_y2_dot_f = y2_dot_f_raw;
		_ftc_dual_y2_dot_prev = y2_dot_f_raw;
	}

	const float hover_thrust = math::constrain(_param_mpc_thr_hover.get(), 0.05f, 0.9f);
	const float thrust_per_input = _param_ca_ftc_indi_m.get() * 9.80665f / (4.f * hover_thrust);
	const float ixx = math::max(_param_ca_ftc_indi_ix.get(), 0.001f);
	const float iyy = math::max(_param_ca_ftc_indi_iy.get(), 0.001f);

	matrix::Matrix<float, 2, 2> B;

	for (int i = 0; i < 2; ++i) {
		const int col = remaining_columns[i];
		const float thrust_abs = math::max(fabsf(effectiveness(5, col)), FLT_EPSILON);
		const float roll_accel_per_input = effectiveness(0, col) / thrust_abs * thrust_per_input / ixx;
		const float pitch_accel_per_input = effectiveness(1, col) / thrust_abs * thrust_per_input / iyy;
		B(0, i) = -R(2, 2) * 9.80665f / (4.f * hover_thrust);
			B(1, i) = h3 * (s * roll_accel_per_input - c * pitch_accel_per_input);
	}

	const float det = B(0, 0) * B(1, 1) - B(0, 1) * B(1, 0);

	if (fabsf(det) < 1e-5f || !PX4_ISFINITE(det)) {
		return false;
	}

	const Vector2f control_error = nu - _ftc_dual_y_ddot_f;
	Vector2f delta_u{
		(B(1, 1) * control_error(0) - B(0, 1) * control_error(1)) / det,
		(-B(1, 0) * control_error(0) + B(0, 0) * control_error(1)) / det
	};

	const float delta_u_limit = math::max(_param_ca_ftc_indi_dul.get(), 0.f) * math::max(dt, 0.0002f);

	for (int i = 0; i < 2; ++i) {
		const int col = remaining_columns[i];
		float delta_min = actuator_min(col) - _ftc_dual_u_f(i);
		float delta_max = actuator_max(col) - _ftc_dual_u_f(i);

		if (delta_u_limit > FLT_EPSILON) {
			delta_min = math::max(delta_min, -delta_u_limit);
			delta_max = math::min(delta_max, delta_u_limit);
		}

		actuator_sp(col) = math::constrain(_ftc_dual_u_f(i) + math::constrain(delta_u(i), delta_min, delta_max),
						   actuator_min(col), actuator_max(col));
		_ftc_dual_debug_u(i) = actuator_sp(col);
		_ftc_dual_u_f(i) = actuator_sp(col);
	}

	float allocated_yaw_moment = 0.f;

	for (int i = 0; i < NUM_ACTUATORS; i++) {
		allocated_yaw_moment += effectiveness(2, i) * actuator_sp(i);
	}

	float sn_accum = 0.f;

	for (int i = 0; i < 2; ++i) {
		sn_accum += effectiveness(2, remaining_columns[i]);
	}

	_ftc_residual_yaw_moment = control_sp(2) - allocated_yaw_moment;
	_ftc_dual_debug_chi = chi;
	_ftc_dual_debug_sl = sl;
	_ftc_dual_debug_sn = (sn_accum >= 0.f) ? 1.f : -1.f;
	_ftc_dual_debug_y = _ftc_dual_y_ddot_f;
	_ftc_dual_debug_nu = nu;
	_ftc_dual_indi_active = true;
	_ftc_indi_latched_active = true;

	PX4_DEBUG("FTC dual INDI m%d/m%d off, m%d/m%d active",
		  failed_motors[0] + 1, failed_motors[1] + 1, remaining_motors[0] + 1, remaining_motors[1] + 1);

	return true;
}

float
ControlAllocator::get_ftc_fault_output_limit() const
{
	if (_param_ca_ftc_dual_en.get() && _ftc_fault_trigger_active) {
		return 0.f;
	}

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
	if (!_ftc_output_fault_active) {
		return;
	}

	const uint16_t fault_mask = get_ftc_fault_motor_mask();

	if (fault_mask == 0) {
		return;
	}

	const float fault_limit = get_ftc_fault_output_limit();

	if (!PX4_ISFINITE(fault_limit)) {
		return;
	}

	for (int motor_idx = 0; motor_idx < MAX_NUM_MOTORS; ++motor_idx) {
		if ((fault_mask & (1u << motor_idx)) == 0u) {
			continue;
		}

		float &fault_control = controls[motor_idx];

		if (PX4_ISFINITE(fault_control)) {
			fault_control = fminf(fault_control, fault_limit);
		}
	}
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
	case FtcMode::FAULT_INDI:
		mode_str = "fault_indi";
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

	PX4_INFO("FTC alloc: indi=%s output_fault=%s nominal=%.3f applied=%.3f limit=%.3f yaw_res=%.3f wheel=%.3f wheel_active=%s",
		 _ftc_indi_control_active ? "yes" : "no",
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
