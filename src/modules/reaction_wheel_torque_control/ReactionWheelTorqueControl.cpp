/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "ReactionWheelTorqueControl.hpp"

#include <px4_platform_common/defines.h>

using namespace time_literals;

ModuleBase::Descriptor ReactionWheelTorqueControl::desc{task_spawn, custom_command, print_usage};

ReactionWheelTorqueControl::ReactionWheelTorqueControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	updateParams();
}

int ReactionWheelTorqueControl::task_spawn(int argc, char *argv[])
{
	ReactionWheelTorqueControl *instance = new ReactionWheelTorqueControl();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	if (!instance->init()) {
		delete instance;
		desc.object.store(nullptr);
		desc.task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

bool ReactionWheelTorqueControl::init()
{
	updateParams();

	if (!_reaction_wheel_setpoint_sub.registerCallback()) {
		PX4_ERR("setpoint callback registration failed");
		return false;
	}

	ScheduleOnInterval(20_ms);
	return true;
}

int ReactionWheelTorqueControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Reaction wheel torque controller for torque-mode SITL.
Combines allocator residual feedforward with yaw-rate damping feedback and
publishes a normalized wheel torque command.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("reaction_wheel_torque_control", "module");
	return 0;
}

void ReactionWheelTorqueControl::updateParamsIfNeeded()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate{};
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}
}

float ReactionWheelTorqueControl::applyTorqueSlewLimit(float torque_cmd_nm, float dt) const
{
	const float slew_limit = math::max(_param_rw_torque_slew.get(), 0.f);

	if (slew_limit <= FLT_EPSILON) {
		return torque_cmd_nm;
	}

	const float max_delta = slew_limit * dt;
	return math::constrain(torque_cmd_nm, _last_torque_command_nm - max_delta, _last_torque_command_nm + max_delta);
}

void ReactionWheelTorqueControl::publishOutputs(hrt_abstime now, float torque_residual, float torque_ff_nm,
		float torque_fb_nm, float torque_cmd_nm, float yaw_rate, float rpm_measured,
		float control_output, bool active, bool feedback_valid, bool saturated)
{
	reaction_wheel_actuator_setpoint_s actuator_setpoint{};
	actuator_setpoint.timestamp = now;
	actuator_setpoint.control = active ? control_output : 0.f;
	actuator_setpoint.active = active;
	_reaction_wheel_actuator_setpoint_pub.publish(actuator_setpoint);

	reaction_wheel_status_s status{};
	status.timestamp = now;
	status.allocator_torque_residual = torque_residual;
	status.torque_command_nm = torque_cmd_nm;
	status.torque_ff_nm = torque_ff_nm;
	status.torque_fb_nm = torque_fb_nm;
	status.torque_cmd_nm = torque_cmd_nm;
	status.yaw_rate = yaw_rate;
	status.rpm_setpoint = 0.f;
	status.rpm_measured = rpm_measured;
	status.rpm_error = 0.f;
	status.control_output = control_output;
	status.active = active;
	status.feedback_valid = feedback_valid;
	status.saturated = saturated;
	status.sign_reversal_active = false;
	_reaction_wheel_status_pub.publish(status);
}

void ReactionWheelTorqueControl::Run()
{
	if (should_exit()) {
		_reaction_wheel_setpoint_sub.unregisterCallback();
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	updateParamsIfNeeded();

	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain((now - _last_run) * 1e-6f, 0.001f, 0.05f);
	_last_run = now;

	reaction_wheel_setpoint_s wheel_setpoint{};
	const bool has_setpoint = _reaction_wheel_setpoint_sub.copy(&wheel_setpoint);
	const bool test_mode = _param_ca_rw_test_en.get() != 0;
	const float test_command = math::constrain(_param_ca_rw_test_cmd.get(), -1.f, 1.f);

	vehicle_status_s vehicle_status{};
	_vehicle_status_sub.copy(&vehicle_status);
	const bool armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	const bool wheel_active = test_mode || (has_setpoint && wheel_setpoint.active);

	rpm_s rpm{};
	const bool feedback_valid = _rpm_sub.copy(&rpm) && PX4_ISFINITE(rpm.rpm_estimate) && rpm.timestamp != 0;
	const float rpm_measured = feedback_valid ? rpm.rpm_estimate : 0.f;

	if (!_param_rw_en.get() || !wheel_active || !armed) {
		_last_torque_command_nm = 0.f;
		publishOutputs(now, test_mode ? test_command : (has_setpoint ? wheel_setpoint.torque : 0.f),
			       0.f, 0.f, 0.f, 0.f, 0.f, 0.f, false, false, false);
		return;
	}

	const float tau_max = math::max(_param_rw_tau_max.get(), 1e-6f);
	const float torque_residual = has_setpoint ? wheel_setpoint.torque : 0.f;

	if (test_mode) {
		const float torque_cmd_nm = test_command * tau_max;
		const float control_output = math::constrain(torque_cmd_nm / tau_max, -1.f, 1.f);
		const bool saturated = fabsf(control_output) >= (1.f - FLT_EPSILON);
		_last_torque_command_nm = torque_cmd_nm;
		publishOutputs(now, test_command, torque_cmd_nm, 0.f, torque_cmd_nm, 0.f, rpm_measured, control_output,
			       true, true, saturated);
		return;
	}

	vehicle_angular_velocity_s angular_velocity{};
	const bool got_gyro = _vehicle_angular_velocity_sub.copy(&angular_velocity)
			      && PX4_ISFINITE(angular_velocity.xyz[2]);
	const hrt_abstime gyro_timestamp = got_gyro ?
					       (angular_velocity.timestamp_sample != 0 ? angular_velocity.timestamp_sample : angular_velocity.timestamp) :
					       0;
	const bool gyro_valid = got_gyro && (gyro_timestamp != 0)
				&& ((now - gyro_timestamp) <= static_cast<hrt_abstime>(_param_rw_gyro_timeout.get() * 1_s));

	if (!gyro_valid) {
		_last_torque_command_nm = 0.f;
		publishOutputs(now, torque_residual, 0.f, 0.f, 0.f, 0.f, rpm_measured, 0.f,
			       false, false, false);
		return;
	}

	const float yaw_rate_raw = angular_velocity.xyz[2];
	const float yaw_rate_deadband = math::max(_param_rw_yawrate_db.get(), 0.f);
	const float yaw_rate = (fabsf(yaw_rate_raw) > yaw_rate_deadband) ? yaw_rate_raw : 0.f;

	const float torque_ff_nm = _param_rw_ff_scale.get() * torque_residual;
	// Positive body yaw rate requires positive wheel acceleration command so the
	// wheel/body reaction pair generates an opposing body yaw torque.
	const float torque_fb_nm = _param_rw_yawrate_k.get() * yaw_rate;
	float torque_cmd_nm = math::constrain(torque_ff_nm + torque_fb_nm, -tau_max, tau_max);
	torque_cmd_nm = applyTorqueSlewLimit(torque_cmd_nm, dt);
	torque_cmd_nm = math::constrain(torque_cmd_nm, -tau_max, tau_max);

	const float control_output = math::constrain(torque_cmd_nm / tau_max, -1.f, 1.f);
	const bool saturated = fabsf(torque_ff_nm + torque_fb_nm) >= tau_max;
	_last_torque_command_nm = torque_cmd_nm;

	publishOutputs(now, torque_residual, torque_ff_nm, torque_fb_nm, torque_cmd_nm, yaw_rate_raw, rpm_measured,
		       control_output, true, true, saturated);
}

extern "C" __EXPORT int reaction_wheel_torque_control_main(int argc, char *argv[])
{
	return ModuleBase::main(ReactionWheelTorqueControl::desc, argc, argv);
}
