/****************************************************************************
 *
 *   Copyright (c) 2026 AMRL FTC Team. All rights reserved.
 *
 ****************************************************************************/

#include "ReactionWheelControl.hpp"

#include <lib/mathlib/math/Limits.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/time.h>

using namespace time_literals;

ModuleBase::Descriptor ReactionWheelControl::desc{task_spawn, custom_command, print_usage};

ReactionWheelControl::ReactionWheelControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

int ReactionWheelControl::task_spawn(int argc, char *argv[])
{
	ReactionWheelControl *instance = new ReactionWheelControl();

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

bool ReactionWheelControl::init()
{
	if (!_reaction_wheel_setpoint_sub.registerCallback()) {
		PX4_ERR("setpoint callback registration failed");
		return false;
	}

	ScheduleOnInterval(20_ms);
	return true;
}

int ReactionWheelControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Reaction wheel speed controller using reaction_wheel_setpoint and esc_status feedback.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("reaction_wheel_control", "module");
	return 0;
}

void ReactionWheelControl::updateParamsIfNeeded()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate{};
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}
}

void ReactionWheelControl::resetControllerState()
{
	_rpm_setpoint = 0.f;
	_rpm_error_integral = 0.f;
	_last_control_output = 0.f;
}

bool ReactionWheelControl::updateFeedback(float &rpm_measured, hrt_abstime &feedback_timestamp)
{
	rpm_s rpm{};

	if (_rpm_sub.copy(&rpm) && PX4_ISFINITE(rpm.rpm_estimate) && rpm.timestamp != 0) {
		rpm_measured = rpm.rpm_estimate;
		feedback_timestamp = rpm.timestamp;
		return true;
	}

	esc_status_s esc_status{};

	if (!_esc_status_sub.copy(&esc_status)) {
		return false;
	}

	const uint8_t actuator_function = actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1 + _param_rw_mot_idx.get() - 1;

	for (uint8_t i = 0; i < esc_status.esc_count && i < esc_status_s::CONNECTED_ESC_MAX; ++i) {
		if (esc_status.esc[i].actuator_function == actuator_function) {
			rpm_measured = static_cast<float>(esc_status.esc[i].esc_rpm);
			feedback_timestamp = esc_status.esc[i].timestamp;
			return PX4_ISFINITE(rpm_measured);
		}
	}

	return false;
}

float ReactionWheelControl::updateReversalLimitedCommand(float desired_control, float dt, bool sign_reversal_active)
{
	const float rate_limit = math::max(_param_rw_rev_rate_lim.get(), 0.01f);
	const float max_delta = rate_limit * dt;

	if (sign_reversal_active) {
		return math::constrain(desired_control, _last_control_output - max_delta, _last_control_output + max_delta);
	}

	return desired_control;
}

void ReactionWheelControl::publishOutputs(hrt_abstime now, float torque_residual, float torque_ff_nm,
		float torque_rate_nm, float torque_momentum_nm, float torque_cmd_nm, float yaw_rate, float omega_w_radps,
		float sigma, float k_omega_eff, float rpm_setpoint, float rpm_measured, float rpm_error, float control_output,
		bool active, bool feedback_valid, bool saturated, bool sign_reversal_active)
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
	status.torque_fb_nm = torque_rate_nm;
	status.torque_cmd_nm = torque_cmd_nm;
	status.yaw_rate = yaw_rate;
	status.omega_w_radps = omega_w_radps;
	status.sigma = sigma;
	status.k_omega_eff = k_omega_eff;
	status.torque_rate_nm = torque_rate_nm;
	status.torque_momentum_nm = torque_momentum_nm;
	status.rpm_setpoint = rpm_setpoint;
	status.rpm_measured = rpm_measured;
	status.rpm_error = rpm_error;
	status.control_output = control_output;
	status.active = active;
	status.feedback_valid = feedback_valid;
	status.saturated = saturated;
	status.sign_reversal_active = sign_reversal_active;
	_reaction_wheel_status_pub.publish(status);
}

void ReactionWheelControl::Run()
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
	// wheel_setpoint.active is a fault-session latch from the allocator.
	// A transient zero residual torque request during that session should not
	// reset the accumulated wheel rpm state.
	const bool wheel_active = test_mode || (has_setpoint && wheel_setpoint.active);

	if (!_param_rw_en.get() || !wheel_active || !armed) {
		resetControllerState();
		publishOutputs(now, test_mode ? test_command : (has_setpoint ? wheel_setpoint.torque : 0.f),
			       0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
			       0.f, 0.f, 0.f, 0.f, false, false, false, false);
		return;
	}

	const float torque_residual = test_mode ? test_command : wheel_setpoint.torque;
	const float wheel_inertia = math::max(_param_rw_j.get(), 1e-6f);
	const float rpm_max = math::max(_param_rw_rpm_max.get(), 1.f);

	float rpm_measured = 0.f;
	hrt_abstime feedback_timestamp = 0;
	const bool got_feedback = updateFeedback(rpm_measured, feedback_timestamp);
	const bool feedback_valid = got_feedback && (feedback_timestamp != 0)
				    && ((now - feedback_timestamp) <= static_cast<hrt_abstime>(_param_rw_fb_timeout.get() * 1_s));

	if (!feedback_valid) {
		resetControllerState();
		publishOutputs(now, torque_residual, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
			       0.f, got_feedback ? rpm_measured : 0.f, 0.f, 0.f,
			       true, false, false, false);
		return;
	}

	_last_feedback_timestamp = feedback_timestamp;
	const float omega_w = rpm_measured * (2.f * M_PI_F / 60.f);
	const float omega_w_max = rpm_max * (2.f * M_PI_F / 60.f);
	const float sigma = math::constrain(fabsf(omega_w) / omega_w_max, 0.f, 1.f);
	const float sigma_n = powf(sigma, math::max(_param_rw_mom_exp.get(), 0.1f));
	const float k_omega_eff = math::max(_param_rw_mom_ksat.get(), 0.f) * sigma_n;

	vehicle_angular_velocity_s angular_velocity{};
	const bool got_yaw_rate = _vehicle_angular_velocity_sub.copy(&angular_velocity)
				  && PX4_ISFINITE(angular_velocity.xyz[2]);
	const float yaw_rate = got_yaw_rate ? angular_velocity.xyz[2] : 0.f;

	const float tau_max = math::max(_param_rw_tau_max.get(), 0.f);
	const float torque_ff_nm = test_mode ? test_command * tau_max : _param_rw_yaw2tau.get() * torque_residual;
	const float torque_rate_nm = test_mode ? 0.f : -_param_rw_yaw_rate_k.get() * (yaw_rate - _param_rw_yawr_sp.get());
	const float torque_momentum_nm = test_mode ? 0.f : -k_omega_eff * omega_w;
	const float torque_command_nm = math::constrain(torque_ff_nm + torque_rate_nm + torque_momentum_nm, -tau_max, tau_max);

	const float alpha_sp = torque_command_nm / wheel_inertia;
	float rpm_sp_dot = alpha_sp * (60.f / (2.f * M_PI_F));

	if (_param_rw_leak_en.get() != 0) {
		rpm_sp_dot -= _rpm_setpoint / math::max(_param_rw_leak_tc.get(), 0.01f);
	}

	_rpm_setpoint += rpm_sp_dot * dt;
	_rpm_setpoint = math::constrain(_rpm_setpoint, -rpm_max, rpm_max);

	const float rpm_error = _rpm_setpoint - rpm_measured;

	const bool sign_reversal_active = (fabsf(_rpm_setpoint) > FLT_EPSILON)
					  && (fabsf(rpm_measured) > _param_rw_zero_rpm_thresh.get())
					  && (signbit(_rpm_setpoint) != signbit(rpm_measured));

	if (sign_reversal_active) {
		_rpm_error_integral = 0.f;
	}

	_rpm_error_integral += rpm_error * dt;
	const float integral_limit = math::max(1.f, rpm_max);
	_rpm_error_integral = math::constrain(_rpm_error_integral, -integral_limit, integral_limit);

	float control_output = _param_rw_rpm_ff.get() * (_rpm_setpoint / rpm_max)
			       + _param_rw_rpm_kp.get() * rpm_error
			       + _param_rw_rpm_ki.get() * _rpm_error_integral;

	control_output = updateReversalLimitedCommand(control_output, dt, sign_reversal_active);
	control_output = math::constrain(control_output, -1.f, 1.f);

	const bool saturated = fabsf(control_output) >= (1.f - FLT_EPSILON);
	if (saturated) {
		_rpm_error_integral -= rpm_error * dt;
	}

	_last_control_output = control_output;

	publishOutputs(now, torque_residual, torque_ff_nm, torque_rate_nm, torque_momentum_nm, torque_command_nm,
		       yaw_rate, omega_w, sigma, k_omega_eff, _rpm_setpoint, rpm_measured, rpm_error, control_output,
		       true, true, saturated, sign_reversal_active);
}

extern "C" __EXPORT int reaction_wheel_control_main(int argc, char *argv[])
{
	return ModuleBase::main(ReactionWheelControl::desc, argc, argv);
}
