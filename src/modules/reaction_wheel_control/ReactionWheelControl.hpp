/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/reaction_wheel_actuator_setpoint.h>
#include <uORB/topics/reaction_wheel_setpoint.h>
#include <uORB/topics/reaction_wheel_status.h>
#include <uORB/topics/rpm.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_status.h>

class ReactionWheelControl : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	ReactionWheelControl();
	~ReactionWheelControl() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);

	bool init();
	void Run() override;

private:
	void resetControllerState();
	void updateParamsIfNeeded();
	bool updateFeedback(float &rpm_measured, hrt_abstime &feedback_timestamp);
	float updateReversalLimitedCommand(float desired_control, float dt, bool sign_reversal_active);
	void publishOutputs(hrt_abstime now, float torque_residual, float torque_ff_nm, float torque_rate_nm,
		float torque_momentum_nm, float torque_cmd_nm, float yaw_rate, float omega_w_radps,
		float sigma, float k_omega_eff, float rpm_setpoint, float rpm_measured, float rpm_error, float control_output,
		bool active, bool feedback_valid, bool saturated, bool sign_reversal_active);

	uORB::SubscriptionCallbackWorkItem _reaction_wheel_setpoint_sub{this, ORB_ID(reaction_wheel_setpoint)};
	uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};
	uORB::Subscription _rpm_sub{ORB_ID(rpm)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1000000};

	uORB::Publication<reaction_wheel_actuator_setpoint_s> _reaction_wheel_actuator_setpoint_pub{ORB_ID(reaction_wheel_actuator_setpoint)};
	uORB::Publication<reaction_wheel_status_s> _reaction_wheel_status_pub{ORB_ID(reaction_wheel_status)};

	float _rpm_setpoint{0.f};
	float _rpm_error_integral{0.f};
	float _last_control_output{0.f};
	hrt_abstime _last_feedback_timestamp{0};
	hrt_abstime _last_run{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::RW_EN>) _param_rw_en,
		(ParamInt<px4::params::RW_MOT_IDX>) _param_rw_mot_idx,
		(ParamInt<px4::params::CA_RW_TEST_EN>) _param_ca_rw_test_en,
		(ParamFloat<px4::params::CA_RW_TEST_CMD>) _param_ca_rw_test_cmd,
		(ParamFloat<px4::params::RW_YAW2TAU>) _param_rw_yaw2tau,
		(ParamFloat<px4::params::RW_TAU_MAX>) _param_rw_tau_max,
		(ParamFloat<px4::params::RW_J>) _param_rw_j,
		(ParamFloat<px4::params::RW_RPM_MAX>) _param_rw_rpm_max,
		(ParamFloat<px4::params::RW_RPM_KP>) _param_rw_rpm_kp,
		(ParamFloat<px4::params::RW_RPM_KI>) _param_rw_rpm_ki,
		(ParamFloat<px4::params::RW_RPM_FF>) _param_rw_rpm_ff,
		(ParamFloat<px4::params::RW_YAW_RATE_K>) _param_rw_yaw_rate_k,
		(ParamFloat<px4::params::RW_YAWR_SP>) _param_rw_yawr_sp,
		(ParamFloat<px4::params::RW_MOM_KSAT>) _param_rw_mom_ksat,
		(ParamFloat<px4::params::RW_MOM_EXP>) _param_rw_mom_exp,
		(ParamInt<px4::params::RW_LEAK_EN>) _param_rw_leak_en,
		(ParamFloat<px4::params::RW_LEAK_TC>) _param_rw_leak_tc,
		(ParamFloat<px4::params::RW_FB_TIMEOUT>) _param_rw_fb_timeout,
		(ParamFloat<px4::params::RW_ZERO_RPM_THR>) _param_rw_zero_rpm_thresh,
		(ParamFloat<px4::params::RW_REV_RATE_LIM>) _param_rw_rev_rate_lim
	)
};
