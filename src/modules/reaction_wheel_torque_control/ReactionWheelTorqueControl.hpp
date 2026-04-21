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
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/reaction_wheel_actuator_setpoint.h>
#include <uORB/topics/reaction_wheel_setpoint.h>
#include <uORB/topics/reaction_wheel_status.h>
#include <uORB/topics/rpm.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_status.h>

class ReactionWheelTorqueControl : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	ReactionWheelTorqueControl();
	~ReactionWheelTorqueControl() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);

	bool init();
	void Run() override;

private:
	void updateParamsIfNeeded();
	void publishOutputs(hrt_abstime now, float torque_residual, float torque_ff_nm, float torque_fb_nm,
		float torque_cmd_nm, float yaw_rate, float rpm_measured, float control_output,
		bool active, bool feedback_valid, bool saturated);
	float applyTorqueSlewLimit(float torque_cmd_nm, float dt) const;

	uORB::SubscriptionCallbackWorkItem _reaction_wheel_setpoint_sub{this, ORB_ID(reaction_wheel_setpoint)};
	uORB::Subscription _rpm_sub{ORB_ID(rpm)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1000000};

	uORB::Publication<reaction_wheel_actuator_setpoint_s> _reaction_wheel_actuator_setpoint_pub{ORB_ID(reaction_wheel_actuator_setpoint)};
	uORB::Publication<reaction_wheel_status_s> _reaction_wheel_status_pub{ORB_ID(reaction_wheel_status)};

	float _last_torque_command_nm{0.f};
	hrt_abstime _last_run{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::RW_EN>) _param_rw_en,
		(ParamInt<px4::params::CA_RW_TEST_EN>) _param_ca_rw_test_en,
		(ParamFloat<px4::params::CA_RW_TEST_CMD>) _param_ca_rw_test_cmd,
		(ParamFloat<px4::params::RW_TAU_MAX>) _param_rw_tau_max,
		(ParamFloat<px4::params::RW_YAWRATE_K>) _param_rw_yawrate_k,
		(ParamFloat<px4::params::RW_YAWRATE_DB>) _param_rw_yawrate_db,
		(ParamFloat<px4::params::RW_FF_SCALE>) _param_rw_ff_scale,
		(ParamFloat<px4::params::RW_TQ_SLEW>) _param_rw_torque_slew,
		(ParamFloat<px4::params::RW_GYR_TO>) _param_rw_gyro_timeout
	)
};
