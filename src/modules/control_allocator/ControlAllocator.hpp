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
 * @file ControlAllocator.hpp
 *
 * Control allocator.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include <ActuatorEffectiveness.hpp>
#include <ActuatorEffectivenessMultirotor.hpp>
#include <ActuatorEffectivenessStandardVTOL.hpp>
#include <ActuatorEffectivenessTiltrotorVTOL.hpp>
#include <ActuatorEffectivenessTailsitterVTOL.hpp>
#include <ActuatorEffectivenessRoverAckermann.hpp>
#include <ActuatorEffectivenessFixedWing.hpp>
#include <ActuatorEffectivenessMCTilt.hpp>
#include <ActuatorEffectivenessCustom.hpp>
#include <ActuatorEffectivenessUUV.hpp>
#include <ActuatorEffectivenessHelicopter.hpp>
#include <ActuatorEffectivenessHelicopterCoaxial.hpp>
#include <ActuatorEffectivenessSpacecraft.hpp>

#include <ControlAllocation.hpp>
#include <ControlAllocationPseudoInverse.hpp>
#include <ControlAllocationSequentialDesaturation.hpp>

#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/actuator_servos_trim.h>
#include <uORB/topics/control_allocator_ftc_debug.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/reaction_wheel_actuator_setpoint.h>
#include <uORB/topics/reaction_wheel_setpoint.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_ftc_physical_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/failure_detector_status.h>
#include <uORB/topics/manual_control_setpoint.h>

class ControlAllocator : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	static constexpr int NUM_ACTUATORS = ControlAllocation::NUM_ACTUATORS;
	static constexpr int NUM_AXES = ControlAllocation::NUM_AXES;

	static constexpr int MAX_NUM_MOTORS = actuator_motors_s::NUM_CONTROLS;
	static constexpr int MAX_NUM_SERVOS = actuator_servos_s::NUM_CONTROLS;

	static constexpr float ICE_SHEDDING_ON_SEC = 2.0f;
	static constexpr float ICE_SHEDDING_OUTPUT = 0.01f;


	using ActuatorVector = ActuatorEffectiveness::ActuatorVector;
	using ActuatorBitmask = ActuatorEffectiveness::ActuatorBitmask;

	ControlAllocator();

	virtual ~ControlAllocator();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void Run() override;

	bool init();

private:

	struct ParamHandles {
		param_t slew_rate_motors[MAX_NUM_MOTORS];
		param_t slew_rate_servos[MAX_NUM_SERVOS];
	};

	struct Params {
		float slew_rate_motors[MAX_NUM_MOTORS];
		float slew_rate_servos[MAX_NUM_SERVOS];
	};

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void parameters_updated();

	void update_allocation_method(bool force);
	bool update_effectiveness_source();

	void update_effectiveness_matrix_if_needed(EffectivenessUpdateReason reason);

	void check_for_motor_failures();

	void publish_control_allocator_status(int matrix_index);
	void publish_control_allocator_ftc_debug(const hrt_abstime now, const float pre_ftc[MAX_NUM_MOTORS],
			const float post_ftc[MAX_NUM_MOTORS], const float final[MAX_NUM_MOTORS]);

	void publish_actuator_controls();
	void fill_motor_controls_from_allocation(float controls[MAX_NUM_MOTORS]) const;
	void fill_motor_outputs_for_publish(float controls[MAX_NUM_MOTORS]) const;
	void fill_motor_saturation_from_allocation(int8_t saturation[MAX_NUM_MOTORS]) const;
	void overlay_reaction_wheel_control(float controls[MAX_NUM_MOTORS]);

	void handle_stopped_motors(const hrt_abstime now);

	float get_ice_shedding_output(hrt_abstime now);

	void update_ftc_state(hrt_abstime now);
	void update_ftc_motor_speed_feedback(hrt_abstime now);
	void update_reaction_wheel_setpoint(float torque_command, float residual_yaw_moment, bool active, hrt_abstime now);
	bool get_motor_column_index(int motor_idx, int matrix_index, int &matrix_column) const;
	void apply_active_ftc_allocation(int matrix_index, const matrix::Vector<float, NUM_AXES> &control_sp, hrt_abstime now,
					 float dt);
	bool apply_active_ftc_indi_allocation(int matrix_index, const matrix::Vector<float, NUM_AXES> &control_sp, hrt_abstime now,
					      float dt);
	bool apply_active_ftc_dual_indi_allocation(int matrix_index, const matrix::Vector<float, NUM_AXES> &control_sp,
			hrt_abstime now, float dt);
	void apply_ftc_output_fault(float controls[MAX_NUM_MOTORS]) const;
	uint16_t get_ftc_fault_motor_mask() const;
	uint16_t get_ftc_remaining_motor_mask() const;
	bool get_dual_pair_motor_indices(int failed_motors[2], int remaining_motors[2]) const;
	float get_ftc_fault_output_limit() const;
	float get_selected_ftc_aux_value() const;
	bool is_ftc_button_pressed(int button_number) const;

	AllocationMethod _allocation_method_id{AllocationMethod::NONE};
	ControlAllocation *_control_allocation[ActuatorEffectiveness::MAX_NUM_MATRICES] {}; 	///< class for control allocation calculations
	int _num_control_allocation{0};
	hrt_abstime _last_effectiveness_update{0};

	enum class EffectivenessSource {
		NONE = -1,
		MULTIROTOR = 0,
		FIXED_WING = 1,
		STANDARD_VTOL = 2,
		TILTROTOR_VTOL = 3,
		TAILSITTER_VTOL = 4,
		ROVER_ACKERMANN = 5,
		ROVER_DIFFERENTIAL = 6,
		MOTORS_6DOF = 7,
		MULTIROTOR_WITH_TILT = 8,
		CUSTOM = 9,
		HELICOPTER_TAIL_ESC = 10,
		HELICOPTER_TAIL_SERVO = 11,
		HELICOPTER_COAXIAL = 12,
		SPACECRAFT_2D = 13,
		SPACECRAFT_3D = 14,
	};

	enum class FailureMode {
		IGNORE = 0,
		REMOVE_FIRST_FAILING_MOTOR = 1,
	};

	enum class FtcMode {
		NORMAL = 0,
		FAULT_INDI = 1,
		FAULT_NOMINAL = 2,
	};

	enum class FtcTriggerMode {
		AUX = 0,
		PARAM = 1,
		BUTTONS = 2,
	};

	EffectivenessSource _effectiveness_source_id{EffectivenessSource::NONE};
	ActuatorEffectiveness *_actuator_effectiveness{nullptr}; 	///< class providing actuator effectiveness

	uint8_t _control_allocation_selection_indexes[NUM_ACTUATORS * ActuatorEffectiveness::MAX_NUM_MATRICES] {};
	int _num_actuators[(int)ActuatorType::COUNT] {};

	// Inputs
	uORB::SubscriptionCallbackWorkItem _vehicle_torque_setpoint_sub{this, ORB_ID(vehicle_torque_setpoint)};  /**< vehicle torque setpoint subscription */
	uORB::Subscription _vehicle_thrust_setpoint_sub{ORB_ID(vehicle_thrust_setpoint)};	 /**< vehicle thrust setpoint subscription */

	uORB::Subscription _vehicle_torque_setpoint1_sub{ORB_ID(vehicle_torque_setpoint), 1};  /**< vehicle torque setpoint subscription (2. instance) */
	uORB::Subscription _vehicle_thrust_setpoint1_sub{ORB_ID(vehicle_thrust_setpoint), 1};	 /**< vehicle thrust setpoint subscription (2. instance) */

	// Outputs
	uORB::Publication<control_allocator_ftc_debug_s> _control_allocator_ftc_debug_pub{ORB_ID(control_allocator_ftc_debug)};
	uORB::PublicationMulti<control_allocator_status_s> _control_allocator_status_pub[2] {ORB_ID(control_allocator_status), ORB_ID(control_allocator_status)};

	uORB::Publication<actuator_motors_s>	_actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_servos_s>	_actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<actuator_servos_trim_s>	_actuator_servos_trim_pub{ORB_ID(actuator_servos_trim)};
	uORB::Publication<reaction_wheel_setpoint_s> _reaction_wheel_setpoint_pub{ORB_ID(reaction_wheel_setpoint)};
	uORB::Subscription _reaction_wheel_actuator_setpoint_sub{ORB_ID(reaction_wheel_actuator_setpoint)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_ftc_physical_setpoint_sub{ORB_ID(vehicle_ftc_physical_setpoint)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_local_position_setpoint_sub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::SubscriptionCallbackWorkItem _esc_status_sub{this, ORB_ID(esc_status)};
	uORB::Subscription _failure_detector_status_sub{ORB_ID(failure_detector_status)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	matrix::Vector3f _torque_sp;
	matrix::Vector3f _thrust_sp;
	matrix::Vector3f _rates_sp;
	matrix::Vector3f _rates_sp_dot;
	matrix::Vector3f _angular_rates;
	matrix::Vector3f _angular_accel;
	matrix::Vector3f _vehicle_acceleration;
	matrix::Quatf _vehicle_attitude_q;
	matrix::Quatf _vehicle_attitude_setpoint_q;
	matrix::Vector3f _local_position;
	matrix::Vector3f _local_velocity;
	matrix::Vector3f _local_acceleration;
	matrix::Vector3f _local_position_sp;
	matrix::Vector3f _local_velocity_sp;
	matrix::Vector3f _local_acceleration_sp;
	float _ftc_physical_fz_des_body{NAN};
	hrt_abstime _ftc_physical_setpoint_timestamp{0};
	bool _publish_controls{true};
	bool _control_setpoint_valid{false};
	bool _vehicle_attitude_valid{false};
	bool _vehicle_attitude_setpoint_valid{false};
	bool _local_position_valid{false};
	bool _local_position_sp_valid{false};
	manual_control_setpoint_s _manual_control_setpoint{};

	// Reflects motor failures that are currently handled, not motor failures that are reported.
	// For example, the system might report two motor failures, but only the first one is handled by CA
	uint16_t _handled_motor_failure_bitmask{0};
	uint16_t _motor_stop_mask{0};

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */
	orb_advert_t _mavlink_log_pub{nullptr};

	bool _armed{false};
	bool _is_vtol{false};
	bool _ftc_fault_trigger_active{false};
	bool _ftc_indi_control_active{false};
	bool _ftc_output_fault_active{false};
	FtcMode _ftc_mode{FtcMode::NORMAL};
	int _ftc_fault_type{0};
	int _ftc_fault_motor_idx{-1};
	hrt_abstime _ftc_fault_timestamp{0};
	float _ftc_current_loe{1.f};
	float _ftc_fault_nominal_command{0.f};
	float _ftc_fault_applied_command{0.f};
	float _ftc_fault_command_limit{1.f};
	float _ftc_residual_yaw_moment{0.f};
	float _reaction_wheel_torque_command{0.f};
	bool _reaction_wheel_active{false};
	bool _reaction_wheel_latched_active{false};
	bool _reaction_wheel_reversible_warned{false};
	bool _ftc_indi_filter_initialized{false};
	bool _ftc_indi_latched_active{false};
	bool _ftc_indi_success{false};
	uint8_t _ftc_indi_fail_reason{control_allocator_ftc_debug_s::INDI_FAIL_NONE};
	int8_t _ftc_indi_fault_column{-1};
	uint8_t _ftc_indi_healthy_count{0};
	bool _ftc_indi_matrix_invertible{false};
	float _ftc_indi_collective_thrust{0.f};
	float _ftc_indi_effectiveness_det{0.f};
	float _ftc_indi_eff_mx[4] {};
	float _ftc_indi_eff_my[4] {};
	float _ftc_indi_eff_fz[4] {};
	float _ftc_indi_ratio_mx_abs_fz[4] {};
	float _ftc_indi_ratio_my_abs_fz[4] {};
	float _ftc_indi_g_raw[9] {};
	float _ftc_indi_g_scaled[9] {};
	float _ftc_indi_row_scale[3] {};
	float _ftc_indi_cond_proxy{0.f};
	float _ftc_indi_scaled_det{0.f};
	matrix::Vector3f _ftc_indi_nu_in{};
	matrix::Vector3f _ftc_indi_error{};
	float _ftc_indi_fz_des{0.f};
	matrix::Vector3f _ftc_indi_delta_omega2{};
	float _ftc_indi_omega2_cmd[NUM_ACTUATORS] {};
	bool _ftc_dual_indi_active{false};
	matrix::Vector3f _ftc_indi_y_dot_f{};
	matrix::Vector2f _ftc_dual_y_ddot_f{};
	matrix::Vector2f _ftc_dual_u_f{};
	float _ftc_dual_zdot_prev{0.f};
	float _ftc_dual_y2_dot_f{0.f};
	float _ftc_dual_y2_dot_prev{0.f};
	matrix::Vector2f _ftc_dual_debug_y{};
	matrix::Vector2f _ftc_dual_debug_nu{};
	matrix::Vector2f _ftc_dual_debug_u{};
	float _ftc_dual_debug_chi{0.f};
	float _ftc_dual_debug_sl{0.f};
	float _ftc_dual_debug_sn{0.f};
	float _ftc_indi_u_f[NUM_ACTUATORS] {};
	float _ftc_indi_input_raw[NUM_ACTUATORS] {};
	float _ftc_motor_omega2_feedback[NUM_ACTUATORS] {};
	hrt_abstime _ftc_motor_feedback_timestamp[NUM_ACTUATORS] {};
	float _ftc_indi_force_error_int{0.f};
	hrt_abstime _last_run{0};
	hrt_abstime _last_rates_sp_timestamp{0};
	hrt_abstime _timestamp_sample{0};
	hrt_abstime _last_status_pub{0};

	ParamHandles _param_handles{};
	Params _params{};
	bool _has_slew_rate{false};


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CA_AIRFRAME>) _param_ca_airframe,
		(ParamInt<px4::params::CA_METHOD>) _param_ca_method,
		(ParamInt<px4::params::CA_FAILURE_MODE>) _param_ca_failure_mode,
		(ParamInt<px4::params::CA_R_REV>) _param_r_rev,
		(ParamFloat<px4::params::CA_ICE_PERIOD>) _param_ice_shedding_period,
		(ParamInt<px4::params::CA_FTC_EN>) _param_ca_ftc_en,
		(ParamInt<px4::params::CA_FTC_MOT>) _param_ca_ftc_mot,
		(ParamInt<px4::params::CA_FTC_TYPE>) _param_ca_ftc_type,
		(ParamFloat<px4::params::CA_FTC_LOE>) _param_ca_ftc_loe,
		(ParamInt<px4::params::CA_FTC_TRIG_MODE>) _param_ca_ftc_trig_mode,
		(ParamInt<px4::params::CA_FTC_STATE>) _param_ca_ftc_state,
		(ParamInt<px4::params::CA_FTC_TRIG_SRC>) _param_ca_ftc_trig_src,
		(ParamInt<px4::params::CA_FTC_BTN_DEG>) _param_ca_ftc_btn_deg,
		(ParamInt<px4::params::CA_FTC_BTN_NOM>) _param_ca_ftc_btn_nom,
		(ParamInt<px4::params::CA_FTC_ALC_MODE>) _param_ca_ftc_alc_mode,
		(ParamInt<px4::params::CA_FTC_INDI_EN>) _param_ca_ftc_indi_en,
		(ParamInt<px4::params::CA_FTC_DUAL_EN>) _param_ca_ftc_dual_en,
		(ParamInt<px4::params::CA_FTC_PAIR>) _param_ca_ftc_pair,
		(ParamFloat<px4::params::CA_FTC_CHI>) _param_ca_ftc_chi,
		(ParamFloat<px4::params::CA_FTC_INDI_K1>) _param_ca_ftc_indi_k1,
		(ParamFloat<px4::params::CA_FTC_INDI_K2>) _param_ca_ftc_indi_k2,
		(ParamFloat<px4::params::CA_FTC_INDI_K3>) _param_ca_ftc_indi_k3,
		(ParamFloat<px4::params::CA_FTC_Z_P>) _param_ca_ftc_z_p,
		(ParamFloat<px4::params::CA_FTC_Z_D>) _param_ca_ftc_z_d,
		(ParamFloat<px4::params::CA_FTC_Y2_P>) _param_ca_ftc_y2_p,
		(ParamFloat<px4::params::CA_FTC_Y2_D>) _param_ca_ftc_y2_d,
		(ParamFloat<px4::params::CA_FTC_INDI_FC>) _param_ca_ftc_indi_fc,
		(ParamFloat<px4::params::CA_FTC_INDI_ILIM>) _param_ca_ftc_indi_ilim,
		(ParamFloat<px4::params::CA_FTC_INDI_M>) _param_ca_ftc_indi_m,
		(ParamFloat<px4::params::CA_FTC_INDI_IX>) _param_ca_ftc_indi_ix,
		(ParamFloat<px4::params::CA_FTC_INDI_IY>) _param_ca_ftc_indi_iy,
		(ParamFloat<px4::params::CA_FTC_KF>) _param_ca_ftc_kf,
		(ParamFloat<px4::params::CA_FTC_OMAX>) _param_ca_ftc_omax,
		(ParamFloat<px4::params::CA_FTC_ERPMAX>) _param_ca_ftc_erpmax,
		(ParamFloat<px4::params::CA_FTC_INDI_SDL>) _param_ca_ftc_indi_sdl,
		(ParamFloat<px4::params::CA_FTC_INDI_DUL>) _param_ca_ftc_indi_dul,
		(ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover,
		(ParamInt<px4::params::CA_RW_MOT_IDX>) _param_ca_rw_mot_idx
	)

};
