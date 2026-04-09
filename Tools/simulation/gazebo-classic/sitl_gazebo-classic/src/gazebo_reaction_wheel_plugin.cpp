/*
 * Copyright 2026 PX4 Development Team. All rights reserved.
 */

#include "gazebo_reaction_wheel_plugin.h"

namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN(GazeboReactionWheelPlugin);

void GazeboReactionWheelPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
	_model = model;

	if (!_model) {
		gzerr << "[gazebo_reaction_wheel_plugin] invalid model\n";
		return;
	}

	if (sdf->HasElement("robotNamespace")) {
		_namespace = sdf->Get<std::string>("robotNamespace");
	}

	_node_handle = transport::NodePtr(new transport::Node());
	_node_handle->Init(_namespace);

	getSdfParam<std::string>(sdf, "jointName", _joint_name, _joint_name, true);
	getSdfParam<std::string>(sdf, "linkName", _link_name, _link_name, true);
	getSdfParam<std::string>(sdf, "commandSubTopic", _command_sub_topic, _command_sub_topic);
	getSdfParam<std::string>(sdf, "wheelRpmPubTopic", _rpm_pub_topic, _rpm_pub_topic);
	getSdfParam<int>(sdf, "motorNumber", _motor_number, _motor_number);
	getSdfParam<double>(sdf, "maxRotVelocity", _max_rot_velocity, _max_rot_velocity);
	getSdfParam<double>(sdf, "maxTorque", _max_torque, _max_torque);
	getSdfParam<double>(sdf, "velocityPGain", _velocity_p_gain, _velocity_p_gain);
	getSdfParam<double>(sdf, "dampingCoefficient", _damping_coefficient, _damping_coefficient);
	getSdfParam<double>(sdf, "timeConstantUp", _time_constant_up, _time_constant_up);
	getSdfParam<double>(sdf, "timeConstantDown", _time_constant_down, _time_constant_down);

	_joint = _model->GetJoint(_joint_name);
	_link = _model->GetLink(_link_name);

	if (!_joint || !_link) {
		gzerr << "[gazebo_reaction_wheel_plugin] failed to get joint or link: "
		      << _joint_name << ", " << _link_name << "\n";
		return;
	}

#if GAZEBO_MAJOR_VERSION < 5
	_joint->SetMaxForce(0, std::numeric_limits<double>::max());
#endif

	_command_sub = _node_handle->Subscribe<mav_msgs::msgs::CommandMotorSpeed>(
		"~/" + _model->GetName() + _command_sub_topic, &GazeboReactionWheelPlugin::VelocityCallback, this);
	_rpm_pub = _node_handle->Advertise<std_msgs::msgs::Float>(
		"~/" + _model->GetName() + _rpm_pub_topic, 1);

	_command_filter.reset(new FirstOrderFilter<double>(_time_constant_up, _time_constant_down, 0.0));
	_update_connection = event::Events::ConnectWorldUpdateBegin(
		boost::bind(&GazeboReactionWheelPlugin::OnUpdate, this, _1));
}

void GazeboReactionWheelPlugin::VelocityCallback(CommandMotorSpeedPtr &rot_velocities)
{
	if (rot_velocities->motor_speed_size() <= _motor_number) {
		return;
	}

	_ref_wheel_rot_vel = constrain(static_cast<double>(rot_velocities->motor_speed(_motor_number)),
				       -_max_rot_velocity, _max_rot_velocity);
}

void GazeboReactionWheelPlugin::OnUpdate(const common::UpdateInfo &info)
{
	const double sim_time = info.simTime.Double();
	const double dt = constrain(sim_time - _prev_sim_time, 1e-4, 0.05);
	_prev_sim_time = sim_time;

	const double filtered_reference = _command_filter->updateFilter(_ref_wheel_rot_vel, dt);
	const double current_velocity = _joint->GetVelocity(0);

	double torque_cmd = _velocity_p_gain * (filtered_reference - current_velocity)
			    - _damping_coefficient * current_velocity;
	torque_cmd = constrain(torque_cmd, -_max_torque, _max_torque);

	_joint->SetForce(0, torque_cmd);

	std_msgs::msgs::Float rpm_msg;
	rpm_msg.set_data(static_cast<float>(current_velocity * 60.0 / (2.0 * M_PI)));
	_rpm_pub->Publish(rpm_msg);
}

} // namespace gazebo
