/*
 * Copyright 2026 PX4 Development Team. All rights reserved.
 */

#include "gazebo_reaction_wheel_torque_plugin.h"

namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN(GazeboReactionWheelTorquePlugin);

void GazeboReactionWheelTorquePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
	_model = model;

	if (!_model) {
		gzerr << "[gazebo_reaction_wheel_torque_plugin] invalid model\n";
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
	getSdfParam<double>(sdf, "maxTorque", _max_torque, _max_torque);
	getSdfParam<double>(sdf, "dampingCoefficient", _damping_coefficient, _damping_coefficient);
	getSdfParam<double>(sdf, "timeConstantUp", _time_constant_up, _time_constant_up);
	getSdfParam<double>(sdf, "timeConstantDown", _time_constant_down, _time_constant_down);

	_joint = _model->GetJoint(_joint_name);
	_link = _model->GetLink(_link_name);

	if (!_joint || !_link) {
		gzerr << "[gazebo_reaction_wheel_torque_plugin] failed to get joint or link: "
		      << _joint_name << ", " << _link_name << "\n";
		return;
	}

#if GAZEBO_MAJOR_VERSION < 5
	_joint->SetMaxForce(0, std::numeric_limits<double>::max());
#endif

	_command_sub = _node_handle->Subscribe<mav_msgs::msgs::CommandMotorSpeed>(
		"~/" + _model->GetName() + _command_sub_topic, &GazeboReactionWheelTorquePlugin::TorqueCommandCallback, this);
	_rpm_pub = _node_handle->Advertise<std_msgs::msgs::Float>(
		"~/" + _model->GetName() + _rpm_pub_topic, 1);

	_command_filter.reset(new FirstOrderFilter<double>(_time_constant_up, _time_constant_down, 0.0));
	_update_connection = event::Events::ConnectWorldUpdateBegin(
		boost::bind(&GazeboReactionWheelTorquePlugin::OnUpdate, this, _1));
}

void GazeboReactionWheelTorquePlugin::TorqueCommandCallback(CommandMotorSpeedPtr &command)
{
	if (command->motor_speed_size() <= _motor_number) {
		return;
	}

	_normalized_torque_cmd = constrain(static_cast<double>(command->motor_speed(_motor_number)), -1.0, 1.0);
}

void GazeboReactionWheelTorquePlugin::OnUpdate(const common::UpdateInfo &info)
{
	const double sim_time = info.simTime.Double();
	const double dt = constrain(sim_time - _prev_sim_time, 1e-4, 0.05);
	_prev_sim_time = sim_time;

	const double filtered_command = _command_filter->updateFilter(_normalized_torque_cmd, dt);
	const double current_velocity = _joint->GetVelocity(0);

	double torque_cmd = filtered_command * _max_torque - _damping_coefficient * current_velocity;
	torque_cmd = constrain(torque_cmd, -_max_torque, _max_torque);
	_joint->SetForce(0, torque_cmd);

	std_msgs::msgs::Float rpm_msg;
	rpm_msg.set_data(static_cast<float>(current_velocity * 60.0 / (2.0 * M_PI)));
	_rpm_pub->Publish(rpm_msg);
}

} // namespace gazebo
