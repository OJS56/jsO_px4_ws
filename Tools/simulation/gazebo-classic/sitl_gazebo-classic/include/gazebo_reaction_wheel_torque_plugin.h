/*
 * Copyright 2026 PX4 Development Team. All rights reserved.
 */

#pragma once

#include <boost/bind.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include "CommandMotorSpeed.pb.h"
#include "Float.pb.h"
#include "common.h"

namespace gazebo
{

typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;

static const std::string kDefaultReactionWheelTorqueCommandSubTopic = "/gazebo/command/motor_speed";
static const std::string kDefaultReactionWheelTorqueRpmPubTopic = "/reaction_wheel/rpm";

class GazeboReactionWheelTorquePlugin : public ModelPlugin
{
public:
	GazeboReactionWheelTorquePlugin() = default;
	~GazeboReactionWheelTorquePlugin() override = default;

	void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
	void OnUpdate(const common::UpdateInfo &info);
	void TorqueCommandCallback(CommandMotorSpeedPtr &command);

	physics::ModelPtr _model;
	physics::JointPtr _joint;
	physics::LinkPtr _link;

	transport::NodePtr _node_handle;
	transport::SubscriberPtr _command_sub;
	transport::PublisherPtr _rpm_pub;

	event::ConnectionPtr _update_connection;
	std::unique_ptr<FirstOrderFilter<double>> _command_filter;

	std::string _namespace;
	std::string _joint_name;
	std::string _link_name;
	std::string _command_sub_topic{kDefaultReactionWheelTorqueCommandSubTopic};
	std::string _rpm_pub_topic{kDefaultReactionWheelTorqueRpmPubTopic};

	int _motor_number{4};
	double _normalized_torque_cmd{0.0};
	double _max_torque{0.45};
	double _damping_coefficient{0.0003};
	double _time_constant_up{1.0 / 80.0};
	double _time_constant_down{1.0 / 40.0};
	double _prev_sim_time{0.0};
};

} // namespace gazebo
