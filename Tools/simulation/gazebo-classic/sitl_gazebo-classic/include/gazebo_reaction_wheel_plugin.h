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

static const std::string kDefaultReactionWheelCommandSubTopic = "/gazebo/command/motor_speed";
static const std::string kDefaultReactionWheelRpmPubTopic = "/reaction_wheel/rpm";

class GazeboReactionWheelPlugin : public ModelPlugin
{
public:
	GazeboReactionWheelPlugin() = default;
	~GazeboReactionWheelPlugin() override = default;

	void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
	void OnUpdate(const common::UpdateInfo &info);
	void VelocityCallback(CommandMotorSpeedPtr &rot_velocities);

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
	std::string _command_sub_topic{kDefaultReactionWheelCommandSubTopic};
	std::string _rpm_pub_topic{kDefaultReactionWheelRpmPubTopic};

	int _motor_number{4};

	double _ref_wheel_rot_vel{0.0};
	double _max_rot_velocity{837.7580409572781}; // 8000 rpm
	double _max_torque{0.05};
	double _velocity_p_gain{0.0025};
	double _damping_coefficient{0.0002};
	double _time_constant_up{1.0 / 80.0};
	double _time_constant_down{1.0 / 40.0};
	double _prev_sim_time{0.0};
};

} // namespace gazebo
