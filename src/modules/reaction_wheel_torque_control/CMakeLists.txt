############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__reaction_wheel_torque_control
	MAIN reaction_wheel_torque_control
	SRCS
		ReactionWheelTorqueControl.cpp
		ReactionWheelTorqueControl.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		px4_work_queue
)
