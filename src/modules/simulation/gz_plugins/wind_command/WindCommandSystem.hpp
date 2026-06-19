/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#ifndef GZ_SIM_SYSTEMS_WINDCOMMANDSYSTEM_HPP_
#define GZ_SIM_SYSTEMS_WINDCOMMANDSYSTEM_HPP_

#include <gz/math/Vector3.hh>
#include <gz/msgs/wind.pb.h>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <mutex>
#include <fstream>
#include <string>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{

class WindCommandSystem:
	public System,
	public ISystemConfigure,
	public ISystemPreUpdate
{
public:
	void Configure(const Entity &_entity,
		       const std::shared_ptr<const sdf::Element> &_sdf,
		       EntityComponentManager &_ecm,
		       EventManager &_eventMgr) override;

	void PreUpdate(const UpdateInfo &_info,
		       EntityComponentManager &_ecm) override;

private:
	void WindCommandCallback(const gz::msgs::Wind &_msg);
	void PublishState(const gz::math::Vector3d &_wind_velocity, bool _enabled);
	void OpenLogFile();
	void WriteLog(double _sim_time_s, const gz::math::Vector3d &_wind_velocity, bool _enabled);

	gz::transport::Node _node;
	gz::transport::Node::Publisher _state_pub;

	std::string _command_topic;
	std::string _state_topic;
	std::string _log_file_path{"/tmp/iris_ftc_wind_events.csv"};
	std::ofstream _log_file;
	double _max_wind_speed{30.0};

	std::mutex _command_mutex;
	gz::math::Vector3d _pending_velocity{0.0, 0.0, 0.0};
	bool _pending_enabled{false};
	bool _has_pending_command{false};
};

} // namespace systems
} // namespace GZ_SIM_VERSION_NAMESPACE
} // namespace sim
} // namespace gz

#endif // GZ_SIM_SYSTEMS_WINDCOMMANDSYSTEM_HPP_
