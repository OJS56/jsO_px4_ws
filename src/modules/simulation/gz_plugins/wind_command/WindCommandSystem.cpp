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

#include "WindCommandSystem.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Wind.hh>

#include <algorithm>
#include <chrono>
#include <iomanip>

using namespace gz;
using namespace sim;
using namespace systems;

void WindCommandSystem::Configure(const Entity &_entity,
				  const std::shared_ptr<const sdf::Element> &_sdf,
				  EntityComponentManager &_ecm,
				  EventManager &/*_eventMgr*/)
{
	const World world(_entity);
	const std::string world_name = world.Name(_ecm).value_or("default");

	_command_topic = "/world/" + world_name + "/wind_cmd";
	_state_topic = "/world/" + world_name + "/wind_cmd/state";

	if (_sdf->HasElement("commandTopic")) {
		_command_topic = _sdf->Get<std::string>("commandTopic");
	}

	if (_sdf->HasElement("stateTopic")) {
		_state_topic = _sdf->Get<std::string>("stateTopic");
	}

	if (_sdf->HasElement("maxWindSpeed")) {
		_max_wind_speed = std::max(0.0, _sdf->Get<double>("maxWindSpeed"));
	}

	if (_sdf->HasElement("logFile")) {
		_log_file_path = _sdf->Get<std::string>("logFile");
	}

	OpenLogFile();

	if (!_node.Subscribe(_command_topic, &WindCommandSystem::WindCommandCallback, this)) {
		gzerr << "[WindCommandSystem] Failed to subscribe to [" << _command_topic << "]" << std::endl;
		return;
	}

	_state_pub = _node.Advertise<gz::msgs::Wind>(_state_topic);

	gzmsg << "[WindCommandSystem] Subscribed to [" << _command_topic << "]" << std::endl;
	gzmsg << "[WindCommandSystem] Publishing state on [" << _state_topic << "]" << std::endl;
	gzmsg << "[WindCommandSystem] maxWindSpeed=" << _max_wind_speed << " m/s" << std::endl;
	gzmsg << "[WindCommandSystem] Logging commands to [" << _log_file_path << "]" << std::endl;
}

void WindCommandSystem::PreUpdate(const UpdateInfo &_info,
				  EntityComponentManager &_ecm)
{
	if (_info.paused) {
		return;
	}

	gz::math::Vector3d commanded_velocity;
	bool commanded_enabled;

	{
		std::lock_guard<std::mutex> lock(_command_mutex);

		if (!_has_pending_command) {
			return;
		}

		commanded_velocity = _pending_velocity;
		commanded_enabled = _pending_enabled;
		_has_pending_command = false;
	}

	gz::math::Vector3d applied_velocity = commanded_enabled ? commanded_velocity : gz::math::Vector3d::Zero;
	const double speed = applied_velocity.Length();

	if (_max_wind_speed > 0.0 && speed > _max_wind_speed) {
		applied_velocity *= _max_wind_speed / speed;
	}

	Entity wind_entity = _ecm.EntityByComponents(components::Wind());

	if (wind_entity == kNullEntity) {
		wind_entity = _ecm.CreateEntity();
		_ecm.CreateComponent(wind_entity, components::Name("wind"));
		_ecm.CreateComponent(wind_entity, components::Wind());
		_ecm.CreateComponent(wind_entity, components::WorldLinearVelocity(applied_velocity));

	} else {
		_ecm.SetComponentData<components::WorldLinearVelocity>(wind_entity, applied_velocity);
	}

	PublishState(applied_velocity, commanded_enabled);
	WriteLog(std::chrono::duration<double>(_info.simTime).count(), applied_velocity, commanded_enabled);
	gzmsg << "[WindCommandSystem] wind=" << applied_velocity
	      << " enabled=" << commanded_enabled << std::endl;
}

void WindCommandSystem::WindCommandCallback(const gz::msgs::Wind &_msg)
{
	gz::math::Vector3d velocity{0.0, 0.0, 0.0};

	if (_msg.has_linear_velocity()) {
		velocity.Set(_msg.linear_velocity().x(), _msg.linear_velocity().y(), _msg.linear_velocity().z());
	}

	std::lock_guard<std::mutex> lock(_command_mutex);
	_pending_velocity = velocity;
	_pending_enabled = _msg.enable_wind() || velocity.Length() > 1e-6;
	_has_pending_command = true;
}

void WindCommandSystem::PublishState(const gz::math::Vector3d &_wind_velocity, bool _enabled)
{
	gz::msgs::Wind msg;
	msg.mutable_linear_velocity()->set_x(_wind_velocity.X());
	msg.mutable_linear_velocity()->set_y(_wind_velocity.Y());
	msg.mutable_linear_velocity()->set_z(_wind_velocity.Z());
	msg.set_enable_wind(_enabled);
	_state_pub.Publish(msg);
}

void WindCommandSystem::OpenLogFile()
{
	_log_file.open(_log_file_path, std::ios::out | std::ios::trunc);

	if (!_log_file.is_open()) {
		gzerr << "[WindCommandSystem] Failed to open wind command log ["
		      << _log_file_path << "]" << std::endl;
		return;
	}

	_log_file << "time_s,x,y,z,enabled\n";
	_log_file.flush();
}

void WindCommandSystem::WriteLog(double _sim_time_s, const gz::math::Vector3d &_wind_velocity, bool _enabled)
{
	if (!_log_file.is_open()) {
		return;
	}

	_log_file << std::fixed << std::setprecision(6)
		  << _sim_time_s << ","
		  << _wind_velocity.X() << ","
		  << _wind_velocity.Y() << ","
		  << _wind_velocity.Z() << ","
		  << (_enabled ? 1 : 0) << "\n";
	_log_file.flush();
}

GZ_ADD_PLUGIN(
	WindCommandSystem,
	gz::sim::System,
	WindCommandSystem::ISystemConfigure,
	WindCommandSystem::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(WindCommandSystem,
		    "gz::sim::systems::WindCommandSystem")
