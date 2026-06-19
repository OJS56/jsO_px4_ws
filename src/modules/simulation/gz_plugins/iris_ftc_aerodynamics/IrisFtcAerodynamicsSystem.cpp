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

#include "IrisFtcAerodynamicsSystem.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Wind.hh>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <stdexcept>

using namespace gz;
using namespace sim;
using namespace systems;

namespace
{
double Sign(double value)
{
	return (0.0 < value) - (value < 0.0);
}

double ReadDouble(const std::shared_ptr<const sdf::Element> &sdf,
		  const char *name,
		  double default_value)
{
	return sdf->HasElement(name) ? sdf->Get<double>(name) : default_value;
}

bool ReadBool(const std::shared_ptr<const sdf::Element> &sdf,
	      const char *name,
	      bool default_value)
{
	return sdf->HasElement(name) ? sdf->Get<bool>(name) : default_value;
}

math::Vector3d ReadVector3(const std::shared_ptr<const sdf::Element> &sdf,
			   const char *name,
			   const math::Vector3d &default_value)
{
	return sdf->HasElement(name) ? sdf->Get<math::Vector3d>(name) : default_value;
}
} // namespace

void IrisFtcAerodynamicsSystem::Configure(const Entity &_entity,
		const std::shared_ptr<const sdf::Element> &_sdf,
		EntityComponentManager &_ecm,
		EventManager &/*_eventMgr*/)
{
	_model = Model(_entity);

	if (_sdf->HasElement("linkName")) {
		_link_name = _sdf->Get<std::string>("linkName");
	}

	_link_entity = _model.LinkByName(_ecm, _link_name);

	if (_link_entity == kNullEntity) {
		throw std::runtime_error("IrisFtcAerodynamicsSystem: link [" + _link_name + "] was not found");
	}

	_link = Link(_link_entity);
	_link.EnableVelocityChecks(_ecm, true);

	_mass = ReadDouble(_sdf, "mass", _mass);
	_cx = ReadDouble(_sdf, "cx", _cx);
	_cy1 = ReadDouble(_sdf, "cy1", _cy1);
	_cy2 = ReadDouble(_sdf, "cy2", _cy2);
	_velocity_cutoff_hz = ReadDouble(_sdf, "velocityCutoffHz", _velocity_cutoff_hz);
	_axis_cutoff_hz = ReadDouble(_sdf, "axisCutoffHz", _axis_cutoff_hz);
	_max_specific_force = std::max(0.0, ReadDouble(_sdf, "maxSpecificForce", _max_specific_force));
	_lateral_sign_multiplier = ReadDouble(_sdf, "lateralSignMultiplier", _lateral_sign_multiplier);
	_min_airspeed = std::max(0.0, ReadDouble(_sdf, "minAirspeed", _min_airspeed));
	_enable_force = ReadBool(_sdf, "enableForce", _enable_force);
	_enable_moment = ReadBool(_sdf, "enableMoment", _enable_moment);
	_moment_scale = ReadDouble(_sdf, "momentScale", _moment_scale);
	_ma_x_bias = ReadDouble(_sdf, "maXBias", _ma_x_bias);
	_ma_x_cos = ReadDouble(_sdf, "maXCos", _ma_x_cos);
	_ma_x_sin = ReadDouble(_sdf, "maXSin", _ma_x_sin);
	_ma_y_bias = ReadDouble(_sdf, "maYBias", _ma_y_bias);
	_ma_y_cos = ReadDouble(_sdf, "maYCos", _ma_y_cos);
	_ma_y_sin = ReadDouble(_sdf, "maYSin", _ma_y_sin);
	_max_moment = std::max(0.0, ReadDouble(_sdf, "maxMoment", _max_moment));
	_thrust_axis_body = NormalizedOr(ReadVector3(_sdf, "thrustAxisBody", _thrust_axis_body),
					 math::Vector3d{0.0, 0.0, 1.0});
	_spin_axis_body = NormalizedOr(ReadVector3(_sdf, "spinAxisBody", _spin_axis_body),
				       math::Vector3d{0.0, 0.0, 1.0});

	if (_sdf->HasElement("logFile")) {
		_log_file_path = _sdf->Get<std::string>("logFile");
	}

	OpenLogFile();

	gzmsg << "[IrisFtcAerodynamicsSystem] link=" << _link_name
	      << " mass=" << _mass
	      << " Cx=" << _cx
	      << " Cy1=" << _cy1
	      << " Cy2=" << _cy2
	      << " enableForce=" << _enable_force
	      << " enableMoment=" << _enable_moment
	      << " log=" << _log_file_path << std::endl;
}

void IrisFtcAerodynamicsSystem::PreUpdate(const UpdateInfo &_info,
		EntityComponentManager &_ecm)
{
	if (_info.paused) {
		return;
	}

	const auto pose = _link.WorldPose(_ecm);
	const auto velocity_world = _link.WorldLinearVelocity(_ecm);
	const auto angular_velocity_world = _link.WorldAngularVelocity(_ecm);

	if (!pose.has_value() || !velocity_world.has_value() || !angular_velocity_world.has_value()) {
		return;
	}

	math::Vector3d wind_world = math::Vector3d::Zero;
	const Entity wind_entity = _ecm.EntityByComponents(components::Wind());

	if (wind_entity != kNullEntity) {
		const auto wind_velocity = _ecm.Component<components::WorldLinearVelocity>(wind_entity);

		if (wind_velocity) {
			wind_world = wind_velocity->Data();
		}
	}

	const double dt = std::chrono::duration<double>(_info.dt).count();
	const math::Vector3d air_velocity_world_raw = velocity_world.value() - wind_world;
	const math::Vector3d thrust_axis_world_raw =
		NormalizedOr(pose->Rot().RotateVector(_thrust_axis_body), math::Vector3d{0.0, 0.0, 1.0});

	if (!_filter_initialized) {
		_filtered_air_velocity_world = air_velocity_world_raw;
		_filtered_thrust_axis_world = thrust_axis_world_raw;
		_filter_initialized = true;

	} else {
		_filtered_air_velocity_world = LowPass(_filtered_air_velocity_world, air_velocity_world_raw,
						       _velocity_cutoff_hz, dt);
		_filtered_thrust_axis_world = NormalizedOr(LowPass(_filtered_thrust_axis_world, thrust_axis_world_raw,
							      _axis_cutoff_hz, dt), thrust_axis_world_raw);
	}

	// The paper defines z_S against the average thrust direction.
	const math::Vector3d z_s_world = NormalizedOr(-_filtered_thrust_axis_world, math::Vector3d{0.0, 0.0, -1.0});
	const math::Vector3d air_in_plane =
		_filtered_air_velocity_world - _filtered_air_velocity_world.Dot(z_s_world) * z_s_world;
	const double airspeed = air_in_plane.Length();

	if (airspeed < _min_airspeed) {
		WriteLog(std::chrono::duration<double>(_info.simTime).count(), wind_world, _filtered_air_velocity_world,
			 math::Vector3d::Zero, math::Vector3d::Zero, z_s_world, airspeed, 0.0, 0.0, 0.0, 0.0,
			 math::Vector3d::Zero, math::Vector3d::Zero, math::Vector3d::Zero);
		return;
	}

	// x_S points against the airspeed direction, and y_S completes a right-handed frame.
	const math::Vector3d x_s_world = NormalizedOr(-air_in_plane, math::Vector3d{1.0, 0.0, 0.0});
	const math::Vector3d y_s_world = NormalizedOr(z_s_world.Cross(x_s_world), math::Vector3d{0.0, 1.0, 0.0});
	const math::Vector3d spin_axis_world =
		NormalizedOr(pose->Rot().RotateVector(_spin_axis_body), thrust_axis_world_raw);
	const double r_about_spin_axis = angular_velocity_world->Dot(spin_axis_world);
	const math::Vector3d body_x_world = NormalizedOr(pose->Rot().RotateVector(math::Vector3d{1.0, 0.0, 0.0}),
					     math::Vector3d{1.0, 0.0, 0.0});
	const double psi_s = std::atan2(body_x_world.Dot(y_s_world), body_x_world.Dot(x_s_world));

	double fx_s = _cx * airspeed;
	double fy_s = _lateral_sign_multiplier * Sign(r_about_spin_axis) *
		      (_cy1 * airspeed + _cy2 * airspeed * airspeed);

	if (_max_specific_force > 0.0) {
		const double force_norm = std::hypot(fx_s, fy_s);

		if (force_norm > _max_specific_force) {
			const double scale = _max_specific_force / force_norm;
			fx_s *= scale;
			fy_s *= scale;
		}
	}

	const math::Vector3d force_world = _mass * (fx_s * x_s_world + fy_s * y_s_world);

	if (_enable_force) {
		_link.AddWorldForce(_ecm, force_world);
	}

	// Ma is intentionally a disabled-by-default surrogate. The paper does not
	// identify a closed-form moment model; this structure is only for controlled
	// disturbance experiments once coefficients are supplied.
	math::Vector3d moment_body = math::Vector3d::Zero;
	math::Vector3d moment_world = math::Vector3d::Zero;

	if (_enable_moment) {
		const double v2 = airspeed * airspeed;
		moment_body.Set(
			_moment_scale * v2 * (_ma_x_bias + _ma_x_cos * std::cos(psi_s) + _ma_x_sin * std::sin(psi_s)),
			_moment_scale * v2 * (_ma_y_bias + _ma_y_cos * std::cos(psi_s) + _ma_y_sin * std::sin(psi_s)),
			0.0);

		if (_max_moment > 0.0) {
			const double moment_norm = moment_body.Length();

			if (moment_norm > _max_moment) {
				moment_body *= _max_moment / moment_norm;
			}
		}

		moment_world = pose->Rot().RotateVector(moment_body);
		_link.AddWorldWrench(_ecm, math::Vector3d::Zero, moment_world);
	}

	WriteLog(std::chrono::duration<double>(_info.simTime).count(), wind_world, _filtered_air_velocity_world,
		 x_s_world, y_s_world, z_s_world, airspeed, r_about_spin_axis, psi_s, fx_s, fy_s, force_world,
		 moment_body, moment_world);
}

math::Vector3d IrisFtcAerodynamicsSystem::NormalizedOr(const math::Vector3d &_value,
		const math::Vector3d &_fallback)
{
	const double length = _value.Length();

	if (length < 1e-9 || !std::isfinite(length)) {
		return _fallback;
	}

	return _value / length;
}

math::Vector3d IrisFtcAerodynamicsSystem::LowPass(const math::Vector3d &_previous,
		const math::Vector3d &_input,
		double _cutoff_hz,
		double _dt)
{
	if (_cutoff_hz <= 0.0 || _dt <= 0.0) {
		return _input;
	}

	const double tau = 1.0 / (2.0 * GZ_PI * _cutoff_hz);
	const double alpha = _dt / (tau + _dt);
	return _previous + alpha * (_input - _previous);
}

void IrisFtcAerodynamicsSystem::OpenLogFile()
{
	_log_file.open(_log_file_path, std::ios::out | std::ios::trunc);

	if (!_log_file.is_open()) {
		gzerr << "[IrisFtcAerodynamicsSystem] Failed to open log [" << _log_file_path << "]" << std::endl;
		return;
	}

	_log_file << "time_s,wind_x,wind_y,wind_z,air_x,air_y,air_z,"
		  << "x_s_x,x_s_y,x_s_z,y_s_x,y_s_y,y_s_z,z_s_x,z_s_y,z_s_z,"
		  << "airspeed,r_about_spin_axis,psi_s,fx_s,fy_s,force_x,force_y,force_z,"
		  << "moment_b_x,moment_b_y,moment_b_z,moment_w_x,moment_w_y,moment_w_z\n";
	_log_file.flush();
}

void IrisFtcAerodynamicsSystem::WriteLog(double _time_s,
		const math::Vector3d &_wind_world,
		const math::Vector3d &_air_velocity_world,
		const math::Vector3d &_x_s_world,
		const math::Vector3d &_y_s_world,
		const math::Vector3d &_z_s_world,
		double _airspeed,
		double _r_about_spin_axis,
		double _psi_s,
		double _fx_s,
		double _fy_s,
		const math::Vector3d &_force_world,
		const math::Vector3d &_moment_body,
		const math::Vector3d &_moment_world)
{
	if (!_log_file.is_open()) {
		return;
	}

	_log_file << std::fixed << std::setprecision(6)
		  << _time_s << ","
		  << _wind_world.X() << "," << _wind_world.Y() << "," << _wind_world.Z() << ","
		  << _air_velocity_world.X() << "," << _air_velocity_world.Y() << "," << _air_velocity_world.Z() << ","
		  << _x_s_world.X() << "," << _x_s_world.Y() << "," << _x_s_world.Z() << ","
		  << _y_s_world.X() << "," << _y_s_world.Y() << "," << _y_s_world.Z() << ","
		  << _z_s_world.X() << "," << _z_s_world.Y() << "," << _z_s_world.Z() << ","
		  << _airspeed << ","
		  << _r_about_spin_axis << ","
		  << _psi_s << ","
		  << _fx_s << ","
		  << _fy_s << ","
		  << _force_world.X() << "," << _force_world.Y() << "," << _force_world.Z() << ","
		  << _moment_body.X() << "," << _moment_body.Y() << "," << _moment_body.Z() << ","
		  << _moment_world.X() << "," << _moment_world.Y() << "," << _moment_world.Z() << "\n";
	_log_file.flush();
}

GZ_ADD_PLUGIN(
	IrisFtcAerodynamicsSystem,
	gz::sim::System,
	IrisFtcAerodynamicsSystem::ISystemConfigure,
	IrisFtcAerodynamicsSystem::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(IrisFtcAerodynamicsSystem,
		    "gz::sim::systems::IrisFtcAerodynamicsSystem")
