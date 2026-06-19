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

#pragma once

#include <gz/math/Vector3.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

#include <fstream>

namespace gz::sim::systems
{

class IrisFtcAerodynamicsSystem:
	public System,
	public ISystemConfigure,
	public ISystemPreUpdate
{
public:
	void Configure(const Entity &_entity,
		       const std::shared_ptr<const sdf::Element> &_sdf,
		       EntityComponentManager &_ecm,
		       EventManager &/*_eventMgr*/) override;

	void PreUpdate(const UpdateInfo &_info,
		       EntityComponentManager &_ecm) override;

private:
	static math::Vector3d NormalizedOr(const math::Vector3d &_value,
					   const math::Vector3d &_fallback);
	static math::Vector3d LowPass(const math::Vector3d &_previous,
				      const math::Vector3d &_input,
				      double _cutoff_hz,
				      double _dt);

	void OpenLogFile();
	void WriteLog(double _time_s,
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
		      const math::Vector3d &_moment_world);

	Model _model{kNullEntity};
	Entity _link_entity{kNullEntity};
	Link _link{kNullEntity};

	std::string _link_name{"base_link"};
	std::string _log_file_path{"/tmp/iris_ftc_aero_debug.csv"};
	std::ofstream _log_file;

	double _mass{1.5};
	double _cx{0.306};
	double _cy1{0.129};
	double _cy2{-0.0339};
	double _velocity_cutoff_hz{2.0};
	double _axis_cutoff_hz{2.0};
	double _max_specific_force{35.0};
	double _lateral_sign_multiplier{1.0};
	double _min_airspeed{0.05};
	bool _enable_force{true};
	bool _enable_moment{false};
	double _moment_scale{1.0};
	double _ma_x_bias{0.0};
	double _ma_x_cos{0.0};
	double _ma_x_sin{0.0};
	double _ma_y_bias{0.0};
	double _ma_y_cos{0.0};
	double _ma_y_sin{0.0};
	double _max_moment{0.0};

	math::Vector3d _thrust_axis_body{0.0, 0.0, 1.0};
	math::Vector3d _spin_axis_body{0.0, 0.0, 1.0};
	math::Vector3d _filtered_air_velocity_world{0.0, 0.0, 0.0};
	math::Vector3d _filtered_thrust_axis_world{0.0, 0.0, 1.0};
	bool _filter_initialized{false};
};

} // namespace gz::sim::systems
