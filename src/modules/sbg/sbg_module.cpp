#include <ecl/geo/geo.h>
#include <systemlib/px4_macros.h>
#include <math.h>

#include "sbg_module.hpp"

int ModuleSBG::print_status()
{
	if (hil_mode) {
		PX4_INFO("Running (HIL)");
	} else {
		PX4_INFO("Running");
	}

	return 0;
}

int ModuleSBG::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int ModuleSBG::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("sbg",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      4096,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

ModuleSBG *ModuleSBG::instantiate(int argc, char *argv[])
{
	ModuleSBG *instance = new ModuleSBG();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

ModuleSBG::ModuleSBG()
	: ModuleParams(nullptr)
{
	hil_mode=false;

}

void ModuleSBG::processHIL() {
	_hil_local_proj_inited=false;

	while(!should_exit()) {
			hil_state_quaternion_s hil_state;
			//HIL impl
			if (_hil_state_sub.update(&hil_state)) {
				const uint64_t timestamp = hrt_absolute_time();
				/* airspeed */
				{
					airspeed_s airspeed{};

					airspeed.timestamp = timestamp;
					airspeed.indicated_airspeed_m_s = hil_state.ind_airspeed * 1e-2f;
					airspeed.true_airspeed_m_s = hil_state.true_airspeed * 1e-2f;

					_airspeed_pub.publish(airspeed);
				}

				/* attitude */
				{
					vehicle_attitude_s hil_attitude{};

					hil_attitude.timestamp = timestamp;

					matrix::Quatf q(hil_state.attitude_quaternion);
					q.copyTo(hil_attitude.q);

					_attitude_pub.publish(hil_attitude);
				}

				/* global position */
				{
					vehicle_global_position_s hil_global_pos{};

					hil_global_pos.timestamp = timestamp;
					hil_global_pos.lat = hil_state.lat / ((double)1e7);
					hil_global_pos.lon = hil_state.lon / ((double)1e7);
					hil_global_pos.alt = hil_state.alt / 1000.0f;
					hil_global_pos.eph = 2.0f;
					hil_global_pos.epv = 4.0f;

					_global_pos_pub.publish(hil_global_pos);
				}

				/* local position */
				{
					double lat = hil_state.lat * 1e-7;
					double lon = hil_state.lon * 1e-7;

					if (!_hil_local_proj_inited) {
						_hil_local_proj_inited = true;
						_hil_local_alt0 = hil_state.alt / 1000.0f;

						map_projection_init(&_hil_local_proj_ref, lat, lon);
					}

					float x = 0.0f;
					float y = 0.0f;
					map_projection_project(&_hil_local_proj_ref, lat, lon, &x, &y);

					vehicle_local_position_s hil_local_pos{};
					hil_local_pos.timestamp = timestamp;

					hil_local_pos.ref_timestamp = _hil_local_proj_ref.timestamp;
					hil_local_pos.ref_lat = math::radians(_hil_local_proj_ref.lat_rad);
					hil_local_pos.ref_lon = math::radians(_hil_local_proj_ref.lon_rad);
					hil_local_pos.ref_alt = _hil_local_alt0;
					hil_local_pos.xy_valid = true;
					hil_local_pos.z_valid = true;
					hil_local_pos.v_xy_valid = true;
					hil_local_pos.v_z_valid = true;
					hil_local_pos.x = x;
					hil_local_pos.y = y;
					hil_local_pos.z = _hil_local_alt0 - hil_state.alt / 1000.0f;
					hil_local_pos.vx = hil_state.vx / 100.0f;
					hil_local_pos.vy = hil_state.vy / 100.0f;
					hil_local_pos.vz = hil_state.vz / 100.0f;

					matrix::Eulerf euler{matrix::Quatf(hil_state.attitude_quaternion)};
					hil_local_pos.heading = euler.psi();
					hil_local_pos.xy_global = true;
					hil_local_pos.z_global = true;
					hil_local_pos.vxy_max = INFINITY;
					hil_local_pos.vz_max = INFINITY;
					hil_local_pos.hagl_min = INFINITY;
					hil_local_pos.hagl_max = INFINITY;

					_local_pos_pub.publish(hil_local_pos);
				}

				/* accelerometer */
				{
					vehicle_acceleration_s va{};
					va.timestamp = timestamp;
					va.timestamp_sample = hil_state.timestamp;
					va.xyz[0]= ((float32)hil_state.xacc)*9.81f/1000.0f;
					va.xyz[1]= ((float32)hil_state.yacc)*9.81f/1000.0f;
					va.xyz[2]= ((float32)hil_state.zacc)*9.81f/1000.0f;
					_vehicle_acceleration_pub.publish(va);
				}

				/* gyroscope */
				{
					vehicle_angular_velocity_s vav{};
					vav.timestamp = timestamp;
					vav.timestamp_sample = hil_state.timestamp;
					vav.xyz[0]= hil_state.rollspeed;
					vav.xyz[1]= hil_state.pitchspeed;
					vav.xyz[2]= hil_state.yawspeed;
					_vehicle_angular_velocity_pub.publish(vav);
				}
			}

			usleep(100);
	}
}

void ModuleSBG::run()
{


	//Mavlink id serve as id
	sys_id=_param_mav_sys_id.get();
	hil_mode=(_param_sys_hitl.get()!=0);

	if (hil_mode) {
		processHIL();
	} else {

		while(!should_exit()) {
			//SBG impl
			usleep(100);
		}

	}


}


int ModuleSBG::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
SBG module
### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ sbg start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sbg", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	//PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	//PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int sbg_main(int argc, char *argv[])
{
	return ModuleSBG::main(argc, argv);
}
