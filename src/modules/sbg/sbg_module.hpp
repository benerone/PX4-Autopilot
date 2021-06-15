#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <parameters/param.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/hil_state_quaternion.h>


extern "C" __EXPORT int sbg_main(int argc, char *argv[]);

class ModuleSBG : public ModuleBase<ModuleSBG>, public ModuleParams
{
public:
	ModuleSBG();

	virtual ~ModuleSBG() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static ModuleSBG *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	uORB::Publication<airspeed_s>				_airspeed_pub{ORB_ID(airspeed)};
	uORB::Publication<vehicle_attitude_s>			_attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_global_position_s>		_global_pos_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_local_position_s>		_local_pos_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_acceleration_s>               _vehicle_acceleration_pub{ORB_ID(vehicle_acceleration)};
	uORB::Publication<vehicle_angular_velocity_s> _vehicle_angular_velocity_pub{ORB_ID(vehicle_angular_velocity)};

	uORB::Subscription	_hil_state_sub{ORB_ID(hil_state_quaternion)};

	void processHIL();


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MAV_SYS_ID>) _param_mav_sys_id,
		(ParamInt<px4::params::SYS_HITL>) _param_sys_hitl
	)

	int32_t sys_id;
	bool hil_mode;

	map_projection_reference_s	_hil_local_proj_ref{};
	float				_hil_local_alt0{0.0f};
	bool				_hil_local_proj_inited{false};

};

