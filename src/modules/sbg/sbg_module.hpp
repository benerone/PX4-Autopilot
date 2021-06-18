#pragma once

#ifdef __PX4_NUTTX
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#endif

#include <termios.h>

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
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
#include <uORB/topics/pipepos_correction.h>
#include <uORB/topics/vehicle_share_position.h>
#include <uORB/topics/sbg_status.h>


#include <sbgECom/src/sbgEComLib.h>
#include <sbgECom/common/interfaces/sbgInterface.h>
#include <sbgECom/common/interfaces/sbgInterfacePx4Serial.h>


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

	void processSBG_EKF_QUAT(const SbgBinaryLogData *pLogData);
	void processSBG_EKF_NAV(const SbgBinaryLogData *pLogData);
	void processSBG_IMU_DATA(const SbgBinaryLogData *pLogData);
	void processSBG_LOG_STATUS(const SbgBinaryLogData *pLogData);

private:

	uORB::Publication<airspeed_s>				_airspeed_pub{ORB_ID(airspeed)};
	uORB::Publication<vehicle_attitude_s>			_attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_global_position_s>		_global_pos_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_local_position_s>		_local_pos_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_acceleration_s>               _vehicle_acceleration_pub{ORB_ID(vehicle_acceleration)};
	uORB::Publication<vehicle_angular_velocity_s> _vehicle_angular_velocity_pub{ORB_ID(vehicle_angular_velocity)};
	uORB::PublicationData<vehicle_share_position_s>		_vehicle_share_position_pub{ORB_ID(vehicle_share_position)};
	uORB::Publication<pipepos_correction_s> 		_pipepos_correction_pub{ORB_ID(pipepos_correction)};
	uORB::Publication<sbg_status_s> 		_sbg_status_pub{ORB_ID(sbg_status)};



	uORB::Subscription	_hil_state_sub{ORB_ID(hil_state_quaternion)};
	uORB::Subscription _r1vehicle_share_position_sub{ORB_ID(r1vehicle_share_position)};
	uORB::Subscription _r2vehicle_share_position_sub{ORB_ID(r2vehicle_share_position)};
	uORB::Subscription _r3vehicle_share_position_sub{ORB_ID(r3vehicle_share_position)};


	void processHIL();

	void processLocalPosition(vehicle_local_position_s &lpos);
	void pipeFuseData(vehicle_local_position_s &lpos,vehicle_share_position_s &spos);

	void prepareSBG();
	void executeSBG();

	void terminateSBG();


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MAV_SYS_ID>) _param_mav_sys_id,
		(ParamInt<px4::params::SYS_HITL>) _param_sys_hitl,
		(ParamFloat<px4::params::EKF2_PI_MUL_X>) _param_ekf2_pi_mul_x,
		(ParamFloat<px4::params::EKF2_PI_MUL_Y>) _param_ekf2_pi_mul_y,
		(ParamFloat<px4::params::EKF2_PI_MUL_Z>) _param_ekf2_pi_mul_z,
		(ParamFloat<px4::params::EKF2_PI_LIM_X>) _param_ekf2_pi_lim_x,
		(ParamFloat<px4::params::EKF2_PI_LIM_Y>) _param_ekf2_pi_lim_y,
		(ParamFloat<px4::params::EKF2_PI_LIM_Z>) _param_ekf2_pi_lim_z,

		(ParamFloat<px4::params::EKF2_PI_MUL_VX>) _param_ekf2_pi_mul_vx,
		(ParamFloat<px4::params::EKF2_PI_MUL_VY>) _param_ekf2_pi_mul_vy,
		(ParamFloat<px4::params::EKF2_PI_MUL_VZ>) _param_ekf2_pi_mul_vz,
		(ParamFloat<px4::params::EKF2_PI_LIM_VX>) _param_ekf2_pi_lim_vx,
		(ParamFloat<px4::params::EKF2_PI_LIM_VY>) _param_ekf2_pi_lim_vy,
		(ParamFloat<px4::params::EKF2_PI_LIM_VZ>) _param_ekf2_pi_lim_vz,

		(ParamFloat<px4::params::EKF2_PI_MUL_HE>) _param_ekf2_pi_mul_he,
		(ParamFloat<px4::params::EKF2_PI_LIM_HE>) _param_ekf2_pi_lim_he,
		(ParamInt<px4::params::SBG_ENABLE_HIL>) _param_sbg_enable_hil
	)

	int32_t sys_id;
	bool hil_mode;

	map_projection_reference_s	_hil_local_proj_ref{};
	float				_hil_local_alt0{0.0f};
	bool				_hil_local_proj_inited{false};

	//SBG
	bool 			enable_sbg_in_hil;
	int				_serial_fd{-1};

	SbgEComHandle			comHandle;
	bool 					comHandleInit;
	SbgInterface			sbgInterface;
	bool 					sbgInterfaceInit;
	SbgEComDeviceInfo		deviceInfo;

	map_projection_reference_s	_local_proj_ref{};
	float				_local_alt0{0.0f};
	bool				_local_proj_inited{false};
	vehicle_attitude_s  g_attitude{};

	//Stat
	int nbEKF_QUAT;
	int nbEKF_NAV;
	int nbIMU_DATA;
	int nbLOG_STATUS;

	double global_lat,global_lon;
	sbg_status_s sbg_status;


};

