#include <ecl/geo/geo.h>
#include <systemlib/px4_macros.h>
#include <math.h>




#include "sbg_module.hpp"
#include "pipetools.h"

#define PORT "/dev/ttyS2"

using namespace zapata;

/*!
 *	Callback definition called each time a new log is received.
 *	\param[in]	pHandle									Valid handle on the sbgECom instance that has called this callback.
 *	\param[in]	msgClass								Class of the message we have received
 *	\param[in]	msg										Message ID of the log received.
 *	\param[in]	pLogData								Contains the received log data as an union.
 *	\param[in]	pUserArg								Optional user supplied argument.
 *	\return												SBG_NO_ERROR if the received log has been used successfully.
 */
SbgErrorCode onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg)
{
	//
	// Handle separately each received data according to the log ID
	//
	switch (msg)
	{
	case SBG_ECOM_LOG_IMU_DATA:
		//PX4_INFO("SBG_ECOM_LOG_IMU_DATA");
		((ModuleSBG*)pUserArg)->processSBG_IMU_DATA(pLogData);
		break;
	case SBG_ECOM_LOG_EKF_EULER:
		//
		// Simply display euler angles in real time
		//
		/*PX4_INFO("Euler Angles: %3.1f\t%3.1f\t%3.1f\tStd Dev:%3.1f\t%3.1f\t%3.1f   \r",
				(double)sbgRadToDegF(pLogData->ekfEulerData.euler[0]), (double)sbgRadToDegF(pLogData->ekfEulerData.euler[1]), (double)sbgRadToDegF(pLogData->ekfEulerData.euler[2]),
				(double)sbgRadToDegF(pLogData->ekfEulerData.eulerStdDev[0]), (double)sbgRadToDegF(pLogData->ekfEulerData.eulerStdDev[1]), (double)sbgRadToDegF(pLogData->ekfEulerData.eulerStdDev[2]));
		*/
		break;
	case SBG_ECOM_LOG_EKF_QUAT:
		//PX4_INFO("SBG_ECOM_LOG_EKF_QUAT");
		((ModuleSBG*)pUserArg)->processSBG_EKF_QUAT(pLogData);
		break;
	case SBG_ECOM_LOG_EKF_NAV:
		//PX4_INFO("SBG_ECOM_LOG_EKF_NAV");
		((ModuleSBG*)pUserArg)->processSBG_EKF_NAV(pLogData);
		break;
	default:
		break;
	}

	return SBG_NO_ERROR;
}



int ModuleSBG::print_status()
{
	if (hil_mode) {
		PX4_INFO("Running (HIL)");
	} else {
		PX4_INFO("Running");
	}
	if (sbgInterfaceInit) {
		PX4_INFO("SBG:Interface Ok");
	} else {
		PX4_INFO("SBG:Interface Fail");
	}

	if (comHandleInit) {
		PX4_INFO("SBG:Init Ok");
		PX4_INFO("SBG:Device : %u found", deviceInfo.serialNumber);
	} else {
		PX4_INFO("SBG:Init Fail");
	}
	PX4_INFO("nbEKF_QUAT: %d",nbEKF_QUAT);
	PX4_INFO("nbEKF_NAV: %d",nbEKF_NAV);
	PX4_INFO("nbIMU_DATA: %d",nbIMU_DATA);
	PX4_INFO("lat: %f",global_lat);
	PX4_INFO("lon: %f",global_lon);

	int smode=solutions & 0b1111;

	switch(smode) {
		case SBG_ECOM_SOL_MODE_UNINITIALIZED:
			PX4_INFO("MODE: NOT INITIALISED: Kalman filter is not initialized, all data invalid");
			break;
		case SBG_ECOM_SOL_MODE_VERTICAL_GYRO:
			PX4_INFO("MODE: VERTICAL_GYRO : The Kalman filter only rely on a vertical reference to compute roll and pitch angles. Heading and navigation data drift freely");
			break;
		case SBG_ECOM_SOL_MODE_AHRS:
			PX4_INFO("MODE: AHRS : A heading reference is available, the Kalman filter provides full orientation but navigation data drift freely");
			break;
		case SBG_ECOM_SOL_MODE_NAV_VELOCITY:
			PX4_INFO("MODE: NAV_VELOCITY : The Kalman filter computes orientation and velocity. Position is freely integrated from velocity estimation");
			break;
		case SBG_ECOM_SOL_MODE_NAV_POSITION:
			PX4_INFO("MODE: NOMINAL : Nominal mode, the Kalman filter computes all parameters (attitude, velocity, position). Absolute position is provided");
			break;
		default:
			PX4_INFO("MODE: Unknown mode !!!!");
	}

	if ((solutions & SBG_ECOM_SOL_MODE_VERTICAL_GYRO)==SBG_ECOM_SOL_MODE_VERTICAL_GYRO) {
		PX4_INFO("The Kalman filter only rely on a vertical reference to compute roll and pitch angles. Heading and navigation data drift freely");
	} else {
		//PX4_INFO("Kalman filter ok");
	}
	if ((solutions & SBG_ECOM_SOL_MODE_AHRS)==SBG_ECOM_SOL_MODE_AHRS) {
		PX4_INFO("A heading reference is available, the Kalman filter provides full orientation but navigation data drift freely");
	} else {
		//PX4_INFO("Kalman filter ok");
	}
	if ((solutions & SBG_ECOM_SOL_MODE_NAV_VELOCITY)==SBG_ECOM_SOL_MODE_NAV_VELOCITY) {
		PX4_INFO("The Kalman filter computes orientation and velocity. Position is freely integrated from velocity estimation");
	} else {
		//PX4_INFO("Kalman filter ok");
	}
	if ((solutions & SBG_ECOM_SOL_MODE_NAV_POSITION)==SBG_ECOM_SOL_MODE_NAV_POSITION) {
		PX4_INFO("Nominal mode, the Kalman filter computes all parameters (attitude, velocity, position). Absolute position is provided");
	} else {
		//PX4_INFO("Kalman filter ok");
	}
	if ((solutions & SBG_ECOM_SOL_ATTITUDE_VALID)==SBG_ECOM_SOL_ATTITUDE_VALID) {
		PX4_INFO("Attitude valid");
	} else {
		PX4_INFO("Attitude invalid");
	}
	if ((solutions & SBG_ECOM_SOL_HEADING_VALID)==SBG_ECOM_SOL_HEADING_VALID) {
		PX4_INFO("Heading valid");
	} else {
		PX4_INFO("Heading invalid");
	}
	if ((solutions & SBG_ECOM_SOL_VELOCITY_VALID)==SBG_ECOM_SOL_VELOCITY_VALID) {
		PX4_INFO("Velocity valid");
	} else {
		PX4_INFO("Velocity invalid");
	}
	if ((solutions & SBG_ECOM_SOL_POSITION_VALID)==SBG_ECOM_SOL_POSITION_VALID) {
		PX4_INFO("Position valid");
	} else {
		PX4_INFO("Position invalid");
	}
	if ((solutions & SBG_ECOM_SOL_VERT_REF_USED)==SBG_ECOM_SOL_VERT_REF_USED) {
		PX4_INFO("Vertical reference used");
	} else {
		PX4_INFO("Vertical reference not used");
	}
	if ((solutions & SBG_ECOM_SOL_MAG_REF_USED)==SBG_ECOM_SOL_MAG_REF_USED) {
		PX4_INFO("Magneto used");
	} else {
		PX4_INFO("Magneto not used");
	}
	if ((solutions & SBG_ECOM_SOL_GPS1_VEL_USED)==SBG_ECOM_SOL_GPS1_VEL_USED) {
		PX4_INFO("GPS1 vel used");
	} else {
		PX4_INFO("GPS1 vel not used");
	}
	if ((solutions & SBG_ECOM_SOL_GPS1_POS_USED)==SBG_ECOM_SOL_GPS1_POS_USED) {
		PX4_INFO("GPS1 pos used");
	} else {
		PX4_INFO("GPS1 pos not used");
	}
	if ((solutions & SBG_ECOM_SOL_GPS1_HDT_USED)==SBG_ECOM_SOL_GPS1_HDT_USED) {
		PX4_INFO("GPS1 heading used");
	} else {
		PX4_INFO("GPS1 heading not used");
	}
	if ((solutions & SBG_ECOM_SOL_GPS2_VEL_USED)==SBG_ECOM_SOL_GPS2_VEL_USED) {
		PX4_INFO("GPS2 vel used");
	} else {
		PX4_INFO("GPS2 vel not used");
	}
	if ((solutions & SBG_ECOM_SOL_GPS2_POS_USED)==SBG_ECOM_SOL_GPS2_POS_USED) {
		PX4_INFO("GPS2 pos used");
	} else {
		PX4_INFO("GPS2 pos not used");
	}
	if ((solutions & SBG_ECOM_SOL_GPS2_HDT_USED)==SBG_ECOM_SOL_GPS2_HDT_USED) {
		PX4_INFO("GPS2 heading used");
	} else {
		PX4_INFO("GPS2 heading not used");
	}
	if ((solutions & SBG_ECOM_SOL_ALIGN_VALID)==SBG_ECOM_SOL_ALIGN_VALID) {
		PX4_INFO("sensor align valid");
	} else {
		PX4_INFO("sensor align not valid");
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
				      10000,
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
	sbgInterfaceInit=false;
	comHandleInit=false;
	nbEKF_QUAT=0;
	nbEKF_NAV=0;
	nbIMU_DATA=0;
	solutions=0;
}

void ModuleSBG::processSBG_EKF_QUAT(const SbgBinaryLogData *pLogData) {
	nbEKF_QUAT++;

	g_attitude.timestamp = hrt_absolute_time();

	matrix::Quatf q(pLogData->ekfQuatData.quaternion);
	q.copyTo(g_attitude.q);

	_attitude_pub.publish(g_attitude);
}

void ModuleSBG::processSBG_EKF_NAV(const SbgBinaryLogData *pLogData) {
	vehicle_global_position_s global_pos{};
	sbg_status_s sbg_status;

	nbEKF_NAV++;

	global_pos.timestamp = hrt_absolute_time();
	global_pos.lat = pLogData->ekfNavData.position[0];
	global_pos.lon = pLogData->ekfNavData.position[1];
	global_pos.alt = pLogData->ekfNavData.position[2];
	global_pos.eph = 2.0f;
	global_pos.epv = 4.0f;

	_global_pos_pub.publish(global_pos);

	double lat =global_pos.lat;
	double lon =global_pos.lon;

	global_lat=lat;
	global_lon=lon;
	solutions=pLogData->ekfNavData.status;

	sbg_status.timestamp=global_pos.timestamp;
	sbg_status.solution_status=pLogData->ekfNavData.status;
	_sbg_status_pub.publish(sbg_status);

	if (!_local_proj_inited) {
		_local_proj_inited = true;
		_local_alt0 = global_pos.alt;

		map_projection_init(&_hil_local_proj_ref, lat, lon);
	}

	float x = 0.0f;
	float y = 0.0f;
	map_projection_project(&_local_proj_ref, lat, lon, &x, &y);

	vehicle_local_position_s local_pos{};
	local_pos.timestamp = global_pos.timestamp;

	local_pos.ref_timestamp = _local_proj_ref.timestamp;
	local_pos.ref_lat = math::radians(_local_proj_ref.lat_rad);
	local_pos.ref_lon = math::radians(_local_proj_ref.lon_rad);
	local_pos.ref_alt = _local_alt0;
	local_pos.xy_valid = true;
	local_pos.z_valid = true;
	local_pos.v_xy_valid = true;
	local_pos.v_z_valid = true;
	local_pos.x = x;
	local_pos.y = y;
	local_pos.z = _local_alt0 - global_pos.alt;
	local_pos.vx = pLogData->ekfNavData.velocity[0];
	local_pos.vy = pLogData->ekfNavData.velocity[1];
	local_pos.vz = pLogData->ekfNavData.velocity[2];

	matrix::Eulerf euler{matrix::Quatf(g_attitude.q)};
	local_pos.heading = euler.psi();
	local_pos.xy_global = true;
	local_pos.z_global = true;
	local_pos.vxy_max = INFINITY;
	local_pos.vz_max = INFINITY;
	local_pos.hagl_min = INFINITY;
	local_pos.hagl_max = INFINITY;

	processLocalPosition(local_pos);

	_local_pos_pub.publish(local_pos);

}

void ModuleSBG::processSBG_IMU_DATA(const SbgBinaryLogData *pLogData) {
	nbIMU_DATA++;
	const uint64_t timestamp = hrt_absolute_time();

	/* accelerometer */

	vehicle_acceleration_s va{};
	va.timestamp = timestamp;
	va.timestamp_sample = timestamp;
	va.xyz[0]= pLogData->imuData.accelerometers[0];
	va.xyz[1]= pLogData->imuData.accelerometers[1];
	va.xyz[2]= pLogData->imuData.accelerometers[2];
	_vehicle_acceleration_pub.publish(va);


	/* gyroscope */

	vehicle_angular_velocity_s vav{};
	vav.timestamp = timestamp;
	vav.timestamp_sample = timestamp;
	vav.xyz[0]= pLogData->imuData.gyroscopes[0];
	vav.xyz[1]= pLogData->imuData.gyroscopes[1];
	vav.xyz[2]= pLogData->imuData.gyroscopes[2];
	_vehicle_angular_velocity_pub.publish(vav);

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

				/* local position + (share)*/
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

					processLocalPosition(hil_local_pos);

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

			//FOR TEST ONLY

			usleep(100);
	}
}


void ModuleSBG::prepareSBG() {
	nbEKF_QUAT=0;
	nbEKF_NAV=0;
	nbIMU_DATA=0;
	solutions=0;
	_serial_fd = ::open(PORT, O_RDWR | O_NOCTTY);

		if (_serial_fd < 0) {
			PX4_ERR("GPS: failed to open serial port: %s err: %d", PORT, errno);
			return;
		}

	SbgErrorCode			errorCode;
	errorCode = sbgInterfacePx4SerialCreate(&sbgInterface, _serial_fd, 115200);

	if (errorCode == SBG_NO_ERROR) {
		sbgInterfaceInit=true;
		errorCode = sbgEComInit(&comHandle, &sbgInterface);
		if (errorCode == SBG_NO_ERROR) {
			comHandleInit=true;

			errorCode = sbgEComCmdGetInfo(&comHandle, &deviceInfo);
			//
			// Display device information if no error
			//
			if (errorCode == SBG_NO_ERROR)
			{
				PX4_INFO("Device : %u found", deviceInfo.serialNumber);
			}
			else
			{
				PX4_ERR("SBG: Unable to get device information.");
			}

			sbgEComSetReceiveLogCallback(&comHandle, onLogReceived, this);

		} else {
			PX4_ERR("SBG:Unable to Unable to initialize the sbgECom library");
			return;
		}
	} else {
		PX4_ERR("SBG:Unable to create the SBG PX4 interface");
		return;
	}
}

void ModuleSBG::terminateSBG() {
	if (comHandleInit) {
		sbgEComClose(&comHandle);
		comHandleInit=false;
	}

	if (sbgInterfaceInit) {
		sbgInterfaceSerialDestroy(&sbgInterface);
		sbgInterfaceInit=false;
	}

	if (_serial_fd >= 0) {
		::close(_serial_fd);
		_serial_fd = -1;
	}
}

void ModuleSBG::executeSBG() {
	SbgErrorCode			errorCode;

	if(comHandleInit) {
		errorCode = sbgEComHandle(&comHandle);
		if (errorCode == SBG_NOT_READY) {
			//PX4_ERR("SBG: Not Ready");
		} else {
			PX4_INFO("SBG*");
		}
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
		prepareSBG();
		while(!should_exit()) {
			//SBG impl
			executeSBG();
			usleep(100);
		}
		terminateSBG();
	}



}
void ModuleSBG::processLocalPosition(vehicle_local_position_s &lpos) {
	vehicle_share_position_s &spos = _vehicle_share_position_pub.get();

	spos.index=sys_id;
	spos.status=vehicle_share_position_s::VSP_STATUS_COMPLETE;

	spos.timestamp = lpos.timestamp;
	spos.x = lpos.x;
	spos.y = lpos.y;
	spos.z = lpos.z;
	spos.vx = lpos.vx;
	spos.vy = lpos.vy;
	spos.vz = lpos.vz;
	spos.xy_valid = lpos.xy_valid;
	spos.z_valid = lpos.z_valid;
	spos.v_xy_valid = lpos.v_xy_valid;
	spos.v_z_valid = lpos.v_z_valid;
	spos.heading = lpos.heading;

	// publish vehicle share position data
	_vehicle_share_position_pub.update();

	pipeFuseData(lpos,spos);

}

void ModuleSBG::pipeFuseData(vehicle_local_position_s &lpos,vehicle_share_position_s &spos) {
	//---------------------------------------------------
	// PIPE: ESTIMATE ERROR
	//---------------------------------------------------
	vehicle_share_position_s _r1vsp;
	vehicle_share_position_s _r2vsp;
	vehicle_share_position_s _r3vsp;
	int nbRemoteValid=0;
	//Get remote integrale
	if (!_r1vehicle_share_position_sub.copy(&_r1vsp)) {
		_r1vsp.status=vehicle_share_position_s::VSP_STATUS_NONE;
	} else {
		if(PipeTools::isVSPValid(_r1vsp)) {
			nbRemoteValid++;
		}
	}
	if (!_r2vehicle_share_position_sub.copy(&_r2vsp)) {
		_r2vsp.status=vehicle_share_position_s::VSP_STATUS_NONE;
	}else {
		if(PipeTools::isVSPValid(_r2vsp)) {
			nbRemoteValid++;
		}
	}
	if (!_r3vehicle_share_position_sub.copy(&_r3vsp)) {
		_r3vsp.status=vehicle_share_position_s::VSP_STATUS_NONE;
	}else {
		if(PipeTools::isVSPValid(_r3vsp)) {
			nbRemoteValid++;
		}
	}
	int nbMedian=0;
	// ------------------ Z ---------------
	double xposError=0.0;
	double yposError=0.0;
	double zposError=0.0;
	double vxposError=0.0;
	double vyposError=0.0;
	double vzposError=0.0;
	double headingError=0.0;
	int32_t medianxpos,medianvxpos,medianypos,medianvypos,medianzpos,medianvzpos,medianheading;
	xposError=(double)spos.x-PipeTools::processMedianVSP(spos,_r1vsp,_r2vsp,_r3vsp,&nbMedian,
			[](const vehicle_share_position_s &r) {
				return r.x;
			},
			[](const vehicle_share_position_s &r) {
				return r.xy_valid;
			},&medianxpos);
	double xCorrection=PipeTools::processMultAndClamp(xposError,_param_ekf2_pi_mul_x.get(),_param_ekf2_pi_lim_x.get());
	lpos.x=lpos.x-(float)xCorrection;
	yposError=(double)spos.y-PipeTools::processMedianVSP(spos,_r1vsp,_r2vsp,_r3vsp,&nbMedian,
			[](const vehicle_share_position_s &r) {
				return r.y;
			},
			[](const vehicle_share_position_s &r) {
				return r.xy_valid;
			},&medianypos);
	double yCorrection=PipeTools::processMultAndClamp(yposError,_param_ekf2_pi_mul_y.get(),_param_ekf2_pi_lim_y.get());
	lpos.y=lpos.y-(float)yCorrection;
	zposError=(double)spos.z-PipeTools::processMedianVSP(spos,_r1vsp,_r2vsp,_r3vsp,&nbMedian,
			[](const vehicle_share_position_s &r) {
				return r.z;
			},
			[](const vehicle_share_position_s &r) {
				return r.z_valid;
			},&medianzpos);
	double zCorrection=PipeTools::processMultAndClamp(zposError,_param_ekf2_pi_mul_z.get(),_param_ekf2_pi_lim_z.get());
	lpos.z=lpos.z-(float)zCorrection;


	vxposError=(double)spos.vx-PipeTools::processMedianVSP(spos,_r1vsp,_r2vsp,_r3vsp,&nbMedian,
			[](const vehicle_share_position_s &r) {
				return r.vx;
			},
			[](const vehicle_share_position_s &r) {
				return r.v_xy_valid;
			},&medianvxpos);
	double vxCorrection=PipeTools::processMultAndClamp(vxposError,_param_ekf2_pi_mul_vx.get(),_param_ekf2_pi_lim_vx.get());
	lpos.vx=lpos.vx-(float)vxCorrection;
	vyposError=(double)spos.vy-PipeTools::processMedianVSP(spos,_r1vsp,_r2vsp,_r3vsp,&nbMedian,
			[](const vehicle_share_position_s &r) {
				return r.vy;
			},
			[](const vehicle_share_position_s &r) {
				return r.v_xy_valid;
			},&medianvypos);
	double vyCorrection=PipeTools::processMultAndClamp(vyposError,_param_ekf2_pi_mul_vy.get(),_param_ekf2_pi_lim_vy.get());
	lpos.vy=lpos.vy-(float)vyCorrection;
	vzposError=(double)spos.vz-PipeTools::processMedianVSP(spos,_r1vsp,_r2vsp,_r3vsp,&nbMedian,
			[](const vehicle_share_position_s &r) {
				return r.vz;
			},
			[](const vehicle_share_position_s &r) {
				return r.v_z_valid;
			},&medianvzpos);
	double vzCorrection=PipeTools::processMultAndClamp(vzposError,_param_ekf2_pi_mul_vz.get(),_param_ekf2_pi_lim_vz.get());
	lpos.vz=lpos.vz-(float)vzCorrection;

	headingError=(double)spos.heading-PipeTools::processMedianVSP(spos,_r1vsp,_r2vsp,_r3vsp,&nbMedian,
			[](const vehicle_share_position_s &r) {
				return r.heading;
			},
			[](const vehicle_share_position_s &r) {
				return true;
			},&medianheading);
	double headingCorrection=PipeTools::processMultAndClamp(headingError,_param_ekf2_pi_mul_he.get(),_param_ekf2_pi_lim_he.get());
	lpos.heading=lpos.heading-(float)headingCorrection;

	//Report
	pipepos_correction_s corr={0L,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0,0,0,0,0,0};
	corr.x_corr=(float)xCorrection;
	corr.y_corr=(float)yCorrection;
	corr.z_corr=(float)zCorrection;
	corr.vx_corr=(float)vxCorrection;
	corr.vy_corr=(float)vyCorrection;
	corr.vz_corr=(float)vzCorrection;
	corr.he_corr=(float)headingCorrection;
	corr.median_x=medianxpos;
	corr.median_y=medianypos;
	corr.median_z=medianzpos;
	corr.median_vx=medianvxpos;
	corr.median_vy=medianvypos;
	corr.median_vz=medianvzpos;
	corr.median_he=medianheading;
	_pipepos_correction_pub.publish(corr);
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
