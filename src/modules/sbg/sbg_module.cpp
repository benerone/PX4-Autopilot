#include <ecl/geo/geo.h>
#include <systemlib/px4_macros.h>
#include <px4_platform_common/time.h>
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
	case SBG_ECOM_LOG_STATUS:
		((ModuleSBG*)pUserArg)->processSBG_LOG_STATUS(pLogData);
		break;
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
	case SBG_ECOM_LOG_UTC_TIME:
		((ModuleSBG*)pUserArg)->processSBG_UTC(pLogData);
		break;
	case SBG_ECOM_LOG_AIR_DATA:
		((ModuleSBG*)pUserArg)->processSBG_AIR_DATA(pLogData);
		break;
	default:
		break;
	}

	return SBG_NO_ERROR;
}


SbgErrorCode onLogRawReceived(uint8_t * buffer,size_t bufferSize , void *pUserArg) {
	ModuleSBG* host=((ModuleSBG*)pUserArg);
	host->writeSbgRawLog(buffer,bufferSize);
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
		if (rawlog_enabled) {
			PX4_INFO("SBG: Raw log enabled %d",nbRawDataWritten);
		} else {
			PX4_INFO("SBG: Raw log disabled");
		}
	} else {
		PX4_INFO("SBG:Init Fail");
	}
	PX4_INFO("nbEKF_QUAT: %d",nbEKF_QUAT);
	PX4_INFO("nbEKF_NAV: %d",nbEKF_NAV);
	PX4_INFO("nbIMU_DATA: %d",nbIMU_DATA);
	PX4_INFO("nbLOG_STATUS: %d",nbLOG_STATUS);
	PX4_INFO("nbUTC: %d",nbUTC);
	PX4_INFO("nbAIR_DATA: %d",nbAIR_DATA);
	PX4_INFO("lat: %f",global_lat);
	PX4_INFO("lon: %f",global_lon);
	PX4_INFO("************** SBG GENERAL *************");
	PX4_INFO("Temperature: %f",(double)sbg_status.temperature);
	if ((sbg_status.general_status & SBG_ECOM_GENERAL_MAIN_POWER_OK)==SBG_ECOM_GENERAL_MAIN_POWER_OK) {
		PX4_INFO("Main Power:Ok");
	} else {
		PX4_INFO("Main Power:Fail");
	}
	if ((sbg_status.general_status & SBG_ECOM_GENERAL_IMU_POWER_OK)==SBG_ECOM_GENERAL_IMU_POWER_OK) {
		PX4_INFO("Imu Power:Ok");
	} else {
		PX4_INFO("Imu Power:Fail");
	}
	if ((sbg_status.general_status & SBG_ECOM_GENERAL_GPS_POWER_OK)==SBG_ECOM_GENERAL_GPS_POWER_OK) {
		PX4_INFO("Gps Power:Ok");
	} else {
		PX4_INFO("Gps Power:Fail");
	}
	if ((sbg_status.general_status & SBG_ECOM_GENERAL_SETTINGS_OK)==SBG_ECOM_GENERAL_SETTINGS_OK) {
		PX4_INFO("Setting:Ok");
	} else {
		PX4_INFO("Setting:Fail");
	}
	if ((sbg_status.general_status & SBG_ECOM_GENERAL_TEMPERATURE_OK)==SBG_ECOM_GENERAL_TEMPERATURE_OK) {
		PX4_INFO("Temperature:Ok");
	} else {
		PX4_INFO("Temperature:Fail");
	}
	if ((sbg_status.general_status & SBG_ECOM_GENERAL_DATALOGGER_OK)==SBG_ECOM_GENERAL_DATALOGGER_OK) {
		PX4_INFO("DataLogger:Ok");
	} else {
		PX4_INFO("DataLogger:Fail");
	}
	if ((sbg_status.general_status & SBG_ECOM_GENERAL_CPU_OK)==SBG_ECOM_GENERAL_CPU_OK) {
		PX4_INFO("CPU:Ok");
	} else {
		PX4_INFO("CPU:Fail");
	}
	PX4_INFO("************** SBG COM *************");
	if ((sbg_status.com_status & SBG_ECOM_PORTA_VALID)==SBG_ECOM_PORTA_VALID) {
		PX4_INFO("PortA:Ok");
	} else {
		PX4_INFO("PortA:Fail");
	}
	if ((sbg_status.com_status & SBG_ECOM_PORTA_RX_OK)==SBG_ECOM_PORTA_RX_OK) {
		PX4_INFO("PortA RX:Ok");
	} else {
		PX4_INFO("PortA RX:Fail");
	}
	if ((sbg_status.com_status & SBG_ECOM_PORTA_TX_OK)==SBG_ECOM_PORTA_TX_OK) {
		PX4_INFO("PortA TX:Ok");
	} else {
		PX4_INFO("PortA TX:Fail");
	}
	if ((sbg_status.com_status & SBG_ECOM_PORTB_VALID)==SBG_ECOM_PORTB_VALID) {
		PX4_INFO("PortB:Ok");
	} else {
		PX4_INFO("PortB:Fail");
	}
	if ((sbg_status.com_status & SBG_ECOM_PORTB_RX_OK)==SBG_ECOM_PORTB_RX_OK) {
		PX4_INFO("PortB RX:Ok");
	} else {
		PX4_INFO("PortB RX:Fail");
	}
	if ((sbg_status.com_status & SBG_ECOM_PORTB_TX_OK)==SBG_ECOM_PORTB_TX_OK) {
		PX4_INFO("PortB TX:Ok");
	} else {
		PX4_INFO("PortB TX:Fail");
	}
	if ((sbg_status.com_status & SBG_ECOM_PORTC_VALID)==SBG_ECOM_PORTC_VALID) {
		PX4_INFO("PortC:Ok");
	} else {
		PX4_INFO("PortC:Fail");
	}
	if ((sbg_status.com_status & SBG_ECOM_PORTC_RX_OK)==SBG_ECOM_PORTC_RX_OK) {
		PX4_INFO("PortC RX:Ok");
	} else {
		PX4_INFO("PortC RX:Fail");
	}
	if ((sbg_status.com_status & SBG_ECOM_PORTC_TX_OK)==SBG_ECOM_PORTC_TX_OK) {
		PX4_INFO("PortC TX:Ok");
	} else {
		PX4_INFO("PortC TX:Fail");
	}
	if ((sbg_status.com_status & SBG_ECOM_PORTD_VALID)==SBG_ECOM_PORTD_VALID) {
		PX4_INFO("PortD:Ok");
	} else {
		PX4_INFO("PortD:Fail");
	}
	if ((sbg_status.com_status & SBG_ECOM_PORTD_RX_OK)==SBG_ECOM_PORTD_RX_OK) {
		PX4_INFO("PortD RX:Ok");
	} else {
		PX4_INFO("PortD RX:Fail");
	}
	if ((sbg_status.com_status & SBG_ECOM_PORTD_TX_OK)==SBG_ECOM_PORTC_TX_OK) {
		PX4_INFO("PortD TX:Ok");
	} else {
		PX4_INFO("PortD TX:Fail");
	}
	if ((sbg_status.com_status & SBG_ECOM_PORTE_VALID)==SBG_ECOM_PORTE_VALID) {
		PX4_INFO("PortE:Ok");
	} else {
		PX4_INFO("PortE:Fail");
	}
	if ((sbg_status.com_status & SBG_ECOM_PORTE_RX_OK)==SBG_ECOM_PORTE_RX_OK) {
		PX4_INFO("PortE RX:Ok");
	} else {
		PX4_INFO("PortE RX:Fail");
	}
	if ((sbg_status.com_status & SBG_ECOM_PORTE_TX_OK)==SBG_ECOM_PORTE_TX_OK) {
		PX4_INFO("PortE TX:Ok");
	} else {
		PX4_INFO("PortE TX:Fail");
	}
	PX4_INFO("************** SBG AIDING *************");
	if ((sbg_status.aiding_status & SBG_ECOM_AIDING_GPS1_POS_RECV)==SBG_ECOM_AIDING_GPS1_POS_RECV) {
		PX4_INFO("GPS1 Position:Ok");
	} else {
		PX4_INFO("GPS1 Position:Fail");
	}
	if ((sbg_status.aiding_status & SBG_ECOM_AIDING_GPS1_VEL_RECV)==SBG_ECOM_AIDING_GPS1_VEL_RECV) {
		PX4_INFO("GPS1 Velocity:Ok");
	} else {
		PX4_INFO("GPS1 Velocity:Fail");
	}
	if ((sbg_status.aiding_status & SBG_ECOM_AIDING_GPS1_HDT_RECV)==SBG_ECOM_AIDING_GPS1_HDT_RECV) {
		PX4_INFO("GPS1 True Heading:Ok");
	} else {
		PX4_INFO("GPS1 True Heading:Fail");
	}
	if ((sbg_status.aiding_status & SBG_ECOM_AIDING_GPS1_UTC_RECV)==SBG_ECOM_AIDING_GPS1_UTC_RECV) {
		PX4_INFO("GPS1 UTC Time:Ok");
	} else {
		PX4_INFO("GPS1 UTC Time:Fail");
	}

	if ((sbg_status.aiding_status & SBG_ECOM_AIDING_GPS2_POS_RECV)==SBG_ECOM_AIDING_GPS2_POS_RECV) {
		PX4_INFO("GPS2 Position:Ok");
	} else {
		PX4_INFO("GPS2 Position:Fail");
	}
	if ((sbg_status.aiding_status & SBG_ECOM_AIDING_GPS2_VEL_RECV)==SBG_ECOM_AIDING_GPS2_VEL_RECV) {
		PX4_INFO("GPS2 Velocity:Ok");
	} else {
		PX4_INFO("GPS2 Velocity:Fail");
	}
	if ((sbg_status.aiding_status & SBG_ECOM_AIDING_GPS2_HDT_RECV)==SBG_ECOM_AIDING_GPS2_HDT_RECV) {
		PX4_INFO("GPS2 True Heading:Ok");
	} else {
		PX4_INFO("GPS2 True Heading:Fail");
	}
	if ((sbg_status.aiding_status & SBG_ECOM_AIDING_GPS2_UTC_RECV)==SBG_ECOM_AIDING_GPS2_UTC_RECV) {
		PX4_INFO("GPS2 UTC Time:Ok");
	} else {
		PX4_INFO("GPS2 UTC Time:Fail");
	}
	if ((sbg_status.aiding_status & SBG_ECOM_AIDING_MAG_RECV)==SBG_ECOM_AIDING_MAG_RECV) {
		PX4_INFO("Magneto:Ok");
	} else {
		PX4_INFO("Magneto:Fail");
	}
	if ((sbg_status.aiding_status & SBG_ECOM_AIDING_ODO_RECV)==SBG_ECOM_AIDING_ODO_RECV) {
		PX4_INFO("Odo:Ok");
	} else {
		PX4_INFO("Odo:Fail");
	}
	if ((sbg_status.aiding_status & SBG_ECOM_AIDING_DVL_RECV)==SBG_ECOM_AIDING_DVL_RECV) {
		PX4_INFO("Dvl:Ok");
	} else {
		PX4_INFO("Dvl:Fail");
	}
	if ((sbg_status.aiding_status & SBG_ECOM_AIDING_USBL_RECV)==SBG_ECOM_AIDING_USBL_RECV) {
		PX4_INFO("Usbl:Ok");
	} else {
		PX4_INFO("Usbl:Fail");
	}
	if ((sbg_status.aiding_status & SBG_ECOM_AIDING_DEPTH_RECV)==SBG_ECOM_AIDING_DEPTH_RECV) {
		PX4_INFO("Depth sensor:Ok");
	} else {
		PX4_INFO("Depth sensor:Fail");
	}
	if ((sbg_status.aiding_status & SBG_ECOM_AIDING_AIR_DATA_RECV)==SBG_ECOM_AIDING_AIR_DATA_RECV) {
		PX4_INFO("Air data:Ok");
	} else {
		PX4_INFO("Air data:Fail");
	}
	PX4_INFO("************** SBG SOLUTION *************");
	int smode=sbg_status.solution_status & 0b1111;

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

	if ((sbg_status.solution_status & SBG_ECOM_SOL_ATTITUDE_VALID)==SBG_ECOM_SOL_ATTITUDE_VALID) {
		PX4_INFO("Attitude valid");
	} else {
		PX4_INFO("Attitude invalid");
	}
	if ((sbg_status.solution_status & SBG_ECOM_SOL_HEADING_VALID)==SBG_ECOM_SOL_HEADING_VALID) {
		PX4_INFO("Heading valid");
	} else {
		PX4_INFO("Heading invalid");
	}
	if ((sbg_status.solution_status & SBG_ECOM_SOL_VELOCITY_VALID)==SBG_ECOM_SOL_VELOCITY_VALID) {
		PX4_INFO("Velocity valid");
	} else {
		PX4_INFO("Velocity invalid");
	}
	if ((sbg_status.solution_status & SBG_ECOM_SOL_POSITION_VALID)==SBG_ECOM_SOL_POSITION_VALID) {
		PX4_INFO("Position valid");
	} else {
		PX4_INFO("Position invalid");
	}
	if ((sbg_status.solution_status & SBG_ECOM_SOL_VERT_REF_USED)==SBG_ECOM_SOL_VERT_REF_USED) {
		PX4_INFO("Vertical reference used");
	} else {
		PX4_INFO("Vertical reference not used");
	}
	if ((sbg_status.solution_status & SBG_ECOM_SOL_MAG_REF_USED)==SBG_ECOM_SOL_MAG_REF_USED) {
		PX4_INFO("Magneto used");
	} else {
		PX4_INFO("Magneto not used");
	}
	if ((sbg_status.solution_status & SBG_ECOM_SOL_GPS1_VEL_USED)==SBG_ECOM_SOL_GPS1_VEL_USED) {
		PX4_INFO("GPS1 vel used");
	} else {
		PX4_INFO("GPS1 vel not used");
	}
	if ((sbg_status.solution_status & SBG_ECOM_SOL_GPS1_POS_USED)==SBG_ECOM_SOL_GPS1_POS_USED) {
		PX4_INFO("GPS1 pos used");
	} else {
		PX4_INFO("GPS1 pos not used");
	}
	if ((sbg_status.solution_status & SBG_ECOM_SOL_GPS1_HDT_USED)==SBG_ECOM_SOL_GPS1_HDT_USED) {
		PX4_INFO("GPS1 heading used");
	} else {
		PX4_INFO("GPS1 heading not used");
	}
	if ((sbg_status.solution_status & SBG_ECOM_SOL_GPS2_VEL_USED)==SBG_ECOM_SOL_GPS2_VEL_USED) {
		PX4_INFO("GPS2 vel used");
	} else {
		PX4_INFO("GPS2 vel not used");
	}
	if ((sbg_status.solution_status & SBG_ECOM_SOL_GPS2_POS_USED)==SBG_ECOM_SOL_GPS2_POS_USED) {
		PX4_INFO("GPS2 pos used");
	} else {
		PX4_INFO("GPS2 pos not used");
	}
	if ((sbg_status.solution_status & SBG_ECOM_SOL_GPS2_HDT_USED)==SBG_ECOM_SOL_GPS2_HDT_USED) {
		PX4_INFO("GPS2 heading used");
	} else {
		PX4_INFO("GPS2 heading not used");
	}
	if ((sbg_status.solution_status & SBG_ECOM_SOL_ALIGN_VALID)==SBG_ECOM_SOL_ALIGN_VALID) {
		PX4_INFO("sensor align valid");
	} else {
		PX4_INFO("sensor align not valid");
	}
	PX4_INFO("************** IMU DATA *************");
	if ((sbg_status.imu_status & SBG_ECOM_IMU_COM_OK)==SBG_ECOM_IMU_COM_OK) {
		PX4_INFO("Imu com ok");
	} else {
		PX4_INFO("Imu com fail");
	}
	if ((sbg_status.imu_status & SBG_ECOM_IMU_STATUS_BIT)==SBG_ECOM_IMU_STATUS_BIT) {
		PX4_INFO("Imu status ok");
	} else {
		PX4_INFO("Imu status fail");
	}
	if ((sbg_status.imu_status & SBG_ECOM_IMU_ACCEL_X_BIT)==SBG_ECOM_IMU_ACCEL_X_BIT) {
		PX4_INFO("Imu Acc X ok");
	} else {
		PX4_INFO("Imu Acc X fail");
	}
	if ((sbg_status.imu_status & SBG_ECOM_IMU_ACCEL_Y_BIT)==SBG_ECOM_IMU_ACCEL_Y_BIT) {
		PX4_INFO("Imu Acc Y ok");
	} else {
		PX4_INFO("Imu Acc Y fail");
	}
	if ((sbg_status.imu_status & SBG_ECOM_IMU_ACCEL_Z_BIT)==SBG_ECOM_IMU_ACCEL_Z_BIT) {
		PX4_INFO("Imu Acc Z ok");
	} else {
		PX4_INFO("Imu Acc Z fail");
	}
	if ((sbg_status.imu_status & SBG_ECOM_IMU_GYRO_X_BIT)==SBG_ECOM_IMU_GYRO_X_BIT) {
		PX4_INFO("Imu Gyro X ok");
	} else {
		PX4_INFO("Imu Gyro X fail");
	}
	if ((sbg_status.imu_status & SBG_ECOM_IMU_GYRO_Y_BIT)==SBG_ECOM_IMU_GYRO_Y_BIT) {
		PX4_INFO("Imu Gyro Y ok");
	} else {
		PX4_INFO("Imu Gyro Y fail");
	}
	if ((sbg_status.imu_status & SBG_ECOM_IMU_GYRO_Z_BIT)==SBG_ECOM_IMU_GYRO_Z_BIT) {
		PX4_INFO("Imu Gyro Z ok");
	} else {
		PX4_INFO("Imu Gyro Z fail");
	}
	if ((sbg_status.imu_status & SBG_ECOM_IMU_ACCELS_IN_RANGE)==SBG_ECOM_IMU_ACCELS_IN_RANGE) {
		PX4_INFO("Imu Accel in range");
	} else {
		PX4_INFO("Imu Accel out of range");
	}
	if ((sbg_status.imu_status & SBG_ECOM_IMU_GYROS_IN_RANGE)==SBG_ECOM_IMU_GYROS_IN_RANGE) {
		PX4_INFO("Imu Gyro in range");
	} else {
		PX4_INFO("Imu Gyro out of range");
	}
	PX4_INFO("************** AIR DATA *************");
	if ((_airdataStatus & SBG_ECOM_AIR_DATA_TIME_IS_DELAY)==SBG_ECOM_AIR_DATA_TIME_IS_DELAY) {
		PX4_INFO("Measurement delay information");
	} else {
		PX4_INFO("Absolute timestamp information");
	}
	if ((_airdataStatus & SBG_ECOM_AIR_DATA_PRESSURE_ABS_VALID)==SBG_ECOM_AIR_DATA_PRESSURE_ABS_VALID) {
		PX4_INFO("Absolute pressure valid");
	} else {
		PX4_INFO("Absolute pressure invalid");
	}
	if ((_airdataStatus & SBG_ECOM_AIR_DATA_ALTITUDE_VALID)==SBG_ECOM_AIR_DATA_ALTITUDE_VALID) {
		PX4_INFO("Barometric altitude valid");
	} else {
		PX4_INFO("Barometric altitude invalid");
	}
	if ((_airdataStatus & SBG_ECOM_AIR_DATA_PRESSURE_DIFF_VALID)==SBG_ECOM_AIR_DATA_PRESSURE_DIFF_VALID) {
		PX4_INFO("Differential pressure valid");
	} else {
		PX4_INFO("Differential pressure invalid");
	}
	if ((_airdataStatus & SBG_ECOM_AIR_DATA_AIRPSEED_VALID)==SBG_ECOM_AIR_DATA_AIRPSEED_VALID) {
		PX4_INFO("True airspeed valid");
	} else {
		PX4_INFO("True airspeed invalid");
	}
	if ((_airdataStatus & SBG_ECOM_AIR_DATA_TEMPERATURE_VALID)==SBG_ECOM_AIR_DATA_TEMPERATURE_VALID) {
		PX4_INFO("Temperature valid");
	} else {
		PX4_INFO("Temperature invalid");
	}
	PX4_INFO("************** Accuracy *************");
	PX4_INFO("Attitude (°) : Roll :%3.1f Pitch :%3.1f Yaw :%3.1f",(double)(sbg_status.roll_acc*180.0f/3.14159f),(double)(sbg_status.pitch_acc*180.0f/3.14159f),(double)(sbg_status.yaw_acc*180.0f/3.14159f));
	PX4_INFO("Position (m): lat %3.1f lon: %3.1f vert:%3.1f",(double)sbg_status.lat_acc,(double)sbg_status.lon_acc,(double)sbg_status.vert_acc);
	PX4_INFO("Velocity (m/s): VN %3.1f VE:%3.1f VD:%3.1f",(double)sbg_status.vel_n_acc,(double)sbg_status.vel_e_acc,(double)sbg_status.vel_d_acc);
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
				      20000,
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
	rawlog_enabled=false;
	nbEKF_QUAT=0;
	nbEKF_NAV=0;
	nbIMU_DATA=0;
	nbLOG_STATUS=0;
	nbUTC=0;
	nbAIR_DATA=0;
	nbRawDataWritten=0;
	nbAIR_DATA=0;
	_airdataStatus=0;
	deviceInfo={};
	sbg_status.solution_status=0;
	sbg_status.aiding_status=0;
	sbg_status.com_status=0;
	sbg_status.general_status=0;
	sbg_status.imu_status=0;
	sbg_status.temperature=0.0f;
}

void ModuleSBG::processSBG_EKF_QUAT(const SbgBinaryLogData *pLogData) {
	nbEKF_QUAT++;

	sbg_status.timestamp=hrt_absolute_time();
	sbg_status.solution_status=pLogData->ekfQuatData.status;
	sbg_status.roll_acc=pLogData->ekfQuatData.eulerStdDev[0];
	sbg_status.pitch_acc=pLogData->ekfQuatData.eulerStdDev[1];
	sbg_status.yaw_acc=pLogData->ekfQuatData.eulerStdDev[2];
	//_sbg_status_pub.publish(sbg_status);

	/*if ((pLogData->ekfQuatData.status & SBG_ECOM_SOL_ATTITUDE_VALID)!=SBG_ECOM_SOL_ATTITUDE_VALID) {
		return;
	}*/
	g_attitude.timestamp = hrt_absolute_time();

	matrix::Quatf q(pLogData->ekfQuatData.quaternion);
	q.copyTo(g_attitude.q);

	_attitude_pub.publish(g_attitude);
}

void ModuleSBG::processSBG_EKF_NAV(const SbgBinaryLogData *pLogData) {
	vehicle_global_position_s global_pos{};
	vehicle_gps_position_s gps_pos{};

	sbg_status.timestamp=hrt_absolute_time();
	sbg_status.solution_status=pLogData->ekfNavData.status;
	sbg_status.vel_n_acc=pLogData->ekfNavData.velocityStdDev[0];
	sbg_status.vel_e_acc=pLogData->ekfNavData.velocityStdDev[1];
	sbg_status.vel_d_acc=pLogData->ekfNavData.velocityStdDev[2];
	sbg_status.lat_acc=pLogData->ekfNavData.positionStdDev[0];
	sbg_status.lon_acc=pLogData->ekfNavData.positionStdDev[1];
	sbg_status.vert_acc=pLogData->ekfNavData.positionStdDev[2];
	//_sbg_status_pub.publish(sbg_status);

	nbEKF_NAV++;

	/*if ((pLogData->ekfQuatData.status & SBG_ECOM_SOL_ATTITUDE_VALID)!=SBG_ECOM_SOL_ATTITUDE_VALID) {
		return;
	}*/

	float eph = std::sqrtf(pLogData->ekfNavData.positionStdDev[0]*pLogData->ekfNavData.positionStdDev[0]
					+ pLogData->ekfNavData.positionStdDev[1]*pLogData->ekfNavData.positionStdDev[1]);
	float epv = pLogData->ekfNavData.positionStdDev[2];
	float evh = std::sqrtf(pLogData->ekfNavData.velocityStdDev[0]*pLogData->ekfNavData.velocityStdDev[0]
					+ pLogData->ekfNavData.velocityStdDev[1]*pLogData->ekfNavData.velocityStdDev[1]);
	float evv = pLogData->ekfNavData.velocityStdDev[2];

	global_pos.timestamp = sbg_status.timestamp;
	global_pos.lat = pLogData->ekfNavData.position[0];
	global_pos.lon = pLogData->ekfNavData.position[1];
	global_pos.alt = pLogData->ekfNavData.position[2];
	global_pos.eph = eph;
	global_pos.epv = epv;

	gps_pos.lat=(int32_t)((double)1e7)*global_pos.lat;
	gps_pos.lon=(int32_t)((double)1e7)*global_pos.lon;
	gps_pos.alt=(int32_t)(1000.0)*global_pos.alt;

	//if ((pLogData->ekfNavData.status & SBG_ECOM_SOL_POSITION_VALID)==SBG_ECOM_SOL_POSITION_VALID) {
		_global_pos_pub.publish(global_pos);
		_gps_pos_pub.publish(gps_pos);
	//}

	double lat =global_pos.lat;
	double lon =global_pos.lon;

	global_lat=lat;
	global_lon=lon;


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
	local_pos.ref_lat = math::degrees(_local_proj_ref.lat_rad);
	local_pos.ref_lon = math::degrees(_local_proj_ref.lon_rad);
	local_pos.ref_alt = _local_alt0;
	/*if ((pLogData->ekfNavData.status & SBG_ECOM_SOL_POSITION_VALID)!=SBG_ECOM_SOL_POSITION_VALID) {
		local_pos.xy_valid = false;
		local_pos.z_valid = false;
	} else {*/
		local_pos.xy_valid = true;
		local_pos.z_valid = true;
	//}
	/*if ((pLogData->ekfNavData.status & SBG_ECOM_SOL_VELOCITY_VALID)!=SBG_ECOM_SOL_VELOCITY_VALID) {
		local_pos.v_xy_valid = false;
		local_pos.v_z_valid = false;
	} else {*/
		local_pos.v_xy_valid = true;
		local_pos.v_z_valid = true;
	//}

	local_pos.x = x;
	local_pos.y = y;
	local_pos.z = _local_alt0 - global_pos.alt;
	local_pos.delta_xy[0] = INFINITY;
	local_pos.delta_xy[1] = INFINITY;
	local_pos.delta_z = INFINITY;
	local_pos.vx = pLogData->ekfNavData.velocity[0];
	local_pos.vy = pLogData->ekfNavData.velocity[1];
	local_pos.vz = pLogData->ekfNavData.velocity[2];
	local_pos.z_deriv = INFINITY;

	local_pos.ax = INFINITY;
	local_pos.ay = INFINITY;
	local_pos.az = INFINITY;

	matrix::Eulerf euler{matrix::Quatf(g_attitude.q)};
	/*if ((pLogData->ekfNavData.status & SBG_ECOM_SOL_HEADING_VALID)!=SBG_ECOM_SOL_HEADING_VALID) {
		local_pos.heading = 0.0f;
	} else {*/
		local_pos.heading = euler.psi();
	//}
	local_pos.delta_heading = INFINITY;

	if ((pLogData->ekfNavData.status & SBG_ECOM_SOL_POSITION_VALID)!=SBG_ECOM_SOL_POSITION_VALID) {
		local_pos.xy_global = false;
		local_pos.z_global = false;
	} else {
		local_pos.xy_global = true;
		local_pos.z_global = true;
	}

	local_pos.vxy_max = INFINITY;
	local_pos.vz_max = INFINITY;
	local_pos.hagl_min = INFINITY;
	local_pos.hagl_max = INFINITY;

	local_pos.dist_bottom = INFINITY;
	local_pos.dist_bottom_valid = false;

	local_pos.eph = eph;
	local_pos.epv = epv;
	local_pos.evh = evh;
	local_pos.evv = evv;

	local_pos.xy_reset_counter = 0;
	local_pos.z_reset_counter = 0;
	local_pos.vxy_reset_counter = 0;
	local_pos.vz_reset_counter = 0;
	local_pos.heading_reset_counter = 0;

	processLocalPosition(local_pos);

	_local_pos_pub.publish(local_pos);

}

void ModuleSBG::processSBG_IMU_DATA(const SbgBinaryLogData *pLogData) {
	nbIMU_DATA++;
	const uint64_t timestamp = hrt_absolute_time();

	sbg_status.timestamp=timestamp;
	sbg_status.imu_status=pLogData->imuData.status;
	sbg_status.temperature=pLogData->imuData.temperature;
	_sbg_status_pub.publish(sbg_status);

	/*if ((sbg_status.imu_status & SBG_ECOM_IMU_COM_OK)!=SBG_ECOM_IMU_COM_OK) {
		return;
	}
	if ((sbg_status.imu_status & SBG_ECOM_IMU_STATUS_BIT)!=SBG_ECOM_IMU_STATUS_BIT) {
		return;
	}*/



	/* accelerometer */
	/*if (((sbg_status.imu_status & SBG_ECOM_IMU_ACCELS_IN_RANGE)==SBG_ECOM_IMU_ACCELS_IN_RANGE)
		&& ((sbg_status.imu_status & SBG_ECOM_IMU_ACCEL_X_BIT)==SBG_ECOM_IMU_ACCEL_X_BIT)
		&& ((sbg_status.imu_status & SBG_ECOM_IMU_ACCEL_Y_BIT)==SBG_ECOM_IMU_ACCEL_Y_BIT)
		&& ((sbg_status.imu_status & SBG_ECOM_IMU_ACCEL_Z_BIT)==SBG_ECOM_IMU_ACCEL_Z_BIT)) {*/
		vehicle_acceleration_s va{};
		va.timestamp = timestamp;
		va.timestamp_sample = timestamp;
		va.xyz[0]= pLogData->imuData.accelerometers[0];
		va.xyz[1]= pLogData->imuData.accelerometers[1];
		va.xyz[2]= pLogData->imuData.accelerometers[2];
		_vehicle_acceleration_pub.publish(va);

	/*}*/
	/* gyroscope */
	/*if (((sbg_status.imu_status & SBG_ECOM_IMU_GYROS_IN_RANGE)==SBG_ECOM_IMU_GYROS_IN_RANGE)
		&& ((sbg_status.imu_status & SBG_ECOM_IMU_GYRO_X_BIT)==SBG_ECOM_IMU_GYRO_X_BIT)
		&& ((sbg_status.imu_status & SBG_ECOM_IMU_GYRO_Y_BIT)==SBG_ECOM_IMU_GYRO_Y_BIT)
		&& ((sbg_status.imu_status & SBG_ECOM_IMU_GYRO_Z_BIT)==SBG_ECOM_IMU_GYRO_Z_BIT)) {*/
			vehicle_angular_velocity_s vav{};
			vav.timestamp = timestamp;
			vav.timestamp_sample = timestamp;
			vav.xyz[0]= pLogData->imuData.gyroscopes[0];
			vav.xyz[1]= pLogData->imuData.gyroscopes[1];
			vav.xyz[2]= pLogData->imuData.gyroscopes[2];
			_vehicle_angular_velocity_pub.publish(vav);
	//}

}

void ModuleSBG::processSBG_LOG_STATUS(const SbgBinaryLogData *pLogData) {
	nbLOG_STATUS++;
	sbg_status.timestamp=hrt_absolute_time();
	sbg_status.general_status=pLogData->statusData.generalStatus;
	sbg_status.com_status=pLogData->statusData.comStatus;
	sbg_status.aiding_status=pLogData->statusData.aidingStatus;
	//_sbg_status_pub.publish(sbg_status);
}

void ModuleSBG::processSBG_UTC(const SbgBinaryLogData *pLogData) {
	nbUTC++;
	//uint16 status=pLogData->utcData.status;
	//uint16 status_utc=status>>SBG_ECOM_CLOCK_UTC_STATUS_SHIFT;
	//if (status_utc==SBG_ECOM_UTC_VALID) {
		sbg_utc_s sbg_utc;
		sbg_utc.timestamp=hrt_absolute_time();
		sbg_utc.year=pLogData->utcData.year;
		sbg_utc.month=pLogData->utcData.month;
		sbg_utc.day=pLogData->utcData.day;
		sbg_utc.hour=pLogData->utcData.hour;
		sbg_utc.minute=pLogData->utcData.minute;
		sbg_utc.second=pLogData->utcData.second;
		_sbg_utc_pub.publish(sbg_utc);
	//}
}

void ModuleSBG::processSBG_AIR_DATA(const SbgBinaryLogData *pLogData) {
	nbAIR_DATA++;
	const uint64_t timestamp = hrt_absolute_time();
	_airdataStatus=pLogData->airData.status;
	vehicle_air_data_s airdata{};
	airdata.timestamp=timestamp;
	airdata.timestamp_sample=timestamp;
	airdata.baro_device_id=0;
	airdata.baro_alt_meter=pLogData->airData.altitude;
	airdata.baro_temp_celcius=pLogData->airData.airTemperature;
	airdata.baro_pressure_pa=pLogData->airData.pressureAbs;
	airdata.rho=0;
	_vehicle_air_data_pub.publish(airdata);
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
					//vehicle_gps_position_s gps_pos{};


					hil_global_pos.timestamp = timestamp;
					hil_global_pos.lat = hil_state.lat / ((double)1e7);
					hil_global_pos.lon = hil_state.lon / ((double)1e7);
					hil_global_pos.alt = hil_state.alt / 1000.0f;
					hil_global_pos.eph = 2.0f;
					hil_global_pos.epv = 4.0f;

					//gps_pos.lat=hil_state.lat;
					//gps_pos.lon=hil_state.lon;
					//gps_pos.alt=hil_state.alt;

					_global_pos_pub.publish(hil_global_pos);
					//_gps_pos_pub.publish(gps_pos);


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

					matrix::Eulerf euler{matrix::Quatf(hil_state.attitude_quaternion)};

					vehicle_local_position_s hil_local_pos{};

					hil_local_pos.timestamp = timestamp;
					hil_local_pos.ref_timestamp = _hil_local_proj_ref.timestamp;
					hil_local_pos.ref_lat = math::degrees(_hil_local_proj_ref.lat_rad);
					hil_local_pos.ref_lon = math::degrees(_hil_local_proj_ref.lon_rad);
					hil_local_pos.ref_alt = _hil_local_alt0;
					hil_local_pos.x = x;
					hil_local_pos.y = y;
					hil_local_pos.z = _hil_local_alt0 - hil_state.alt / 1000.0f;
					hil_local_pos.delta_xy[0] = INFINITY;
					hil_local_pos.delta_xy[1] = INFINITY;
					hil_local_pos.delta_z = INFINITY;
					hil_local_pos.vx = hil_state.vx / 100.0f;
					hil_local_pos.vy = hil_state.vy / 100.0f;
					hil_local_pos.vz = hil_state.vz / 100.0f;
					hil_local_pos.z_deriv = INFINITY;
					hil_local_pos.ax = INFINITY;
					hil_local_pos.ay = INFINITY;
					hil_local_pos.az = INFINITY;
					hil_local_pos.heading = euler.psi();
					hil_local_pos.delta_heading = INFINITY;
					hil_local_pos.dist_bottom = INFINITY;
					hil_local_pos.eph = 0.0;
					hil_local_pos.epv = 0.0;
					hil_local_pos.evh = 0.0;
					hil_local_pos.evv = 0.0;
					hil_local_pos.vxy_max = INFINITY;
					hil_local_pos.vz_max = INFINITY;
					hil_local_pos.hagl_min = INFINITY;
					hil_local_pos.hagl_max = INFINITY;

					hil_local_pos.xy_valid = true;
					hil_local_pos.z_valid = true;
					hil_local_pos.v_xy_valid = true;
					hil_local_pos.v_z_valid = true;

					hil_local_pos.xy_reset_counter = 0;
					hil_local_pos.z_reset_counter = 0;
					hil_local_pos.vxy_reset_counter = 0;
					hil_local_pos.vz_reset_counter = 0;
					hil_local_pos.heading_reset_counter = 0;

					hil_local_pos.xy_global = true;
					hil_local_pos.z_global = true;
					hil_local_pos.dist_bottom_valid = false;

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
			if (enable_sbg_in_hil) {
				executeSBG();
			}
			usleep(100);
	}
}


void ModuleSBG::writeSbgRawLog(uint8_t * buffer,size_t bufferSize) {

	//TODO
	if (rawlog_enabled && (_fd >= 0) && bufferSize>0) {
		ssize_t ret = ::write(_fd, buffer, bufferSize);
		if (ret<0) {
			PX4_ERR("SBG: Unable to write raw data, disabling raw log");
			rawlog_enabled=false;
			::close(_fd);
			_fd = -1;
		} else {
			nbRawDataWritten+=bufferSize;
			::fsync(_fd);
		}
	}
}

void ModuleSBG::prepareSBG() {
	usleep(10000);
	nbEKF_QUAT=0;
	nbEKF_NAV=0;
	nbIMU_DATA=0;
	nbLOG_STATUS=0;
	nbUTC=0;
	nbAIR_DATA=0;
	nbRawDataWritten=0;
	sbg_status.solution_status=0;
	sbg_status.aiding_status=0;
	sbg_status.com_status=0;
	sbg_status.general_status=0;
	sbg_status.imu_status=0;
	sbg_status.temperature=0.0f;
	global_lat=0.0;
	global_lon=0.0;
	deviceInfo={};
	_serial_fd = ::open(PORT, O_RDWR | O_NOCTTY);

		if (_serial_fd < 0) {
			PX4_ERR("GPS: failed to open serial port: %s err: %d", PORT, errno);
			return;
		}

	SbgErrorCode			errorCode;
	errorCode = sbgInterfacePx4SerialCreate(&sbgInterface, _serial_fd, 921600);
	sbgInterfacePx4SerialSetReadRawCallback(&sbgInterface,onLogRawReceived,this);


	if (rawlog_enabled) {
		//TODO open raw file, if fail disable rawlog
		time_t utc_time_sec;

		struct timespec ts = {};
		px4_clock_gettime(CLOCK_REALTIME, &ts);
		utc_time_sec = ts.tv_sec + (ts.tv_nsec / 1e9);

		tm tt = {};
		gmtime_r(&utc_time_sec, &tt);
		char log_file_name_time[16] = "";
		strftime(log_file_name_time, sizeof(log_file_name_time), "%H_%M_%S", &tt);
		char log_file_name[64];
		snprintf(log_file_name, 64, "/fs/microsd/%d_%s.sbg.bin",sys_id,log_file_name_time);
		_fd = ::open(log_file_name, O_CREAT | O_WRONLY, PX4_O_MODE_666);
		if (_fd < 0) {
			PX4_ERR("Can't open sbg raw file %s, errno: %d", log_file_name, errno);
			rawlog_enabled=false;
		}

	}

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
				sbg_status.serial_number=deviceInfo.serialNumber;
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
	if (rawlog_enabled && (_fd >= 0)) {
		//TODO close raw file
		::close(_fd);
		_fd = -1;
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
	rawlog_enabled=false;//(_param_sbg_enable_rawlog.get()!=0);
	enable_sbg_in_hil=(_param_sbg_enable_hil.get()!=0);



	if (hil_mode) {
		if (enable_sbg_in_hil) {
			prepareSBG();
		}
		processHIL();
		if (enable_sbg_in_hil) {
			terminateSBG();
		}
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
