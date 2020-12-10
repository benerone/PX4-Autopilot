
#include <pthread.h>

/*static pthread_mutex_t mut;
pthread_mutex_init(&mut, NULL);
pthread_mutex_lock(&mut);
pthread_mutex_unlock(&mut);*/

#include "can_integrale_module.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

using namespace time_literals;

typedef UAVCAN_DRIVER::CanInitHelper<> CanInitHelper;

typedef struct {
	float32 v1;
	float32 v2;
} IntegralCanData;

#define OFFSET_PITCH_ROLL 	0
#define OFFSET_YOW 		8


int ModuleCanIntegrale::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module
	PX4_INFO("RxNb:%d TxNb:%d ri=%lf pi=%lf yi=%lf",nbReceived,nbEmitted,(double)r_integrale.pitch_rate_integral,(double)r_integrale.roll_rate_integral,(double)r_integrale.yaw_rate_integral);
	PX4_INFO("RxNbE:%d TxNbE:%d",nbReceivedError,nbEmittedError);
	return 0;
}

int ModuleCanIntegrale::custom_command(int argc, char *argv[])
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


int ModuleCanIntegrale::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("can_integrale",
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

ModuleCanIntegrale *ModuleCanIntegrale::instantiate(int argc, char *argv[])
{
	ModuleCanIntegrale *instance = new ModuleCanIntegrale();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

ModuleCanIntegrale::ModuleCanIntegrale()
	: ModuleParams(nullptr)
{
}

void ModuleCanIntegrale::run()
{
	//Mavlink id serve as id
	int32_t sys_id=_param_mav_sys_id.get();
	PX4_INFO("canid %d",sys_id);
	nbReceived=0;
	nbEmitted=0;
	nbReceivedError=0;
	nbEmittedError=0;
	postYow=false;
	//Can start if necessary
	static CanInitHelper *can = nullptr;
	if (can == nullptr) {

		can = new CanInitHelper();

		if (can == nullptr) {                    // We don't have exceptions so bad_alloc cannot be thrown
			PX4_ERR("Out of memory");
			return;
		}
		uavcan::uint32_t bitrate=125000; //1000000;
		const int can_init_res = can->init(bitrate);

		if (can_init_res < 0) {
			PX4_ERR("CAN driver init failed %i", can_init_res);
			return;
		}
	}

	uavcan::ICanIface * iFace=can->driver.getIface(0);

	while(!should_exit()) {
		//Get can interface

		//TODO
		integrale_s integrale;
		IntegralCanData tmp;


		if (_integrale_sub.update(&integrale)) {
			//Publish on can
			tmp.v1=integrale.pitch_rate_integral;
			tmp.v2=integrale.roll_rate_integral;
			uavcan::CanFrame canFrame=uavcan::CanFrame(sys_id | OFFSET_PITCH_ROLL,(const uavcan::uint8_t *)&tmp,8);
			auto sendResult=iFace->send(canFrame,uavcan::MonotonicTime::fromUSec(20000),0);
			if (sendResult==0) {
				//PX4_ERR("CAN driver TX full");
				nbEmittedError++;
			}
			if (sendResult<0) {
				PX4_ERR("CAN driver TX error");
				nbEmittedError++;
			}
			if (sendResult==1) {
				nbEmitted++;
			}
			yowIntegraleValue=integrale.yaw_rate_integral;
			postYow=true;
			usleep(1000);

		} else {
			if(postYow) {
				postYow=false;
				tmp.v1=yowIntegraleValue;
				tmp.v2=0.0f;
				uavcan::CanFrame canFrameY=uavcan::CanFrame(sys_id | OFFSET_YOW,(const uavcan::uint8_t *)&tmp,8);
				auto sendResultY=iFace->send(canFrameY,uavcan::MonotonicTime::fromUSec(20000),0);
				if (sendResultY==0) {
					//PX4_ERR("CAN driver TX full");
					nbEmittedError++;
				}
				if (sendResultY<0) {
					PX4_ERR("CAN driver TX error");
					nbEmittedError++;
				}
				if (sendResultY==1) {
					nbEmitted++;
				}
			}
		}

		//Check for new incoming can msg
		uavcan::int16_t receiveResult=1;
		while (receiveResult>0)  {
			uavcan::MonotonicTime mtime;
			uavcan::UtcTime out_ts_utc;
			uavcan::CanIOFlags out_flags;
			uavcan::CanFrame canFrame;
			receiveResult=iFace->receive(canFrame,mtime,out_ts_utc,out_flags);
			if (receiveResult==0) {
				//PX4_ERR("CAN driver RX empty");
				//nbReceivedError++;
			}
			if (receiveResult<0) {
				PX4_ERR("CAN driver RX error");
				nbReceivedError++;
			}
			if (receiveResult==1) {
				//Process received
				IntegralCanData *tmpp=(IntegralCanData *)canFrame.data;
				if ((canFrame.id & OFFSET_YOW) == OFFSET_YOW) {
					r_integrale.yaw_rate_integral=tmpp->v1;
				} else {
					r_integrale.pitch_rate_integral=tmpp->v1;
					r_integrale.roll_rate_integral=tmpp->v2;
				}

				nbReceived++;
			}
		}
		//sleep
		usleep(20000); //50Hz
	}


	//Can stop


}

int ModuleCanIntegrale::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Can module to share integrale
### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ can_integrale start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("can_integrale", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	//PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	//PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int can_integrale_main(int argc, char *argv[])
{
	return ModuleCanIntegrale::main(argc, argv);
}
