
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
#define OFFSET_YOW 		16
#define OFFSET_CORR_PITCH_ROLL 	32
#define OFFSET_CORR_YOW 	64
#define MASK_ID 		0b111
#define MAX_DELAY_REFRESH	500000
//Time Cycle in us
//#define TIME_CYCLE 		1160
#define TIME_CYCLE 		2000

int ModuleCanIntegrale::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module
	PX4_INFO("RxNb:%d TxNb:%d",nbReceived,nbEmitted);
	PX4_INFO("R1 Nb:%d ri=%lf pi=%lf yi=%lf status=%d",nbReceivedR1,(double)r1_integrale.pitch_rate_integral,(double)r1_integrale.roll_rate_integral,(double)r1_integrale.yaw_rate_integral,(int)r1_integrale.status);
	PX4_INFO("R2 Nb:%d ri=%lf pi=%lf yi=%lf status=%d",nbReceivedR2,(double)r2_integrale.pitch_rate_integral,(double)r2_integrale.roll_rate_integral,(double)r2_integrale.yaw_rate_integral,(int)r2_integrale.status);
	PX4_INFO("R3 Nb:%d ri=%lf pi=%lf yi=%lf status=%d",nbReceivedR3,(double)r3_integrale.pitch_rate_integral,(double)r3_integrale.roll_rate_integral,(double)r3_integrale.yaw_rate_integral,(int)r3_integrale.status);
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
	nbReceivedR1=0;
	nbReceivedR2=0;
	nbReceivedR3=0;
	nbEmitted=0;
	nbReceivedError=0;
	nbEmittedError=0;
	int32_t cycle=0;
	postYow=false;
	postCorrection=false;

	r1_integrale={0L,0.0,0.0,0.0,integrale_s::INTEGRALE_STATUS_NONE};
	r2_integrale={0L,0.0,0.0,0.0,integrale_s::INTEGRALE_STATUS_NONE};
	r3_integrale={0L,0.0,0.0,0.0,integrale_s::INTEGRALE_STATUS_NONE};
	_r1integrale_pub.publish(r1_integrale);
	_r2integrale_pub.publish(r2_integrale);
	_r3integrale_pub.publish(r3_integrale);
	rx_integrales[0]=&r1_integrale;
	rx_integrales[1]=&r2_integrale;
	rx_integrales[2]=&r3_integrale;


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

	usleep(sys_id*TIME_CYCLE*4); //id*packet_transfert_delay*nbpacket (2I and 2C)

	while(!should_exit()) {

		integrale_s integrale;
		pipe_correction_s pipe_correction;
		IntegralCanData tmp;

		auto startTime=hrt_absolute_time();
		//Transmit
		switch(cycle) {
			case 0: //I Pitch Roll
			if (_integrale_sub.updated()) {
				_integrale_sub.copy(&integrale);
				tmp.v1=integrale.pitch_rate_integral;
				tmp.v2=integrale.roll_rate_integral;
				yowIntegraleValue=integrale.yaw_rate_integral;
				if (sendFrame(iFace,sys_id | OFFSET_PITCH_ROLL,(const uavcan::uint8_t *)&tmp,8)) {
					cycle++;
					postYow=true;
				}
			} else {
				cycle++;
			}
			break;
			case 1: //I Yow
			if (postYow) {
				tmp.v1=yowIntegraleValue;
				tmp.v2=0.0f;
				if (sendFrame(iFace,sys_id | OFFSET_YOW,(const uavcan::uint8_t *)&tmp,8)) {
					cycle++;
					postYow=false;
				}
			} else {
				cycle++;
			}
			break;
			case 2: //C Pitch Roll
			if (_pipe_correction_sub.updated()) {
				_pipe_correction_sub.copy(&pipe_correction);
				tmp.v1=pipe_correction.pitch_correction;
				tmp.v2=pipe_correction.roll_correction;
				yowPipeCorrectionValue=pipe_correction.yaw_correction;
				nbMedianValue=pipe_correction.nb_median;
				if (sendFrame(iFace,sys_id | OFFSET_CORR_PITCH_ROLL,(const uavcan::uint8_t *)&tmp,8)) {
					cycle++;
					postCorrection=true;
				}
			} else {
				cycle++;
			}
			break;
			case 3: //C Yow
			if (postCorrection) {
				tmp.v1=yowPipeCorrectionValue;
				tmp.v2=nbMedianValue;
				if (sendFrame(iFace,sys_id | OFFSET_CORR_YOW,(const uavcan::uint8_t *)&tmp,8)) {
					cycle++;
					postCorrection=false;
				}
			} else {
				cycle++;
			}
			break;
			default:
				cycle++;
				if (cycle%16==0) {
					cycle=0;
				}
			break;
		}

		//Receive
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
				int32_t id=(int32_t)(canFrame.id & MASK_ID);
				if (id<5) {
					bool isValid=true;
					auto offset=id;
					if (id<sys_id) {
						offset=id-1;
					} else {
						if (id>sys_id)  {
							offset=id-2;
						} else {
							PX4_ERR("Duplicate id in use for can");
							isValid=false;
						}
					}
					if (isValid) {
						//PX4_INFO(" can id %x",canFrame.id);
						if ((canFrame.id & OFFSET_YOW) == OFFSET_YOW) {
							//PX4_INFO(" offset yow id %x status:%d",canFrame.id,(int)rx_integrales[offset]->status);
							rx_integrales[offset]->yaw_rate_integral=tmpp->v1;
							if (rx_integrales[offset]->status==integrale_s::INTEGRALE_STATUS_PARTIAL) {
								rx_integrales[offset]->status=	integrale_s::INTEGRALE_STATUS_COMPLETE;
								if (offset==0) {
									r1_integrale.timestamp=hrt_absolute_time();
									_r1integrale_pub.publish(r1_integrale);
									nbReceivedR1++;
								}
								if (offset==1) {
									r2_integrale.timestamp=hrt_absolute_time();
									_r2integrale_pub.publish(r2_integrale);
									nbReceivedR2++;
								}
								if (offset==2) {
									r3_integrale.timestamp=hrt_absolute_time();
									_r3integrale_pub.publish(r3_integrale);
									nbReceivedR3++;
								}
							}
							rx_integrales[offset]->status=	integrale_s::INTEGRALE_STATUS_NONE;
						} else {
							if (((canFrame.id & OFFSET_CORR_PITCH_ROLL) == 0) && ((canFrame.id & OFFSET_CORR_YOW) == 0)) {
								//PX4_INFO(" offset pitch-roll id %x status:%d",canFrame.id,(int)rx_integrales[offset]->status);
								rx_integrales[offset]->pitch_rate_integral=tmpp->v1;
								rx_integrales[offset]->roll_rate_integral=tmpp->v2;
								rx_integrales[offset]->status=integrale_s::INTEGRALE_STATUS_PARTIAL;
							}

						}
					}

				} else {
					PX4_ERR("Invalid can id %x",canFrame.id);
					PX4_ERR("Invalid id %x",id);
				}


				nbReceived++;
			}
		}
		//Check unrefresh
		uint64_t currentTime=hrt_absolute_time();
		if (currentTime-r1_integrale.timestamp>MAX_DELAY_REFRESH) {
			r1_integrale.status=integrale_s::INTEGRALE_STATUS_NONE;
			_r1integrale_pub.publish(r1_integrale);
		}
		if (currentTime-r2_integrale.timestamp>MAX_DELAY_REFRESH) {
			r2_integrale.status=integrale_s::INTEGRALE_STATUS_NONE;
			_r2integrale_pub.publish(r2_integrale);
		}
		if (currentTime-r3_integrale.timestamp>MAX_DELAY_REFRESH) {
			r3_integrale.status=integrale_s::INTEGRALE_STATUS_NONE;
			_r3integrale_pub.publish(r3_integrale);
		}

		int64_t deltaTime=((int64_t)TIME_CYCLE)-((int64_t)(currentTime-startTime));
		if (deltaTime<0) {
			deltaTime=0;
		}

		//sleep
		usleep(deltaTime);
	}


	//Can stop


}
bool ModuleCanIntegrale::sendFrame(uavcan::ICanIface * iFace,uint32_t can_id, const uint8_t* can_data, uint8_t data_len) {
	uavcan::CanFrame canFrame=uavcan::CanFrame(can_id,can_data,data_len);
	//PX4_INFO("send pr can id %x",canFrame.id);
	auto sendResult=iFace->send(canFrame,uavcan::MonotonicTime::fromUSec(20000),0);
	if (sendResult==0) {
		//PX4_ERR("CAN driver TX full");
		nbEmittedError++;
		return false;
	}
	if (sendResult<0) {
		PX4_ERR("CAN driver TX error");
		nbEmittedError++;
		return false;
	}
	if (sendResult==1) {
		nbEmitted++;
	}
	return true;
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
