
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
typedef struct {
	int32_t v1;
	int32_t v2;
} IntegralCanDataInt;
typedef struct {
	int32_t v1;
	float32 v2;
} IntegralCanDataMix;


#define OFFSET_PITCH_ROLL 	1
#define OFFSET_YOW 		2
#define OFFSET_VXY 		3
#define OFFSET_THR_ACT	 	4
#define OFFSET_POSXY	 	5
#define OFFSET_POSZ_VELZ	6
#define OFFSET_VELXY	 	7
#define OFFSET_STATUS	 	8
#define OFFSET_ACT_1	 	9
#define MASK_ID 		0b111
#define TYPE_SHIFT		3
#define MAX_DELAY_REFRESH	500000

#define VALID_XY		1
#define VALID_Z			2
#define VALID_VXY		4
#define VALID_VZ		8

//Time Cycle in us
//#define TIME_CYCLE 		1160
#define TIME_CYCLE 		2000

int ModuleCanIntegrale::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module
	PX4_INFO("Can0 RxNb:%d TxNb:%d",nbReceived[0],nbEmitted[0]);
	PX4_INFO("Can1 RxNb:%d TxNb:%d",nbReceived[1],nbEmitted[1]);
	PX4_INFO("R1 Nb:%d ri=%lf pi=%lf yi=%lf status=%d",nbReceivedR1,(double)r1_integrale.pitch_rate_integral,(double)r1_integrale.roll_rate_integral,(double)r1_integrale.yaw_rate_integral,(int)r1_integrale.status);
	PX4_INFO("R2 Nb:%d ri=%lf pi=%lf yi=%lf status=%d",nbReceivedR2,(double)r2_integrale.pitch_rate_integral,(double)r2_integrale.roll_rate_integral,(double)r2_integrale.yaw_rate_integral,(int)r2_integrale.status);
	PX4_INFO("R3 Nb:%d ri=%lf pi=%lf yi=%lf status=%d",nbReceivedR3,(double)r3_integrale.pitch_rate_integral,(double)r3_integrale.roll_rate_integral,(double)r3_integrale.yaw_rate_integral,(int)r3_integrale.status);
	PX4_INFO("Can0 RxNbE:%d TxNbE:%d %s",nbReceivedError[0],nbEmittedError[0],can_status.can_error_1?"Error":"Ok");
	PX4_INFO("Can1 RxNbE:%d TxNbE:%d %s",nbReceivedError[1],nbEmittedError[1],can_status.can_error_2?"Error":"Ok");
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
	sys_id=_param_mav_sys_id.get();
	PX4_INFO("canid %d",sys_id);
	nbReceived[0]=0;
	nbReceived[1]=0;
	nbReceivedR1=0;
	nbReceivedR2=0;
	nbReceivedR3=0;
	nbEmitted[0]=0;
	nbEmitted[1]=0;
	nbReceivedError[0]=0;
	nbReceivedError[1]=0;
	nbEmittedError[0]=0;
	nbEmittedError[1]=0;
	int32_t cycle=0;
	postYow=false;
	postVxy=false;
	postThrAct=false;
	int32_t countIt=0;

	can_status={0L,0,0,0,0,0,0,0,0,0,0,0,false,false};

	r1_integrale={0L,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0,integrale_s::INTEGRALE_STATUS_NONE};
	r2_integrale={0L,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0,integrale_s::INTEGRALE_STATUS_NONE};
	r3_integrale={0L,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0,integrale_s::INTEGRALE_STATUS_NONE};
	_r1integrale_pub.publish(r1_integrale);
	_r2integrale_pub.publish(r2_integrale);
	_r3integrale_pub.publish(r3_integrale);
	rx_integrales[0]=&r1_integrale;
	rx_integrales[1]=&r2_integrale;
	rx_integrales[2]=&r3_integrale;
	r1_shpos={0L,0.0,0.0,0.0,0.0,0.0,0.0,false,false,false,false,0,vehicle_share_position_s::VSP_STATUS_NONE};
	r2_shpos={0L,0.0,0.0,0.0,0.0,0.0,0.0,false,false,false,false,0,vehicle_share_position_s::VSP_STATUS_NONE};
	r3_shpos={0L,0.0,0.0,0.0,0.0,0.0,0.0,false,false,false,false,0,vehicle_share_position_s::VSP_STATUS_NONE};
	_r1vehicle_share_position_pub.publish(r1_shpos);
	_r2vehicle_share_position_pub.publish(r2_shpos);
	_r3vehicle_share_position_pub.publish(r3_shpos);
	rx_shpos[0]=&r1_shpos;
	rx_shpos[1]=&r2_shpos;
	rx_shpos[2]=&r3_shpos;
	rx_act[0]=&act_r1;
	rx_act[1]=&act_r2;
	rx_act[2]=&act_r3;
	for(int i=0;i<3;i++) {
		rx_act[i]->timestamp=0L;
		for(int j=0;j<4;j++) {
			rx_act[i]->control[j]=0.0f;
		}
		rx_act[i]->index=0;
		rx_act[i]->status=actuator_controls_re_s::ACT_STATUS_NONE;
	}
	_r1act_pub.publish(act_r1);
	_r2act_pub.publish(act_r2);
	_r3act_pub.publish(act_r3);
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
		} else {

		}
	}

	iFace=can->driver.getIface(0);
	iFace2=can->driver.getIface(1);

	usleep(sys_id*TIME_CYCLE*9); //id*packet_transfert_delay*nbpacket (2I and 2C)

	while(!should_exit()) {

		integrale_s integrale;
		vehicle_share_position_s vehicle_share_position;
		//pipe_correction_s pipe_correction;
		IntegralCanData tmp;
		IntegralCanDataMix tmpm;
		bool resultSend,resultSend2;

		auto startTime=hrt_absolute_time();
		//Transmit
		switch(cycle) {
			case 0: //I Pitch Roll
			if (_integrale_sub.updated()) {
				_integrale_sub.copy(&integrale);
				tmp.v1=integrale.pitch_rate_integral;
				tmp.v2=integrale.roll_rate_integral;
				yowIntegraleValue=integrale.yaw_rate_integral;
				thrustValue=integrale.thrust;
				vxValue=integrale.vx;
				vyValue=integrale.vy;
				thrAct=integrale.throttle_act;
				resultSend=sendFrame(iFace,sys_id | (OFFSET_PITCH_ROLL<<TYPE_SHIFT),(const uavcan::uint8_t *)&tmp,8);
				resultSend2=sendFrame(iFace2,sys_id | (OFFSET_PITCH_ROLL<<TYPE_SHIFT),(const uavcan::uint8_t *)&tmp,8);
				if (resultSend || resultSend2) {
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
				tmp.v2=thrustValue;
				resultSend=sendFrame(iFace,sys_id | (OFFSET_YOW<<TYPE_SHIFT),(const uavcan::uint8_t *)&tmp,8);
				resultSend2=sendFrame(iFace2,sys_id |  (OFFSET_YOW<<TYPE_SHIFT),(const uavcan::uint8_t *)&tmp,8);
				if (resultSend || resultSend2) {
					cycle++;
					postYow=false;
					postVxy=true;
				}
			} else {
				cycle++;
			}
			break;
			case 2: //I Vxy
			if (postVxy) {
				tmp.v1=vxValue;
				tmp.v2=vyValue;
				resultSend=sendFrame(iFace,sys_id | (OFFSET_VXY<<TYPE_SHIFT),(const uavcan::uint8_t *)&tmp,8);
				resultSend2=sendFrame(iFace2,sys_id | (OFFSET_VXY<<TYPE_SHIFT),(const uavcan::uint8_t *)&tmp,8);
				if (resultSend || resultSend2) {
					cycle++;
					postVxy=false;
					postThrAct=true;
				}
			} else {
				cycle++;
			}
			break;
			case 3: //I ThrAct
			if (postThrAct) {
				tmp.v1=thrAct;
				tmp.v2=0.0f;
				resultSend=sendFrame(iFace,sys_id | (OFFSET_THR_ACT<<TYPE_SHIFT),(const uavcan::uint8_t *)&tmp,8);
				resultSend2=sendFrame(iFace2,sys_id | (OFFSET_THR_ACT<<TYPE_SHIFT),(const uavcan::uint8_t *)&tmp,8);
				if (resultSend || resultSend2) {
					cycle++;
					postThrAct=false;
				}
			} else {
				cycle++;
			}
			break;
			case 4: //POS XY
			if (_vehicle_share_position_sub.updated()) {
				_vehicle_share_position_sub.copy(&vehicle_share_position);
				tmp.v1=vehicle_share_position.x;
				tmp.v2=vehicle_share_position.y;
				V_ZPos=vehicle_share_position.z;
				V_ZVel=vehicle_share_position.vz;
				V_XVel=vehicle_share_position.vx;
				V_YVel=vehicle_share_position.vy;
				heading=vehicle_share_position.heading;
				valid_xy=vehicle_share_position.xy_valid;
				valid_z=vehicle_share_position.z_valid;
				valid_vxy=vehicle_share_position.v_xy_valid;
				valid_vz=vehicle_share_position.v_z_valid;
				resultSend=sendFrame(iFace,sys_id | (OFFSET_POSXY<<TYPE_SHIFT),(const uavcan::uint8_t *)&tmp,8);
				resultSend2=sendFrame(iFace2,sys_id | (OFFSET_POSXY<<TYPE_SHIFT),(const uavcan::uint8_t *)&tmp,8);
				if (resultSend || resultSend2) {
					cycle++;
					postV_Zinfo=true;
				}
			} else {
				cycle++;
			}
			break;
			case 5: //POS Z and VEL Z
			if (postV_Zinfo) {
				tmp.v1=V_ZPos;
				tmp.v2=V_ZVel;
				resultSend=sendFrame(iFace,sys_id | (OFFSET_POSZ_VELZ<<TYPE_SHIFT),(const uavcan::uint8_t *)&tmp,8);
				resultSend2=sendFrame(iFace2,sys_id | (OFFSET_POSZ_VELZ<<TYPE_SHIFT),(const uavcan::uint8_t *)&tmp,8);
				if (resultSend || resultSend2) {
					cycle++;
					postV_Zinfo=false;
					postV_VXY=true;
				}
			} else {
				cycle++;
			}
			break;
			case 6: //VEL XY
			if (postV_VXY) {
				tmp.v1=V_XVel;
				tmp.v2=V_YVel;
				resultSend=sendFrame(iFace,sys_id | (OFFSET_VELXY<<TYPE_SHIFT),(const uavcan::uint8_t *)&tmp,8);
				resultSend2=sendFrame(iFace2,sys_id | (OFFSET_VELXY<<TYPE_SHIFT),(const uavcan::uint8_t *)&tmp,8);
				if (resultSend || resultSend2) {
					cycle++;
					postV_VXY=false;
					postV_status=true;
				}
			} else {
				cycle++;
			}
			break;
			case 7: //Valid
			if (postV_status) {
				tmpm.v1=(valid_xy?VALID_XY:0)|(valid_z?VALID_Z:0)|(valid_vxy?VALID_VXY:0)|(valid_vz?VALID_VZ:0);
				tmpm.v2=heading;
				resultSend=sendFrame(iFace,sys_id | (OFFSET_STATUS<<TYPE_SHIFT),(const uavcan::uint8_t *)&tmpm,8);
				resultSend2=sendFrame(iFace2,sys_id | (OFFSET_STATUS<<TYPE_SHIFT),(const uavcan::uint8_t *)&tmpm,8);
				if (resultSend || resultSend2) {
					cycle++;
					postV_status=false;
				}
			} else {
				cycle++;
			}
			break;
			case 8:
			if (_actuator_controls_0_sub.updated()) {
				actuator_controls_s actuator_controls;
				_actuator_controls_0_sub.copy(&actuator_controls);
				tmp.v1=actuator_controls.control[actuator_controls_s::INDEX_YAW];
				tmp.v2=actuator_controls.control[actuator_controls_s::INDEX_THROTTLE];
				resultSend=sendFrame(iFace,sys_id | (OFFSET_ACT_1<<TYPE_SHIFT),(const uavcan::uint8_t *)&tmp,8);
				resultSend2=sendFrame(iFace2,sys_id | (OFFSET_ACT_1<<TYPE_SHIFT),(const uavcan::uint8_t *)&tmp,8);
				if (resultSend || resultSend2) {
					cycle++;
					postV_status=false;
				}
			} else {
				cycle++;
			}
			break;
			default:
				cycle++;
				if (cycle%36==0) {
					cycle=0;
				}
			break;
		}

		//Receive
		//Check for new incoming can msg
		uavcan::int16_t receiveResult=1;
		uavcan::MonotonicTime mtime;
		uavcan::UtcTime out_ts_utc;
		uavcan::CanIOFlags out_flags;
		uavcan::CanFrame canFrame;
		if (iFace!=nullptr) {
			while (receiveResult>0)  {

				receiveResult=iFace->receive(canFrame,mtime,out_ts_utc,out_flags);
				if (receiveResult==0) {
					//PX4_ERR("CAN driver RX empty");
					//nbReceivedError++;
				}
				if (receiveResult<0) {
					PX4_ERR("CAN driver RX error");
					nbReceivedError[0]++;
				}
				if (receiveResult==1) {
					processReceivedFrame(iFace,canFrame);
				}
			}
		}

		receiveResult=1;
		if (iFace2!=nullptr) {
			while (receiveResult>0)  {
				receiveResult=iFace2->receive(canFrame,mtime,out_ts_utc,out_flags);
				if (receiveResult==0) {
					//PX4_ERR("CAN driver RX empty");
					//nbReceivedError++;
				}
				if (receiveResult<0) {
					PX4_ERR("CAN driver RX error");
					nbReceivedError[1]++;
				}
				if (receiveResult==1) {
					processReceivedFrame(iFace2,canFrame);
				}
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
		if (currentTime-r1_shpos.timestamp>MAX_DELAY_REFRESH) {
			r1_shpos.status=vehicle_share_position_s::VSP_STATUS_NONE;
			_r1vehicle_share_position_pub.publish(r1_shpos);
		}
		if (currentTime-r2_shpos.timestamp>MAX_DELAY_REFRESH) {
			r2_shpos.status=vehicle_share_position_s::VSP_STATUS_NONE;
			_r2vehicle_share_position_pub.publish(r2_shpos);
		}
		if (currentTime-r3_shpos.timestamp>MAX_DELAY_REFRESH) {
			r3_shpos.status=vehicle_share_position_s::VSP_STATUS_NONE;
			_r3vehicle_share_position_pub.publish(r3_shpos);
		}
		if (currentTime-act_r1.timestamp>MAX_DELAY_REFRESH) {
			act_r1.status=actuator_controls_re_s::ACT_STATUS_NONE;
			_r1act_pub.publish(act_r1);
		}
		if (currentTime-act_r2.timestamp>MAX_DELAY_REFRESH) {
			act_r2.status=actuator_controls_re_s::ACT_STATUS_NONE;
			_r2act_pub.publish(act_r2);
		}
		if (currentTime-act_r3.timestamp>MAX_DELAY_REFRESH) {
			act_r3.status=actuator_controls_re_s::ACT_STATUS_NONE;
			_r3act_pub.publish(act_r3);
		}
		if (countIt==0) {
			can_status.timestamp=currentTime;
			can_status.nbr_1=nbReceivedR1;
			can_status.nbr_2=nbReceivedR2;
			can_status.nbr_3=nbReceivedR3;
			can_status.nb_emitted_1=nbEmitted[0];
			can_status.nb_emitted_2=nbEmitted[1];
			can_status.nb_received_1=nbReceived[0];
			can_status.nb_received_2=nbReceived[1];
			can_status.can_error_1=(can_status.nb_emitted_error_1!=nbEmittedError[0]) || (can_status.nb_received_error_1!=nbReceivedError[0]);
			can_status.can_error_2=(can_status.nb_emitted_error_2!=nbEmittedError[1]) || (can_status.nb_received_error_2!=nbReceivedError[1]);
			can_status.nb_emitted_error_1=nbEmittedError[0];
			can_status.nb_emitted_error_2=nbEmittedError[1];
			can_status.nb_received_error_1=nbReceivedError[0];
			can_status.nb_received_error_2=nbReceivedError[1];
			_can_status_pub.publish(can_status);
		}
		countIt++;
		if (countIt>100) {
			countIt=0;
		}
		currentTime=hrt_absolute_time();
		int64_t deltaTime=((int64_t)TIME_CYCLE)-((int64_t)(currentTime-startTime));
		if (deltaTime<0) {
			deltaTime=0;
		}

		//sleep
		usleep(deltaTime);
	}


	//Can stop


}
void ModuleCanIntegrale::processReceivedFrame(uavcan::ICanIface * iFacePart,uavcan::CanFrame &canFrame) {
//Process received
	IntegralCanData *tmpp=(IntegralCanData *)canFrame.data;
	IntegralCanDataMix *tmpm=(IntegralCanDataMix *)canFrame.data;
	int32_t id=(int32_t)(canFrame.id & MASK_ID);
	int32_t typecmd=(int32_t)(canFrame.id >> TYPE_SHIFT);
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
			if (typecmd == OFFSET_THR_ACT) {
				rx_integrales[offset]->throttle_act=tmpp->v1;
				if (rx_integrales[offset]->status==integrale_s::INTEGRALE_STATUS_PARTIAL) {
					rx_integrales[offset]->status=	integrale_s::INTEGRALE_STATUS_COMPLETE;
					if (offset==0) {
						r1_integrale.timestamp=hrt_absolute_time();
						r1_integrale.index=id;
						_r1integrale_pub.publish(r1_integrale);
						nbReceivedR1++;
					}
					if (offset==1) {
						r2_integrale.timestamp=hrt_absolute_time();
						r2_integrale.index=id;
						_r2integrale_pub.publish(r2_integrale);
						nbReceivedR2++;
					}
					if (offset==2) {
						r3_integrale.timestamp=hrt_absolute_time();
						r3_integrale.index=id;
						_r3integrale_pub.publish(r3_integrale);
						nbReceivedR3++;
					}
				}
				rx_integrales[offset]->status=	integrale_s::INTEGRALE_STATUS_NONE;
			}
			if (typecmd == OFFSET_PITCH_ROLL) {
				//PX4_INFO(" offset pitch-roll id %x status:%d",canFrame.id,(int)rx_integrales[offset]->status);
				rx_integrales[offset]->pitch_rate_integral=tmpp->v1;
				rx_integrales[offset]->roll_rate_integral=tmpp->v2;
				rx_integrales[offset]->status=integrale_s::INTEGRALE_STATUS_PARTIAL;
			}
			if (typecmd == OFFSET_YOW) {
				//PX4_INFO(" offset pitch-roll id %x status:%d",canFrame.id,(int)rx_integrales[offset]->status);
				rx_integrales[offset]->yaw_rate_integral=tmpp->v1;
				rx_integrales[offset]->thrust=tmpp->v2;
				rx_integrales[offset]->status=integrale_s::INTEGRALE_STATUS_PARTIAL;
			}
			if (typecmd == OFFSET_VXY) {
				//PX4_INFO(" offset pitch-roll id %x status:%d",canFrame.id,(int)rx_integrales[offset]->status);
				rx_integrales[offset]->vx=tmpp->v1;
				rx_integrales[offset]->vy=tmpp->v2;
				rx_integrales[offset]->status=integrale_s::INTEGRALE_STATUS_PARTIAL;
			}
			if (typecmd == OFFSET_POSXY) {
				rx_shpos[offset]->x=tmpp->v1;
				rx_shpos[offset]->y=tmpp->v2;
				rx_shpos[offset]->status=vehicle_share_position_s::VSP_STATUS_PARTIAL;
			}
			if (typecmd == OFFSET_POSZ_VELZ) {
				rx_shpos[offset]->z=tmpp->v1;
				rx_shpos[offset]->vz=tmpp->v2;
				rx_shpos[offset]->status=vehicle_share_position_s::VSP_STATUS_PARTIAL;
			}
			if (typecmd == OFFSET_VELXY) {
				rx_shpos[offset]->vx=tmpp->v1;
				rx_shpos[offset]->vy=tmpp->v2;
				rx_shpos[offset]->status=vehicle_share_position_s::VSP_STATUS_PARTIAL;
			}
			if (typecmd == OFFSET_STATUS) {
				rx_shpos[offset]->xy_valid=(tmpm->v1 & VALID_XY)==VALID_XY;
				rx_shpos[offset]->z_valid=(tmpm->v1 & VALID_Z)==VALID_Z;
				rx_shpos[offset]->v_xy_valid=(tmpm->v1 & VALID_VXY)==VALID_VXY;
				rx_shpos[offset]->v_z_valid=(tmpm->v1 & VALID_VZ)==VALID_VZ;
				rx_shpos[offset]->heading=tmpm->v2;
				if (rx_shpos[offset]->status==vehicle_share_position_s::VSP_STATUS_PARTIAL) {
					rx_shpos[offset]->status=vehicle_share_position_s::VSP_STATUS_COMPLETE;
					if (offset==0) {
						r1_shpos.timestamp=hrt_absolute_time();
						r1_shpos.index=id;
						_r1vehicle_share_position_pub.publish(r1_shpos);
						nbReceivedR1++;
					}
					if (offset==1) {
						r2_shpos.timestamp=hrt_absolute_time();
						r2_shpos.index=id;
						_r2vehicle_share_position_pub.publish(r2_shpos);
						nbReceivedR2++;
					}
					if (offset==2) {
						r3_shpos.timestamp=hrt_absolute_time();
						r3_shpos.index=id;
						_r3vehicle_share_position_pub.publish(r3_shpos);
						nbReceivedR3++;
					}
				}
				rx_shpos[offset]->status=vehicle_share_position_s::VSP_STATUS_NONE;
			}
			if (typecmd == OFFSET_ACT_1) {
				rx_act[offset]->control[actuator_controls_s::INDEX_YAW]=tmpp->v1;
				rx_act[offset]->control[actuator_controls_s::INDEX_THROTTLE]=tmpp->v2;
				rx_act[offset]->timestamp=hrt_absolute_time();
				rx_act[offset]->index=id;
				rx_act[offset]->status=actuator_controls_re_s::ACT_STATUS_COMPLETE;
				if (offset==0) {
					_r1act_pub.publish(act_r1);
					nbReceivedR1++;
				}
				if (offset==1) {
					_r2act_pub.publish(act_r2);
					nbReceivedR2++;
				}
				if (offset==2) {
					_r3act_pub.publish(act_r3);
					nbReceivedR3++;
				}
			}
		}
	} else {
		PX4_ERR("Invalid can id %x",canFrame.id);
		PX4_ERR("Invalid id %x",id);
	}

	if (iFacePart==iFace) {
		nbReceived[0]++;
	}
	if (iFacePart==iFace2) {
		nbReceived[1]++;
	}

}
bool ModuleCanIntegrale::sendFrame(uavcan::ICanIface * iFacePart,uint32_t can_id, const uint8_t* can_data, uint8_t data_len) {
	if (iFacePart==nullptr) {
		return false;
	}
	uavcan::CanFrame canFrame=uavcan::CanFrame(can_id,can_data,data_len);
	//PX4_INFO("send pr can id %x",canFrame.id);
	auto sendResult=iFacePart->send(canFrame,uavcan::MonotonicTime::fromUSec(20000),0);
	if (sendResult==0) {
		//PX4_ERR("CAN driver TX full");
		if (iFacePart==iFace) {
			nbEmittedError[0]++;
		}
		if (iFacePart==iFace2) {
			nbEmittedError[1]++;
		}
		return false;
	}
	if (sendResult<0) {
		PX4_ERR("CAN driver TX error");
		if (iFacePart==iFace) {
			nbEmittedError[0]++;
		}
		if (iFacePart==iFace2) {
			nbEmittedError[1]++;
		}
		return false;
	}
	if (sendResult==1) {
		if (iFacePart==iFace) {
			nbEmitted[0]++;
		}
		if (iFacePart==iFace) {
			nbEmitted[1]++;
		}
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
