#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <parameters/param.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/integrale.h>
#include <uORB/topics/pipe_correction.h>

# include <uavcan_stm32/uavcan_stm32.hpp>

#define MAX_REMOTE_INTEGRALE 3



extern "C" __EXPORT int can_integrale_main(int argc, char *argv[]);


class ModuleCanIntegrale : public ModuleBase<ModuleCanIntegrale>, public ModuleParams
{
public:
	ModuleCanIntegrale();

	virtual ~ModuleCanIntegrale() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static ModuleCanIntegrale *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MAV_SYS_ID>) _param_mav_sys_id
	)
	// Subscriptions=
	uORB::Subscription 	_integrale_sub{ORB_ID(integrale)};
	uORB::Subscription	_pipe_correction_sub{ORB_ID(pipe_correction)};
	uORB::Publication<integrale_s>			_r1integrale_pub{ORB_ID(r1integrale)};
	uORB::Publication<integrale_s>			_r2integrale_pub{ORB_ID(r2integrale)};
	uORB::Publication<integrale_s>			_r3integrale_pub{ORB_ID(r3integrale)};
	integrale_s r1_integrale;
	integrale_s r2_integrale;
	integrale_s r3_integrale;
	integrale_s * rx_integrales[MAX_REMOTE_INTEGRALE];
	int32_t nbReceivedR1;
	int32_t nbReceivedR2;
	int32_t nbReceivedR3;
	int32_t nbReceived;
	int32_t nbEmitted;
	int32_t nbReceivedError;
	int32_t nbEmittedError;
	bool postYow;
	bool postCorrection;
	float yowIntegraleValue;
	float yowPipeCorrectionValue;
	float nbMedianValue;
};

