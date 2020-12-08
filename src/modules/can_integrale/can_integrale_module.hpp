#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <parameters/param.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/integrale.h>


# include <uavcan_stm32/uavcan_stm32.hpp>




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
};

