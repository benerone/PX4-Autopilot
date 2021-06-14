#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <parameters/param.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>


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
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MAV_SYS_ID>) _param_mav_sys_id
	)

	int32_t sys_id;

};

