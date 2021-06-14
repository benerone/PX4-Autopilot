
#include "sbg_module.hpp"

int ModuleSBG::print_status()
{
	PX4_INFO("Running");
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
				      4096,
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
}

void ModuleSBG::run()
{


	//Mavlink id serve as id
	sys_id=_param_mav_sys_id.get();
	while(!should_exit()) {
		usleep(100);
	}

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
