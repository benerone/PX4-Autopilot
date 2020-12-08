#include "can_integrale_module.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

using namespace time_literals;

int ModuleCanIntegrale::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

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
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
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
	//Can start if necessary

	while(!should_exit()) {

		//TODO
		integrale_s integrale;

		if (_integrale_sub.update(&integrale)) {
			//Publish on can
		}

		//Check for new incoming can msg
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
