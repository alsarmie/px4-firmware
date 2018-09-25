/**
 * @file fake_mocap_main.cpp
 * @author Ludvig Ericson <ludvig@lericson.se>
 *
 * Fake mocap publisher
 */

#include "fake_mocap_main.hpp"

FakeMocapModule::FakeMocapModule() :
	_pub_mocap(ORB_ID(att_pos_mocap), -1)
{
	_pub_mocap.get().id = 0;
	_pub_mocap.get().id = 0;
	_pub_mocap.get().q[0] = 1.0f;
	_pub_mocap.get().q[1] = 0.0f;
	_pub_mocap.get().q[2] = 0.0f;
	_pub_mocap.get().q[3] = 0.0f;
	_pub_mocap.get().x = 0.0f;	// north
	_pub_mocap.get().y = 0.0f;	// east
	_pub_mocap.get().z = 0.0f;	// down
}

int FakeMocapModule::publish()
{
	uint64_t _timeStamp = hrt_absolute_time();
	_pub_mocap.get().timestamp = _timeStamp;
	_pub_mocap.get().timestamp_received = _timeStamp;
	_pub_mocap.update();
	return 0;
}

int FakeMocapModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Publish fake MOCAP readings

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fake_mocap", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int FakeMocapModule::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FakeMocapModule::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("fake_mocap",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT,
					 1000,
					 (px4_main_t)&run_trampoline,
					 (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

FakeMocapModule *FakeMocapModule::instantiate(int argc, char *argv[])
{
	FakeMocapModule *instance = new FakeMocapModule();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

void FakeMocapModule::run()
{
	unsigned frequency = 10; // Hz
	unsigned interval = 1000000/frequency;
	while (!should_exit()) {
		publish();
		usleep(interval);
	}
}

int fake_mocap_main(int argc, char *argv[])
{
	return FakeMocapModule::main(argc, argv);
}
