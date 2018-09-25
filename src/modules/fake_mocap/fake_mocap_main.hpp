#pragma once

#include <drivers/drv_hrt.h>
#include <px4_log.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_module.h>

// uORB Publications
#include <uORB/Publication.hpp>
#include <uORB/topics/att_pos_mocap.h>

extern "C" __EXPORT int fake_mocap_main(int argc, char *argv[]);

class FakeMocapModule : public ModuleBase<FakeMocapModule>
{
public:
	virtual ~FakeMocapModule() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static FakeMocapModule *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

private:
	FakeMocapModule();
	int publish();

	uORB::Publication<att_pos_mocap_s> _pub_mocap;
};
