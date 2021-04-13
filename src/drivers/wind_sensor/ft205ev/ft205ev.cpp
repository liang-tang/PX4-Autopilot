/****************************************************************************
 *
 *   Copyright (c) 2018-19 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ft205ev.cpp
 *
 * @author Mark Sauder <mcsauder@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 * @author Jake Dahl <dahl.jakejacob@gmail.com>
 * @author Mohammed Kabir <mhkabir@mit.edu>
 */

#include "ft205ev.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

#ifndef GPIO_HEATER_OUTPUT
#error "To use the ft205ev driver, the board_config.h must define and initialize GPIO_HEATER_OUTPUT"
#endif

FT205EV::FT205EV() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{

}

FT205EV::~FT205EV()
{

}

int FT205EV::custom_command(int argc, char *argv[])
{
	// Check if the driver is running.
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	return print_usage("Unrecognized command.");
}

void FT205EV::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	windvane_s windvane{};
	windvane.timestamp = hrt_absolute_time();
	_windvane_pub.publish(windvane);

	bool updated = _windvane_sub.updated();

	if (updated) {
		if (_windvane_sub.copy(&windvane)) {
			PX4_INFO("time %llu\n", windvane.timestamp);
		}
	}

	ScheduleDelayed(CONTROLLER_PERIOD_DEFAULT);
}

void FT205EV::initialize_topics()
{

}

int FT205EV::print_status()
{
	PX4_INFO("Sensor ID: - Temperature: Setpoint: FT205EV State:");

	return PX4_OK;
}

int FT205EV::start()
{
	initialize_topics();

	ScheduleNow();

	return PX4_OK;
}

int FT205EV::task_spawn(int argc, char *argv[])
{
	FT205EV *ft205ev = new FT205EV();

	if (!ft205ev) {
		PX4_ERR("driver allocation failed");
		return PX4_ERROR;
	}

	_object.store(ft205ev);
	_task_id = task_id_is_work_queue;

	ft205ev->start();

	return 0;
}

int FT205EV::print_usage(const char *reason)
{
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

/**
 * Main entry point for the ft205ev driver module
 */
int ft205ev_main(int argc, char *argv[])
{
	return FT205EV::main(argc, argv);
}
