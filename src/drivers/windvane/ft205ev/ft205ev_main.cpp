/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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

#include "ft205ev.hpp"

#include <px4_platform_common/getopt.h>

/**
 * Local functions in support of the shell command.
 */
namespace ft205ev
{

FT205EV	*g_dev{nullptr};

int start(const char *port1, const char *port2);
int status();
int stop();
int usage();

int
start(const char *port1, const char *port2)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_OK;
	}

	// Instantiate the driver.
	g_dev = new FT205EV(port1, port2);

	if (g_dev == nullptr) {
		PX4_ERR("driver start failed");
		return PX4_ERROR;
	}

	if (OK != g_dev->init()) {
		PX4_ERR("driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return 0;
}

int stop()
{
	if (g_dev != nullptr) {
		PX4_INFO("stopping driver");
		delete g_dev;
		g_dev = nullptr;
		PX4_INFO("driver stopped");

	} else {
		PX4_ERR("driver not running");
		return 1;
	}

	return PX4_OK;
}

int
usage()
{
	PRINT_MODULE_USAGE_NAME("ft205ev", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device1", false);
	PRINT_MODULE_USAGE_PARAM_STRING('e', nullptr, nullptr, "Serial device2", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Driver status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print driver status");
	return PX4_OK;
}

} // namespace

extern "C" __EXPORT int ft205ev_main(int argc, char *argv[])
{
	int ch = 0;
	// tty port1
	const char *device_path1 = FT205EV_DEFAULT_PORT1;
	// tty port2
	const char *device_path2 = FT205EV_DEFAULT_PORT2;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	// parse CLI arguments if have
	while ((ch = px4_getopt(argc, argv, "e:e:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_path1 = myoptarg;
			break;

		case 'e':
			device_path2 = myoptarg;
			break;

		default:
			PX4_WARN("Unknown option!");
			return PX4_ERROR;
		}
	}

	if (myoptind >= argc) {
		PX4_ERR("unrecognized command");
		return ft205ev::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {
		if (strcmp(device_path1, "") != 0 && strcmp(device_path2, "") != 0) {
			return ft205ev::start(device_path1, device_path2);

		} else {
			PX4_WARN("Please specify device path!");
			return ft205ev::usage();
		}

	} else if (!strcmp(argv[myoptind], "stop")) {
		return ft205ev::stop();

	} else if (!strcmp(argv[myoptind], "status")) {
		return ft205ev::status();
	}

	return ft205ev::usage();
}
