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

/**
 * @file tfmini.cpp
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Greg Hulands
 * @author Ayush Gaud <ayush.gaud@gmail.com>
 * @author Christoph Tobler <christoph@px4.io>
 * @author Mohammed Kabir <mhkabir@mit.edu>
 *
 * Driver for the Benewake TFmini laser rangefinder series
 */

#pragma once

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/topics/windvane_sensor.h>

#include "windvane_nmea.h"

// default ports
#define FT205EV_DEFAULT_PORT1 "/dev/ttyS2"
#define FT205EV_DEFAULT_PORT2 "/dev/ttyS3"

using namespace time_literals;

class FT205EV : public px4::ScheduledWorkItem
{
public:
	// constructor
	FT205EV(const char *port1, const char *port2);

	// destructor
	virtual ~FT205EV();

	// This method initializes the general driver for a windvane sensor
	int init();

	// print some basic information about the driver
	void print_info();

private:
	// init serial ports
	int init_ports(int port);

	// read data fom serial port
	int collect(int port);

	// Perform a reading cycle
	void Run() override;

	// Initialise the automatic measurement state machine and start it
	void start();

	// Stops the automatic measurement state machine
	void stop();

	// serial port name
	char _ports[2][20] {};

	// The file descriptor
	int _fd[2]{-1, -1};

	// parser
	AP_WindVane_NMEA *windvane_nmea[2];

	// data flag
	bool updated[2];

	// Publication
	uORB::Publication<windvane_sensor_s> _windvane_sensor_pub{ORB_ID(windvane_sensor)};

	// perf counter
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
};
