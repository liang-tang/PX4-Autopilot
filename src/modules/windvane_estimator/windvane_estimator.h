/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/windvane.h>
#include <uORB/topics/windvane_sensor.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>

extern "C" __EXPORT int windvane_estimator_main(int argc, char *argv[]);

using namespace math;
using namespace matrix;

class WINDVANE_ESTIMATOR : public ModuleBase<WINDVANE_ESTIMATOR> , public ModuleParams
{
public:
	// constructor
	WINDVANE_ESTIMATOR();

	// destructor
	virtual ~WINDVANE_ESTIMATOR();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static WINDVANE_ESTIMATOR *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

private:
	// calculate and publish wind date
	void calculate_and_publish();

	// log wind data on sdcard
	void log_on_sdcard();

	void update_test_case(Eulerf &euler, Vector3f &Vgg, windvane_sensor_s &sensor);

	// file descriptor
	int _fd = -1;

	// messages define
	windvane_s windvane{};
	windvane_sensor_s windvane_sensor{};
	vehicle_attitude_s attitude{};
	vehicle_local_position_s local_pos{};
	vehicle_global_position_s global_pos{};
	vehicle_gps_position_s gps_pos{};

	// Publication
	uORB::Publication<windvane_s> _windvane_pub{ORB_ID(windvane)};

	// Subscriptions
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _gps_position_sub{ORB_ID(vehicle_gps_position)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::WV_TEST_CASE>) _test_case
	)
};
