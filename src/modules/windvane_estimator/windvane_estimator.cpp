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

#include "windvane_estimator.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>

#define GPS_EPOCH_SECS ((time_t)1234567890ULL)

int WINDVANE_ESTIMATOR::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int WINDVANE_ESTIMATOR::custom_command(int argc, char *argv[])
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


int WINDVANE_ESTIMATOR::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("windvane_estimator",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024 * 4,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

WINDVANE_ESTIMATOR *WINDVANE_ESTIMATOR::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	WINDVANE_ESTIMATOR *instance = new WINDVANE_ESTIMATOR(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

WINDVANE_ESTIMATOR::WINDVANE_ESTIMATOR(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void WINDVANE_ESTIMATOR::run()
{
	int windvane_sensor_sub = orb_subscribe(ORB_ID(windvane_sensor));

	px4_pollfd_struct_t fds[1];
	fds[0].fd = windvane_sensor_sub;
	fds[0].events = POLLIN;

	while (!should_exit()) {
		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(windvane_sensor), windvane_sensor_sub, &windvane_sensor);
			calculate_and_publish();
		}
	}
	orb_unsubscribe(windvane_sensor_sub);
}

void WINDVANE_ESTIMATOR::calculate_and_publish()
{
	if (_att_sub.updated()) {
		_att_sub.copy(&attitude);
	}

	if (_local_pos_sub.updated()) {
		_local_pos_sub.copy(&local_pos);
	}

	if (_global_pos_sub.updated()) {
		_global_pos_sub.copy(&global_pos);
	}

	// matrix::Vector3f rel_wind(0, 0, 0);
	// matrix::Dcmf R_body_to_earth(matrix::Quatf(attitude.q));
	// rel_wind = R_body_to_earth.transpose() * rel_wind;

	// PX4_INFO("222: %.2lf %.2lf %.2lf\n", (double)rel_wind(0)), (double)rel_wind(1), (double)rel_wind(2));

	const matrix::Eulerf euler(matrix::Quatf(attitude.q));

	windvane.timestamp = hrt_absolute_time();
	windvane.speed_hor = windvane_sensor.speed_hor;
	windvane.angle_hor = windvane_sensor.angle_hor;
	windvane.speed_ver = windvane_sensor.speed_ver;
	windvane.angle_ver = windvane_sensor.angle_ver;
	windvane.attitude[0] = math::degrees(euler.phi());
	windvane.attitude[1] = math::degrees(euler.theta());
	windvane.attitude[2] = math::degrees(euler.psi());
	windvane.ground_speed[0] = local_pos.vx;
	windvane.ground_speed[1] = local_pos.vy;
	windvane.ground_speed[2] = local_pos.vz;

	windvane.lat = global_pos.lat;
	windvane.lon = global_pos.lon;
	windvane.alt = global_pos.alt;

	windvane.wind_speed_calculated[0] = 0.0f;
	windvane.wind_speed_calculated[1] = 0.0f;
	windvane.wind_speed_calculated[2] = 0.0f;

	windvane.wind_speed_hor_calculated = 0.0f;
	windvane.wind_angle_hor_calculated = 0.0f;

	_windvane_pub.publish(windvane);

	log_on_sdcard();
}
void WINDVANE_ESTIMATOR::log_on_sdcard()
{
	tm tt = {};

	if (!get_log_time(&tt, 28800)) {
		return;
	}

	char time_now_str[16] = "";
	strftime(time_now_str, sizeof(time_now_str), "%m%d_%H%M%S", &tt);
	PX4_INFO("time now: %s\n", time_now_str);

	if (_fd == -1) {
		char log_file_name_str[40] = "";
		snprintf(log_file_name_str, sizeof(log_file_name_str), PX4_STORAGEDIR"/%s.csv", time_now_str);
		PX4_INFO("log file name: %s\n", log_file_name_str);

		_fd = ::open(log_file_name_str, O_CREAT | O_WRONLY, PX4_O_MODE_666);
		if (_fd == -1) {
			PX4_INFO("open error");
			return;
		}
	}

	char *file = nullptr;
	if (asprintf(&file, "%s,"
			    "%.2f,%.2f,%.2f,%.2f," //raw
			    "%.2f,%.2f,%.2f," // attitude
			    "%.2f,%.2f,%.2f," // ground_speed
			    "%.7f,%.7f,%.2f," // Latitude Longitude Alt
			    "%.2f,%.2f,%.2f," // Vax, Vay, Vax
			    "%.2f,%.2f\n",    // Vwxy,0wxy
		time_now_str,
		(double)windvane.speed_hor, (double)windvane.angle_hor,
		(double)windvane.speed_ver, (double)windvane.angle_ver,
		(double)windvane.attitude[0], (double)windvane.attitude[1], (double)windvane.attitude[2],
		(double)windvane.ground_speed[0], (double)windvane.ground_speed[1], (double)windvane.ground_speed[2],
		windvane.lat, windvane.lon, (double)windvane.alt,
		(double)windvane.wind_speed_calculated[0],
		(double)windvane.wind_speed_calculated[1],
		(double)windvane.wind_speed_calculated[2],
		(double)windvane.wind_speed_hor_calculated,
		(double)windvane.wind_angle_hor_calculated) < 0) {
		return;
	}

	::write(_fd, file, strlen(file));
	fsync(_fd);
	free(file);
}

bool WINDVANE_ESTIMATOR::get_log_time(struct tm *tt, int utc_offset_sec)
{
	uORB::Subscription vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};

	time_t utc_time_sec;
	bool use_clock_time = true;

	/* Get the latest GPS publication */
	vehicle_gps_position_s gps_pos;

	if (vehicle_gps_position_sub.copy(&gps_pos)) {
		utc_time_sec = gps_pos.time_utc_usec / 1e6;

		if (gps_pos.fix_type >= 2 && utc_time_sec >= GPS_EPOCH_SECS) {
			use_clock_time = false;
		}
	}

	if (use_clock_time) {
		/* take clock time if there's no fix (yet) */
		struct timespec ts = {};
		px4_clock_gettime(CLOCK_REALTIME, &ts);
		utc_time_sec = ts.tv_sec + (ts.tv_nsec / 1e9);

		if (utc_time_sec < GPS_EPOCH_SECS) {
			return false;
		}
	}

	/* apply utc offset */
	utc_time_sec += utc_offset_sec;

	return gmtime_r(&utc_time_sec, tt) != nullptr;
}

int WINDVANE_ESTIMATOR::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int windvane_estimator_main(int argc, char *argv[])
{
	return WINDVANE_ESTIMATOR::main(argc, argv);
}
