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

#include <dirent.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

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

	// wind speed (body frame)
	matrix::Vector3f Vba(0, 0, 0);
	Vba(0) = windvane_sensor.speed_horiz * cosf(math::radians(windvane_sensor.angle_horiz + 180));
	Vba(1) = windvane_sensor.speed_horiz * sinf(math::radians(windvane_sensor.angle_horiz + 180));
	Vba(2) = windvane_sensor.speed_vert * cosf(math::radians(windvane_sensor.angle_vert + 180));

	// wind speed (world frame)
	matrix::Vector3f Vga(0, 0, 0);
	matrix::Dcmf R_body_to_earth(matrix::Quatf(attitude.q));
	Vga = R_body_to_earth.transpose() * Vba;

	// ground speed (world frame)
	matrix::Vector3f Vgg(local_pos.vx, local_pos.vy, local_pos.vz);

	// real wind speed (world frame)
	matrix::Vector3f Vgw = Vgg - Vga;

	// horizontal wind speed
	float Vgwxy_len = sqrtf(Vgw(0) * Vgw(0) + Vgw(1) * Vgw(1));

	float Owxy = 0;

	matrix::Vector2f Vgwxy(Vgw(0), Vgw(1));
	matrix::Vector2f n(1, 0);

	if (Vgw(1) > 0) {
		Owxy = math::degrees(acosf((Vgwxy * n) / (Vgwxy_len * n.length())));
	} else if (Vgw(1) < 0) {
		Owxy = math::degrees(acosf((Vgwxy * n) / (Vgwxy_len * n.length()))) + 180;
	}

	const matrix::Eulerf euler(matrix::Quatf(attitude.q));

	windvane.timestamp = hrt_absolute_time();

	windvane.lat = global_pos.lat;
	windvane.lon = global_pos.lon;
	windvane.alt = global_pos.alt;

	windvane.ground_speed[0] = local_pos.vx;
	windvane.ground_speed[1] = local_pos.vy;
	windvane.ground_speed[2] = local_pos.vz;

	windvane.attitude[0] = math::degrees(euler.phi());
	windvane.attitude[1] = math::degrees(euler.theta());
	windvane.attitude[2] = math::degrees(euler.psi());

	windvane.speed_horiz_sensor = windvane_sensor.speed_horiz;
	windvane.angle_horiz_sensor = windvane_sensor.angle_horiz;
	windvane.speed_vert_sensor = windvane_sensor.speed_vert;
	windvane.angle_vert_sensor = windvane_sensor.angle_vert;

	windvane.wind_speed_3d[0] = Vgw(0);
	windvane.wind_speed_3d[1] = Vgw(1);
	windvane.wind_speed_3d[2] = Vgw(2);

	windvane.wind_speed_horiz = Vgwxy_len;
	windvane.wind_angle_horiz = Owxy;

	_windvane_pub.publish(windvane);

	log_on_sdcard();
}

void WINDVANE_ESTIMATOR::log_on_sdcard()
{
	/* take clock time if there's no fix (yet) */
	struct timespec ts = {};
	px4_clock_gettime(CLOCK_REALTIME, &ts);
	time_t utc_time_sec = ts.tv_sec + (ts.tv_nsec / 1e9);
	time_t utc_time_ms = ts.tv_nsec / 1e6;

	if (utc_time_sec < GPS_EPOCH_SECS) {
		return;
	}

	/* apply utc offset */
	utc_time_sec += 28800;

	tm tt = {};
	if (gmtime_r(&utc_time_sec, &tt) == nullptr) {
		return;
	}

	char time_now_str[32] = "";
	strftime(time_now_str, sizeof(time_now_str), "%Y_%m_%d-%H_%M_%S", &tt);

	if (_fd == -1) {
		char log_file_name_str[64] = "";
		snprintf(log_file_name_str, sizeof(log_file_name_str), PX4_STORAGEDIR"/%s.csv", time_now_str);

		_fd = ::open(log_file_name_str, O_CREAT | O_WRONLY, PX4_O_MODE_666);
		if (_fd == -1) {
			PX4_INFO("open error");
			return;
		}
		const char* head =
				"Time,"
				"Lat,Lon,Alt,"
				"ground_speed.x,ground_speed.y,ground_speed.z,"
				"Roll,Pitch,Yaw,"
				"speed_horiz_sensor,angle_horiz_sensor,"
				"speed_vert_sensor,angle_vert_sensor,"
				"wind_speed_3d.x,wind_speed_3d.y, wind_speed_3d.z,"
				"wind_speed_horiz,wind_angle_horiz\n";
		::write(_fd, head, strlen(head));
		fsync(_fd);
	}

	char *file = nullptr;
	if (asprintf(&file, "%s:%03u,"
			    "%.7f,%.7f,%.2f," // latitude longitude alt
			    "%.2f,%.2f,%.2f," // ground_speed
			    "%.2f,%.2f,%.2f," // attitude
			    "%.2f,%.2f,%.2f,%.2f," // raw sensor
			    "%.2f,%.2f,%.2f," // wind_speed_3d
			    "%.2f,%.2f\n",    // wind_speed_horiz, wind_angle_horiz
		&time_now_str[11], utc_time_ms,
		windvane.lat, windvane.lon, (double)windvane.alt,
		(double)windvane.ground_speed[0], (double)windvane.ground_speed[1], (double)windvane.ground_speed[2],
		(double)windvane.attitude[0], (double)windvane.attitude[1], (double)windvane.attitude[2],
		(double)windvane.speed_horiz_sensor, (double)windvane.angle_horiz_sensor,
		(double)windvane.speed_vert_sensor, (double)windvane.angle_vert_sensor,
		(double)windvane.wind_speed_3d[0],
		(double)windvane.wind_speed_3d[1],
		(double)windvane.wind_speed_3d[2],
		(double)windvane.wind_speed_horiz,
		(double)windvane.wind_angle_horiz) < 0) {
		return;
	}

	::write(_fd, file, strlen(file));
	fsync(_fd);
	free(file);
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