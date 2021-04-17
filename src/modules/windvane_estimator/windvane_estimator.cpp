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

#include <dirent.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

using namespace math;
using namespace matrix;

#define GPS_EPOCH_SECS ((time_t)1234567890ULL)

int WINDVANE_ESTIMATOR::custom_command(int argc, char *argv[])
{
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
	WINDVANE_ESTIMATOR *instance = new WINDVANE_ESTIMATOR();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

WINDVANE_ESTIMATOR::WINDVANE_ESTIMATOR()
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
	// copy attitude
	if (_att_sub.updated()) {
		_att_sub.copy(&attitude);
	}

	// copy local position
	if (_local_pos_sub.updated()) {
		_local_pos_sub.copy(&local_pos);
	}

	// copy global position
	if (_global_pos_sub.updated()) {
		_global_pos_sub.copy(&global_pos);
	}

	// attitude
	Eulerf euler(Quatf(attitude.q));
	// ground speed (world frame)
	Vector3f Vgg(local_pos.vx, local_pos.vy, local_pos.vz);
#if 1
	euler.phi() = radians(0);
	euler.theta() = radians(0);
	euler.psi() = radians(45.0f);

	windvane_sensor.speed_horiz = 2.0f*1.414f;
	windvane_sensor.speed_vert = 2.0f*1.414f;
	windvane_sensor.angle_horiz = 0;
	windvane_sensor.angle_vert = 90;

	Vgg(0) = 1;
	Vgg(1) = 1;
	Vgg(2) = 0;
#endif

	// wind speed (body frame)
	Vector3f Vba(0, 0, 0);
	Vba(0) = windvane_sensor.speed_horiz * cosf(radians(windvane_sensor.angle_horiz));
	Vba(1) = windvane_sensor.speed_horiz * sinf(radians(windvane_sensor.angle_horiz));
	Vba(2) = windvane_sensor.speed_vert * cosf(radians(windvane_sensor.angle_vert));

	matrix::Dcmf matrix  = matrix::Dcmf{matrix::Eulerf{
							euler.phi(),
							euler.theta(),
							euler.psi()}};

	// wind speed (world frame)
	Vector3f Vga(0, 0, 0);
	Vga = matrix * Vba;

	// real wind speed (world frame)
	Vector3f Vgw = Vgg - Vga;

	// horizontal wind speed
	float Vgwxy_len = sqrtf(Vgw(0) * Vgw(0) + Vgw(1) * Vgw(1));

	// horizontal wind angle
	float Owxy = 0;

	// horizontal wind vector
	Vector2f Vgwxy(Vgw(0), Vgw(1));
	Vector2f n(1, 0);

	// calculate horizontal wind angle
	if (Vgw(1) > 0) {
		Owxy = degrees(acosf((Vgwxy * n) / (Vgwxy_len * n.length())));
	} else if (Vgw(1) < 0) {
		Owxy = 360 -degrees(acosf((Vgwxy * n) / (Vgwxy_len * n.length())));
	}

	windvane.timestamp = hrt_absolute_time();

	windvane.lat = global_pos.lat;
	windvane.lon = global_pos.lon;
	windvane.alt = global_pos.alt;

	windvane.ground_speed[0] = local_pos.vx;
	windvane.ground_speed[1] = local_pos.vy;
	windvane.ground_speed[2] = local_pos.vz;

	windvane.attitude[0] = degrees(euler.phi());
	windvane.attitude[1] = degrees(euler.theta());
	windvane.attitude[2] = degrees(wrap_2pi(euler.psi()));

	windvane.speed_horiz_sensor = windvane_sensor.speed_horiz;
	windvane.angle_horiz_sensor = windvane_sensor.angle_horiz;
	windvane.speed_vert_sensor = windvane_sensor.speed_vert;
	windvane.angle_vert_sensor = windvane_sensor.angle_vert;

	windvane.wind_speed_3d[0] = Vgw(0);
	windvane.wind_speed_3d[1] = Vgw(1);
	windvane.wind_speed_3d[2] = Vgw(2);

	windvane.wind_speed_horiz = Vgwxy_len;
	windvane.wind_angle_horiz = Owxy;

	PX4_INFO("Vgw %.2f %.2f %.2f", (double)Vgw(0), (double)Vgw(1), (double)Vgw(2));
	PX4_INFO("Vgwxy %.2f", (double)Vgwxy_len);
	PX4_INFO("Owxy %.2f", (double)Owxy);

	// publish to mavlink interface
	_windvane_pub.publish(windvane);

	log_on_sdcard();
}

void WINDVANE_ESTIMATOR::log_on_sdcard()
{
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

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int windvane_estimator_main(int argc, char *argv[])
{
	return WINDVANE_ESTIMATOR::main(argc, argv);
}
