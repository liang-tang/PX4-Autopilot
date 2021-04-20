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

#include <drivers/drv_adc.h>

#include <dirent.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

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
	: ModuleParams(nullptr)
{
}

WINDVANE_ESTIMATOR::~WINDVANE_ESTIMATOR()
{
	// close fd
	if (_fd > 0) {
		::close(_fd);
		_fd = -1;
	}
}

void WINDVANE_ESTIMATOR::run()
{
	while (!should_exit()) {
		// run at 10hz
		px4_usleep(100000);
		calculate_and_publish();
	}
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

	// copy gps position
	if (_gps_position_sub.updated()) {
		_gps_position_sub.copy(&gps_pos);
	}

	// copy adc report
	if (_adc_report_sub.updated()) {
		_adc_report_sub.copy(&adc_report);
	}

	// copy airspeed
	if (_airspeed_sub.updated()) {
		_airspeed_sub.copy(&airspeed);
	}

	// copy vehicle air data
	if (_vehicle_air_data_sub.updated()) {
		_vehicle_air_data_sub.copy(&vehicle_air_data);
	}

	// copy differential pressure
	if (_diff_pres_sub.updated()) {
		_diff_pres_sub.copy(&differential_pressure);
	}

	// voltage raw
	float voltage_aoa_raw = 0.0f;
	float voltage_aos_raw = 0.0f;

	// get voltage raw data
	for (unsigned i = 0; i < PX4_MAX_ADC_CHANNELS; ++i) {
		// 6.6v ADC
		if (adc_report.channel_id[i] == 4) {
			voltage_aoa_raw = adc_report.raw_data[i];
		}

		// 3.3v ADC
		if (adc_report.channel_id[i] == 14) {
			voltage_aos_raw = adc_report.raw_data[i];
		}
	}

	// convert to real voltage
	float voltage_aoa = voltage_aoa_raw * adc_report.v_ref / adc_report.resolution * 2.0f;
	float voltage_aos = voltage_aos_raw * adc_report.v_ref / adc_report.resolution;

	// convert to AOA, AOS
	float aoa = voltage_aoa* 61.12f - 151.0f;
	float aos = voltage_aos * 61.12f - 151.0f;

	// PX4_INFO("adc4 %.2f adc14 %.2f", (double)voltage_aoa, (double)voltage_aos);

	// attitude
	Eulerf euler = Quatf(attitude.q);
	// ground speed (world frame)
	Vector3f Vgg(local_pos.vx, local_pos.vy, local_pos.vz);

	// test case
	update_test_case(euler, Vgg, aoa, aos);

	// airspeed (airflow frame)
	float Vaa = fabs(airspeed.true_airspeed_m_s) / (cosf(radians(aoa)) * cosf(radians(aos)));

	// wind speed (body frame)
	Vector3f Vba(0, 0, 0);
	Vba(0) = Vaa * cosf(radians(aoa)) * cosf(radians(aos));
	Vba(1) = Vaa * sinf(radians(aos));
	Vba(2) = Vaa * sinf(radians(aoa)) * cosf(radians(aos));

	// get current rotation matrix
	Dcmf matrix  = Dcmf{Eulerf{
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
	if (fabsf(Vgw(1)) < FLT_EPSILON) {
		// do nothing
	} else if (Vgw(1) > 0) {
		Owxy = degrees(acosf((Vgwxy * n) / (Vgwxy_len * n.length())));
	} else if (Vgw(1) < 0) {
		Owxy = 360 - degrees(acosf((Vgwxy * n) / (Vgwxy_len * n.length())));
	}

	// timestamp
	windvane.timestamp = hrt_absolute_time();

	// lat lon alt
	windvane.lat = global_pos.lat;
	windvane.lon = global_pos.lon;
	windvane.alt = global_pos.alt;

	// ground speed
	windvane.ground_speed[0] = local_pos.vx * 1e2f;
	windvane.ground_speed[1] = local_pos.vy * 1e2f;
	windvane.ground_speed[2] = local_pos.vz * 1e2f;

	// roll pitch yaw
	windvane.attitude[0] = degrees(euler.phi()) * 1e2f;
	windvane.attitude[1] = degrees(euler.theta()) * 1e2f;
	windvane.attitude[2] = degrees(wrap_2pi(euler.psi())) * 1e2f;

	// voltage AOA AOS airspeed
	windvane.voltage[0] = voltage_aoa * 1e2f;
	windvane.voltage[1] = voltage_aos * 1e2f;
	windvane.aoa = aoa * 1e2f;
	windvane.aos = aos * 1e2f;
	windvane.airspeed = airspeed.true_airspeed_m_s * 1e2f;

	// differential pressure, baro pressure
	windvane.differential_pressure_pa = differential_pressure.differential_pressure_filtered_pa;
	windvane.baro_pressure_pa = vehicle_air_data.baro_pressure_pa;

	// wind speed
	windvane.wind_speed_3d[0] = Vgw(0);
	windvane.wind_speed_3d[1] = Vgw(1);
	windvane.wind_speed_3d[2] = Vgw(2);

	// horizontal wind speed angle
	windvane.wind_speed_horiz = Vgwxy_len;
	windvane.wind_angle_horiz = Owxy;

	// PX4_INFO("Vgw %.2f %.2f %.2f", (double)Vgw(0), (double)Vgw(1), (double)Vgw(2));
	// PX4_INFO("Vgwxy %.2f", (double)Vgwxy_len);
	// PX4_INFO("Owxy %.2f", (double)Owxy);

	// publish to mavlink interface
	_windvane_pub.publish(windvane);

	// log
	log_on_sdcard();
}

void WINDVANE_ESTIMATOR::update_test_case(Eulerf &euler, Vector3f &Vgg, float &aoa, float &aos)
{
	int test_case = _test_case.get();

	if (test_case == 0) {
		return;
	}

	if (test_case == 1) {
		Vgg(0) = 0.0f;
		Vgg(1) = 0.0f;
		Vgg(2) = 0.0f;
		aoa = 60;
		aos = 0;

		euler.phi() = radians(0.0f);
		euler.theta() = radians(15.0f);
		euler.psi() = radians(0.0f);

		airspeed.true_airspeed_m_s = 1.0f;

	} else if (test_case == 2) {
		euler.phi() = radians(0.0f);
		euler.theta() = radians(0.0f);
		euler.psi() = radians(-60.0f);

		Vgg(0) = 0.0f;
		Vgg(1) = 0.0f;
		Vgg(2) = 0.0f;

		aoa = 0;
		aos = 60;
		airspeed.true_airspeed_m_s = 1.0f;
	}
}

void WINDVANE_ESTIMATOR::log_on_sdcard()
{
	// get current utc time
	struct timespec ts = {};
	px4_clock_gettime(CLOCK_REALTIME, &ts);
	time_t utc_time_sec = ts.tv_sec + (ts.tv_nsec / 1e9);
	time_t utc_time_ms = ts.tv_nsec / 1e6;

	// valid time?
	if (utc_time_sec < GPS_EPOCH_SECS || gps_pos.fix_type < 3) {
		return;
	}

	// apply utc offset (8 hours)
	utc_time_sec += 28800;

	// convert to gm time
	tm tt = {};
	if (gmtime_r(&utc_time_sec, &tt) == nullptr) {
		return;
	}

	char time_now_str[32] = "";
	// format time string (eg. 2021_04_18-12_00_00)
	strftime(time_now_str, sizeof(time_now_str), "%Y_%m_%d-%H_%M_%S", &tt);

	if (_fd == -1) {
		char log_file_name_str[64] = "";
		// format file string (eg. /fs/microsd/2021_04_18-12_00_00.csv)
		snprintf(log_file_name_str, sizeof(log_file_name_str), PX4_STORAGEDIR"/%s.csv", time_now_str);

		// create file on sdcard
		_fd = ::open(log_file_name_str, O_CREAT | O_WRONLY, PX4_O_MODE_666);
		if (_fd == -1) {
			PX4_INFO("open error");
			return;
		}

		// log file head (first line content)
		const char* head =
				"Time,"
				"Lat,Lon,Alt,"
				"ground_speed.x,ground_speed.y,ground_speed.z,"
				"Roll,Pitch,Yaw,"
				"voltage1,voltage2,"
				"aoa,aos,"
				"airspeed,"
				"differential_pressure_pa,"
				"baro_pressure_pa,"
				"wind_speed_3d.x,wind_speed_3d.y, wind_speed_3d.z,"
				"wind_speed_horiz,wind_angle_horiz\n";
		// write to file
		::write(_fd, head, strlen(head));
		// sync file
		fsync(_fd);
	}

	// log string
	char *log_str = nullptr;
	// format data
	if (asprintf(&log_str, "%s:%03u,"
			    "%.7f,%.7f,%.2f," // latitude longitude alt
			    "%.2f,%.2f,%.2f," // ground_speed
			    "%.2f,%.2f,%.2f," // attitude
			    "%.2f,%.2f,"      // voltage
			    "%.2f,%.2f,"      // aoa aos
			    "%.2f,"           // airspeed
			    "%.2f,"           // differential_pressure_pa
			    "%.2f,"           // baro_pressure_pa
			    "%.2f,%.2f,%.2f," // wind_speed_3d
			    "%.2f,%.2f\n",    // wind_speed_horiz, wind_angle_horiz
		&time_now_str[11], utc_time_ms,
		windvane.lat, windvane.lon, (double)windvane.alt,
		(double)windvane.ground_speed[0]*1e-2, (double)windvane.ground_speed[1]*1e-2, (double)windvane.ground_speed[2]*1e-2,
		(double)windvane.attitude[0]*1e-2, (double)windvane.attitude[1]*1e-2, (double)windvane.attitude[2]*1e-2,
		(double)windvane.voltage[0]*1e-2, (double)windvane.voltage[1]*1e-2,
		(double)windvane.aoa*1e-2, (double)windvane.aos*1e-2,
		(double)windvane.airspeed*1e-2,
		(double)windvane.differential_pressure_pa,
		(double)windvane.baro_pressure_pa,
		(double)windvane.wind_speed_3d[0],
		(double)windvane.wind_speed_3d[1],
		(double)windvane.wind_speed_3d[2],
		(double)windvane.wind_speed_horiz,
		(double)windvane.wind_angle_horiz) < 0) {
		return;
	}

	// write to file
	::write(_fd, log_str, strlen(log_str));

	// sync to file
	fsync(_fd);

	// free log_str
	free(log_str);
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
