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

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

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
			//PX4_INFO("time %llu\n", windvane.timestamp);
		}
	}

	vehicle_global_position_s global_pos;
	vehicle_local_position_s local_pos;

	if (_global_pos_sub.update(&global_pos) && _local_pos_sub.update(&local_pos)) {

	}

	vehicle_attitude_s attitude;

	if (_att_sub.update(&attitude)) {
		const matrix::Eulerf euler(matrix::Quatf(attitude.q));
		const float roll(euler.phi());
		const float pitch(euler.theta());
		const float yaw(euler.psi());

		PX4_INFO("roll pitch yaw %.2f %.2f %.2f\n", (double)roll, (double)pitch, (double)yaw);
	}

	ScheduleDelayed(CONTROLLER_PERIOD_DEFAULT);
}

void FT205EV::update()
{
	// two serial needs
	if (_serial_fd[0] < 0 || _serial_fd[1] < 0) {
		return;
	}

	for (uint8_t i = 0; i < 2; i++) {
		// read from the uart. This must be non-blocking, so check first if there is data available
		int bytes_available = 0;
		int ret = ioctl(_serial_fd[i], FIONREAD, (unsigned long)&bytes_available);

		if (ret != 0 || bytes_available <= 0) {
			continue;
		}

		const int buf_length = 24;
		uint8_t buf[buf_length];

		int num_read = read(_serial_fd[i], buf, buf_length);

		for (int j = 0; j < num_read; ++j) {
			for (int k = 0; k < 24 - 1; k++) {
				_frame_buffer[i][k] = _frame_buffer[i][k + 1];
			}
			_frame_buffer[i][ 24 - 1] = buf[j];

			// $WI,WVP=020.0,045,0*73<cr><lf>
 		}

	}
}

void FT205EV::initialize_ports()
{
	const char *port1 = "/dev/ttyS3";
	const char *port2 = "/dev/ttyS4";
	/* open the serial port */
	_serial_fd[0] = ::open(port1, O_RDWR | O_NOCTTY);

	if (_serial_fd[0] < 0) {
		PX4_ERR("FT205EV: failed to open serial port: %s err: %d", port1, errno);
		return;
	}

	_serial_fd[1] = ::open(port2, O_RDWR | O_NOCTTY);

	if (_serial_fd[1] < 0) {
		PX4_ERR("FT205EV: failed to open serial port: %s err: %d", port2, errno);
		return;
	}

	if (setBaudrate(0, 9600) != 0) {
		PX4_ERR("FT205EV: failed to set serial port: %s err: %d", port1, errno);
	}

	if (setBaudrate(1, 9600) != 0) {
		PX4_ERR("FT205EV: failed to set serial port: %s err: %d", port2, errno);
	}
}

int FT205EV::setBaudrate(unsigned index, unsigned baud)
{
	int speed;

	switch (baud) {
	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

	default:
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_serial_fd[index], &uart_config);

	//
	// Input flags - Turn off input processing
	//
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
				 INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	//
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	uart_config.c_oflag = 0;

	//
	// No line processing
	//
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	/* no parity, one stop bit, disable flow control */
	uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		return -errno;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		return -errno;
	}

	if ((termios_state = tcsetattr(_serial_fd[index], TCSANOW, &uart_config)) < 0) {
		return -errno;
	}

	return 0;
}

int FT205EV::print_status()
{
	PX4_INFO("Sensor ID: - Temperature: Setpoint: FT205EV State:");

	return PX4_OK;
}

int FT205EV::start()
{
	initialize_ports();

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
