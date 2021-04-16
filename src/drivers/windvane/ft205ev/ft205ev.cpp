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

#include <fcntl.h>

FT205EV::FT205EV(const char *port1, const char *port2) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port1))
{
	// store port name
	strncpy(_ports[0], port1, sizeof(_ports[0]) - 1);

	// enforce null termination
	_ports[0][sizeof(_ports[0]) - 1] = '\0';

	// store port name
	strncpy(_ports[1], port2, sizeof(_ports[1]) - 1);

	// enforce null termination
	_ports[1][sizeof(_ports[1]) - 1] = '\0';
}

FT205EV::~FT205EV()
{
	// make sure we are truly inactive
	stop();

	if (windvane_nmea[0] != nullptr) {
		delete windvane_nmea[0] ;
		windvane_nmea[0] = nullptr;
	}

	if (windvane_nmea[1] != nullptr) {
		delete windvane_nmea[1] ;
		windvane_nmea[1] = nullptr;
	}

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
FT205EV::init()
{
	int ret1 = init_ports(0);
	int ret2 = init_ports(1);
	if (ret1 == PX4_OK && ret2 == PX4_OK) {
		start();
	}

	return ret1 & ret2;
}

int
FT205EV::init_ports(int port)
{
	// status
	int ret = 0;

	do { // create a scope to handle exit conditions using break

		// open fd
		_fd[port] = ::open(_ports[port], O_RDWR | O_NOCTTY);

		if (_fd[port] < 0) {
			PX4_ERR("Error opening fd");
			return -1;
		}

		// baudrate 115200, 8 bits, no parity, 1 stop bit
		unsigned speed = B115200;
		termios uart_config{};
		int termios_state{};

		tcgetattr(_fd[port], &uart_config);

		// clear ONLCR flag (which appends a CR for every LF)
		uart_config.c_oflag &= ~ONLCR;

		// set baud rate
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d ISPD", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_ERR("CFG: %d OSPD\n", termios_state);
			ret = -1;
			break;
		}

		if ((termios_state = tcsetattr(_fd[port], TCSANOW, &uart_config)) < 0) {
			PX4_ERR("baud %d ATTR", termios_state);
			ret = -1;
			break;
		}

		uart_config.c_cflag |= (CLOCAL | CREAD);	// ignore modem controls
		uart_config.c_cflag &= ~CSIZE;
		uart_config.c_cflag |= CS8;			// 8-bit characters
		uart_config.c_cflag &= ~PARENB;			// no parity bit
		uart_config.c_cflag &= ~CSTOPB;			// only need 1 stop bit
		uart_config.c_cflag &= ~CRTSCTS;		// no hardware flowcontrol

		// setup for non-canonical mode
		uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		uart_config.c_oflag &= ~OPOST;

		// fetch bytes as they become available
		uart_config.c_cc[VMIN] = 1;
		uart_config.c_cc[VTIME] = 1;

		if (_fd[port] < 0) {
			PX4_ERR("FAIL: laser fd");
			ret = -1;
			break;
		}
	} while (0);

	// close the fd
	::close(_fd[port]);
	_fd[port] = -1;

	return ret;
}

int
FT205EV::collect(int port)
{
	// the buffer for read chars is buflen minus null termination
	char readbuf[27] {};
	unsigned readlen = sizeof(readbuf);

	// Check the number of bytes available in the buffer
	int bytes_available = 0;
	::ioctl(_fd[port], FIONREAD, (unsigned long)&bytes_available);

	if (!bytes_available) {
		return 0;
	}

	int ret = 0;
	do {
		// read from the sensor (uart buffer)
		ret = ::read(_fd[port], &readbuf[0], readlen);

		if (ret < 0) {
			PX4_ERR("read err: %d", ret);
			return -EAGAIN;
		}

		// parse buffer
		for (int i = 0; i < ret; i++) {
			if (windvane_nmea[port]->decode(readbuf[i])) {
				updated[port] = true;
			}
		}

		// bytes left to parse
		bytes_available -= ret;

	} while (bytes_available > 0);

	return PX4_OK;
}

void
FT205EV::start()
{
	windvane_nmea[0] = new AP_WindVane_NMEA();
	windvane_nmea[1] = new AP_WindVane_NMEA();
	// schedule a cycle to start things (the sensor sends at 10Hz, but we run a bit faster to avoid missing data)
	ScheduleOnInterval(10_ms);
}

void
FT205EV::stop()
{
	ScheduleClear();
}

void
FT205EV::Run()
{
	// fds initialized?
	if (_fd[0] < 0) {
		// open fd
		_fd[0] = ::open(_ports[0], O_RDWR | O_NOCTTY);
	}

	if (_fd[1] < 0) {
		// open fd
		_fd[1] = ::open(_ports[1], O_RDWR | O_NOCTTY);
	}

	perf_begin(_sample_perf);

	if (collect(0) == -EAGAIN) {
		perf_count(_comms_errors);

	}

	if (collect(1) == -EAGAIN) {
		perf_count(_comms_errors);
	}

	if (updated[0] && updated[1]) {
		float speed_horiz = windvane_nmea[0]->get_wind_speed();
		float angle_horiz = windvane_nmea[0]->get_wind_dir();

		float speed_vert = windvane_nmea[1]->get_wind_speed();
		float angle_vert = windvane_nmea[1]->get_wind_dir();

		windvane_sensor_s windvane_sensor{};
		windvane_sensor.timestamp = hrt_absolute_time();
		windvane_sensor.speed_horiz = speed_horiz;
		windvane_sensor.angle_horiz = angle_horiz;
		windvane_sensor.speed_vert = speed_vert;
		windvane_sensor.angle_vert = angle_vert;

		_windvane_sensor_pub.publish(windvane_sensor);

		updated[0] = false;
		updated[1] = false;
	}

	perf_end(_sample_perf);
}

void
FT205EV::print_info()
{
	printf("Using port '%s' '%s'\n", _ports[0], _ports[1]);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
