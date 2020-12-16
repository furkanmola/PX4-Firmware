/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
 *   Author: Holger Steinhaus <hsteinhaus@gmx.de>
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
 * @file motor_test.c
 *
 * Tool for drive testing
 */

#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/test_motor.h>

extern "C" __EXPORT int motor_test_main(int argc, char *argv[]);

static void motor_test(unsigned channel, float value, uint8_t driver_instance, int timeout_ms);
static void usage(const char *reason);

static int _min_pwm = 1000;
static int _max_pwm = 2000;
static float _ramp_time;
static bool _thread_should_exit = false;		/**< motor_ramp exit flag */
static bool _thread_running = false;		/**< motor_ramp status flag */
static int _motor_ramp_task;

int motor_ramp_thread_main_test(int argc, char *argv[]);

enum Mode {
	RAMP
};

static Mode _mode;

void motor_test(unsigned channel, float value, uint8_t driver_instance, int timeout_ms)
{
	test_motor_s test_motor{};
	test_motor.timestamp = hrt_absolute_time();
	test_motor.motor_number = channel;
	test_motor.value = value;
	test_motor.action = value >= 0.f ? test_motor_s::ACTION_RUN : test_motor_s::ACTION_STOP;
	test_motor.driver_instance = driver_instance;
	test_motor.timeout_ms = timeout_ms;

	uORB::Publication<test_motor_s> test_motor_pub{ORB_ID(test_motor)};
	test_motor_pub.publish(test_motor);

	if (test_motor.action == test_motor_s::ACTION_STOP) {
		PX4_INFO("motors stop command sent");

	} else {
		/* Adjust for 1-based motor indexing */
		//PX4_INFO("motor %d set to %.2f", channel + 1, (double)value);
	}
}

static void usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Utility to test motors.

WARNING: remove all props before using this command.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("motor_test", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Set motor(s) to a specific output value");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 1, 8, "Motor to test (1...8, all if not specified)", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 100, "Power (0...100)", true);
	PRINT_MODULE_USAGE_PARAM_INT('t', 0, 0, 100, "Timeout in seconds (default=no timeout)", true);
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 4, "driver instance", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop all motors");
	PRINT_MODULE_USAGE_COMMAND_DESCR("ramp", "Implement ramp function to motors");
	PRINT_MODULE_USAGE_PARAM_INT('r', 1, 0, 20, "Time of ramp function", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("iterate", "Iterate all motors starting and stopping one after the other");

}

int motor_test_main(int argc, char *argv[])
{
	int channel = -1; //default to all channels
	unsigned long lval;
	uint8_t driver_instance = 0;
	_thread_should_exit = false;

	int ch;
	int timeout_ms = 0;

	//float dt = 0.001f;
	float value = 0.0f;
	//float timer = 0.0f;
	//bool error_flag = false;
	//hrt_abstime last_run = 0;
	//if(last_run > 0)
	//{
	//	dt = hrt_elapsed_time(&last_run) * 1e-7;
	//}

	//last_run = hrt_absolute_time();

	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "i:m:p:t:r:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		case 'i':
			driver_instance = (uint8_t)strtol(myoptarg, nullptr, 0);
			break;

		case 'm':
			/* Read in motor number and adjust for 1-based indexing */
			channel = (int)strtol(myoptarg, nullptr, 0) - 1;
			break;

		case 'p':
			/* Read in power value */
			lval = strtoul(myoptarg, nullptr, 0);

			if (lval > 100) {
				usage("value invalid");
				return 1;
			}

			value = ((float)lval) / 100.f;
			break;

		case 't':
			timeout_ms = strtol(myoptarg, nullptr, 0) * 1000;
			break;
		case 'r':
			_ramp_time = atof(myoptarg) * 2.0;

			if (_ramp_time <= 0) {
				usage("ramp time must be greater than 0");
				//error_flag = true;
			}

			break;

		default:
			usage(nullptr);
			return 1;
		}
	}

	bool run_test = true;

	if (myoptind >= 0 && myoptind < argc) {

		if (strcmp("stop", argv[myoptind]) == 0) {
			channel = 0;
			value = -1.f;

		} else if (strcmp("iterate", argv[myoptind]) == 0) {
			value = 0.15f;

			for (int i = 0; i < 8; ++i) {
				motor_test(i, value, driver_instance, 0);
				px4_usleep(500000);
				motor_test(i, -1.f, driver_instance, 0);
				px4_usleep(10000);
			}

			run_test = false;

		} else if (strcmp("test", argv[myoptind]) == 0) {
			// nothing to do
		} else if (strcmp("ramp", argv[myoptind]) == 0){
			_mode = RAMP;
			run_test = false;
		}else {
			usage(nullptr);
			return 0;
		}

	} else {
		usage(nullptr);
		return 0;
	}

	if (run_test) {
		if (channel < 0) {
			for (int i = 0; i < 8; ++i) {
				motor_test(i, value, driver_instance, timeout_ms);
				px4_usleep(10000);
			}

		} else {
			motor_test(channel, value, driver_instance, timeout_ms);
		}
	}

	_motor_ramp_task = px4_task_spawn_cmd("motor_ramp",
					      SCHED_DEFAULT,
					      SCHED_PRIORITY_DEFAULT + 40,
					      2000,
					      motor_ramp_thread_main_test,
					      (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);

	return 0;
}

int motor_ramp_thread_main_test(int argc, char *argv[])
{
	_thread_running = true;

	int myoptind = 1;
	float value = 0.0f;
	uint8_t driver_instance = 0;
	float dt = 0.001f;
	hrt_abstime last_run = 0;
	if(last_run > 0)
	{
		dt = hrt_elapsed_time(&last_run) * 1e-7;
	}

	last_run = hrt_absolute_time();


	while (!_thread_should_exit) {

		if (!strcmp(argv[myoptind], "stop"))
		{
			_thread_should_exit = true;
			value = 0.0f;

		}
		if (_mode == RAMP){

			value = 0;
			while(value <= 1.0f)
			{
				value += 1000.0f * dt / (_max_pwm - _min_pwm) / _ramp_time;
				motor_test(0, value, driver_instance, 0);
				motor_test(1, value, driver_instance, 0);
				motor_test(2, value, driver_instance, 0);
				motor_test(3, value, driver_instance, 0);
			}
			while(value >= 0.0f)
			{
				value -= 1000.0f * dt / (_max_pwm - _min_pwm) / _ramp_time;
				motor_test(0, value, driver_instance, 0);
				motor_test(1, value, driver_instance, 0);
				motor_test(2, value, driver_instance, 0);
				motor_test(3, value, driver_instance, 0);
			}
			_thread_should_exit = true;
		}
	}
	_thread_running = false;
	return 0;
}
