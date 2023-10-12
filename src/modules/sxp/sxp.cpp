/****************************************************************************
 *
 *   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
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
 * @file sxp.hpp
 * Simulator with X-Plane (SXP)
 *
 * @author Romain Chiappinelli      <romain.chiap@gmail.com>
 *
 * Altitude R&D Chiappinellli - October 2023
 */

#include "sxp.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <drivers/drv_pwm_output.h>         // to get PWM flags
#include <lib/drivers/device/Device.hpp>

using namespace math;
using namespace matrix;
using namespace time_literals;

Sxp::Sxp() :
	ModuleParams(nullptr)
{}

Sxp::~Sxp()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

void Sxp::run()
{
	parameters_updated();
	init_variables();
	// gps_no_fix();

	// const hrt_abstime task_start = hrt_absolute_time();
	_last_run = hrt_absolute_time();
	// _gps_time = task_start;
	// _airspeed_time = task_start;
	// _time = task_start;
	// _dist_snsr_time = task_start;
	_vehicle = (VehicleType)constrain(_sih_vtype.get(), static_cast<typeof _sih_vtype.get()>(0),
					  static_cast<typeof _sih_vtype.get()>(2));

	if (_sys_ctrl_alloc.get()) {
		_actuator_out_sub = uORB::Subscription{ORB_ID(actuator_outputs_sim)};
	}

	realtime_loop();

	exit_and_cleanup();
}

void Sxp::realtime_loop()
{
	int rate = _imu_gyro_ratemax.get();

	// default to 250 Hz (4000 us interval)
	if (rate <= 0) {
		rate = 250;
	}

	// 200 - 2000 Hz
	int interval_us = math::constrain(int(roundf(1e6f / rate)), 500, 5000);

	px4_sem_init(&_data_semaphore, 0, 0);
	hrt_call_every(&_timer_call, interval_us, interval_us, timer_callback, &_data_semaphore);

	while (!should_exit()) {
		px4_sem_wait(&_data_semaphore);     // periodic real time wakeup
		perf_begin(_loop_perf);
		perf_count(_loop_interval_perf);
		read_motors();
		publish_sxp();
		perf_end(_loop_perf);
	}

	hrt_cancel(&_timer_call);
	px4_sem_destroy(&_data_semaphore);
}


void Sxp::timer_callback(void *sem)
{
	px4_sem_post((px4_sem_t *)sem);
}

// store the parameters in a more convenient form
void Sxp::parameters_updated()
{

}

// initialization of the variables for the simulator
void Sxp::init_variables()
{
	srand(1234);    // initialize the random seed once before calling generate_wgn()

	_p_I = Vector3f(0.0f, 0.0f, 0.0f);
	_v_I = Vector3f(0.0f, 0.0f, 0.0f);
	_q = Quatf(1.0f, 0.0f, 0.0f, 0.0f);
	_w_B = Vector3f(0.0f, 0.0f, 0.0f);

	_u[0] = _u[1] = _u[2] = _u[3] = 0.0f;

	_px4_accel.set_temperature(T1_C);
	_px4_gyro.set_temperature(T1_C);
	// _px4_mag.set_temperature(T1_C);

	// open the socket for Xplane connect
	//IP Address of computer running X-Plane
	_xpc_sock = openUDP("127.0.0.1");

	float values[2] = {1,1};
	int size = 1;
	sendDREF(_xpc_sock, "sim/operation/override/override_joystick", values, size);

}

// read the motor signals outputted from the mixer
void Sxp::read_motors()
{
	actuator_outputs_s actuators_out;

	float pwm_middle = 0.5f * (PWM_DEFAULT_MIN + PWM_DEFAULT_MAX);

	if (_actuator_out_sub.update(&actuators_out)) {
		_last_actuator_output_time = actuators_out.timestamp;

		if (_sys_ctrl_alloc.get()) {
			for (int i = 0; i < NB_MOTORS; i++) { // saturate the motor signals
				_u[i] = actuators_out.output[i];
			}

		} else {
			for (int i = 0; i < NB_MOTORS; i++) { // saturate the motor signals
				if ((_vehicle == VehicleType::FW && i < 3) || (_vehicle == VehicleType::TS
						&& i > 3)) { // control surfaces in range [-1,1]
					_u[i] = constrain(2.0f * (actuators_out.output[i] - pwm_middle) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN), -1.0f, 1.0f);

				} else { // throttle signals in range [0,1]
					_u[i] = constrain((actuators_out.output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN), 0.0f, 1.0f);
				}
			}
		}
	}

	float command[6]={-_u[1],_u[0],_u[2],_u[3]};
	// [Elevator, Aileron, Rudder, Throttle, Gear, Flaps, Speed Brakes]
	int size = 4;
	_sendCTRLres = sendCTRL(_xpc_sock, command, size, 0);
}

void Sxp::publish_sxp()
{
	// [Lat, Lon, Alt, Pitch, Roll, Yaw, Gear]
	_getPOSIres = getPOSI(_xpc_sock, _v_states, 0);
	if (_getPOSIres==0) {
		_now = hrt_absolute_time();
		_dt = (_now - _last_run) * 1e-6f;
		if (_dt < 1.0e-5f) {
			return;
		}
		_last_run = _now;

		// publish attitude
		_att.timestamp = hrt_absolute_time();
		_rpy = matrix::Eulerf(radians(_v_states[4]), radians(_v_states[3]), radians(_v_states[5]));
		Quatf q = matrix::Quatf(_rpy);
		q.copyTo(_att.q);
		_att_pub.publish(_att);

		// compute the angular rates
		Eulerf _rpy_dot = (_rpy - _rpy_old) / _dt;
		_rpy_old = _rpy;
		float S_[3][3] = {
			{1, 0, -sinf(_rpy.theta())},
			{0, cosf(_rpy.phi()), sinf(_rpy.phi())*cosf(_rpy.theta())},
			{0, -sinf(_rpy.phi()), cosf(_rpy.phi())*cosf(_rpy.theta())}
		};
		Matrix3f S = Matrix3f(S_);
		_w_B = S*_rpy_dot;	// correct transformation

		// publish angular velocity groundtruth
		_vehicle_angular_velocity.timestamp = hrt_absolute_time();
		_vehicle_angular_velocity.xyz[0] = _w_B(0); // rollspeed;
		_vehicle_angular_velocity.xyz[1] = _w_B(1); // pitchspeed;
		_vehicle_angular_velocity.xyz[2] = _w_B(2); // yawspeed;
		_vehicle_angular_velocity_pub.publish(_vehicle_angular_velocity);

		// publish the global position
		_gpos.timestamp = hrt_absolute_time();
		_gpos.lat = _v_states[0];
		_gpos.lon = _v_states[1];
		_gpos.alt = (float)_v_states[2];
		_gpos_pub.publish(_gpos);

		// publish simulated estimator status
		_estim_s.timestamp_sample = hrt_absolute_time();
		_estim_s.timestamp = hrt_absolute_time();
		_estim_s.output_tracking_error[0]=0.0019f;
		_estim_s.output_tracking_error[1]=0.0106f;
		_estim_s.output_tracking_error[2]=0.0229f;
		_estim_s.control_mode_flags = 2147484183;
		_estim_s.pos_horiz_accuracy = 0.2048f;
		_estim_s.pos_vert_accuracy = 0.3553f;
		_estim_s.mag_test_ratio = 0.2860f;
		_estim_s.vel_test_ratio = 0.0338f;
		_estim_s.pos_test_ratio = 0.0644f;
		_estim_s.hgt_test_ratio = 0.0059f;
		_estim_s.accel_device_id = 1310988;
		_estim_s.gyro_device_id = 1310988;
		_estim_s.baro_device_id = 6620172;
		_estim_s.mag_device_id = 197388;
		_estim_s.solution_status_flags = 895;
		_estim_s_pub.publish(_estim_s);

		// publish random sensor value so the commander passes the check
		_px4_accel.update(_now, 0, 0, 9.81f);
		_px4_gyro.update(_now, _w_B(0), _w_B(1), _w_B(2));
		// _px4_mag.update(_now, 0, 0, 0);
		// sensor_baro_s sensor_baro{};
		// sensor_baro.timestamp_sample = _now;
		// sensor_baro.device_id = 6620172; // 6620172: DRV_BARO_DEVTYPE_BAROSIM, BUS: 1, ADDR: 4, TYPE: SIMULATION
		// sensor_baro.timestamp = _now;
		// _sensor_baro_pub.publish(sensor_baro);
	} else if (_getPOSIres==-3) {
		_xpc_length_error_count++;
	}
	_total_loops++;
}


int Sxp::print_status()
{

	PX4_INFO("Running Fixed-Wing simulator with X-Plane");

	// PX4_INFO("vehicle landed: %d", _grounded);
	PX4_INFO("dt [us]: %d", (int)(_dt * 1e6f));
	PX4_INFO("xpc socket.port: %d, xpc socket.xpPort: %d, xpc socket.sock: %d", (int)_xpc_sock.port, (int)_xpc_sock.xpPort, _xpc_sock.sock);
	PX4_INFO("last send control res: %d, last get posi res: %d", _sendCTRLres, _getPOSIres);
	PX4_INFO("length error count: %u, %%: %.2f", _xpc_length_error_count, ((double)_xpc_length_error_count)*100/_total_loops);

	PX4_INFO("inertial position NED (m)");
	_p_I.print();
	PX4_INFO("inertial velocity NED (m/s)");
	_v_I.print();
	PX4_INFO("attitude roll-pitch-yaw (deg)");
	(matrix::Eulerf(_q) * 180.0f / M_PI_F).print();
	PX4_INFO("angular acceleration roll-pitch-yaw (deg/s)");
	(_w_B * 180.0f / M_PI_F).print();
	PX4_INFO("actuator signals");
	Vector<float, 8> u = Vector<float, 8>(_u);
	u.transpose().print();
	PX4_INFO("Global pos (lat, lon, alt): %.2f, %.2f, %.2f", _v_states[0], _v_states[1], _v_states[2]);
	PX4_INFO("Attitude (roll, pitch, yaw): %.2f, %.2f, %.2f", _v_states[4], _v_states[3], _v_states[5]);
	return 0;
}


int Sxp::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("sih",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_MAX,
				      1250,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

Sxp *Sxp::instantiate(int argc, char *argv[])
{
	Sxp *instance = new Sxp();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

int Sxp::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Sxp::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module provide a simulator for fixed-wing running with X-Plane.

This simulator subscribes to "actuator_outputs" which are the actuator pwm
signals given by the mixer.

This simulator publishes the sensors signals corrupted with realistic noise
in order to incorporate the state estimator in the loop.

### Implementation
The simulator implements the equations of motion using matrix algebra.
Quaternion representation is used for the attitude.
Forward Euler is used for integration.
Most of the variables are declared global in the .hpp file to avoid stack overflow.


)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("sxp", "simulation");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int sxp_main(int argc, char *argv[])
{
	return Sxp::main(argc, argv);
}
