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
	_last_gyro = hrt_absolute_time();
	_last_gpos = hrt_absolute_time();
	// _gps_time = task_start;
	// _airspeed_time = task_start;
	// _time = task_start;
	// _dist_snsr_time = task_start;
	// _vehicle = (VehicleType)constrain(_sih_vtype.get(), static_cast<typeof _sih_vtype.get()>(0),
	// 				  static_cast<typeof _sih_vtype.get()>(2));

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
	_sensor_baro.device_id = 6620172; // 6620172: DRV_BARO_DEVTYPE_BAROSIM, BUS: 1, ADDR: 4, TYPE: SIMULATION
	_px4_mag.set_temperature(T1_C);

	// open the socket for Xplane connect
	//IP Address of computer running X-Plane
	_xpc_sock = openUDP("127.0.0.1");

	float values[2] = {1,1};
	int size = 1;
	sendDREF(_xpc_sock, "sim/operation/override/override_joystick", values, size);

	// init the gps
	_sensor_gps.fix_type = 3;  // 3D fix
	_sensor_gps.satellites_used = 7;
	_sensor_gps.heading = NAN;
	_sensor_gps.heading_offset = NAN;
	_sensor_gps.s_variance_m_s = 0.5f;
	_sensor_gps.c_variance_rad = 0.1f;
	_sensor_gps.eph = 0.9f;
	_sensor_gps.epv = 1.78f;
	_sensor_gps.hdop = 0.7f;
	_sensor_gps.vdop = 1.1f;

	// init the estimator
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
}

// read the motor signals outputted from the mixer
void Sxp::read_motors()
{
	actuator_outputs_s actuators_out;

	float pwm_middle = 0.5f * (PWM_DEFAULT_MIN + PWM_DEFAULT_MAX);

	if (_actuator_out_sub.update(&actuators_out)) {
		_last_actuator_output_time = actuators_out.timestamp;

		if (_sys_ctrl_alloc.get()) {
			for (int i = 0; i < NB_ACTUATORS; i++) { // saturate the motor signals
				_u[i] = actuators_out.output[i];
			}

		} else {
			for (int i = 0; i < NB_ACTUATORS; i++) { // saturate the motor signals
				if (i != 3) { // control surfaces in range [-1,1]
					_u[i] = constrain(2.0f * (actuators_out.output[i] - pwm_middle) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN), -1.0f, 1.0f);

				} else { // throttle signals in range [0,1]
					_u[i] = constrain((actuators_out.output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN), 0.0f, 1.0f);
				}
			}
		}
	}

	// send the main commands
	float command[7]={}; 	// [Elevator, Aileron, Rudder, Throttle, Gear, Flaps, Speed Brakes]
	if (_armed) {
		command[0] = -_u[1];	// elevator
		command[1] = _u[0];	// ailerons
		// command[2] = _u[2]+_u[6];	// we mix the wheel in there
		command[2] = _u[2];	// rudder
		command[3] = _u[3];	// throttle
		command[5] = _u[4];	// flaps
	}
	int size = 6;
	_sendCTRLres = sendCTRL(_xpc_sock, command, size, 0);

	actuator_armed_s actuator_armed = {};
	_actuator_armed_sub.copy(&actuator_armed);
	if (actuator_armed.armed && !_armed) {
		// release the parking brake if we just armed
		float brake[3] = {0,0,0};
		sendDREF(_xpc_sock, "sim/flightmodel/controls/parkbrake", brake, 3);
	}
	_armed = actuator_armed.armed;
}

void Sxp::publish_sxp()
{
	// [Lat, Lon, Alt, Pitch, Roll, Yaw, Gear]
	_getPOSIres = getPOSI(_xpc_sock, _v_states, 0);
	if (_getPOSIres==0) {

		// publish attitude
		_att.timestamp = hrt_absolute_time();
		_rpy = matrix::Eulerf(radians(_v_states[4]), radians(_v_states[3]), radians(_v_states[5]));
		Quatf q = matrix::Quatf(_rpy);
		q.copyTo(_att.q);
		_att_pub.publish(_att);

		_now = hrt_absolute_time();
		// guards against derivative of a constant
		if (Vector3f(_rpy - _rpy_old).norm()>1e-7f)  {
			_dt = (_now - _last_gyro) * 1e-6f;
			if (_dt < 1.0e-5f) {
				return;
			}
			_last_gyro = _now;

			// compute the angular rates
			_rpy_dot = (_rpy - _rpy_old) / _dt;
			_rpy_old = _rpy;
			float S_[3][3] = {
				{1, 0, -sinf(_rpy.theta())},
				{0, cosf(_rpy.phi()), sinf(_rpy.phi())*cosf(_rpy.theta())},
				{0, -sinf(_rpy.phi()), cosf(_rpy.phi())*cosf(_rpy.theta())}
			};
			Matrix3f S = Matrix3f(S_);
			_w_B = S*_rpy_dot;	// correct transformation
		}

		// publish angular velocity
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
		_estim_s_pub.publish(_estim_s);

		// publish random sensor value so the commander passes the check
		_px4_accel.update(_now, 0, 0, 9.81f);
		_px4_gyro.update(_now, _w_B(0), _w_B(1), _w_B(2));
		_px4_mag.update(_now, 0, 0, 0);

		// barometer
		_sensor_baro.timestamp_sample = _now;
		_sensor_baro.timestamp = _now;
		float altitude = (float)_gpos.alt;
		float baro_p_mBar = CONSTANTS_STD_PRESSURE_MBAR *        // reconstructed pressure in mBar
			powf((1.0f + altitude * TEMP_GRADIENT / T1_K), -CONSTANTS_ONE_G / (TEMP_GRADIENT * CONSTANTS_AIR_GAS_CONST));
		float baro_temp_c = T1_K + CONSTANTS_ABSOLUTE_NULL_CELSIUS + TEMP_GRADIENT * altitude; // reconstructed temperture in celcius
		_sensor_baro.pressure = baro_p_mBar * 100.f;
		_sensor_baro.temperature = baro_temp_c;
		_sensor_baro_pub.publish(_sensor_baro);

		// pubish the local position
		if (!_map_proj.isInitialized() && fabs(_gpos.lat*_gpos.lon)>0.1) {
			_map_proj.initReference(_gpos.lat,_gpos.lon);
			_alt0 =  (float)_gpos.alt;
			_local_pos.xy_global = true;
			_local_pos.z_global = true;
			_local_pos.ref_timestamp = _now;
			_local_pos.ref_lat = _gpos.lat;
			_local_pos.ref_lon = _gpos.lon;
			_local_pos.ref_alt = _alt0;
		}
		_pos_I = _map_proj.project(_gpos.lat,_gpos.lon);
		// guards against derivative of a constant
		if (Vector2f(_pos_I - _pos_I_old).norm()>1e-7f)  {
			_dt_gpos = (_now - _last_gpos) * 1e-6f;
			if (_dt_gpos < 1.0e-5f) {
				return;
			}
			_last_gpos = _now;
			// compute the local velocity
			_vel_I = (_pos_I - _pos_I_old)/_dt_gpos;
			_local_pos.v_z_valid = true;
			_local_pos.v_xy_valid = true;
			_local_pos.vx = _vel_I(0);
			_local_pos.vy = _vel_I(1);
			float z = _alt0 - (float)_gpos.alt;
			_local_pos.vz = (z - _local_pos.z)/_dt_gpos;
		}
		_local_pos.xy_valid = true;
		_local_pos.z_valid = true;
		_local_pos.x = _pos_I(0);
		_local_pos.y = _pos_I(1);
		_local_pos.z = _alt0 - (float)_gpos.alt;
		_local_pos.timestamp = _now;
		_local_pos.timestamp_sample = _now;
		_pos_I_old = _pos_I;
		_local_pos_pub.publish(_local_pos);

		// publish the gps
		_sensor_gps.timestamp = _now;
		_sensor_gps.lat = (int32_t)(_gpos.lat*1e7);
		_sensor_gps.lon = (int32_t)(_gpos.lon*1e7);
		_sensor_gps.alt = (int32_t)(_gpos.alt*1000);
		_sensor_gps.vel_ned_valid = true;
		_sensor_gps.vel_m_s = _vel_I.norm();
		_sensor_gps.vel_n_m_s = _vel_I(0);
		_sensor_gps.vel_e_m_s = _vel_I(1);
		_sensor_gps.vel_d_m_s = _local_pos.vz;
		_sensor_gps.cog_rad = atan2(_vel_I(1),_vel_I(0));
		_gps_pub.publish(_sensor_gps);

		// publish the airspeed
		// float airspeed_i[1]={};
		// int size=26;
		// // if (getDREF(_xpc_sock, "sim/flightmodel/position/indicated_airspeed", airspeed_i, &size)==0 && size>0) {
		// // 	_indicated_airspeed = airspeed_i[0];
		// // }
		// float airspeed_t[26]={};
		// if (getDREF(_xpc_sock, "sim/flightmodel/position/true_airspeed", airspeed_t, &size)==0 && size>0) {
		// 	_true_airspeed = airspeed_t[0];
		// }
		// _indicated_airspeed = _true_airspeed * 1;

		// _airspeed.timestamp = _now;
		// _airspeed.timestamp_sample = _now;
		// _airspeed.indicated_airspeed_m_s = _indicated_airspeed;
		// _airspeed.true_airspeed_m_s = _true_airspeed;
		// _airspeed.air_temperature_celsius = baro_temp_c;
		// _airspeed_pub.publish(_airspeed);

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
  	PX4_INFO("armed: %u", _armed);
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
