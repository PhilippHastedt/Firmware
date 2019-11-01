/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 *
 * This module is a modification of the fixed wing module and it is designed for ground rovers.
 * It has been developed starting from the fw module, simplified and improved with dedicated items.
 *
 * All the acknowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */

#include "uuv_att_control.hpp"


/**
 * GroundRover attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int uuv_att_control_main(int argc, char *argv[]);

namespace att_uuv_control
{
    UUVAttitudeControl	*g_control = nullptr;
}

UUVAttitudeControl::UUVAttitudeControl():
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "gnda_dt")),
	_nonfinite_input_perf(perf_alloc(PC_COUNT, "gnda_nani")),
	_nonfinite_output_perf(perf_alloc(PC_COUNT, "gnda_nano"))
{
    _parameter_handles.roll_p = param_find("UUV_ROLL_P");
    _parameter_handles.roll_i = param_find("UUV_ROLL_I");
    _parameter_handles.roll_d = param_find("UUV_ROLL_D");
    _parameter_handles.roll_imax = param_find("UUV_ROLL_IMAX");
    _parameter_handles.roll_ff = param_find("UUV_ROLL_FF");

    _parameter_handles.pitch_p = param_find("UUV_PITCH_P");
    _parameter_handles.pitch_i = param_find("UUV_PITCH_I");
    _parameter_handles.pitch_d = param_find("UUV_PITCH_D");
    _parameter_handles.pitch_imax = param_find("UUV_PITCH_IMAX");
    _parameter_handles.pitch_ff = param_find("UUV_PITCH_FF");

    _parameter_handles.yaw_p = param_find("UUV_YAW_P");
    _parameter_handles.yaw_i = param_find("UUV_YAW_I");
    _parameter_handles.yaw_d = param_find("UUV_YAW_D");
    _parameter_handles.yaw_imax = param_find("UUV_YAW_IMAX");
    _parameter_handles.yaw_ff = param_find("UUV_YAW_FF");

    _parameter_handles.roll_geo_p = param_find("UUV_GEO_ROLL_P");
    _parameter_handles.roll_geo_d = param_find("UUV_GEO_ROLL_D");
    _parameter_handles.pitch_geo_p = param_find("UUV_GEO_PITCH_P");
    _parameter_handles.pitch_geo_d = param_find("UUV_GEO_PITCH_D");
    _parameter_handles.yaw_geo_p = param_find("UUV_GEO_YAW_P");
    _parameter_handles.yaw_geo_d = param_find("UUV_GEO_YAW_D");

    _parameter_handles.roll_geo_max = param_find("UUV_GEO_X_ROLL");
    _parameter_handles.pitch_geo_max = param_find("UUV_GEO_X_PITCH");
    _parameter_handles.yaw_geo_max = param_find("UUV_GEO_X_YAW");
    _parameter_handles.thrust_geo_max = param_find("UUV_GEO_X_THRUST");

    _parameter_handles.test_roll = param_find("TEST_ROLL");
    _parameter_handles.test_pitch = param_find("TEST_PITCH");
    _parameter_handles.test_yaw = param_find("TEST_YAW");
    _parameter_handles.test_thrust = param_find("TEST_THRUST");
    _parameter_handles.is_test_mode = param_find("IS_TEST_MODE");
    _parameter_handles.direct_roll = param_find("DIRECT_ROLL");
    _parameter_handles.direct_pitch = param_find("DIRECT_PITCH");
    _parameter_handles.direct_yaw = param_find("DIRECT_YAW");
    _parameter_handles.direct_thrust = param_find("DIRECT_THRUST");
    _parameter_handles.is_direct_mode = param_find("IS_DIRECT_MODE");

    /* fetch initial parameter values */
	parameters_update();
}

UUVAttitudeControl::~UUVAttitudeControl()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			px4_usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}
	perf_free(_loop_perf);
	perf_free(_nonfinite_input_perf);
	perf_free(_nonfinite_output_perf);

    att_uuv_control::g_control = nullptr;
}

void UUVAttitudeControl::parameters_update()
{
    param_get(_parameter_handles.roll_p, &(_parameters.roll_p));
    param_get(_parameter_handles.roll_i, &(_parameters.roll_i));
    param_get(_parameter_handles.roll_d, &(_parameters.roll_d));
    param_get(_parameter_handles.roll_imax, &(_parameters.roll_imax));
    param_get(_parameter_handles.roll_ff, &(_parameters.roll_ff));

    param_get(_parameter_handles.pitch_p, &(_parameters.pitch_p));
    param_get(_parameter_handles.pitch_i, &(_parameters.pitch_i));
    param_get(_parameter_handles.pitch_d, &(_parameters.pitch_d));
    param_get(_parameter_handles.pitch_imax, &(_parameters.pitch_imax));
    param_get(_parameter_handles.pitch_ff, &(_parameters.pitch_ff));

    param_get(_parameter_handles.yaw_p, &(_parameters.yaw_p));
    param_get(_parameter_handles.yaw_i, &(_parameters.yaw_i));
    param_get(_parameter_handles.yaw_d, &(_parameters.yaw_d));
    param_get(_parameter_handles.yaw_imax, &(_parameters.yaw_imax));
    param_get(_parameter_handles.yaw_ff, &(_parameters.yaw_ff));

    param_get(_parameter_handles.roll_geo_p, &(_parameters.roll_geo_p));
    param_get(_parameter_handles.roll_geo_d, &(_parameters.roll_geo_d));
    param_get(_parameter_handles.pitch_geo_p, &(_parameters.pitch_geo_p));
    param_get(_parameter_handles.pitch_geo_d, &(_parameters.pitch_geo_d));
    param_get(_parameter_handles.yaw_geo_p, &(_parameters.yaw_geo_p));
    param_get(_parameter_handles.yaw_geo_d, &(_parameters.yaw_geo_d));

    param_get(_parameter_handles.roll_geo_max, &(_parameters.roll_geo_max));
    param_get(_parameter_handles.pitch_geo_max, &(_parameters.pitch_geo_max));
    param_get(_parameter_handles.yaw_geo_max, &(_parameters.yaw_geo_max));
    param_get(_parameter_handles.thrust_geo_max, &(_parameters.thrust_geo_max));

    param_get(_parameter_handles.test_roll, &(_parameters.test_roll));
    param_get(_parameter_handles.test_pitch, &(_parameters.test_pitch));
    param_get(_parameter_handles.test_yaw, &(_parameters.test_yaw));
    param_get(_parameter_handles.test_thrust, &(_parameters.test_thrust));
    param_get(_parameter_handles.is_test_mode, &(_parameters.is_test_mode));
    param_get(_parameter_handles.direct_roll, &(_parameters.direct_roll));
    param_get(_parameter_handles.direct_pitch, &(_parameters.direct_pitch));
    param_get(_parameter_handles.direct_yaw, &(_parameters.direct_yaw));
    param_get(_parameter_handles.direct_thrust, &(_parameters.direct_thrust));
    param_get(_parameter_handles.is_direct_mode, &(_parameters.is_direct_mode));

    /* pid controller parameters*/
    pid_init(&_roll_ctrl, PID_MODE_DERIVATIV_SET, 0.01f);
    pid_set_parameters(&_roll_ctrl,_parameters.roll_p,_parameters.roll_i,_parameters.roll_d,_parameters.roll_imax,1.0f);
    pid_init(&_pitch_ctrl, PID_MODE_DERIVATIV_SET, 0.01f);
    pid_set_parameters(&_pitch_ctrl,_parameters.pitch_p,_parameters.pitch_i,_parameters.pitch_d,_parameters.pitch_imax,1.0f);
    pid_init(&_yaw_ctrl, PID_MODE_DERIVATIV_SET, 0.01f);
    pid_set_parameters(&_yaw_ctrl,_parameters.yaw_p,_parameters.yaw_i,_parameters.yaw_d,_parameters.yaw_imax,1.0f);
}

void UUVAttitudeControl::vehicle_control_mode_poll()
{
	bool updated = false;
	orb_check(_vcontrol_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void UUVAttitudeControl::manual_control_setpoint_poll()
{
	bool updated = false;
	orb_check(_manual_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}
}

void UUVAttitudeControl::vehicle_attitude_setpoint_poll()
{
	bool updated = false;
	orb_check(_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
	}
}

void UUVAttitudeControl::battery_status_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}
}

int UUVAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
    att_uuv_control::g_control->task_main();
	return 0;
}

void UUVAttitudeControl::task_main()
{
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    _angular_velocity_sub = orb_subscribe(ORB_ID(vehicle_angular_velocity));
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));

	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_attitude_setpoint_poll();
	vehicle_control_mode_poll();
	manual_control_setpoint_poll();
	battery_status_poll();

	/* wakeup source */
	px4_pollfd_struct_t fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _att_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {
		static int loop_counter = 0;

		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			PX4_ERR("poll error %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if attitude changed */
		if (fds[1].revents & POLLIN) {
			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f ||
			    fabsf(deltaT) < 0.00001f ||
			    !PX4_ISFINITE(deltaT)) {
				deltaT = 0.01f;
			}

			/* load local copies */
			orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);
            orb_copy(ORB_ID(vehicle_angular_velocity), _angular_velocity_sub, &_angular_velocity);

			vehicle_attitude_setpoint_poll();
			vehicle_control_mode_poll();
			manual_control_setpoint_poll();
			battery_status_poll();
            //PX4_INFO("att_setpoints x %8.4f",(double)_att_sp.thrust_body[0]);
            //PX4_INFO("att_setpoints z %8.4f",(double)_att_sp.thrust_body[2]);

			/* decide if in stabilized or full manual control */
			if (_vcontrol_mode.flag_control_rates_enabled) {
				/* Run attitude controllers */
                if (_vcontrol_mode.flag_control_attitude_enabled) {
                    Eulerf euler_angles(matrix::Quatf(_att.q));
                    //PX4_INFO("quat: %.2f; %.2f; %.2f; %.2f", (double)_att.q[0], (double)_att.q[1],(double)_att.q[2],(double)_att.q[3]);
                    float roll_body;
                    float pitch_body;
                    float yaw_body;
                    float thrust_body;

                    if(_parameters.is_direct_mode == 1){
                        roll_body = _parameters.direct_roll;
                        pitch_body = _parameters.direct_pitch;
                        yaw_body = _parameters.direct_yaw;
                        thrust_body = _parameters.direct_thrust;
                        //roll_body = _att_sp.roll_body;
                        //pitch_body = _att_sp.pitch_body;
                        //yaw_body = _att_sp.yaw_body;
                        //thrust_body = _att_sp.thrust_body[0];
                    } else if (_parameters.is_direct_mode == 2) {
                        roll_body = _att_sp.roll_body;
                        pitch_body = _att_sp.pitch_body;
                        yaw_body = _att_sp.yaw_body;

                        thrust_body = _att_sp.thrust_body[0];
                    } else {
                        roll_body = _att_sp.roll_body;
                        pitch_body = _att_sp.pitch_body;
                        yaw_body = _att_sp.yaw_body;
                        //thrust_body = -_att_sp.thrust_body[2];
                        thrust_body = _att_sp.thrust_body[0];
                        // Map from [0,1] to [-1,1]
                        //thrust_body = thrust_body;
                    }
                    //PX4_INFO("received thrust: %.4f", (double)thrust_body);
                    float roll_u = 0.0;
                    float pitch_u = 0.0;
                    float yaw_u = 0.0;
                    float thrust_u = 0.0f;
                    // euler_angles.psi() += _parameters.test_roll;

                    // Geometric Controller
                    if (_parameters.is_direct_mode == 2) {
                        roll_body = _att_sp.roll_body;
                        pitch_body = _att_sp.pitch_body;
                        yaw_body = _att_sp.yaw_body;

                        /* geometric control start */
                        // Matrix3f _R_sp;

                        /* get attitude setpoint rotational matrix */

                        Dcmf _rot_des = Eulerf(_att_sp.roll_body,
                                              _att_sp.pitch_body,
                                              _att_sp.yaw_body);
                        ///Dcmf _rot_att = dcm(_att.q);

                        /* get current rotation matrix from control state quaternions */
                        Quatf q_att(_att.q[0], _att.q[1], _att.q[2], _att.q[3]);
                        Matrix3f _rot_att = q_att.to_dcm();

                        Vector3f e_R_vec;
                        Vector3f torques;
                        Vector3f omega;

                        //_angular_velocity.xyz[0]
                        /* get current rates from sensors */
                        //omega(0) = _v_att.rollspeed;
                        //omega(1) = _v_att.pitchspeed;
                        //omega(2) = _v_att.yawspeed;

                        /* Compute matrix: attitude error */
                        Matrix3f e_R =  (_rot_des.transpose() * _rot_att - _rot_att.transpose() * _rot_des) * 0.5;


                        /* vee-map the error to get a vector instead of matrix e_R */
                        e_R_vec(0) = e_R(2,1);  // Roll
                        e_R_vec(1) = e_R(0,2);  // Pitch
                        e_R_vec(2) = e_R(1,0);  // Yaw

                        omega(0) = _angular_velocity.xyz[0] - 0.0f;
                        omega(1) = _angular_velocity.xyz[1] - 0.0f;
                        omega(2) = _angular_velocity.xyz[2] - 0.0f;

                        /**< P-Control */
                        torques(0) = - e_R_vec(0) * _parameters.roll_geo_p;   /**< Roll  */
                        torques(1) = - e_R_vec(1) * _parameters.pitch_geo_p;  /**< Pitch */
                        torques(2) = - e_R_vec(2) * _parameters.yaw_geo_p;    /**< Yaw   */

                        /**< PD-Control */
                        torques(0) = torques(0) - omega(0) * _parameters.roll_geo_p;  /**< Roll  */
                        torques(1) = torques(1) - omega(1) * _parameters.pitch_geo_d; /**< Pitch */
                        torques(2) = torques(2) - omega(2) * _parameters.yaw_geo_d;   /**< Yaw   */
                        roll_u = torques(0);
                        pitch_u = torques(1);
                        yaw_u = torques(2);
                        /* geometric control end */
                        thrust_u = _att_sp.thrust_body[0];

                    } else {
                        /* regular PID-Controller */
                        /* Calculate the control output for the steering as yaw */
                        roll_u = pid_calculate(&_roll_ctrl, roll_body, euler_angles.phi(), _angular_velocity.xyz[0], deltaT);
                        pitch_u = pid_calculate(&_pitch_ctrl, pitch_body, euler_angles.theta(), _angular_velocity.xyz[1], deltaT);
                        yaw_u = pid_calculate(&_yaw_ctrl, yaw_body, euler_angles.psi(), _angular_velocity.xyz[2], deltaT);
                        thrust_u = thrust_body;

                        float roll_angle_diff = 0.0f;
                        float pitch_angle_diff = 0.0f;
                        float yaw_angle_diff = 0.0f;

                        if (roll_body * euler_angles.phi() < 0.0f) {
                            if (roll_body < 0.0f) {
                                roll_angle_diff = euler_angles.phi() - roll_body ;

                            } else {
                                roll_angle_diff = roll_body - euler_angles.phi();
                            }

                            // a switch might have happened
                            if ((double)roll_angle_diff > M_PI) {
                                roll_u = -roll_u;
                            }
                        }
                        if (pitch_body * euler_angles.theta() < 0.0f) {
                            if (pitch_body < 0.0f) {
                                pitch_angle_diff = euler_angles.theta() - pitch_body ;

                            } else {
                                pitch_angle_diff = pitch_body - euler_angles.theta();
                            }

                            // a switch might have happened
                            if ((double)pitch_angle_diff > M_PI) {
                                pitch_u = -pitch_u;
                            }
                        }
                        if (yaw_body * euler_angles.psi() < 0.0f) {
                            if (yaw_body < 0.0f) {
                                yaw_angle_diff = euler_angles.psi() - yaw_body ;

                            } else {
                                yaw_angle_diff = yaw_body - euler_angles.psi();
                            }

                            // a switch might have happened
                            if ((double)yaw_angle_diff > M_PI) {
                                yaw_u = -yaw_u;
                            }
                        }
                    }

                    if (_parameters.is_direct_mode == 2){
                    if (PX4_ISFINITE(roll_u)) {
                        if (roll_u >_parameters.roll_geo_max){
                            roll_u = _parameters.roll_geo_max;
                        }else if(roll_u < -_parameters.roll_geo_max){
                            roll_u = -_parameters.roll_geo_max;
                        }

                        _actuators.control[actuator_controls_s::INDEX_ROLL] = roll_u;
                        //PX4_INFO("writing roll %.4f", (double)roll_u);
                    } else {
                        _actuators.control[actuator_controls_s::INDEX_ROLL] = 0.0f;

                        perf_count(_nonfinite_output_perf);

                        if (_debug && loop_counter % 10 == 0) {
                            PX4_INFO("roll_u %.4f", (double)roll_u);
                        }
                    }
                    if (PX4_ISFINITE(pitch_u)) {
                        if (pitch_u >_parameters.pitch_geo_max){
                            pitch_u = _parameters.pitch_geo_max;
                        }else if(pitch_u < -_parameters.pitch_geo_max){
                            pitch_u = -_parameters.pitch_geo_max;
                        }
                        _actuators.control[actuator_controls_s::INDEX_PITCH] = pitch_u;
                        //PX4_ERR("writing pitch %.4f", (double)pitch_u);
                    } else {
                        _actuators.control[actuator_controls_s::INDEX_PITCH] = 0.0f;

                        perf_count(_nonfinite_output_perf);

                        if (_debug && loop_counter % 10 == 0) {
                            PX4_INFO("pitch_u %.4f", (double)pitch_u);
                        }
                    }
                    if (PX4_ISFINITE(yaw_u)) {
                        if (yaw_u >_parameters.yaw_geo_max){
                            yaw_u = _parameters.yaw_geo_max;
                        }else if(yaw_u < -_parameters.yaw_geo_max){
                            yaw_u = -_parameters.yaw_geo_max;
                        }
                        _actuators.control[actuator_controls_s::INDEX_YAW] = yaw_u;
                        //PX4_INFO("writing yaw %.4f", (double)yaw_u);
                    } else {
                        _actuators.control[actuator_controls_s::INDEX_YAW] = 0.0f;

                        perf_count(_nonfinite_output_perf);

                        if (_debug && loop_counter % 10 == 0) {
                            PX4_INFO("yaw_u %.4f", (double)yaw_u);
                        }
                    }

                    /* throttle passed through if it is finite and if no engine failure was detected */
                    _actuators.control[actuator_controls_s::INDEX_THROTTLE] = thrust_u;
                    //PX4_INFO("writing thrust %.4f", (double)thrust_u);
                    } else {
                        if (PX4_ISFINITE(roll_u)) {
                            _actuators.control[actuator_controls_s::INDEX_ROLL] = roll_u;
                            //PX4_INFO("writing roll %.4f", (double)roll_u);
                        } else {
                            _actuators.control[actuator_controls_s::INDEX_ROLL] = 0.0f;

                            perf_count(_nonfinite_output_perf);

                            if (_debug && loop_counter % 10 == 0) {
                                PX4_INFO("roll_u %.4f", (double)roll_u);
                            }
                        }
                        if (PX4_ISFINITE(pitch_u)) {
                            _actuators.control[actuator_controls_s::INDEX_PITCH] = pitch_u;
                            //PX4_INFO("writing pitch %.4f", (double)pitch_u);
                        } else {
                            _actuators.control[actuator_controls_s::INDEX_PITCH] = 0.0f;

                            perf_count(_nonfinite_output_perf);

                            if (_debug && loop_counter % 10 == 0) {
                                PX4_INFO("pitch_u %.4f", (double)pitch_u);
                            }
                        }
                        if (PX4_ISFINITE(yaw_u)) {
                            _actuators.control[actuator_controls_s::INDEX_YAW] = yaw_u;
                            //PX4_INFO("writing yaw %.4f", (double)yaw_u);
                        } else {
                            _actuators.control[actuator_controls_s::INDEX_YAW] = 0.0f;

                            perf_count(_nonfinite_output_perf);

                            if (_debug && loop_counter % 10 == 0) {
                                PX4_INFO("yaw_u %.4f", (double)yaw_u);
                            }
                        }

                        /* throttle passed through if it is finite and if no engine failure was detected */
                        _actuators.control[actuator_controls_s::INDEX_THROTTLE] = thrust_u;
                        //PX4_INFO("writing thrust %.4f", (double)thrust_body);
                    }

                    if(_parameters.is_test_mode){
                    /* throttle passed through if it is finite and if no engine failure was detected */
                    _actuators.control[actuator_controls_s::INDEX_THROTTLE] = _parameters.test_thrust;//thrust_body[0];
                    _actuators.control[actuator_controls_s::INDEX_YAW] = _parameters.test_yaw;
                    _actuators.control[actuator_controls_s::INDEX_ROLL] = _parameters.test_roll;
                    _actuators.control[actuator_controls_s::INDEX_PITCH] = _parameters.test_pitch;
                    }
					}

            } else {
                /* manual/direct control */
                _actuators.control[actuator_controls_s::INDEX_ROLL] = _manual.y;
                _actuators.control[actuator_controls_s::INDEX_PITCH] = -_manual.x;
                _actuators.control[actuator_controls_s::INDEX_YAW] = _manual.r;
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z;
            }

			/* lazily publish the setpoint only once available */
			_actuators.timestamp = hrt_absolute_time();
			_actuators.timestamp_sample = _att.timestamp;

			/* Only publish if any of the proper modes are enabled */
			if (_vcontrol_mode.flag_control_attitude_enabled ||
			    _vcontrol_mode.flag_control_manual_enabled) {

				/* publish the actuator controls */
				if (_actuators_0_pub != nullptr) {
					orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, _actuators_0_pub, &_actuators);

				} else {
					_actuators_0_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &_actuators);
				}
			}
		}

		loop_counter++;
		perf_end(_loop_perf);
	}

	PX4_INFO("exiting.");

    _control_task = -1;
	_task_running = false;
}

int UUVAttitudeControl::start()
{
	/* start the task */
    _control_task = px4_task_spawn_cmd("uuv_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
                       (px4_main_t)&UUVAttitudeControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		PX4_ERR("task start failed");
		return -errno;
	}

	return PX4_OK;
}

int uuv_att_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_INFO("usage: uuv2_att_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

        if (att_uuv_control::g_control != nullptr) {
			PX4_WARN("already running");
			return 1;
		}

        att_uuv_control::g_control = new UUVAttitudeControl;

        if (att_uuv_control::g_control == nullptr) {
			PX4_ERR("alloc failed");
			return 1;
		}

        if (PX4_OK != att_uuv_control::g_control->start()) {
            delete att_uuv_control::g_control;
            att_uuv_control::g_control = nullptr;
			PX4_ERR("start failed");
			return 1;
		}

		/* check if the waiting is necessary at all */
        if (att_uuv_control::g_control == nullptr || !att_uuv_control::g_control->task_running()) {

			/* avoid memory fragmentation by not exiting start handler until the task has fully started */
            while (att_uuv_control::g_control == nullptr || !att_uuv_control::g_control->task_running()) {
				px4_usleep(50000);
				printf(".");
				fflush(stdout);
			}

			printf("\n");
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
        if (att_uuv_control::g_control == nullptr) {
			PX4_WARN("not running");
			return 1;
		}

        delete att_uuv_control::g_control;
        att_uuv_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
        if (att_uuv_control::g_control) {
			PX4_INFO("running");
			return 0;

		} else {
			PX4_INFO("not running");
			return 1;
		}
	}

	PX4_WARN("unrecognized command");
	return 1;
}
