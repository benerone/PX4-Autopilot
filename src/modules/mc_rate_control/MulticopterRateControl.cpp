/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "MulticopterRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;
using namespace time_literals;
using math::radians;

MulticopterRateControl::MulticopterRateControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_actuators_0_pub(vtol ? ORB_ID(actuator_controls_virtual_mc) : ORB_ID(actuator_controls_0)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
	cnt=0;
}

MulticopterRateControl::~MulticopterRateControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity callback registration failed!");
		return false;
	}

	return true;
}

void
MulticopterRateControl::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled_by_val(_param_cbrk_rate_ctrl.get(), CBRK_RATE_CTRL_KEY);

	_pi_limit[0]=_param_mc_pi_limit_roll.get();
	_pi_limit[1]=_param_mc_pi_limit_pitch.get();
	_pi_limit[2]=_param_mc_pi_limit_yow.get();
	_pi_limit[3]=_param_mc_pi_limit_vx.get();
	_pi_limit[4]=_param_mc_pi_limit_vy.get();
	_pi_limit[5]=_param_mc_pi_limit_thrust.get();
	_pi_mult[0]=_param_mc_pi_mul_roll.get();
	_pi_mult[1]=_param_mc_pi_mul_pitch.get();
	_pi_mult[2]=_param_mc_pi_mul_yow.get();
	_pi_mult[3]=_param_mc_pi_mul_vx.get();
	_pi_mult[4]=_param_mc_pi_mul_vy.get();
	_pi_mult[5]=_param_mc_pi_mul_thrust.get();
	moyCorItems_pitch=_param_mc_pi_moy_cor_pitch.get();
	moyCorItems_roll=_param_mc_pi_moy_cor_roll.get();
	moyCorItems_yaw=_param_mc_pi_moy_cor_yaw.get();
	moyCorItems_vx=_param_mc_pi_moy_cor_vx.get();
	moyCorItems_vy=_param_mc_pi_moy_cor_vy.get();
	moyCorItems_thrust=_param_mc_pi_moy_cor_thrust.get();
	sys_id=_param_mav_sys_id.get();
}

float
MulticopterRateControl::get_landing_gear_state()
{
	// Only switch the landing gear up if we are not landed and if
	// the user switched from gear down to gear up.
	// If the user had the switch in the gear up position and took off ignore it
	// until he toggles the switch to avoid retracting the gear immediately on takeoff.
	if (_landed) {
		_gear_state_initialized = false;
	}

	float landing_gear = landing_gear_s::GEAR_DOWN; // default to down

	if (_manual_control_setpoint.gear_switch == manual_control_setpoint_s::SWITCH_POS_ON && _gear_state_initialized) {
		landing_gear = landing_gear_s::GEAR_UP;

	} else if (_manual_control_setpoint.gear_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
		// Switching the gear off does put it into a safe defined state
		_gear_state_initialized = true;
	}

	return landing_gear;
}

void
MulticopterRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		// grab corresponding vehicle_angular_acceleration immediately after vehicle_angular_velocity copy
		vehicle_angular_acceleration_s v_angular_acceleration{};
		_vehicle_angular_acceleration_sub.copy(&v_angular_acceleration);

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f angular_accel{v_angular_acceleration.xyz};
		const Vector3f rates{angular_velocity.xyz};

		/* check for updates in other topics */
		_v_control_mode_sub.update(&_v_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		_vehicle_status_sub.update(&_vehicle_status);

		const bool manual_control_updated = _manual_control_setpoint_sub.update(&_manual_control_setpoint);

		// generate the rate setpoint from sticks?
		bool manual_rate_sp = false;

		if (_v_control_mode.flag_control_manual_enabled &&
		    !_v_control_mode.flag_control_altitude_enabled &&
		    !_v_control_mode.flag_control_velocity_enabled &&
		    !_v_control_mode.flag_control_position_enabled) {

			// landing gear controlled from stick inputs if we are in Manual/Stabilized mode
			//  limit landing gear update rate to 10 Hz
			if ((now - _landing_gear.timestamp) > 100_ms) {
				_landing_gear.landing_gear = get_landing_gear_state();
				_landing_gear.timestamp = hrt_absolute_time();
				_landing_gear_pub.publish(_landing_gear);
			}

			if (!_v_control_mode.flag_control_attitude_enabled) {
				manual_rate_sp = true;
			}

			// Check if we are in rattitude mode and the pilot is within the center threshold on pitch and roll
			//  if true then use published rate setpoint, otherwise generate from manual_control_setpoint (like acro)
			if (_v_control_mode.flag_control_rattitude_enabled) {
				manual_rate_sp =
					(fabsf(_manual_control_setpoint.y) > _param_mc_ratt_th.get()) ||
					(fabsf(_manual_control_setpoint.x) > _param_mc_ratt_th.get());
			}

		} else {
			_landing_gear_sub.update(&_landing_gear);
		}

		if (manual_rate_sp) {
			if (manual_control_updated) {

				// manual rates control - ACRO mode
				const Vector3f man_rate_sp{
					math::superexpo(_manual_control_setpoint.y, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(-_manual_control_setpoint.x, _param_mc_acro_expo.get(), _param_mc_acro_supexpo.get()),
					math::superexpo(_manual_control_setpoint.r, _param_mc_acro_expo_y.get(), _param_mc_acro_supexpoy.get())};

				_rates_sp = man_rate_sp.emult(_acro_rate_max);
				_thrust_sp = _manual_control_setpoint.z;

				// publish rate setpoint
				vehicle_rates_setpoint_s v_rates_sp{};
				v_rates_sp.roll = _rates_sp(0);
				v_rates_sp.pitch = _rates_sp(1);
				v_rates_sp.yaw = _rates_sp(2);
				v_rates_sp.thrust_body[0] = 0.0f;
				v_rates_sp.thrust_body[1] = 0.0f;
				v_rates_sp.thrust_body[2] = -_thrust_sp;
				v_rates_sp.timestamp = hrt_absolute_time();

				_v_rates_sp_pub.publish(v_rates_sp);
			}

		} else {
			// use rates setpoint topic
			vehicle_rates_setpoint_s v_rates_sp;

			if (_v_rates_sp_sub.update(&v_rates_sp)) {
				_rates_sp(0) = v_rates_sp.roll;
				_rates_sp(1) = v_rates_sp.pitch;
				_rates_sp(2) = v_rates_sp.yaw;
				_thrust_sp = -v_rates_sp.thrust_body[2];
			}
		}

		// run the rate controller
		if (_v_control_mode.flag_control_rates_enabled && !_actuators_0_circuit_breaker_enabled) {

			// reset integral if disarmed
			if (!_v_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_rate_control.resetIntegral();
			}

			// update saturation status from mixer feedback
			if (_motor_limits_sub.updated()) {
				multirotor_motor_limits_s motor_limits;

				if (_motor_limits_sub.copy(&motor_limits)) {
					MultirotorMixer::saturation_status saturation_status;
					saturation_status.value = motor_limits.saturation_status;

					_rate_control.setSaturationStatus(saturation_status);
				}
			}

			// run rate controller
			const Vector3f att_control = _rate_control.update(rates, _rates_sp, angular_accel, dt, _maybe_landed || _landed);

			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			//publish integrale
			integrale_s integrale_data{};
			auto integrale_values=_rate_control.getRateIntegral();
			integrale_data.timestamp= hrt_absolute_time();
			integrale_data.roll_rate_integral=integrale_values(0);
			integrale_data.pitch_rate_integral=integrale_values(1);
			integrale_data.yaw_rate_integral=integrale_values(2);
			integralepos_s integralepos_data;
			if (_integralepos_sub.copy(&integralepos_data)) {
				integrale_data.thrust=integralepos_data.thrust_vel_integral;
				integrale_data.vx=integralepos_data.x_vel_integral;
				integrale_data.vy=integralepos_data.y_vel_integral;
			} else {
				integrale_data.thrust=0.0f;
				integrale_data.vx=0.0f;
				integrale_data.vy=0.0f;
			}
			//TEST
			//integrale_data.thrust=_thrust_sp;

			integrale_data.status=integrale_s::INTEGRALE_STATUS_COMPLETE;
			integrale_data.index=sys_id;
			_integrales_pub.publish(integrale_data);

			//---------------------------------------------------
			// PIPE: ESTIMATE ERROR
			//---------------------------------------------------
			integrale_s _r1integrale;
			integrale_s _r2integrale;
			integrale_s _r3integrale;
			int nbRemoteValid=0;
			//Get remote integrale
			if (!_r1integrale_sub.copy(&_r1integrale)) {
				_r1integrale.status=integrale_s::INTEGRALE_STATUS_NONE;
			} else {
				if(PipeTools::isIntegraleValid(_r1integrale)) {
					nbRemoteValid++;
				}
			}
			if (!_r2integrale_sub.copy(&_r2integrale)) {
				_r2integrale.status=integrale_s::INTEGRALE_STATUS_NONE;
			}else {
				if(PipeTools::isIntegraleValid(_r2integrale)) {
					nbRemoteValid++;
				}
			}
			if (!_r3integrale_sub.copy(&_r3integrale)) {
				_r3integrale.status=integrale_s::INTEGRALE_STATUS_NONE;
			}else {
				if(PipeTools::isIntegraleValid(_r3integrale)) {
					nbRemoteValid++;
				}
			}
			double rollError,pitchError,yawError,trustError,vxError,vyError;
			int nbMedian=0;
			rollError=0.0f;
			pitchError=0.0f;
			yawError=0.0f;
			vxError=0.0f;
			vyError=0.0f;
			trustError=0.0f;

			int32_t medianRoll,medianPitch,medianYaw,medianThrust,medianVx,medianVy;
			medianRoll=0;
			medianPitch=0;
			medianYaw=0;
			medianVx=0;
			medianVy=0;
			medianThrust=0;
			if (nbRemoteValid!=1) {
				//Case 1,3 or 4
				rollError=(double)integrale_data.roll_rate_integral-PipeTools::processMedian(integrale_data,_r1integrale,_r2integrale,_r3integrale,&nbMedian,[](const integrale_s &r) {
					return r.roll_rate_integral;
				},&medianRoll);

				pitchError=(double)integrale_data.pitch_rate_integral-PipeTools::processMedian(integrale_data,_r1integrale,_r2integrale,_r3integrale,&nbMedian,[](const integrale_s &r) {
					return r.pitch_rate_integral;
				},&medianPitch);
				yawError=(double)integrale_data.yaw_rate_integral-PipeTools::processMedian(integrale_data,_r1integrale,_r2integrale,_r3integrale,&nbMedian,[](const integrale_s &r) {
					return r.yaw_rate_integral;
				},&medianYaw);
				vxError=(double)integrale_data.vx-PipeTools::processMedian(integrale_data,_r1integrale,_r2integrale,_r3integrale,&nbMedian,[](const integrale_s &r) {
					return r.vx;
				},&medianThrust);
				vyError=(double)integrale_data.vy-PipeTools::processMedian(integrale_data,_r1integrale,_r2integrale,_r3integrale,&nbMedian,[](const integrale_s &r) {
					return r.vy;
				},&medianThrust);
				trustError=(double)integrale_data.thrust-PipeTools::processMedian(integrale_data,_r1integrale,_r2integrale,_r3integrale,&nbMedian,[](const integrale_s &r) {
					return r.thrust;
				},&medianThrust);
			} else {
				PX4_INFO("!!!!!Case 2");
				//Case 2
				if (sys_id==1 ||
					(sys_id==2 && !PipeTools::isIntegraleValid(_r1integrale)) ||
					(sys_id==3 && !PipeTools::isIntegraleValid(_r1integrale) && !PipeTools::isIntegraleValid(_r2integrale)) ||
					(sys_id==4 && !PipeTools::isIntegraleValid(_r1integrale) && !PipeTools::isIntegraleValid(_r2integrale) && !PipeTools::isIntegraleValid(_r3integrale))) {
					rollError=0.0f;
					pitchError=0.0f;
					yawError=0.0f;
					trustError=0.0f;
					vxError=0.0f;
					vyError=0.0f;
					medianRoll=sys_id;
					medianPitch=sys_id;
					medianYaw=sys_id;
					medianVx=sys_id;
					medianVy=sys_id;
					medianThrust=sys_id;
				} else {
					if (sys_id==2) {
						rollError=(double)(integrale_data.roll_rate_integral-_r1integrale.roll_rate_integral);
						pitchError=(double)(integrale_data.pitch_rate_integral-_r1integrale.pitch_rate_integral);
						yawError=(double)(integrale_data.yaw_rate_integral-_r1integrale.yaw_rate_integral);
						vxError=(double)(integrale_data.vx-_r1integrale.vx);
						vyError=(double)(integrale_data.vy-_r1integrale.vy);
						trustError=(double)(integrale_data.thrust-_r1integrale.thrust);
						medianRoll=_r1integrale.index;
						medianPitch=_r1integrale.index;
						medianYaw=_r1integrale.index;
						medianVx=_r1integrale.index;
						medianVy=_r1integrale.index;
						medianThrust=_r1integrale.index;
					}
					if (sys_id==3) {
						integrale_s _vintegrale={0L,0.0f,0.0f,0.0f,0.0f,integrale_s::INTEGRALE_STATUS_NONE};
						if (PipeTools::isIntegraleValid(_r1integrale)) {
							_vintegrale=_r1integrale;
						} else {
							if (PipeTools::isIntegraleValid(_r2integrale)) {
								_vintegrale=_r2integrale;
							}
						}

						rollError=(double)(integrale_data.roll_rate_integral-_vintegrale.roll_rate_integral);
						pitchError=(double)(integrale_data.pitch_rate_integral-_vintegrale.pitch_rate_integral);
						yawError=(double)(integrale_data.yaw_rate_integral-_vintegrale.yaw_rate_integral);
						vxError=(double)(integrale_data.vx-_vintegrale.vx);
						vyError=(double)(integrale_data.vy-_vintegrale.vy);
						trustError=(double)(integrale_data.thrust-_vintegrale.thrust);
						medianRoll=_vintegrale.index;
						medianPitch=_vintegrale.index;
						medianYaw=_vintegrale.index;
						medianVx=_vintegrale.index;
						medianVy=_vintegrale.index;
						medianThrust=_vintegrale.index;
					}
					if (sys_id==4) {
						integrale_s _vintegrale={0L,0.0f,0.0f,0.0f,0.0f,integrale_s::INTEGRALE_STATUS_NONE};
						if (PipeTools::isIntegraleValid(_r1integrale)) {
							_vintegrale=_r1integrale;
						} else {
							if (PipeTools::isIntegraleValid(_r2integrale)) {
								_vintegrale=_r2integrale;
							} else {
								if (PipeTools::isIntegraleValid(_r3integrale)) {
									_vintegrale=_r3integrale;
								}
							}
						}
						rollError=(double)(integrale_data.roll_rate_integral-_vintegrale.roll_rate_integral);
						pitchError=(double)(integrale_data.pitch_rate_integral-_vintegrale.pitch_rate_integral);
						yawError=(double)(integrale_data.yaw_rate_integral-_vintegrale.yaw_rate_integral);
						vxError=(double)(integrale_data.vx-_vintegrale.vx);
						vyError=(double)(integrale_data.vy-_vintegrale.vy);
						trustError=(double)(integrale_data.thrust-_vintegrale.thrust);
						medianRoll=_vintegrale.index;
						medianPitch=_vintegrale.index;
						medianYaw=_vintegrale.index;
						medianVx=_vintegrale.index;
						medianVy=_vintegrale.index;
						medianThrust=_vintegrale.index;
					}
				}
			}

			//---------------------------------------------------
			// PIPE: APPLY CORRECTION
			//---------------------------------------------------
			double rollCorrection=PipeTools::processMultAndClamp(rollError,_pi_mult[0],_pi_limit[0]);
			double pitchCorrection=PipeTools::processMultAndClamp(pitchError,_pi_mult[1],_pi_limit[1]);
			double yawCorrection=PipeTools::processMultAndClamp(yawError,_pi_mult[2],_pi_limit[2]);
			double vxCorrection=PipeTools::processMultAndClamp(vxError,_pi_mult[3],_pi_limit[3]);
			double vyCorrection=PipeTools::processMultAndClamp(vyError,_pi_mult[4],_pi_limit[4]);
			double thrustCorrection=PipeTools::processMultAndClamp(trustError,_pi_mult[5],_pi_limit[5]);
			rollCorrection=PipeTools::processAverage(accuCorrectionRoll,rollCorrection,moyCorItems_roll);
			pitchCorrection=PipeTools::processAverage(accuCorrectionPitch,pitchCorrection,moyCorItems_pitch);
			yawCorrection=PipeTools::processAverage(accuCorrectionYaw,yawCorrection,moyCorItems_yaw);
			vxCorrection=PipeTools::processAverage(accuCorrectionVx,vxCorrection,moyCorItems_vx);
			vyCorrection=PipeTools::processAverage(accuCorrectionVy,vyCorrection,moyCorItems_vy);
			thrustCorrection=PipeTools::processAverage(accuCorrectionThrust,thrustCorrection,moyCorItems_thrust);
			/*cnt++;
			if (cnt>1000) {
				cnt=0;
				PX4_INFO("RE:%f RC:%f",(double)rollError,(double)(rollCorrection*1000.0f));
				PX4_INFO("PE:%f PC:%f",(double)pitchError,(double)(pitchCorrection*1000.0f));
				PX4_INFO("YE:%f YC:%f",(double)yawError,(double)(yawCorrection*1000.0f));
				PX4_INFO("TE:%f TC:%f",(double)trustError,(double)(thrustCorrection*1000.0f));
				PX4_INFO("NbRemote:%d",nbRemoteValid);
				PX4_INFO("R1:%d R2:%d R3:%d",(int)_r1integrale.status,(int)_r2integrale.status,(int)_r3integrale.status);
				PX4_INFO("Median: R:%c P:%c Y:%c T:%c",medianRoll==sys_id?'M':'_',medianPitch==sys_id?'M':'_',
				medianYaw==sys_id?'M':'_',medianThrust==sys_id?'M':'_');
				PX4_INFO("MedianIndex: R:%d P:%d Y:%d T:%d",medianRoll,medianPitch,
				medianYaw,medianThrust);
				PX4_INFO("MedianIndex: L:%d R1:%d R2:%d R3:%d",integrale_data.index,_r1integrale.index,
				_r2integrale.index,_r3integrale.index);
			}*/
			matrix::Vector3f rateIntegrale;
			rateIntegrale(0)=integrale_values(0)-(float)rollCorrection;
			rateIntegrale(1)=integrale_values(1)-(float)pitchCorrection;
			rateIntegrale(2)=integrale_values(2)-(float)yawCorrection;
			_rate_control.setRateIntegral(rateIntegrale);

			pipe_correction_s pipe_correction{};
			pipe_correction.timestamp=hrt_absolute_time();
			pipe_correction.roll_correction=(float)rollCorrection;
			pipe_correction.pitch_correction=(float)pitchCorrection;
			pipe_correction.yaw_correction=(float)yawCorrection;
			pipe_correction.vx_correction=(float)vxCorrection;
			pipe_correction.vy_correction=(float)vyCorrection;
			pipe_correction.thrust_correction=(float)thrustCorrection;
			pipe_correction.nb_median=(float)nbMedian;
			pipe_correction.param_mr=_pi_mult[0];
			pipe_correction.param_mp=_pi_mult[1];
			pipe_correction.param_my=_pi_mult[2];
			pipe_correction.param_mt=_pi_mult[3];
			pipe_correction.median_roll=medianRoll;
			pipe_correction.median_pitch=medianPitch;
			pipe_correction.median_yaw=medianYaw;
			pipe_correction.median_vx=medianVx;
			pipe_correction.median_vy=medianVy;
			pipe_correction.median_thrust=medianThrust;
			_pipe_correction_pub.publish(pipe_correction);

			// publish actuator controls
			actuator_controls_s actuators{};
			actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_thrust_sp) ? _thrust_sp : 0.0f;
			actuators.control[actuator_controls_s::INDEX_LANDING_GEAR] = (float)_landing_gear.landing_gear;
			actuators.timestamp_sample = angular_velocity.timestamp_sample;

			// scale effort by battery status if enabled
			if (_param_mc_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status)) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.0f) {
					for (int i = 0; i < 4; i++) {
						actuators.control[i] *= _battery_status_scale;
					}
				}
			}

			actuators.timestamp = hrt_absolute_time();
			_actuators_0_pub.publish(actuators);

		} else if (_v_control_mode.flag_control_termination_enabled) {
			if (!_vehicle_status.is_vtol) {
				// publish actuator controls
				actuator_controls_s actuators{};
				actuators.timestamp = hrt_absolute_time();
				_actuators_0_pub.publish(actuators);
			}
		}
	}

	perf_end(_loop_perf);
}

int MulticopterRateControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterRateControl *instance = new MulticopterRateControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_rate_control_main(int argc, char *argv[])
{
	return MulticopterRateControl::main(argc, argv);
}
