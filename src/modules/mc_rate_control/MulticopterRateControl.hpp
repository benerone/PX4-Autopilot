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

#pragma once

#include <RateControl.hpp>

#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/integrale.h>
#include <uORB/topics/integralepos.h>
#include <uORB/topics/pipe_correction.h>

#include "pipetools.h"
using namespace zapata;

class MulticopterRateControl : public ModuleBase<MulticopterRateControl>, public ModuleParams, public px4::WorkItem
{
public:
	MulticopterRateControl(bool vtol = false);
	~MulticopterRateControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void		parameters_updated();

	/**
	 * Get the landing gear state based on the manual control switch position
	 * @return vehicle_attitude_setpoint_s::LANDING_GEAR_UP or vehicle_attitude_setpoint_s::LANDING_GEAR_DOWN
	 */
	float		get_landing_gear_state();

	RateControl _rate_control; ///< class for rate control calculations

	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _landing_gear_sub{ORB_ID(landing_gear)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _motor_limits_sub{ORB_ID(multirotor_motor_limits)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _v_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _v_rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _vehicle_angular_acceleration_sub{ORB_ID(vehicle_angular_acceleration)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _integralepos_sub{ORB_ID(integralepos)};


	uORB::Subscription _act_ctrl_sub{ORB_ID(actuator_controls_0)};
	uORB::Subscription _r1integrale_sub{ORB_ID(r1integrale)};
	uORB::Subscription _r2integrale_sub{ORB_ID(r2integrale)};
	uORB::Subscription _r3integrale_sub{ORB_ID(r3integrale)};

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::Publication<actuator_controls_s>		_actuators_0_pub;
	uORB::PublicationMulti<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status), ORB_PRIO_DEFAULT};	/**< controller status publication */
	uORB::Publication<landing_gear_s>		_landing_gear_pub{ORB_ID(landing_gear)};
	uORB::Publication<vehicle_rates_setpoint_s>	_v_rates_sp_pub{ORB_ID(vehicle_rates_setpoint)};			/**< rate setpoint publication */
	uORB::Publication<integrale_s>			_integrales_pub{ORB_ID(integrale)};

	uORB::Publication<pipe_correction_s>	_pipe_correction_pub{ORB_ID(pipe_correction)};

	landing_gear_s 			_landing_gear{};
	manual_control_setpoint_s	_manual_control_setpoint{};
	vehicle_control_mode_s		_v_control_mode{};
	vehicle_status_s		_vehicle_status{};

	bool _actuators_0_circuit_breaker_enabled{false};	/**< circuit breaker to suppress output */
	bool _landed{true};
	bool _maybe_landed{true};

	float _battery_status_scale{0.0f};

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	matrix::Vector3f _rates_sp;			/**< angular rates setpoint */

	float		_thrust_sp{0.0f};		/**< thrust setpoint */

	bool _gear_state_initialized{false};		/**< true if the gear state has been initialized */

	hrt_abstime _last_run{0};

	float _pi_limit[7];
	float _pi_mult[7];
	int32_t moyCorItems_pitch;
	int32_t moyCorItems_roll;
	int32_t moyCorItems_yaw;
	int32_t moyCorItems_vx;
	int32_t moyCorItems_vy;
	int32_t moyCorItems_thrust;
	int32_t moyCorItems_thr_act;
	int32_t sys_id;
	int oldNbRemoteValid=0;
	int cnt;

	zapata::StdVector<double> accuCorrectionPitch;
	zapata::StdVector<double> accuCorrectionRoll;
	zapata::StdVector<double> accuCorrectionYaw;
	zapata::StdVector<double> accuCorrectionVx;
	zapata::StdVector<double> accuCorrectionVy;
	zapata::StdVector<double> accuCorrectionThrust;
	zapata::StdVector<double> accuCorrectionThrAct;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_ROLLRATE_P>) _param_mc_rollrate_p,
		(ParamFloat<px4::params::MC_ROLLRATE_I>) _param_mc_rollrate_i,
		(ParamFloat<px4::params::MC_RR_INT_LIM>) _param_mc_rr_int_lim,
		(ParamFloat<px4::params::MC_ROLLRATE_D>) _param_mc_rollrate_d,
		(ParamFloat<px4::params::MC_ROLLRATE_FF>) _param_mc_rollrate_ff,
		(ParamFloat<px4::params::MC_ROLLRATE_K>) _param_mc_rollrate_k,

		(ParamFloat<px4::params::MC_PITCHRATE_P>) _param_mc_pitchrate_p,
		(ParamFloat<px4::params::MC_PITCHRATE_I>) _param_mc_pitchrate_i,
		(ParamFloat<px4::params::MC_PR_INT_LIM>) _param_mc_pr_int_lim,
		(ParamFloat<px4::params::MC_PITCHRATE_D>) _param_mc_pitchrate_d,
		(ParamFloat<px4::params::MC_PITCHRATE_FF>) _param_mc_pitchrate_ff,
		(ParamFloat<px4::params::MC_PITCHRATE_K>) _param_mc_pitchrate_k,

		(ParamFloat<px4::params::MC_YAWRATE_P>) _param_mc_yawrate_p,
		(ParamFloat<px4::params::MC_YAWRATE_I>) _param_mc_yawrate_i,
		(ParamFloat<px4::params::MC_YR_INT_LIM>) _param_mc_yr_int_lim,
		(ParamFloat<px4::params::MC_YAWRATE_D>) _param_mc_yawrate_d,
		(ParamFloat<px4::params::MC_YAWRATE_FF>) _param_mc_yawrate_ff,
		(ParamFloat<px4::params::MC_YAWRATE_K>) _param_mc_yawrate_k,

		(ParamFloat<px4::params::MPC_MAN_Y_MAX>) _param_mpc_man_y_max,			/**< scaling factor from stick to yaw rate */

		(ParamFloat<px4::params::MC_ACRO_R_MAX>) _param_mc_acro_r_max,
		(ParamFloat<px4::params::MC_ACRO_P_MAX>) _param_mc_acro_p_max,
		(ParamFloat<px4::params::MC_ACRO_Y_MAX>) _param_mc_acro_y_max,
		(ParamFloat<px4::params::MC_ACRO_EXPO>) _param_mc_acro_expo,				/**< expo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_EXPO_Y>) _param_mc_acro_expo_y,				/**< expo stick curve shape (yaw) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPO>) _param_mc_acro_supexpo,			/**< superexpo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPOY>) _param_mc_acro_supexpoy,			/**< superexpo stick curve shape (yaw) */

		(ParamFloat<px4::params::MC_RATT_TH>) _param_mc_ratt_th,

		(ParamBool<px4::params::MC_BAT_SCALE_EN>) _param_mc_bat_scale_en,

		(ParamInt<px4::params::CBRK_RATE_CTRL>) _param_cbrk_rate_ctrl,

		(ParamFloat<px4::params::MC_PI_LIM_PITCH>) _param_mc_pi_limit_pitch,
		(ParamFloat<px4::params::MC_PI_LIM_ROLL>) _param_mc_pi_limit_roll,
		(ParamFloat<px4::params::MC_PI_LIM_YOW>) _param_mc_pi_limit_yow,
		(ParamFloat<px4::params::MC_PI_LIM_THRUST>) _param_mc_pi_limit_thrust,
		(ParamFloat<px4::params::MC_PI_LIM_THRACT>) _param_mc_pi_limit_thr_act,
		(ParamFloat<px4::params::MC_PI_LIM_VX>) _param_mc_pi_limit_vx,
		(ParamFloat<px4::params::MC_PI_LIM_VY>) _param_mc_pi_limit_vy,
		(ParamFloat<px4::params::MC_PI_MUL_PITCH>) _param_mc_pi_mul_pitch,
		(ParamFloat<px4::params::MC_PI_MUL_ROLL>) _param_mc_pi_mul_roll,
		(ParamFloat<px4::params::MC_PI_MUL_YOW>) _param_mc_pi_mul_yow,
		(ParamFloat<px4::params::MC_PI_MUL_VX>) _param_mc_pi_mul_vx,
		(ParamFloat<px4::params::MC_PI_MUL_VY>) _param_mc_pi_mul_vy,
		(ParamFloat<px4::params::MC_PI_MUL_THRUST>) _param_mc_pi_mul_thrust,
		(ParamFloat<px4::params::MC_PI_MUL_THRACT>) _param_mc_pi_mul_thr_act,
		(ParamInt<px4::params::MC_PI_MOY_COR_PI>) _param_mc_pi_moy_cor_pitch,
		(ParamInt<px4::params::MC_PI_MOY_COR_RO>) _param_mc_pi_moy_cor_roll,
		(ParamInt<px4::params::MC_PI_MOY_COR_YO>) _param_mc_pi_moy_cor_yaw,
		(ParamInt<px4::params::MC_PI_MOY_COR_VX>) _param_mc_pi_moy_cor_vx,
		(ParamInt<px4::params::MC_PI_MOY_COR_VY>) _param_mc_pi_moy_cor_vy,
		(ParamInt<px4::params::MC_PI_MOY_COR_TH>) _param_mc_pi_moy_cor_thrust,
		(ParamInt<px4::params::MC_PI_MOY_COR_TA>) _param_mc_pi_moy_cor_thr_act,
		(ParamInt<px4::params::MAV_SYS_ID>) _param_mav_sys_id
	)

	matrix::Vector3f _acro_rate_max;	/**< max attitude rates in acro mode */

};
