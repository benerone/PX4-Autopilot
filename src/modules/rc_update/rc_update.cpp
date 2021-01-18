/****************************************************************************
 *
 *   Copyright (c) 2016-2019 PX4 Development Team. All rights reserved.
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
 * @file rc_update.cpp
 *
 * @author Beat Kueng <beat-kueng@gmx.net>
 */

#include "rc_update.h"

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MINABS(a,b) ((abs(a)) < (abs(b)) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MAX_CNT_R 3
#define MOY_CORREC_NBITEM 5

using namespace time_literals;

namespace RCUpdate
{

RCUpdate::RCUpdate() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME))
{
	// initialize parameter handles
	for (unsigned i = 0; i < RC_MAX_CHAN_COUNT; i++) {
		char nbuf[16];

		/* min values */
		sprintf(nbuf, "RC%d_MIN", i + 1);
		_parameter_handles.min[i] = param_find(nbuf);

		/* trim values */
		sprintf(nbuf, "RC%d_TRIM", i + 1);
		_parameter_handles.trim[i] = param_find(nbuf);

		/* max values */
		sprintf(nbuf, "RC%d_MAX", i + 1);
		_parameter_handles.max[i] = param_find(nbuf);

		/* channel reverse */
		sprintf(nbuf, "RC%d_REV", i + 1);
		_parameter_handles.rev[i] = param_find(nbuf);

		/* channel deadzone */
		sprintf(nbuf, "RC%d_DZ", i + 1);
		_parameter_handles.dz[i] = param_find(nbuf);
	}

	// RC to parameter mapping for changing parameters with RC
	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		// shifted by 1 because param name starts at 1
		char name[rc_parameter_map_s::PARAM_ID_LEN];
		snprintf(name, rc_parameter_map_s::PARAM_ID_LEN, "RC_MAP_PARAM%d", i + 1);
		_parameter_handles.rc_map_param[i] = param_find(name);
	}

	rc_parameter_map_poll(true /* forced */);

	parameters_updated();
	cntR1=-1;
	cntR2=-1;
	cntR3=-1;
}

RCUpdate::~RCUpdate()
{
	perf_free(_loop_perf);
}

bool
RCUpdate::init()
{
	if (!_input_rc_sub.registerCallback()) {
		PX4_ERR("input_rc callback registration failed!");
		return false;
	}

	return true;
}

void
RCUpdate::parameters_updated()
{
	// rc values
	for (unsigned int i = 0; i < RC_MAX_CHAN_COUNT; i++) {

		float min = 0.0f;
		param_get(_parameter_handles.min[i], &min);
		_parameters.min[i] = min;

		float trim = 0.0f;
		param_get(_parameter_handles.trim[i], &trim);
		_parameters.trim[i] = trim;

		float max = 0.0f;
		param_get(_parameter_handles.max[i], &max);
		_parameters.max[i] = max;

		float rev = 0.0f;
		param_get(_parameter_handles.rev[i], &rev);
		_parameters.rev[i] = rev < 0.0f;

		float dz = 0.0f;
		param_get(_parameter_handles.dz[i], &dz);
		_parameters.dz[i] = dz;
	}

	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		param_get(_parameter_handles.rc_map_param[i], &(_parameters.rc_map_param[i]));
	}

	//update coef integrale
	_pi_coef=matrix::Vector4f(_param_rc_pi_coef_roll.get(),_param_rc_pi_coef_pitch.get(),_param_rc_pi_coef_yow.get(),_param_rc_pi_coef_thrust.get());
	_pi_limit=matrix::Vector4f(_param_rc_pi_limit_roll.get(),_param_rc_pi_limit_pitch.get(),_param_rc_pi_limit_yow.get(),_param_rc_pi_limit_thrust.get());
	_pi_mult=matrix::Vector4f(_param_rc_pi_mul_roll.get(),_param_rc_pi_mul_pitch.get(),_param_rc_pi_mul_yow.get(),_param_rc_pi_mul_thrust.get());
	moyCorItems=_param_rc_pi_moy_cor.get();
	update_rc_functions();
}

void
RCUpdate::update_rc_functions()
{
	/* update RC function mappings */
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_THROTTLE] = _param_rc_map_throttle.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_ROLL] = _param_rc_map_roll.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PITCH] = _param_rc_map_pitch.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_YAW] = _param_rc_map_yaw.get() - 1;

	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_MODE] = _param_rc_map_mode_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_RETURN] = _param_rc_map_return_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_RATTITUDE] = _param_rc_map_ratt_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_POSCTL] = _param_rc_map_posctl_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_LOITER] = _param_rc_map_loiter_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_ACRO] = _param_rc_map_acro_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_OFFBOARD] = _param_rc_map_offb_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_KILLSWITCH] = _param_rc_map_kill_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_ARMSWITCH] = _param_rc_map_arm_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_TRANSITION] = _param_rc_map_trans_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_GEAR] = _param_rc_map_gear_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_STAB] = _param_rc_map_stab_sw.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_MAN] = _param_rc_map_man_sw.get() - 1;

	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_FLAPS] = _param_rc_map_flaps.get() - 1;

	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_1] = _param_rc_map_aux1.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_2] = _param_rc_map_aux2.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_3] = _param_rc_map_aux3.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_4] = _param_rc_map_aux4.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_5] = _param_rc_map_aux5.get() - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_6] = _param_rc_map_aux6.get() - 1;

	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i] = _parameters.rc_map_param[i] - 1;
	}
}

void
RCUpdate::rc_parameter_map_poll(bool forced)
{
	if (_rc_parameter_map_sub.updated() || forced) {
		_rc_parameter_map_sub.copy(&_rc_parameter_map);

		/* update parameter handles to which the RC channels are mapped */
		for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
			if (_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i] < 0 || !_rc_parameter_map.valid[i]) {
				/* This RC channel is not mapped to a RC-Parameter Channel (e.g. RC_MAP_PARAM1 == 0)
				 * or no request to map this channel to a param has been sent via mavlink
				 */
				continue;
			}

			/* Set the handle by index if the index is set, otherwise use the id */
			if (_rc_parameter_map.param_index[i] >= 0) {
				_parameter_handles.rc_param[i] = param_for_used_index((unsigned)_rc_parameter_map.param_index[i]);

			} else {
				_parameter_handles.rc_param[i] = param_find(&_rc_parameter_map.param_id[i * (rc_parameter_map_s::PARAM_ID_LEN + 1)]);
			}

		}

		PX4_DEBUG("rc to parameter map updated");

		for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
			PX4_DEBUG("\ti %d param_id %s scale %.3f value0 %.3f, min %.3f, max %.3f",
				  i,
				  &_rc_parameter_map.param_id[i * (rc_parameter_map_s::PARAM_ID_LEN + 1)],
				  (double)_rc_parameter_map.scale[i],
				  (double)_rc_parameter_map.value0[i],
				  (double)_rc_parameter_map.value_min[i],
				  (double)_rc_parameter_map.value_max[i]
				 );
		}
	}
}

float
RCUpdate::get_rc_value(uint8_t func, float min_value, float max_value)
{
	if (_rc.function[func] >= 0) {
		float value = _rc.channels[_rc.function[func]];
		return math::constrain(value, min_value, max_value);

	} else {
		return 0.0f;
	}
}

switch_pos_t
RCUpdate::get_rc_sw3pos_position(uint8_t func, float on_th, bool on_inv, float mid_th, bool mid_inv)
{
	if (_rc.function[func] >= 0) {
		float value = 0.5f * _rc.channels[_rc.function[func]] + 0.5f;

		if (on_inv ? value < on_th : value > on_th) {
			return manual_control_setpoint_s::SWITCH_POS_ON;

		} else if (mid_inv ? value < mid_th : value > mid_th) {
			return manual_control_setpoint_s::SWITCH_POS_MIDDLE;

		} else {
			return manual_control_setpoint_s::SWITCH_POS_OFF;
		}

	} else {
		return manual_control_setpoint_s::SWITCH_POS_NONE;
	}
}

switch_pos_t
RCUpdate::get_rc_sw2pos_position(uint8_t func, float on_th, bool on_inv)
{
	if (_rc.function[func] >= 0) {
		float value = 0.5f * _rc.channels[_rc.function[func]] + 0.5f;

		if (on_inv ? value < on_th : value > on_th) {
			return manual_control_setpoint_s::SWITCH_POS_ON;

		} else {
			return manual_control_setpoint_s::SWITCH_POS_OFF;
		}

	} else {
		return manual_control_setpoint_s::SWITCH_POS_NONE;
	}
}

void
RCUpdate::set_params_from_rc()
{
	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		if (_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i] < 0 || !_rc_parameter_map.valid[i]) {
			/* This RC channel is not mapped to a RC-Parameter Channel (e.g. RC_MAP_PARAM1 == 0)
			 * or no request to map this channel to a param has been sent via mavlink
			 */
			continue;
		}

		float rc_val = get_rc_value((rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i), -1.0, 1.0);

		/* Check if the value has changed,
		 * maybe we need to introduce a more aggressive limit here */
		if (rc_val > _param_rc_values[i] + FLT_EPSILON || rc_val < _param_rc_values[i] - FLT_EPSILON) {
			_param_rc_values[i] = rc_val;
			float param_val = math::constrain(
						  _rc_parameter_map.value0[i] + _rc_parameter_map.scale[i] * rc_val,
						  _rc_parameter_map.value_min[i], _rc_parameter_map.value_max[i]);

			param_set(_parameter_handles.rc_param[i], &param_val);
		}
	}
}

void
RCUpdate::Run()
{
	if (should_exit()) {
		_input_rc_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
		parameters_updated();
	}

	rc_parameter_map_poll();

	/* read low-level values from FMU or IO RC inputs (PPM, Spektrum, S.Bus) */
	input_rc_s rc_input;

	if (_input_rc_sub.copy(&rc_input)) {

		// warn if the channel count is changing (possibly indication of error)
		if (!rc_input.rc_lost && (_channel_count_previous != rc_input.channel_count) && (_channel_count_previous > 0)) {
			PX4_DEBUG("RC channel count changed %d -> %d", _channel_count_previous, rc_input.channel_count);
		}

		_channel_count_previous = rc_input.channel_count;

		/* detect RC signal loss */
		bool signal_lost = true;

		/* check flags and require at least four channels to consider the signal valid */
		if (rc_input.rc_lost || rc_input.rc_failsafe || rc_input.channel_count < 4) {
			/* signal is lost or no enough channels */
			signal_lost = true;


		} else if ((rc_input.input_source == input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM ||
			    rc_input.input_source == input_rc_s::RC_INPUT_SOURCE_PX4IO_PPM)
			   && rc_input.channel_count == 16) {

			// This is a specific RC lost check for RFD 868+/900 Modems on PPM.
			// The observation was that when RC is lost, 16 channels are active and the first 12 are 1000
			// and the remaining ones are 0.

			for (unsigned int i = 0; i < 16; i++) {
				if (i < 12 && rc_input.values[i] > 999 && rc_input.values[i] < 1005) {
					signal_lost = true;

				} else if (rc_input.values[i] == 0) {
					signal_lost = true;

				} else {
					signal_lost = false;
					break;
				}
			}

		} else {
			/* signal looks good */
			signal_lost = false;

			/* check failsafe */
			int8_t fs_ch = _rc.function[_param_rc_map_failsafe.get()]; // get channel mapped to throttle

			if (_param_rc_map_failsafe.get() > 0) { // if not 0, use channel number instead of rc.function mapping
				fs_ch = _param_rc_map_failsafe.get() - 1;
			}

			if (_param_rc_fails_thr.get() > 0 && fs_ch >= 0) {
				/* failsafe configured */
				if ((_param_rc_fails_thr.get() < _parameters.min[fs_ch] && rc_input.values[fs_ch] < _param_rc_fails_thr.get()) ||
				    (_param_rc_fails_thr.get() > _parameters.max[fs_ch] && rc_input.values[fs_ch] > _param_rc_fails_thr.get())) {
					/* failsafe triggered, signal is lost by receiver */
					signal_lost = true;
				}
			}
		}

		unsigned channel_limit = rc_input.channel_count;

		if (channel_limit > RC_MAX_CHAN_COUNT) {
			channel_limit = RC_MAX_CHAN_COUNT;
		}

		//Apply deadzone before pipe
		for (unsigned int i = 0; i < channel_limit; i++) {
			if (_rc.channels[i]<(_parameters.trim[i] + _parameters.dz[i]) || _rc.channels[i]>(_parameters.trim[i] - _parameters.dz[i])){
				_rc.channels[i]=_parameters.trim[i];
			}
		}

		//Pipe depending on integrale
		if (channel_limit>=rc_channels_s::RC_CHANNELS_FUNCTION_YAW) {
			int nbMedian=0;
			matrix::Vector4f correction=pipeIntegrale(&nbMedian);
			accuCorrection.push_back(correction);
			if (accuCorrection.size()>(unsigned int)moyCorItems) {
				for(unsigned int i=0;i<accuCorrection.size()-1;i++) {
					accuCorrection[i]=accuCorrection[i+1];
				}
				accuCorrection.pop_back();
			}
			matrix::Vector4f finalCorrection=matrix::Vector4f(0.0f,0.0f,0.0f,0.0f);
			for(unsigned int i=0;i<accuCorrection.size();i++) {
				finalCorrection+=accuCorrection[i];
			}
			finalCorrection=finalCorrection/accuCorrection.size();
			pipe_correction_s pipe_correction{};
			pipe_correction.timestamp=hrt_absolute_time();
			pipe_correction.roll_correction=-finalCorrection(0);
			pipe_correction.pitch_correction=-finalCorrection(1);
			pipe_correction.yaw_correction=-finalCorrection(2);
			pipe_correction.thrust_correction=-finalCorrection(3);
			pipe_correction.nb_median=(float)nbMedian;
			pipe_correction.param_mr=_pi_coef(0);
			pipe_correction.param_mp=_pi_coef(1);
			pipe_correction.param_my=_pi_coef(2);
			pipe_correction.param_mt=_pi_coef(3);
			rc_input.values[rc_channels_s::RC_CHANNELS_FUNCTION_ROLL]-=finalCorrection(0);
			rc_input.values[rc_channels_s::RC_CHANNELS_FUNCTION_PITCH]-=finalCorrection(1);
			rc_input.values[rc_channels_s::RC_CHANNELS_FUNCTION_YAW]-=finalCorrection(2);
			rc_input.values[rc_channels_s::RC_CHANNELS_FUNCTION_THROTTLE]-=finalCorrection(3);
			_pipe_correction_pub.publish(pipe_correction);
		}


		/* read out and scale values from raw message even if signal is invalid */
		for (unsigned int i = 0; i < channel_limit; i++) {

			/*
			 * 1) Constrain to min/max values, as later processing depends on bounds.
			 */
			rc_input.values[i] = math::constrain(rc_input.values[i], _parameters.min[i], _parameters.max[i]);

			/*
			 * 2) Scale around the mid point differently for lower and upper range.
			 *
			 * This is necessary as they don't share the same endpoints and slope.
			 *
			 * First normalize to 0..1 range with correct sign (below or above center),
			 * the total range is 2 (-1..1).
			 * If center (trim) == min, scale to 0..1, if center (trim) == max,
			 * scale to -1..0.
			 *
			 * As the min and max bounds were enforced in step 1), division by zero
			 * cannot occur, as for the case of center == min or center == max the if
			 * statement is mutually exclusive with the arithmetic NaN case.
			 *
			 * DO NOT REMOVE OR ALTER STEP 1!
			 */
			if (rc_input.values[i] > _parameters.trim[i]) {
				_rc.channels[i] = (rc_input.values[i] - _parameters.trim[i]) / (float)(
							  _parameters.max[i] - _parameters.trim[i]);

			} else if (rc_input.values[i] < _parameters.trim[i]) {
				_rc.channels[i] = (rc_input.values[i] - _parameters.trim[i]) / (float)(
							  _parameters.trim[i] - _parameters.min[i]);

			} else {
				/* in the configured dead zone, output zero */
				_rc.channels[i] = 0.0f;
			}

			if (_parameters.rev[i]) {
				_rc.channels[i] = -_rc.channels[i];
			}


			/* handle any parameter-induced blowups */
			if (!PX4_ISFINITE(_rc.channels[i])) {
				_rc.channels[i] = 0.0f;
			}
		}

		_rc.channel_count = rc_input.channel_count;
		_rc.rssi = rc_input.rssi;
		_rc.signal_lost = signal_lost;
		_rc.timestamp = rc_input.timestamp_last_signal;
		_rc.frame_drop_count = rc_input.rc_lost_frame_count;

		/* publish rc_channels topic even if signal is invalid, for debug */
		_rc_pub.publish(_rc);

		/* only publish manual control if the signal is still present and was present once */
		if (!signal_lost && rc_input.timestamp_last_signal > 0) {

			/* initialize manual setpoint */
			manual_control_setpoint_s manual_control_setpoint{};
			/* set mode slot to unassigned */
			manual_control_setpoint.mode_slot = manual_control_setpoint_s::MODE_SLOT_NONE;
			/* set the timestamp to the last signal time */
			manual_control_setpoint.timestamp = rc_input.timestamp_last_signal;
			manual_control_setpoint.data_source = manual_control_setpoint_s::SOURCE_RC;

			/* limit controls */
			manual_control_setpoint.y = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_ROLL, -1.0, 1.0);
			manual_control_setpoint.x = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_PITCH, -1.0, 1.0);
			manual_control_setpoint.r = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_YAW, -1.0, 1.0);
			manual_control_setpoint.z = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_THROTTLE, 0.0, 1.0);
			manual_control_setpoint.flaps = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_FLAPS, -1.0, 1.0);
			manual_control_setpoint.aux1 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_1, -1.0, 1.0);
			manual_control_setpoint.aux2 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_2, -1.0, 1.0);
			manual_control_setpoint.aux3 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_3, -1.0, 1.0);
			manual_control_setpoint.aux4 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_4, -1.0, 1.0);
			manual_control_setpoint.aux5 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_5, -1.0, 1.0);
			manual_control_setpoint.aux6 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_6, -1.0, 1.0);

			if (_param_rc_map_fltmode.get() > 0) {
				/* number of valid slots */
				const int num_slots = manual_control_setpoint_s::MODE_SLOT_NUM;

				/* the half width of the range of a slot is the total range
				 * divided by the number of slots, again divided by two
				 */
				const float slot_width_half = 2.0f / num_slots / 2.0f;

				/* min is -1, max is +1, range is 2. We offset below min and max */
				const float slot_min = -1.0f - 0.05f;
				const float slot_max = 1.0f + 0.05f;

				/* the slot gets mapped by first normalizing into a 0..1 interval using min
				 * and max. Then the right slot is obtained by multiplying with the number of
				 * slots. And finally we add half a slot width to ensure that integer rounding
				 * will take us to the correct final index.
				 */
				manual_control_setpoint.mode_slot = (((((_rc.channels[_param_rc_map_fltmode.get() - 1] - slot_min) * num_slots) +
								       slot_width_half)
								      / (slot_max - slot_min)) + (1.0f / num_slots)) + 1;

				if (manual_control_setpoint.mode_slot > num_slots) {
					manual_control_setpoint.mode_slot = num_slots;
				}
			}

			/* mode switches */
			manual_control_setpoint.mode_switch = get_rc_sw3pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_MODE,
							      _param_rc_auto_th.get(), _param_rc_auto_th.get() < 0.f,
							      _param_rc_assist_th.get(), _param_rc_assist_th.get() < 0.f);

			manual_control_setpoint.rattitude_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_RATTITUDE,
					_param_rc_ratt_th.get(), _param_rc_ratt_th.get() < 0.f);
			manual_control_setpoint.posctl_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_POSCTL,
								_param_rc_posctl_th.get(), _param_rc_posctl_th.get() < 0.f);
			manual_control_setpoint.return_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_RETURN,
								_param_rc_return_th.get(), _param_rc_return_th.get() < 0.f);
			manual_control_setpoint.loiter_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_LOITER,
								_param_rc_loiter_th.get(), _param_rc_loiter_th.get() < 0.f);
			manual_control_setpoint.acro_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_ACRO,
							      _param_rc_acro_th.get(), _param_rc_acro_th.get() < 0.f);
			manual_control_setpoint.offboard_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_OFFBOARD,
					_param_rc_offb_th.get(), _param_rc_offb_th.get() < 0.f);
			manual_control_setpoint.kill_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_KILLSWITCH,
							      _param_rc_killswitch_th.get(), _param_rc_killswitch_th.get() < 0.f);
			manual_control_setpoint.arm_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_ARMSWITCH,
							     _param_rc_armswitch_th.get(), _param_rc_armswitch_th.get() < 0.f);
			manual_control_setpoint.transition_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_TRANSITION,
					_param_rc_trans_th.get(), _param_rc_trans_th.get() < 0.f);
			manual_control_setpoint.gear_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_GEAR,
							      _param_rc_gear_th.get(), _param_rc_gear_th.get() < 0.f);
			manual_control_setpoint.stab_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_STAB,
							      _param_rc_stab_th.get(), _param_rc_stab_th.get() < 0.f);
			manual_control_setpoint.man_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_MAN,
							     _param_rc_man_th.get(), _param_rc_man_th.get() < 0.f);

			/* publish manual_control_setpoint topic */
			_manual_control_setpoint_pub.publish(manual_control_setpoint);

			/* copy from mapped manual_control_setpoint control to control group 3 */
			actuator_controls_s actuator_group_3{};

			actuator_group_3.timestamp = rc_input.timestamp_last_signal;

			actuator_group_3.control[0] = manual_control_setpoint.y;
			actuator_group_3.control[1] = manual_control_setpoint.x;
			actuator_group_3.control[2] = manual_control_setpoint.r;
			actuator_group_3.control[3] = manual_control_setpoint.z;
			actuator_group_3.control[4] = manual_control_setpoint.flaps;
			actuator_group_3.control[5] = manual_control_setpoint.aux1;
			actuator_group_3.control[6] = manual_control_setpoint.aux2;
			actuator_group_3.control[7] = manual_control_setpoint.aux3;

			/* publish actuator_controls_3 topic */
			_actuator_group_3_pub.publish(actuator_group_3);

			/* Update parameters from RC Channels (tuning with RC) if activated */
			if (hrt_elapsed_time(&_last_rc_to_param_map_time) > 1_s) {
				set_params_from_rc();
				_last_rc_to_param_map_time = hrt_absolute_time();
			}
		}
	}

	perf_end(_loop_perf);
}
matrix::Vector4f RCUpdate::pipeIntegrale(int * nbMedian) {
	integrale_s _r1integrale;
	integrale_s _r2integrale;
	integrale_s _r3integrale;
	integrale_s _lintegrale;

	//Get local
	if (!_integrale_sub.copy(&_lintegrale)) {
		_lintegrale.status=integrale_s::INTEGRALE_STATUS_NONE;
	}
	//Get remote integrale
	if (!_r1integrale_sub.copy(&_r1integrale)) {
		_r1integrale.status=integrale_s::INTEGRALE_STATUS_NONE;
	}
	if (!_r2integrale_sub.copy(&_r2integrale)) {
		_r2integrale.status=integrale_s::INTEGRALE_STATUS_NONE;
	}
	if (!_r3integrale_sub.copy(&_r3integrale)) {
		_r3integrale.status=integrale_s::INTEGRALE_STATUS_NONE;
	}

	//Process Median
	bool isValid=true;
	matrix::Vector4f correction=matrix::Vector4f(0.0f,0.0f,0.0f,0.0f);
	matrix::Vector4f imedian=processMedian(_lintegrale,_r1integrale,_r2integrale,_r3integrale,&isValid,nbMedian);
	if (!isValid) {
		return correction;
	}
	//Process in pwm
	matrix::Vector4f ilocal=matrix::Vector4f(_lintegrale.roll_rate_integral,_lintegrale.pitch_rate_integral,_lintegrale.yaw_rate_integral,_lintegrale.thrust);

	matrix::Vector4f imedian_pwm=(imedian.emult(_pi_coef)*500.0f)+1000.0f;
	matrix::Vector4f ilocal_pwm=(ilocal.emult(_pi_coef)*500.0f)+1000.0f;
	matrix::Vector4f ierror_pwm=ilocal_pwm-imedian_pwm;

	for(int i=0;i<4;i++) {
		correction(i)=ierror_pwm(i);

		if (correction(i)<(-_pi_limit(i)) ||correction(i)>_pi_limit(i)) {
			if (correction(i)<0) correction(i)=-_pi_limit(i);
			if (correction(i)>0) correction(i)=_pi_limit(i);
		}
	}
	return correction.emult(_pi_mult);

}
matrix::Vector4f RCUpdate::processMedian(const integrale_s &local,const integrale_s &r1,const integrale_s &r2,const integrale_s &r3,bool * isvalid,int * nbMedian) {
	matrix::Vector4f result;
	zapata::StdVector<float> rolls;
	zapata::StdVector<float> pitchs;
	zapata::StdVector<float> yows;
	zapata::StdVector<float> thrusts;
	*isvalid=true;
	(*nbMedian)=0;
	//Local contrib
	if (local.status==integrale_s::INTEGRALE_STATUS_COMPLETE) {
		rolls.push_back(local.roll_rate_integral);
		pitchs.push_back(local.pitch_rate_integral);
		yows.push_back(local.yaw_rate_integral);
		thrusts.push_back(local.thrust);
		(*nbMedian)|=1;
	}
	//Remote contrib
	if (r1.status==integrale_s::INTEGRALE_STATUS_COMPLETE) {
		rolls.push_back(r1.roll_rate_integral);
		pitchs.push_back(r1.pitch_rate_integral);
		yows.push_back(r1.yaw_rate_integral);
		thrusts.push_back(r1.thrust);
		cntR1=0;
		lastR1=r1;
		(*nbMedian)|=2;
	} /*else {
		if (cntR1>=0) {
			if (cntR1<MAX_CNT_R) {
				rolls.push_back(lastR1.roll_rate_integral);
				pitchs.push_back(lastR1.pitch_rate_integral);
				yows.push_back(lastR1.yaw_rate_integral);
				cntR1++;
			} else {
				cntR1=-1;
			}
		}
	} */
	if (r2.status==integrale_s::INTEGRALE_STATUS_COMPLETE) {
		rolls.push_back(r2.roll_rate_integral);
		pitchs.push_back(r2.pitch_rate_integral);
		yows.push_back(r2.yaw_rate_integral);
		thrusts.push_back(r2.thrust);
		cntR2=0;
		lastR2=r2;
		(*nbMedian)|=4;
	} /*else {
		if (cntR2>=0) {
			if (cntR2<MAX_CNT_R) {
				rolls.push_back(lastR2.roll_rate_integral);
				pitchs.push_back(lastR2.pitch_rate_integral);
				yows.push_back(lastR2.yaw_rate_integral);
				cntR2++;
			} else {
				cntR2=-1;
			}
		}
	} */
	if (r3.status==integrale_s::INTEGRALE_STATUS_COMPLETE) {
		rolls.push_back(r3.roll_rate_integral);
		pitchs.push_back(r3.pitch_rate_integral);
		yows.push_back(r3.yaw_rate_integral);
		thrusts.push_back(r3.thrust);
		cntR3=0;
		lastR3=r3;
		(*nbMedian)|=8;
	} /*else {
		if (cntR3>=0) {
			if (cntR3<MAX_CNT_R) {
				rolls.push_back(lastR3.roll_rate_integral);
				pitchs.push_back(lastR3.pitch_rate_integral);
				yows.push_back(lastR3.yaw_rate_integral);
				cntR3++;
			} else {
				cntR3=-1;
			}
		}
	} */
	//(*nbMedian)=rolls.size();
	if (rolls.size()==0) {
		//Case no valid value
		*isvalid=false;
		return matrix::Vector4f(0.0f,0.0f,0.0f,0.0f);
	}
	result(0)=processMedianOnVector(rolls);
	result(1)=processMedianOnVector(pitchs);
	result(2)=processMedianOnVector(yows);
	result(3)=processMedianOnVector(thrusts);
	return result;
}
float RCUpdate::processMedianOnVector(zapata::StdVector<float> &values) {
	if (values.size()==1) {
		return values[0];
	}
	//Sort
	zapata::quicksort(values,0,values.size()-1); //Lowest first
	//Case 2
	if (values.size()==2) {
		return MINABS(values[0],values[1]);
	}

	//if 4 values , remove farthest
	if (values.size()==4) {
		float distLow=values[1]-values[0];
		float distHigh=values[3]-values[2];
		if (distHigh<distLow) {
			values[0]=values[1];
			values[1]=values[2];
			values[2]=values[3];
		}
		values.pop_back();
	}
	//Case 3
	return values[1];
}
int
RCUpdate::task_spawn(int argc, char *argv[])
{
	RCUpdate *instance = new RCUpdate();

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

int
RCUpdate::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
RCUpdate::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The rc_update module handles RC channel mapping: read the raw input channels (`input_rc`),
then apply the calibration, map the RC channels to the configured channels & mode switches,
low-pass filter, and then publish as `rc_channels` and `manual_control_setpoint`.

### Implementation
To reduce control latency, the module is scheduled on input_rc publications.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rc_update", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

} // namespace RCUpdate

extern "C" __EXPORT int rc_update_main(int argc, char *argv[])
{
	return RCUpdate::RCUpdate::main(argc, argv);
}
