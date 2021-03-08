/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file KMixer.cpp
 *
 * Simple summing mixer.
 */

#include "KMixer.hpp"

#include <stdio.h>
#include <stdlib.h>

#define debug(fmt, args...)	do { } while(0)
//#define debug(fmt, args...)	do { printf("[mixer] " fmt "\n", ##args); } while(0)

#define THROTTLE_GC 0
#define THROTTLE_IC 3

KMixer::KMixer(ControlCallback control_cb, uintptr_t cb_handle, kmixer_s *mixinfo) :
	Mixer(control_cb, cb_handle),
	_pinfo(mixinfo)
{
}

KMixer::~KMixer()
{
	if (_pinfo != nullptr) {
		free(_pinfo);
	}
}
KMixer *
KMixer::from_text(Mixer::ControlCallback control_cb, uintptr_t cb_handle, const char *buf, unsigned &buflen)
{
	KMixer *sm = nullptr;
	kmixer_s *mixinfo = nullptr;
	unsigned inputs;
	int minV,maxV;
	const char *end = buf + buflen;

	/* enforce that the mixer ends with a new line */
	if (!string_well_formed(buf, buflen)) {
		return nullptr;
	}

	/* get the base info for the mixer */
	if (sscanf(buf, "K: %u %d %d", &inputs, &minV,&maxV) != 3) {
		debug("K parse failed on '%s'", buf);
		goto out;
	}

	/* at least 1 input is required */
	if (inputs == 0) {
		debug("K parse got 0 inputs");
		goto out;
	}

	buf = skipline(buf, buflen);

	if (buf == nullptr) {
		debug("no line ending, line is incomplete");
		goto out;
	}

	mixinfo = (kmixer_s *)malloc(KMIXER_SIZE(inputs));

	if (mixinfo == nullptr) {
		debug("could not allocate memory for mixer info");
		goto out;
	}

	mixinfo->control_count = inputs;
	mixinfo->min=minV/10000.0f;
	mixinfo->max=maxV/10000.0f;
	mixinfo->throttle_index=-1;

	for (unsigned i = 0; i < inputs; i++) {
		if (parse_control_scaler(end - buflen, buflen,
					 mixinfo->controls[i].scaler,
					 mixinfo->controls[i].control_group,
					 mixinfo->controls[i].control_index)) {
			debug("simple mixer parser failed parsing ctrl scaler tag, ret: '%s'", buf);
			goto out;
		}
		if (mixinfo->controls[i].control_group==THROTTLE_GC && mixinfo->controls[i].control_index==THROTTLE_IC) {
			mixinfo->throttle_index=i;
		}
	}

	sm = new KMixer(control_cb, cb_handle, mixinfo);

	if (sm != nullptr) {
		mixinfo = nullptr;
		debug("loaded mixer with %d input(s)", inputs);

	} else {
		debug("could not allocate memory for mixer");
	}

out:

	if (mixinfo != nullptr) {
		free(mixinfo);
	}

	return sm;
}
int KMixer::parse_control_scaler(const char *buf, unsigned &buflen, float &scaler, uint8_t &control_group,
					uint8_t &control_index) {
	unsigned u[2];
	int scaleV;
	buf = findtag(buf, buflen, 'P');

	if ((buf == nullptr) ) {
		debug("K control parser failed finding tag, ret: '%s'", buf);
		return -1;
	}

	if (sscanf(buf, "P: %u %u %d",
		   &u[0], &u[1], &scaleV) != 3) {
		debug("control parse failed on '%s'", buf);
		return -1;
	}

	buf = skipline(buf, buflen);

	if (buf == nullptr) {
		debug("no line ending, line is incomplete");
		return -1;
	}

	control_group		= u[0];
	control_index		= u[1];
	scaler = scaleV/10000.0f;

	return 0;

}
void
KMixer::groups_required(uint32_t &groups)
{
	for (unsigned i = 0; i < _pinfo->control_count; i++) {
		groups |= 1 << _pinfo->controls[i].control_group;
	}
}
void  KMixer::print(PrinterFunction pF) {
	(*pF)("K Mixer with %d controls and min:%f max:%f thIndex:%d",_pinfo->control_count,
	(double)_pinfo->min,(double)_pinfo->max,_pinfo->throttle_index);

	for(int i=0;i<(int)_pinfo->control_count;i++) {
		(*pF)("->Ctrl %d (%c)[%d:%d]: %f ",i,i==_pinfo->throttle_index?'T':' ',_pinfo->controls[i].control_group,_pinfo->controls[i].control_index,
			(double)_pinfo->controls[i].scaler);
	}

}



unsigned
KMixer::mix(float *outputs, unsigned space)
{
	float sum = 0.0f;

	if (_pinfo == nullptr) {
		return 0;
	}

	if (space < 1) {
		return 0;
	}

	//Check if throttle is used
	if (_pinfo->throttle_index>=0) {
		//Used
		//Process throttle part
		float inputTr = 0.0f;

		_control_cb(_cb_handle,
				_pinfo->controls[_pinfo->throttle_index].control_group,
				_pinfo->controls[_pinfo->throttle_index].control_index,
				inputTr);

		float throttle_sum=inputTr*_pinfo->controls[_pinfo->throttle_index].scaler;

		bool useNeg=true;
		bool usePos=true;
		if (throttle_sum<_pinfo->min) {
			useNeg=false;
			throttle_sum=_pinfo->min;
		}
		if (throttle_sum>_pinfo->max) {
			usePos=false;
			throttle_sum=_pinfo->max;
		}
		sum=throttle_sum;
		for (int i = 0; i < _pinfo->control_count; i++) {
			if (i!=_pinfo->throttle_index) {
				float input = 0.0f;

				_control_cb(_cb_handle,
					_pinfo->controls[i].control_group,
					_pinfo->controls[i].control_index,
					input);

				input=input*_pinfo->controls[i].scaler;
				if ((input<0.0f && useNeg) || (input>0.0f && usePos)) {
					sum+=input;
				}
			}

		}
	} else {
		//Not used
		for (unsigned i = 0; i < _pinfo->control_count; i++) {
			float input = 0.0f;

			_control_cb(_cb_handle,
				_pinfo->controls[i].control_group,
				_pinfo->controls[i].control_index,
				input);

			sum += _pinfo->controls[i].scaler*input;
		}
	}

	//Clamp
	if (sum<_pinfo->min) {
		sum=_pinfo->min;
	}
	if (sum>_pinfo->max) {
		sum=_pinfo->max;
	}
	if (sum>=0) {
		*outputs=((1.0f-_trim)*sum/_pinfo->max)+_trim;
	} else {
		*outputs=((1.0f+_trim)*(_pinfo->min-sum)/_pinfo->min)-1.0f;
	}
	return 1;
}
/**
 * @brief Set trim offset for this mixer
 *
 * @return the number of outputs this mixer feeds to
 */
unsigned KMixer::set_trim(float trim) {
	_trim=trim;
	return 1;
}

/**
 * @brief Get trim offset for this mixer
 *
 * @return the number of outputs this mixer feeds to
 */
unsigned KMixer::get_trim(float *trim) {
	(*trim)=_trim;
	return 1;
}
