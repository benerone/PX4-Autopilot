/****************************************************************************
 *
 *   Copyright (C) 2012-2019 PX4 Development Team. All rights reserved.
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

#include <mixer/MixerBase/Mixer.hpp>

/** mixer input */
struct kmixer_control_s {
	uint8_t			control_group;	/**< group from which the input reads */
	uint8_t			control_index;	/**< index within the control group */
	float			scaler;		/**< scaling applied to the input before use */
};

#define KMIXER_SIZE(_icount)	(sizeof(struct kmixer_s) + (_icount) * sizeof(struct kmixer_control_s))

/** simple mixer */
struct kmixer_s {
	uint8_t			control_count;	/**< number of inputs */
	float min;
	float max;
	int throttle_index;
	kmixer_control_s	controls[];	/**< actual size of the array is set by control_count */
};

/**
 * KMixer summing mixer.
 *
 * Collects zero or more inputs and mixes them to a single output.
 */
class KMixer : public Mixer
{
public:
	/**
	 * Constructor
	 *
	 * @param mixinfo		Mixer configuration.  The pointer passed
	 *				becomes the property of the mixer and
	 *				will be freed when the mixer is deleted.
	 */
	KMixer(ControlCallback control_cb, uintptr_t cb_handle, kmixer_s *mixinfo);
	virtual ~KMixer();

	// no copy, assignment, move, move assignment
	KMixer(const KMixer &) = delete;
	KMixer &operator=(const KMixer &) = delete;
	KMixer(KMixer &&) = delete;
	KMixer &operator=(KMixer &&) = delete;

	/**
	 * Factory method with full external configuration.
	 *
	 * Given a pointer to a buffer containing a text description of the mixer,
	 * returns a pointer to a new instance of the mixer.
	 *
	 * @param control_cb		The callback to invoke when fetching a
	 *				control value.
	 * @param cb_handle		Handle passed to the control callback.
	 * @param buf			Buffer containing a text description of
	 *				the mixer.
	 * @param buflen		Length of the buffer in bytes, adjusted
	 *				to reflect the bytes consumed.
	 * @return			A new SimpleMixer instance, or nullptr
	 *				if the text format is bad.
	 */
	static KMixer			*from_text(Mixer::ControlCallback control_cb, uintptr_t cb_handle, const char *buf,
			unsigned &buflen);

	unsigned			mix(float *outputs, unsigned space) override;

	void				groups_required(uint32_t &groups) override;

	void                            print(PrinterFunction pF);

	/**
	 * @brief Set trim offset for this mixer
	 *
	 * @return the number of outputs this mixer feeds to
	 */
	unsigned set_trim(float trim);

	/**
	 * @brief Get trim offset for this mixer
	 *
	 * @return the number of outputs this mixer feeds to
	 */
	unsigned get_trim(float *trim);
private:
	static int parse_control_scaler(const char *buf, unsigned &buflen, float &scaler, uint8_t &control_group,
					uint8_t &control_index);


	kmixer_s			*_pinfo;
	float _trim;
};
