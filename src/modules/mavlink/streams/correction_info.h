#ifndef CORRECTION_INFO_HPP
#define CORRECTION_INFO_HPP

#include <uORB/topics/pipe_correction.h>

class MavlinkStreamCorrection : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamCorrection(mavlink); }

	static constexpr const char *get_name_static() { return "CORRECTION"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_CORRECTION_VALUES; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_CORRECTION_VALUES_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamCorrection(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _correction_sub{ORB_ID(pipe_correction)};

protected:
	bool send(const hrt_abstime t) override
	{

		pipe_correction_s pipe_correction;

		if (_correction_sub.update(&pipe_correction)) {
			mavlink_correction_values_t msg{};
			msg.time_usec = pipe_correction.timestamp;
			msg.corrroll = pipe_correction.roll_correction;
			msg.corrpitch = pipe_correction.pitch_correction;
			msg.corryaw = pipe_correction.yaw_correction;
			msg.corrvx = pipe_correction.vx_correction;
			msg.corrvy = pipe_correction.vy_correction;
			msg.corrthrust = pipe_correction.thrust_correction;
			msg.medianroll=pipe_correction.median_roll;
			msg.medianpitch=pipe_correction.median_pitch;
			msg.medianyaw=pipe_correction.median_yaw;
			msg.medianthrust=pipe_correction.median_thrust;
			msg.medianthract=pipe_correction.median_thr_act;
			mavlink_msg_correction_values_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};


#endif
