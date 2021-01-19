#ifndef RC_INPUTCH_INFO_HPP
#define RC_INPUTCH_INFO_HPP

#include <uORB/topics/input_rc_changed.h>

class MavlinkStreamRCInputChanged : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamRCInputChanged(mavlink); }

	static constexpr const char *get_name_static() { return "RC_CHANNELS_CH"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_RC_CHANNELS_CH; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_RC_CHANNELS_CH_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamRCInputChanged(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _input_rc_sub{ORB_ID(input_rc_changed)};
protected:
	bool send(const hrt_abstime t) override
	{

		input_rc_changed_s ch;

		if (_input_rc_sub.update(&ch)) {
			mavlink_rc_channels_ch_t msg{};
			msg.time_boot_ms = ch.timestamp;
			msg.chan1_raw=ch.values[0];
			msg.chan2_raw=ch.values[1];
			msg.chan3_raw=ch.values[2];
			msg.chan4_raw=ch.values[3];
			msg.chan5_raw=ch.values[4];
			msg.chan6_raw=ch.values[5];
			msg.chan7_raw=ch.values[6];
			msg.chan8_raw=ch.values[7];
			msg.chan9_raw=ch.values[8];
			msg.chan10_raw=ch.values[9];
			msg.chan11_raw=ch.values[10];
			msg.chan12_raw=ch.values[11];
			msg.chan13_raw=ch.values[12];
			msg.chan14_raw=ch.values[13];
			msg.chan15_raw=ch.values[14];
			msg.chan16_raw=ch.values[15];
			msg.chan17_raw=ch.values[16];
			msg.chan18_raw=ch.values[17];
			mavlink_msg_rc_channels_ch_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};


#endif
