#ifndef SBG_UTC_HPP
#define SBG_UTC_HPP

#include <uORB/topics/sbg_utc.h>

class MavlinkStreamSbgUTC : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamSbgUTC(mavlink); }

	static constexpr const char *get_name_static() { return "UTC_SBG"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_UTC_SBG; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		//return _integrale_sub.advertised() ? MAVLINK_MSG_ID_INTEGRALE_VALUES_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
		return MAVLINK_MSG_ID_UTC_SBG_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamSbgUTC(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sbg_utc_sub{ORB_ID(sbg_utc)};

protected:
	bool send(const hrt_abstime t) override
	{

		sbg_utc_s sbg_utc;
		if (_sbg_utc_sub.update(&sbg_utc)) {
			mavlink_utc_sbg_t msg{};
			msg.year=sbg_utc.year;
			msg.month=sbg_utc.month;
			msg.day=sbg_utc.day;
			msg.hour=sbg_utc.hour;
			msg.minute=sbg_utc.minute;
			msg.second=sbg_utc.second;
			mavlink_msg_utc_sbg_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};


#endif
