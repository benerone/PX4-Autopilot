#ifndef CAN_INFO_HPP
#define CAN_INFO_HPP

#include <uORB/topics/can_status.h>

class MavlinkStreamCanStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamCanStatus(mavlink); }

	static constexpr const char *get_name_static() { return "EXTRA_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_EXTRA_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		//return _integrale_sub.advertised() ? MAVLINK_MSG_ID_INTEGRALE_VALUES_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
		return MAVLINK_MSG_ID_EXTRA_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamCanStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _can_status_sub{ORB_ID(can_status)};

protected:
	bool send(const hrt_abstime t) override
	{

		can_status_s can_status;

		if (_can_status_sub.update(&can_status)) {
			mavlink_extra_status_t msg{};
			msg.can1_state=can_status.can_error_1?0:1;
			msg.can2_state=can_status.can_error_2?0:1;
			mavlink_msg_extra_status_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};


#endif
