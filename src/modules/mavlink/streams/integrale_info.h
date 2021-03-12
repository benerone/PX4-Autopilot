#ifndef INTEGRALE_INFO_HPP
#define INTEGRALE_INFO_HPP

#include <uORB/topics/integrale.h>

class MavlinkStreamIntegrale : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamIntegrale(mavlink); }

	static constexpr const char *get_name_static() { return "INTEGRALE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_INTEGRALE_VALUES; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		//return _integrale_sub.advertised() ? MAVLINK_MSG_ID_INTEGRALE_VALUES_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
		return MAVLINK_MSG_ID_INTEGRALE_VALUES_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamIntegrale(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _integrale_sub{ORB_ID(integrale)};

protected:
	bool send(const hrt_abstime t) override
	{

		integrale_s integrale;

		if (_integrale_sub.update(&integrale)) {
			mavlink_integrale_values_t msg{};
			msg.time_usec = integrale.timestamp;
			msg.rollspeedintegral = integrale.roll_rate_integral;
			msg.pitchspeedintegral = integrale.pitch_rate_integral;
			msg.yawspeedintegral = integrale.yaw_rate_integral;
			msg.vx = integrale.vx;
			msg.vy = integrale.vy;
			msg.thrust = integrale.thrust;
			mavlink_msg_integrale_values_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};


#endif
