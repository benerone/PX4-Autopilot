#ifndef CAN_INFO_HPP
#define CAN_INFO_HPP

#include <uORB/topics/can_status.h>
#include <uORB/topics/sbg_status.h>

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
	uORB::Subscription _sbg_status_sub{ORB_ID(sbg_status)};

protected:
	bool send(const hrt_abstime t) override
	{

		can_status_s can_status;

		if (_can_status_sub.update(&can_status)) {
			mavlink_extra_status_t msg{};
			msg.can1_state=can_status.can_error_1?0:1;
			msg.can2_state=can_status.can_error_2?0:1;
			sbg_status_s sbg_status;
			if (_sbg_status_sub.update(&sbg_status)) {
				msg.sbg_solution=sbg_status.solution_status;
				msg.sbg_general=sbg_status.general_status;
				msg.sbg_com=sbg_status.com_status;
				msg.sbg_aiding=sbg_status.aiding_status;
				msg.sbg_imu=sbg_status.imu_status;
				msg.roll_acc=sbg_status.roll_acc;
				msg.pitch_acc=sbg_status.pitch_acc;
				msg.yaw_acc=sbg_status.yaw_acc;
				msg.vel_n_acc=sbg_status.vel_n_acc;
				msg.vel_e_acc=sbg_status.vel_e_acc;
				msg.vel_d_acc=sbg_status.vel_d_acc;
				msg.lat_acc=sbg_status.lat_acc;
				msg.lon_acc=sbg_status.lon_acc;
				msg.vert_acc=sbg_status.vert_acc;
			} else {
				msg.sbg_solution=0;
				msg.sbg_general=0;
				msg.sbg_com=0;
				msg.sbg_aiding=0;
				msg.sbg_imu=0;
				msg.roll_acc=0.0;
				msg.pitch_acc=0.0;
				msg.yaw_acc=0.0;
				msg.vel_n_acc=0.0;
				msg.vel_e_acc=0.0;
				msg.vel_d_acc=0.0;
				msg.lat_acc=0.0;
				msg.lon_acc=0.0;
				msg.vert_acc=0.0;
			}
			mavlink_msg_extra_status_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};


#endif
