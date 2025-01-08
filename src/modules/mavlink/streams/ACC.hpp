/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#ifndef ACC_HPP
#define ACC_HPP

#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_acceleration.h>
class MavlinkStreamAcc : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamAcc(mavlink); }

	static constexpr const char *get_name_static() { return "ACC"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ACC; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _vehicle_acceleration_sub.advertised() ? MAVLINK_MSG_ID_ACC_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamAcc(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};

	bool send() override
	{
		vehicle_acceleration_s acc;
		if (_vehicle_acceleration_sub.update(&acc)) {

			mavlink_acc_t msg{};

			msg.time_boot_ms = acc.timestamp / 1000;
			msg.x = acc.xyz[0];
			msg.y = acc.xyz[1];
			msg.z = acc.xyz[2];


			mavlink_msg_acc_send_struct(_mavlink->get_channel(), &msg);

			return true;

		}

		return false;
	}
};

#endif // ACC_HPP
