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

#ifndef CUSTOM_MSG_HPP
#define CUSTOM_MSG_HPP

#include <uORB/topics/custom_msg.h>
//This is almost cookie cutter stuff to deal with uORB to Mavlink
class MavlinkStreamCustomMsg : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamCustomMsg(mavlink); }

	static constexpr const char *get_name_static() { return "CUSTOM_MSG"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_CUSTOM_MSG; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	bool const_rate() override { return true; }

	unsigned get_size() override
	{
		return _custom_msg_sub.advertised() ? MAVLINK_MSG_ID_CUSTOM_MSG_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamCustomMsg(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _custom_msg_sub{ORB_ID(custom_msg)};
	bool send() override
	{
		//this gets called if a uORB message is found
		bool sent = false;
		custom_msg_s umsg;
		while ((_mavlink->get_free_tx_buf() >= get_size()) &&_custom_msg_sub.update(&umsg)) {
			mavlink_custom_msg_t msg{};
			msg.value = umsg.value;
			mavlink_msg_custom_msg_send_struct(_mavlink->get_channel(), &msg);
			sent = true;
			PX4_INFO("SET CUSTOM MESSAGE to MAVLINK OUT");
		}


		return sent;
	}
};

#endif
