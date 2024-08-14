/**
 * @file px4_simple_app.c
 * 1) read data from uORB about vehicle
 * 2) Send data out using mavlink through uORB with custom message
 * 3) Read and set RC inputs/outputs
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>

#include <uORB/topics/vehicle_local_position.h>

#include <uORB/topics/custom_msg.h>

#include <uORB/topics/actuator_motors.h>

extern "C" __EXPORT int custom_main(int argc, char *argv[]);

int custom_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");

	int att_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
	int angv_sub_fd = orb_subscribe(ORB_ID(vehicle_angular_velocity));
	orb_set_interval(att_sub_fd, 100);

	int vlp_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position));
	orb_set_interval(vlp_sub_fd, 100);

	px4_pollfd_struct_t fds[] = {
		//{ .fd = 0, .events = POLLIN},
		{ .fd = att_sub_fd,	.events = POLLIN },
		{ .fd = vlp_sub_fd,   	.events = POLLIN },
	};


	//setup the uorb message out to mavlink
	struct custom_msg_s cmsg;
	memset(&cmsg, 0, sizeof(cmsg));
	orb_advert_t cm_ad = orb_advertise(ORB_ID(custom_msg),&cmsg);
	//set motor outputs
	//struct actuator_motors_s am_msg;
	//orb_advert_t am_ad = orb_advertise(ORB_ID(actuator_motors),&am_msg);

	PX4_INFO("START");
	int error_counter = 0;

	/*for(int i=0;i<1000;i++)
	{
		uint64_t timestamp_us = hrt_absolute_time();
		am_msg.timestamp = timestamp_us;
		am_msg.timestamp_sample = timestamp_us;
		am_msg.control[0] = 0.99f;
		am_msg.control[1] = 0.99f;
		am_msg.control[2] = 0.99f;
		am_msg.control[3] = 0.99f;
		orb_publish(ORB_ID(actuator_motors), am_ad, &am_msg);
	}*/
	for (int i = 0; i < 25; i++) {
		uint64_t timestamp_us = hrt_absolute_time();
		cmsg.timestamp = timestamp_us;
		cmsg.value = 42;
		orb_publish(ORB_ID(custom_msg), cm_ad, &cmsg);

		int poll_ret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);
		PX4_INFO("POLL RESULTS %d", poll_ret);
		if (poll_ret == 0) {
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			if (error_counter < 10 || error_counter % 50 == 0) {
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}
			error_counter++;
		} else
		{
			if(fds[0].revents & POLLIN) {
				struct vehicle_attitude_s vehicle_attitude;
				orb_copy(ORB_ID(vehicle_attitude), att_sub_fd, &vehicle_attitude);
				struct vehicle_angular_velocity_s angular_velocity;
				orb_copy(ORB_ID(vehicle_angular_velocity), angv_sub_fd, &angular_velocity);
				PX4_INFO("Quat:\t\t%8.4f\t%8.4f\t%8.4f\t%8.4f",
					 (double)vehicle_attitude.q[0],
					 (double)vehicle_attitude.q[1],
					 (double)vehicle_attitude.q[2],
					 (double)vehicle_attitude.q[3]);
				PX4_INFO("rpy speed:\t\t%8.4f\t%8.4f\t%8.4f",
					(double)angular_velocity.xyz[0],
					(double)angular_velocity.xyz[1],
					(double)angular_velocity.xyz[2]);
			}
			if(fds[1].revents & POLLIN) {
				struct vehicle_local_position_s vlp;
				orb_copy(ORB_ID(vehicle_local_position), vlp_sub_fd, &vlp);
				PX4_INFO("Postion:\t\t%8.4f\t%8.4f\t%8.4f",
					 (double)vlp.x,
					 (double)vlp.y,
					 (double)vlp.z);
				PX4_INFO("Velocity:\t%8.4f\t%8.4f\t%8.4f",
					 (double)vlp.vx,
					 (double)vlp.vy,
					 (double)vlp.vz);
				PX4_INFO("Acceleration:\t%8.4f\t%8.4f\t%8.4f",
					 (double)vlp.ax,
					 (double)vlp.ay,
					 (double)vlp.az);
				PX4_INFO("Heading:\t\t%8.4f\t%8.4f\t%8.4f",
					 (double)vlp.heading,
					 (double)0,
					 (double)0);
			}
		}
	}

	PX4_INFO("exiting");

	return 0;
}
