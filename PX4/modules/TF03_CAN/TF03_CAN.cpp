/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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

/**
 * @file TF03_CAN.cpp
 * Collect and publish altimeter data from the benewake TF03 Lidar
 *
 * @author Example User <r.reinfeld@campus.tu-berlin.de>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>


#include <nuttx/can/can.h>
#include <arch/board/board.h>



#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <fcntl.h>
#include <debug.h>
#include <stdbool.h>
#include <semaphore.h>
#include <nuttx/arch.h>
#include <pthread.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_sem.h>

#include <uORB/uORB.h>
// Publish
#include <uORB/topics/distance_sensor.h>

#include <sys/time.h>
#include <time.h>


extern "C" int TF03_CAN_main(int argc, char *argv[]);

	

class TF03_CAN : public ModuleBase<TF03_CAN>
{
public:
	TF03_CAN();

	virtual ~TF03_CAN() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static TF03_CAN *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;
	
	
private:

	int CAN_INIT()
	
};


int TF03_CAN::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
 Treiber für Benewake TF03 Lidar über CAN
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	//PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	//PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int TF03_CAN::print_status()
{
	PX4_INFO("Running");
	PX4_INFO("Version: 9");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int TF03_CAN::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int TF03_CAN::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("TF03_CAN",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      2000,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return 1;
	}

	return 0;
}

TF03_CAN *TF03_CAN::instantiate(int argc, char *argv[])
{

	TF03_CAN *instance = new TF03_CAN();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

TF03_CAN::TF03_CAN()
{
}

void TF03_CAN::run()
{
	/*File Descriptor for CAN device File*/
	int fd=0;
	/*INIT*/
	fd=CAN_INIT();
	//Topic Publish
	orb_advert_t distance_sensor_pub;
	struct distance_sensor_s 		lidar;
	memset(&_combined, 0, sizeof(_distance));
	_combined_pub = orb_advertise(ORB_ID(distance_sensor), &lidar);
	// variables for CAN Read 
	int msgdlc;
	size_t msgsize;
	size_t nbytes;
	struct can_msg_s rxmsg;
	msgsize = sizeof(struct can_msg_s);
	while(!should_exit())
	{
		// read incoming lidar data
		nbytes = read(fd, &rxmsg, msgsize);
		if (nbytes < CAN_MSGLEN(0) || nbytes > msgsize)
		{
			printf("ERROR: read(%ld) returned %ld\n",
            (long)msgsize, (long)nbytes);
		}
		// publish lidar data
		lidar.timestamp = hrt_absolute_time();
		lidar.orientation =25;
		lidar.current_distance = float((rxmsg.cm_data[0]+rxmsg.cm_data[1])/2);
		lidar.min_distance = 0.1f;
		lidar.max_distance = 180.f;
		lidar.variance = float((rxmsg.cm_data[0]-rxmsg.cm_data[1])/2);
		lidar.signal_quality = -1;

		/* TODO: set proper ID */
		lidar.id = 0;
		orb_publish(ORB_ID(distance_sensor),_distance_sensor_pub, &lidar);	
	}
}

int TF03_CAN_main(int argc, char *argv[])
{
	return TF03_CAN::main(argc, argv);
}

int TF03_CAN::CAN_INIT(void)
{
	int ret=0;
// Variable CAN
	struct canioc_bittiming_s bt;
	int fd;
	fd = open("/dev/uavcan", O_RDWR);
	if (fd < 0)
	{
		PX4_ERR("ERROR: open %s failed: %d\n",
           "/dev/uavcan", errno);
	}
	//Get Bittiming	
	ret = ioctl(fd, CANIOC_GET_BITTIMING, (unsigned long)((uintptr_t)&bt));
	if (ret < 0)
	{
		PX4_ERR("Bit timing not available: %d\n", errno);
	}
	else
	{
		PX4_INFO("Bit timing successed:\n");
	}
return fd;
}




