/*
 * Copyright (c) 2012, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file /kobuki_keyop/src/keyop_core.cpp
 *
 * @brief Creates a node for remote controlling parts of robot_core.
 *
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <ros/ros.h>
#include <ecl/time.hpp>
#include <ecl/exceptions.hpp>
#include <std_srvs/Empty.h>
#include <kobuki_msgs/MotorPower.h>
#include "../include/keyop_core/keyopcore.hpp"


/*****************************************************************************
 ** Namespaces
 *****************************************************************************/
using roskobuki_navigation::NaviController;
namespace keyopcore {

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

/**
 * @brief Default constructor, needs initialisation.
 */

KeyOpCore::KeyOpCore(NaviController* robotcontroller)
{

	quit_requested = false;
	key_file_descriptor = 0;
	tcgetattr(key_file_descriptor, &original_terminal_state); // get terminal properties
	this->robotcontroller = robotcontroller;
}

KeyOpCore::~KeyOpCore() {
	tcsetattr(key_file_descriptor, TCSANOW, &original_terminal_state);
}

bool KeyOpCore::initialize() {
	// start keyboard input thread
	thread.start(&KeyOpCore::keyboardInputLoop, *this);
	return true;
}


void KeyOpCore::spin() {
	ros::Rate loop_rate(10);

	while (!quit_requested && ros::ok()) {
		loop_rate.sleep();
	}

}

/*****************************************************************************
 ** Implementation [Keyboard]
 *****************************************************************************/

/**
 * @brief The worker thread function that accepts input keyboard commands.
 *
 * This is ok here - but later it might be a good idea to make a node which
 * posts keyboard events to a topic. Recycle common code if used by many!
 */
void KeyOpCore::keyboardInputLoop() {
	struct termios raw;
	memcpy(&raw, &original_terminal_state, sizeof(struct termios));

	raw.c_lflag &= ~(ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(key_file_descriptor, TCSANOW, &raw);

	puts("Reading from keyboard");
	puts("---------------------------");
	puts("w/s arrows : linear velocity forward/backward.");
	puts("a/d arrows : angular velocity turn left/right.");
	puts("x : stop /start automove.");
	puts("q : quit.");

	char c;
	while (!quit_requested) {
		if (read(key_file_descriptor, &c, 1) < 0) {
			perror("read char failed():");
			exit(-1);
		}
		printf("input :%c\r\n", c);
		processKeyboardInput(c);
	}
}

/**
 * @brief Process individual keyboard inputs.
 *
 * @param c keyboard input.
 */
void KeyOpCore::processKeyboardInput(char c) {
	/*
	 * Arrow keys are a bit special, they are escape characters - meaning they
	 * trigger a sequence of keycodes. In this case, 'esc-[-Keycode_xxx'. We
	 * ignore the esc-[ and just parse the last one. So long as we avoid using
	 * the last one for its actual purpose (e.g. left arrow corresponds to
	 * esc-[-D) we can keep the parsing simple.
	 */
	switch (c) {
	case 'a': {
		robotcontroller->turnleft(90, robotcontroller->vel_ang_);
		break;
	}

	case 'd': {
		robotcontroller->turnright(90, robotcontroller->vel_ang_);
		break;
	}

	case 'w': {
		robotcontroller->move();
		break;
	}

	case 's': {
		robotcontroller->moveback();
		break;
	}

	case 'x': {
		robotcontroller->stop_automove();
		break;
	}

	case 'n': {
		robotcontroller->next_move();
		break;
	}

	case 'q': {
		robotcontroller->exitrequest();
		quit_requested = true;
		break;
	}

	default: {
		break;
	}
	}
}
} // namespace keyop_core
