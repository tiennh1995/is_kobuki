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
 * @file /include/teleop_core/keyop_core.hpp
 *
 * @brief The controlling node for remote operations on robot_core.
 *
 **/

/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef KEYOPCORE_NODE_HPP_
#define KEYOPCORE_NODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/


#include <termios.h> // for keyboard input
#include <ecl/threads.hpp>
#include <kobuki_msgs/KeyboardInput.h> // keycodes from remote teleops.

#include "../roskobuki_navi_controller.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

using roskobuki_navigation::NaviController;
namespace keyopcore
{

/*****************************************************************************
 ** Interface
 *****************************************************************************/
/**
 * @brief Keyboard remote control for our robot core (mobile base).
 *
 */
class KeyOpCore
{
public:
  /*********************
   ** C&D
   **********************/

  KeyOpCore(NaviController* robotcontroller);
  ~KeyOpCore();

  bool initialize();
  /*********************
   ** Runtime
   **********************/
  void spin();

private:



  NaviController* robotcontroller;
  /*********************
   ** Commands
   **********************/


  /*********************
   ** Keylogging
   **********************/

  void keyboardInputLoop();
  void processKeyboardInput(char c);

  bool quit_requested;
  int key_file_descriptor;
  struct termios original_terminal_state;
  ecl::Thread thread;
};

} // namespace keyop_core

#endif /* KEYOP_CORE_NODE_HPP_ */
