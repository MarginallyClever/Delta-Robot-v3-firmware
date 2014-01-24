#ifndef ARM_H
#define ARM_H
//------------------------------------------------------------------------------
// Delta Robot v8 - Supports RUMBA 6-axis motor controller
// dan@marginallycelver.com 2014-01-07
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/DeltaRobotv8 for more information.


#include "Joint.h"
#include <Servo.h>


struct Arm {
  Vector3 shoulder;
  Joint elbow;
  Joint wrist;
  Joint wop;
  
  float angle;

  // for motors  
  int last_step;
  int new_step;
  
  int delta;
  int absdelta;
  int dir;
  int over;

  Vector3 plane_ortho;
  Vector3 plane_normal;

  // for limit switches
  int motor_step_pin;
  int motor_dir_pin;
  int motor_enable_pin;
  int limit_switch_pin;
  int limit_switch_state;  
};


/**
* This file is part of Delta Robot v8.
*
* Delta Robot v8 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Delta Robot v8 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Delta Robot v8. If not, see <http://www.gnu.org/licenses/>.
*/
#endif

