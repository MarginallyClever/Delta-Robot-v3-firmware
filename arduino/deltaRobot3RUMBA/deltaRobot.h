#ifndef DELTAROBOT_H
#define DELTAROBOT_H
//------------------------------------------------------------------------------
// Delta Robot v8 - Supports RUMBA 6-axis motor controller
// dan@marginallycelver.com 2014-01-07
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/DeltaRobotv8 for more information.


#include "vector3.h"


//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------

#define MACHINE_STYLE_MARGINALLYCLEVER_V8  1
#define MACHINE_STYLE_JUDAH                2

// change this bit to match your machine
#define MACHINE_STYLE MACHINE_STYLE_MARGINALLYCLEVER_V8
//#define MACHINE_STYLE MACHINE_STYLE_JUDAH


#if MACHINE_STYLE == MACHINE_STYLE_MARGINALLYCLEVER_V8
// physical measurements of the machine
#define CENTER_TO_SHOULDER       (3.77f)  // cm
#define SHOULDER_TO_ELBOW        (5.0f)  // cm
#define ELBOW_TO_WRIST           (16.5f)  // cm
#define EFFECTOR_TO_WRIST        (1.724f)  // cm
#define CENTER_TO_FLOOR          (18.9)  // cm
#define DEGREES_ABOVE_HORIZONTAL (24.0f)
#endif


#if MACHINE_STYLE == MACHINE_STYLE_JUDAH
// physical measurements of the machine
#define CENTER_TO_SHOULDER       (12.65682f)  // cm (f)
#define SHOULDER_TO_ELBOW        (20.32f)  // cm (Rf)
#define ELBOW_TO_WRIST           (55.82f)  // cm (Re)
#define EFFECTOR_TO_WRIST        (1.724f)  // cm (E)
#define CENTER_TO_FLOOR          (8.89)  // cm
#define DEGREES_ABOVE_HORIZONTAL (24.0f)
#endif


//------------------------------------------------------------------------------
// STRUCTS
//------------------------------------------------------------------------------


struct Joint {
  Vector3 pos;
  Vector3 relative;
};


struct Arm {
  Vector3 shoulder;
  Joint elbow;
  Joint wrist;
  Joint wop;
  
  float angle;

  // for motors
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


struct DeltaRobot {
  Arm arms[NUM_AXIES];
  Vector3 ee;
  Joint base;
  Vector3 tool_offset[NUM_TOOLS];
  int current_tool;
  float default_height;
};


extern int mode_abs;
extern DeltaRobot robot;


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

