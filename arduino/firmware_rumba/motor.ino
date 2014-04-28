//------------------------------------------------------------------------------
// Delta Robot v8 - Supports RUMBA 6-axis motor controller
// dan@marginallycelver.com 2014-01-07
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/DeltaRobotv8 for more information.


//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include "configuration.h"


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
#if VERBOSE > 2
char *letter="XYZ";
#endif


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

/**
 * Supports movement with both styles of Motor Shield
 * @input motor which motor to move
 * @input dir which direction to step
 **/
void motor_onestep(int motor,int dir) {
#if VERBOSE > 2
  Serial.print(letter[motor]);
#endif
  digitalWrite(robot.arms[motor].motor_dir_pin,dir<0?LOW:HIGH);
  digitalWrite(robot.arms[motor].motor_step_pin,LOW);
  digitalWrite(robot.arms[motor].motor_step_pin,HIGH);
}


void outputsteps() {
#if VERBOSE > 1
  Serial.print(F("\tSteps="));   Serial.print(robot.arms[0].new_step);
  Serial.print(F(","));   Serial.print(robot.arms[1].new_step);
  Serial.print(F(","));   Serial.println(robot.arms[2].new_step);
#endif
}


/**
 * Use Bresenham's line algorithm to synchronize the movement of all three motors and approximate movement in a straight line.
 */
void motor_segment(float a0,float a1,float a2,float fr) {
  robot.arms[0].new_step = a0 * MICROSTEP_PER_DEGREE;
  robot.arms[1].new_step = a1 * MICROSTEP_PER_DEGREE;
  robot.arms[2].new_step = a2 * MICROSTEP_PER_DEGREE;

  motor_prepare_segment(robot.arms[0].new_step,
                        robot.arms[1].new_step,
                        robot.arms[2].new_step,
                        fr);
}


void motor_enable() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    digitalWrite(robot.arms[i].motor_enable_pin,LOW);
  }
}


void motor_disable() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    digitalWrite(robot.arms[i].motor_enable_pin,HIGH);
  }
}


void motor_setup() {
  // these values are from the RUMBA specification
  robot.arms[0].motor_step_pin=17;
  robot.arms[0].motor_dir_pin=16;
  robot.arms[0].motor_enable_pin=48;

  robot.arms[1].motor_step_pin=54;
  robot.arms[1].motor_dir_pin=47;
  robot.arms[1].motor_enable_pin=55;

  robot.arms[2].motor_step_pin=57;
  robot.arms[2].motor_dir_pin=56;
  robot.arms[2].motor_enable_pin=62;
  
  for(int i=0;i<NUM_AXIES;++i) {  
    // set the motor pin & scale
    pinMode(robot.arms[i].motor_step_pin,OUTPUT);
    pinMode(robot.arms[i].motor_dir_pin,OUTPUT);
    pinMode(robot.arms[i].motor_enable_pin,OUTPUT);
    // set the limit switches
    robot.arms[i].limit_switch_pin=37-i; 
    robot.arms[i].limit_switch_state=HIGH;
    pinMode(robot.arms[i].limit_switch_pin,INPUT);
    digitalWrite(robot.arms[i].limit_switch_pin,HIGH);
  }
  
  pinMode(13,OUTPUT);
}


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

