//------------------------------------------------------------------------------
// Delta Robot v8 - Supports RUMBA 6-axis motor controller
// dan@marginallycelver.com 2014-01-07
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/DeltaRobotv8 for more information.


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


/**
 * print the current motor positions in steps
 */
void motor_where() {
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
  robot.arms[0].motor_step_pin   = MOTOR_0_STEP_PIN;
  robot.arms[0].motor_dir_pin    = MOTOR_0_DIR_PIN;
  robot.arms[0].motor_enable_pin = MOTOR_0_ENABLE_PIN;
  robot.arms[0].limit_switch_pin = MOTOR_0_LIMIT_PIN;

  robot.arms[1].motor_step_pin   = MOTOR_1_STEP_PIN;
  robot.arms[1].motor_dir_pin    = MOTOR_1_DIR_PIN;
  robot.arms[1].motor_enable_pin = MOTOR_1_ENABLE_PIN;
  robot.arms[1].limit_switch_pin = MOTOR_1_LIMIT_PIN;

  robot.arms[2].motor_step_pin   = MOTOR_2_STEP_PIN;
  robot.arms[2].motor_dir_pin    = MOTOR_2_DIR_PIN;
  robot.arms[2].motor_enable_pin = MOTOR_2_ENABLE_PIN;
  robot.arms[2].limit_switch_pin = MOTOR_2_LIMIT_PIN;
  
  for(int i=0;i<NUM_AXIES;++i) {
    // set the motor pin & scale
    pinMode(robot.arms[i].motor_step_pin,OUTPUT);
    pinMode(robot.arms[i].motor_dir_pin,OUTPUT);
    pinMode(robot.arms[i].motor_enable_pin,OUTPUT);
    // set the limit switches
    robot.arms[i].limit_switch_state=HIGH;
    pinMode(robot.arms[i].limit_switch_pin,INPUT);
    digitalWrite(robot.arms[i].limit_switch_pin,HIGH);
  }
}


void motor_set_step_count(long a0,long a1,long a2) {  
  if( current_segment==last_segment ) {
    Segment &old_seg = line_segments[get_prev_segment(last_segment)];
    old_seg.a[0].step_count=a0;
    old_seg.a[1].step_count=a1;
    old_seg.a[2].step_count=a2;
/*
    laststep[0]=a0;
    laststep[1]=a1;
    laststep[2]=a2;
*/
  }
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

