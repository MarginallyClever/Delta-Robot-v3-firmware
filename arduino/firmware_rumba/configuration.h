#ifndef CONFIGURATION_H
#define CONFIGURATION_H
//------------------------------------------------------------------------------
// Delta Robot v8 - Supports RUMBA 6-axis motor controller
// dan@marginallycelver.com 2014-01-07
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/DeltaRobotv8 for more information.


//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
#define VERBOSE              (0)  // increasing number increases output

#define EEPROM_VERSION       (1)  // firmware version

#define BAUD                 (57600)  // How fast is the Arduino talking?
#define MAX_BUF              (64)  // What is the longest message Arduino can store?

#define STEPS_PER_TURN       (400.0)  // default number of steps per turn * microsteps
#define MICROSTEPS           (16.0)  // microstepping on this microcontroller

#define MAX_FEEDRATE         (30000.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (1000.0)
#define DEFAULT_FEEDRATE     (2500.0)
#define DEFAULT_ACCELERATION (250.0)

#define MAX_ANGLE            (90+85)
#define MIN_ANGLE            (90-30)

// split long lines into pieces to make them more correct.
#define MM_PER_SEGMENT       (10)
#define NUM_TOOLS            (6)
// related to number of instructions that can be buffered.  must be a power of two > 1.
#define MAX_SEGMENTS         (32)

// turn this on if you need the robot to NOT buffer commands
//#define ONE_COMMAND_AT_A_TIME  (1)

#define TWOPI                (PI*2.0)
#define DEG2RAD              (PI/180.0)
#define RAD2DEG              (180.0/PI)

#define MICROSTEPS_PER_TURN  (STEPS_PER_TURN*MICROSTEPS)
#define MICROSTEP_PER_DEGREE (MICROSTEPS_PER_TURN/360.0)


//------------------------------------------------------------------------------
// TIMERS
//------------------------------------------------------------------------------
// time passed with no instruction?  Make sure PC knows we are waiting.
#define TIMEOUT_OK           (1000)
// timer stuff
#define CLOCK_FREQ           (16000000L)
#define MAX_COUNTER          (65536L)
// optimize code, please
#define FORCE_INLINE         __attribute__((always_inline)) inline


#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG;  cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START



// arduino pins for motor control
#define MOTHERBOARD 1  // RUMBA
//#define MOTHERBOARD 2  // RAMPS 1.4

#if MOTHERBOARD == 1  // RUMBA
#define NUM_AXIES          (3)  // can go up to 6


#define MOTOR_0_DIR_PIN    (16)
#define MOTOR_0_STEP_PIN   (17)
#define MOTOR_0_ENABLE_PIN (48)
#define MOTOR_0_LIMIT_PIN  (37)

#define MOTOR_1_DIR_PIN    (47)
#define MOTOR_1_STEP_PIN   (54)
#define MOTOR_1_ENABLE_PIN (55)
#define MOTOR_1_LIMIT_PIN  (36)

#define MOTOR_2_DIR_PIN    (56)
#define MOTOR_2_STEP_PIN   (57)
#define MOTOR_2_ENABLE_PIN (62)
#define MOTOR_2_LIMIT_PIN  (35)

#define MOTOR_3_DIR_PIN    (22)
#define MOTOR_3_STEP_PIN   (23)
#define MOTOR_3_ENABLE_PIN (27)
#define MOTOR_3_LIMIT_PIN  (34)

#define MOTOR_4_DIR_PIN    (25)
#define MOTOR_4_STEP_PIN   (26)
#define MOTOR_4_ENABLE_PIN (24)
#define MOTOR_4_LIMIT_PIN  (33)

#define MOTOR_5_DIR_PIN    (28)
#define MOTOR_5_STEP_PIN   (29)
#define MOTOR_5_ENABLE_PIN (39)
#define MOTOR_5_LIMIT_PIN  (32)
#endif

#if MOTHERBOARD == 2  // RAMPS 1.4
#define NUM_AXIES          (3)  // can go up to 4.

#define MOTOR_0_DIR_PIN    (55)
#define MOTOR_0_STEP_PIN   (54)
#define MOTOR_0_ENABLE_PIN (38)
#define MOTOR_0_LIMIT_PIN  (3)

#define MOTOR_1_DIR_PIN    (61)
#define MOTOR_1_STEP_PIN   (60)
#define MOTOR_1_ENABLE_PIN (56)
#define MOTOR_1_LIMIT_PIN  (14)

#define MOTOR_2_DIR_PIN    (48)
#define MOTOR_2_STEP_PIN   (46)
#define MOTOR_2_ENABLE_PIN (62)
#define MOTOR_2_LIMIT_PIN  (18)

#define MOTOR_3_DIR_PIN    (28)
#define MOTOR_3_STEP_PIN   (26)
#define MOTOR_3_ENABLE_PIN (24)
#endif

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

