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

#define VERSION              (1)  // firmware version
#define BAUD                 (57600)  // How fast is the Arduino talking?
#define MAX_BUF              (64)  // What is the longest message Arduino can store?

#define STEPS_PER_TURN       (400)  // depends on your stepper motor.  most are 200.
#define MICROSTEPS           (16.0)
#define MAX_FEEDRATE         (16000.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (1)
#define DEFAULT_FEEDRATE     (4000)
#define ACCELERATION         (200)

#define MAX_ANGLE            (90+85)
#define MIN_ANGLE            (90-30)

// related to number of instructions that can be buffered.  must be a power of two > 1.
#define MAX_SEGMENTS         (32)

#define SEGMENTS_PER_CM      (2.0)
#define SEGMENTS_PER_DEG     (5.0)

// turn this on if you need the robot to NOT buffer commands
//#define ONE_COMMAND_AT_A_TIME  (1)

// ** Nothing below this line needs to be configured **

#define NUM_AXIES            (3)
#define NUM_TOOLS            (6)

#define TWOPI                (PI*2.0)
#define DEG2RAD              (PI/180.0)
#define RAD2DEG              (180.0/PI)

#define MICROSTEPS_PER_TURN  (STEPS_PER_TURN*MICROSTEPS)
#define CIRCUMFERENCE        (BICEP_LENGTH*TWOPI)
#define MICROSTEP_DISTANCE   (CIRCUMFERENCE/MICROSTEPS_PER_TURN)  // distance elbow moves in a single microstep
#define MICROSTEP_PER_DEGREE (MICROSTEPS_PER_TURN/360.0)



#define SHOULDER_TO_ELBOW    (5)  // cm
#define ELBOW_TO_WRIST       (16.5f)  // cm

//#define CENTER_TO_SHOULDER   (5.248f)  // cm
#define CENTER_TO_SHOULDER   (3.77f)  // cm
#define EFFECTOR_TO_WRIST    (1.724f)  // cm

#define CENTER_TO_FLOOR    (18.9)  // cm


// timer stuff
#define CLOCK_FREQ           (16000000L)
#define MAX_COUNTER          (65536L)


// optimize code, please
#define FORCE_INLINE         __attribute__((always_inline)) inline


#define MOTHERBOARD 1 // RUMBA
//#define MOTHERBOARD 2 // RAMPS

#if MOTHERBOARD == 1
#define MOTOR_0_DIR_PIN    (16)
#define MOTOR_0_STEP_PIN   (17)
#define MOTOR_0_ENABLE_PIN (48)

#define MOTOR_1_DIR_PIN    (47)
#define MOTOR_1_STEP_PIN   (54)
#define MOTOR_1_ENABLE_PIN (55)

#define MOTOR_2_DIR_PIN    (56)
#define MOTOR_2_STEP_PIN   (57)
#define MOTOR_2_ENABLE_PIN (62)
#endif

#if MOTHERBOARD == 2
#endif


#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG;  cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START

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

