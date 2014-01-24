//------------------------------------------------------------------------------
// Delta robot w/ stepper motors & adafruit motor shield v2
// dan@marginallycelver.com 2013-11-25
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/i-make-robots/Delta-Robot for more information.


//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include "configuration.h"
#include "Vector3.h"
#include "Joint.h"
#include "Arm.h"
#include "DeltaRobot.h"


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
float feed_rate=10;  // how fast the EE moves in cm/s

// did you put it together backwards?
// @TODO: store this in EEPROM
int reverse=0;

// absolute or relative movements?
int mode_abs=1;


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------
/**
  * finds angle of dy/dx as a value from 0...2PI
  * @return the angle
  */
float atan3(float dy,float dx) {
  float a=atan2(dy,dx);
  if(a<0) a=(PI*2.0)+a;
  return a;
}


/**
 * delay for the appropriate number of microseconds
 * @input ms how many milliseconds to wait
 */
void pause(long ms) {
  delay(ms/1000);
  delayMicroseconds(ms%1000);  // delayMicroseconds doesn't work for values > ~16k.
}


/**
 * Set the feedrate (speed motors will move)
 * @input nfr the new speed in steps/second
 */
float feedrate(float nfr) {
  if(feed_rate==nfr) return nfr;  // same as last time?  quit now.

  if(nfr>MAX_FEEDRATE) {
    Serial.print(F("Feedrate set to maximum ("));
    Serial.print(MAX_FEEDRATE);
    Serial.println(F("steps/s)"));
    nfr=MAX_FEEDRATE;
  }
  if(nfr<MIN_FEEDRATE) {  // don't allow crazy feed rates
    Serial.print(F("Feedrate set to minimum ("));
    Serial.print(MIN_FEEDRATE);
    Serial.println(F("steps/s)"));
    nfr=MIN_FEEDRATE;
  }
  feed_rate=nfr;
  
  return feed_rate;
}


/**
 * display helpful information
 */
void help() {
  Serial.print(F("Delta Robot v8-"));
  Serial.println(VERSION);
  Serial.println(F("http://github.com/i-make-robots/Delta-Robot/"));
  Serial.println(F("Commands:"));
  Serial.println(F("G00/G01 [X(steps)] [Y(steps)] [Z(steps)] [E(steps)] [F(feedrate)]; - linear move"));
  Serial.println(F("G04 P[seconds]; - delay"));
  Serial.println(F("G90; - absolute mode"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X(steps)] [Y(steps)] [Z(steps)]; - change logical position"));
  Serial.println(F("M18; - disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
}


/**
 * runs once when board turns on/resets.  Initializes variables.
 */
void setup() {
  Serial.begin(BAUD);  // open coms

  motor_setup();

  setup_robot();
  // @TODO: Is this necessary?
  deltarobot_position(0,0,0);
  
  help();  // say hello
  feedrate(200);  // set default speed
  parser_ready();
}


/**
 * runs over and over.  if loop() finishes then the board calls it again. globals do not get reset if loop() ends.
 * listen for instructions coming from the serial connection.
 * See: http://www.marginallyclever.com/2011/10/controlling-your-arduino-through-the-serial-monitor/
 */
void loop() {
  parser_listen();
}


/**
* This file is part of Delta-Robot.
*
* Delta-Robot is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Delta-Robot is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Delta-Robot. If not, see <http://www.gnu.org/licenses/>.
*/

