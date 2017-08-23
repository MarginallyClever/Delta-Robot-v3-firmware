//------------------------------------------------------------------------------
// Delta Robot v8 - Supports RUMBA 6-axis motor controller
// dan@marginallycelver.com 2014-01-07
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/DeltaRobotv8 for more information.


//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include "vector3.h"
#include "configuration.h"
#include "segment.h"


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
float feed_rate=DEFAULT_FEEDRATE;  // how fast the EE moves in cm/s
float acceleration=DEFAULT_ACCELERATION;

// did you put it together backwards?
int reverse=0;

// absolute or relative movements?
int mode_abs=1;

long robot_uid=0;


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
  Serial.print(F("\n\nHELLO WORLD! I AM DELTA ROBOT V3-"));
  Serial.print(EEPROM_VERSION);
  Serial.print(F(" #"));
  Serial.println(robot_uid);
  Serial.println(F("Please see http://www.marginallyclever.com/ for more information."));
  Serial.println(F("Commands:"));
  Serial.println(F("M17/M18; - enable/disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
  Serial.println(F("F, G00, G01, G04, G17, G18, G28, G54-G59, G90, G91, G92, M06 as described by http://en.wikipedia.org/wiki/G-code"));
}


/**
 * runs once when board turns on/resets.  Initializes variables.
 */
void setup() {
  loadConfig();
  
  Serial.begin(BAUD);  // open coms

  motor_setup();
  segment_setup();

  deltarobot_setup();
  feedrate(DEFAULT_FEEDRATE);  // set default speed
  
  help();  // say hello
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

