//------------------------------------------------------------------------------
// Delta Robot v8 - Supports RUMBA 6-axis motor controller
// dan@marginallycelver.com 2014-01-07
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/DeltaRobotv8 for more information.


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
static char buffer[MAX_BUF];  // where we store the message until we get a ';'
static int sofar;  // how much is in the buffer


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------
/**
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
float parsenumber(char code,float val) {
  char *ptr=buffer;
  while(ptr && *ptr && ptr<buffer+sofar) {
    if(*ptr==code) {
      return atof(ptr+1);
    }
    ptr=strchr(ptr,' ')+1;
  }
  return val;
} 


/**
 * write a string followed by a float to the serial line.  Convenient for debugging.
 * @input code the string.
 * @input val the float.
 */
void output(char *code,float val) {
  Serial.print(code);
  Serial.print(F("="));
  Serial.println(val);
}


void outputvector(Vector3 &v,char*name) {
  Serial.print(name);
  Serial.print(F("="));
  Serial.print(v.x);
  Serial.print(F(","));
  Serial.print(v.y);
  Serial.print(F(","));
  Serial.println(v.z);
}


/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */
void parser_processCommand() {
  int cmd = parsenumber('G',-1);
  switch(cmd) {
  case  0:
  case  1: {  // move in a line
      Vector3 offset=deltarobot_get_end_plus_offset();
      deltarobot_line( parsenumber('X',(mode_abs?offset.x:0)) + (mode_abs?0:offset.x),
                       parsenumber('Y',(mode_abs?offset.y:0)) + (mode_abs?0:offset.y),
                       parsenumber('Z',(mode_abs?offset.z:0)) + (mode_abs?0:offset.z),
                       feedrate(parsenumber('F',feed_rate)) );
    break;
  }
  case  4:  pause(parsenumber('P',0)*1000);  break;  // dwell
  case 28:  deltarobot_find_home();  break;
  case 54:
  case 55:
  case 56:
  case 57:
  case 58:
  case 59: {  // 54-59 tool offsets
    int axis=cmd-54;
    deltarobot_tool_offset(axis,parsenumber('X',robot.tool_offset[axis].x),
                                parsenumber('Y',robot.tool_offset[axis].y),
                                parsenumber('Z',robot.tool_offset[axis].z));
    break;
  }
  case 90:  mode_abs=1;  break;  // absolute mode
  case 91:  mode_abs=0;  break;  // relative mode

  // See hexapod_position() for note about why G92 is removed
  case 92: { // set logical position
    Vector3 offset = deltarobot_get_end_plus_offset();
    deltarobot_position( parsenumber('X',offset.x),
                         parsenumber('Y',offset.y),
                         parsenumber('Z',offset.z) );
    }
    break;
  default:  break;
  }

  cmd = parsenumber('M',-1);
  switch(cmd) {
  case 6:
    deltarobot_tool_change(parsenumber('T',0));
    break;
  case 17:  motor_enable();  break;
  case 18:  motor_disable();  break;
  case 100:  help();  break;
  case 114:  deltarobot_where();  break;
  default:  break;
  }
}


/**
 * Prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
 */
void parser_ready() {
  sofar=0;  // clear input buffer
  Serial.print(F(">"));  // signal ready to receive input
}


/**
 * Listen to the serial port for incoming commands and deal with them
 */
void parser_listen() {
  // listen for serial commands
  while(Serial.available() > 0) {  // if something is available
    char c=Serial.read();  // get it
    Serial.print(c);  // repeat it back so I know you got the message
    if(sofar<MAX_BUF) buffer[sofar++]=c;  // store it
    if(buffer[sofar-1]==';') break;  // entire message received
  }

  if(sofar>0 && buffer[sofar-1]==';') {
    // we got a message and it ends with a semicolon
    buffer[sofar]=0;  // end the buffer so string functions work right
    Serial.print(F("\r\n"));  // echo a return character for humans
    parser_processCommand();  // do something with the command

#ifdef ONE_COMMAND_AT_A_TIME
    while( current_segment != last_segment );
#endif

    parser_ready();
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
