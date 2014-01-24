//------------------------------------------------------------------------------
// Delta Robot v8 - Supports RUMBA controller
// dan@marginallycelver.com 2014-01-07
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/DeltaRobotv8 for more information.

/**
 * top center to wrist hole: X76.35 Y+/-5.53 Z8.7
 * base center to shoulder hole: X80.93 Y+/-21.5 Z78.31
 */

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
//#define VERBOSE              (1)  // add to get a lot more serial output.
//#define DEBUG_SWITCHES       (1)

#define VERSION              (1)  // firmware version
#define BAUD                 (57600)  // How fast is the Arduino talking?
#define MAX_BUF              (64)  // What is the longest message Arduino can store?
#define STEPS_PER_TURN       (400)  // depends on your stepper motor.  most are 200.
#define MIN_STEP_DELAY       (100)
#define MAX_FEEDRATE         (1000000/MIN_STEP_DELAY)
#define MIN_FEEDRATE         (0.01)
#define NUM_AXIES            (3)
#define MICROSTEP_MUL        (16.0)

// measurements based on computer model of robot
#define BICEP_LENGTH         (5.01)
#define FOREARM_LENGTH       (16.75)
#define CIRCUMFERENCE        (BICEP_LENGTH*PI*2.0)
#define STEP_DISTANCE        ((CIRCUMFERENCE/MICROSTEP_MUL)/STEPS_PER_TURN)


//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// STRUCTS
//------------------------------------------------------------------------------
// for line()
typedef struct {
  long delta;
  long absdelta;
  int dir;
  long over;
} Axis;


typedef struct {
  int motor_step_pin;
  int motor_dir_pin;
  int motor_enable_pin;
  int limit_switch_pin;
  int limit_switch_state;  
} Arm;


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
Arm arms[NUM_AXIES];

Axis a[NUM_AXIES];  // for line()
Axis atemp;  // for line()

char buffer[MAX_BUF];  // where we store the message until we get a ';'
int sofar;  // how much is in the buffer

// speeds
float fr=0;  // human version
long step_delay;  // machine version
int step_delay_s;
int step_delay_us;

float px,py,pz;  // position

// settings
char mode_abs=1;  // absolute mode?

#ifdef VERBOSE
char *letter="XYZ";
#endif


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------


/**
 * delay for the appropriate number of microseconds
 * @input ms how many milliseconds to wait
 */
void pause(long ms) {
  delay( ms/1000 );
  delayMicroseconds( ms%1000 );  // delayMicroseconds doesn't work for ms values > ~16k.
}


/**
 * delay for the appropriate number of microseconds
 * @input ms how many milliseconds to wait
 */
void pause_step() {
  delay( step_delay_s );
  delayMicroseconds( step_delay_us );  // delayMicroseconds doesn't work for ms values > ~16k.
}


/**
 * Set the feedrate (speed motors will move)
 * @input nfr the new speed in steps/second
 */
void feedrate(float nfr) {
  if(fr==nfr) return;  // same as last time?  quit now.

  if(nfr>MAX_FEEDRATE) {
    Serial.print(F("New feedrate must be less than "));
    Serial.print(MAX_FEEDRATE);
    Serial.println(F("steps/s."));
    nfr=MAX_FEEDRATE;
  }
  if(nfr<MIN_FEEDRATE) {  // don't allow crazy feed rates
    Serial.print(F("New feedrate must be greater than "));
    Serial.print(MIN_FEEDRATE);
    Serial.println(F("steps/s."));
    nfr=MIN_FEEDRATE;
  }
  step_delay = 1000000.0/nfr;
  step_delay_s  = step_delay / 1000;
  step_delay_us = step_delay % 1000;
  fr=nfr;
}


/**
 * Set the logical position
 * @input npx new position x
 * @input npy new position y
 */
void position(float npx,float npy,float npz) {
  // here is a good place to add sanity tests
  px=npx;
  py=npy;
  pz=npz;
}


/**
 * Move one motor in a given direction
 * @input the motor number [0...6]
 * @input the direction to move 1 for forward, -1 for backward
 **/
void onestep(int motor,int dir) {
#ifdef VERBOSE
  Serial.print(letter[motor]);
#endif
  digitalWrite(arms[motor].motor_dir_pin,dir<0?LOW:HIGH);
  digitalWrite(arms[motor].motor_step_pin,LOW);
  digitalWrite(arms[motor].motor_step_pin,HIGH);
}


/**
 * Grips the power on the motors
 **/
void motor_enable() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    digitalWrite(arms[i].motor_enable_pin,LOW);
  }
}


/**
 * Releases the power on the motors
 **/
void motor_disable() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    digitalWrite(arms[i].motor_enable_pin,HIGH);
  }
}


/**
 * Uses bresenham's line algorithm to move both motors
 * @input newx the destination x position
 * @input newy the destination y position
 **/
void line(float newx,float newy,float newz) {
  a[0].delta = newx-px;
  a[1].delta = newy-py;
  a[2].delta = newz-pz;
  
  long i,j,maxsteps=0;

  for(i=0;i<NUM_AXIES;++i) {
    a[i].dir = (a[i].delta > 0 ? 1:-1);
    a[i].absdelta = abs(a[i].delta);
    a[i].over=0;
    if(maxsteps<a[i].absdelta) maxsteps=a[i].absdelta;
  }
  
#ifdef VERBOSE
  Serial.println(F("Start >"));
#endif
  
  for(i=0;i<maxsteps;++i) {
    for(j=1;j<NUM_AXIES;++j) {
      a[j].over += a[j].absdelta;
      if(a[j].over >= maxsteps) {
        a[j].over -= maxsteps;
        onestep(j,a[j].dir);
      }
    }
    pause_step();
  }

#ifdef VERBOSE
  Serial.println(F("< Done."));
#endif

  // This does not take into account movements of a fraction of a step.  They will be misreported and lead to error.
  position(newx,newy,newz);
}


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
  Serial.println(val);
}


/**
 * print the current position, feedrate, and absolute mode.
 */
void where() {
  output("X",px);
  output("Y",py);
  output("Z",pz);
  output("F",fr);
  Serial.println(mode_abs?"ABS":"REL");
} 


/**
 * display helpful information
 */
void help() {
  Serial.print(F("Delta Robot v8-"));
  Serial.println(VERSION);
  Serial.println(F("Commands:"));
  Serial.println(F("M17/M18; - enable/disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
  Serial.println(F("F, G00, G01, G04, G17, G18, G28, G90, G91, G92 as described by http://en.wikipedia.org/wiki/G-code"));
}


/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */
void processCommand() {
  int cmd = parsenumber('G',-1);
  switch(cmd) {
  case  0:
  case  1: // line
    feedrate(parsenumber('F',fr));
    line( parsenumber('X',(mode_abs?px:0)) + (mode_abs?0:px),
          parsenumber('Y',(mode_abs?py:0)) + (mode_abs?0:py),
          parsenumber('Z',(mode_abs?pz:0)) + (mode_abs?0:pz) );
    break;
  case  4:  pause(parsenumber('P',0)*1000);  break;  // dwell
  case 28:  find_home();  break;  
  case 90:  mode_abs=1;  break;  // absolute mode
  case 91:  mode_abs=0;  break;  // relative mode
  case 92:  // set logical position
    position( parsenumber('X',0),
              parsenumber('Y',0),
              parsenumber('Z',0) );
    break;
  default:  break;
  }

  cmd = parsenumber('M',-1);
  switch(cmd) {
  case 17:  motor_enable();  break;
  case 18:  motor_disable();  break;
  case 100:  help();  break;
  case 114:  where();  break;
  default:  break;
  }
}


/**
 * prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
 */
void ready() {
  sofar=0;  // clear input buffer
  Serial.print(F(">"));  // signal ready to receive input
}


/**
 * read the limit switch states
 * @return 1 if a switch is being hit
 */
char read_switches() {
  char i, hit=0;
  int state;
  
  for(i=0;i<NUM_AXIES;++i) {
    state=digitalRead(arms[i].limit_switch_pin);
#ifdef DEBUG_SWITCHES
    Serial.print(state);
    Serial.print('\t');
#endif
    if(arms[i].limit_switch_state != state) {
      arms[i].limit_switch_state = state;
#ifdef DEBUG_SWITCHES
      Serial.print(F("Switch "));
      Serial.println(i,DEC);
#endif
    }
    if(state == LOW) ++hit;
  }
#ifdef DEBUG_SWITCHES
  Serial.print('\n');
#endif
  return hit;
}


/**
 * setup the limit switches
 */
void setup_switches() {
  for(int i=0;i<NUM_AXIES;++i) { 
    arms[i].limit_switch_pin=37-i; 
    arms[i].limit_switch_state=HIGH;
    pinMode(arms[i].limit_switch_pin,INPUT);
    digitalWrite(arms[i].limit_switch_pin,HIGH);
  }
}


/**
 * Move the motors until they connect with the limit switches, then return to "zero" position.
 */
void find_home() {
  char i;

  motor_disable();

  // The arms are 24 degrees from straight sideways when they hit the switch.
  // @TODO: This could be better customized in firmware.
  float horizontal = 24;
  long steps_to_zero = STEPS_PER_TURN * MICROSTEP_MUL * horizontal / 360.00;
#ifdef VERBOSE
  Serial.print("steps=");
  Serial.println(step_size);
#endif

  for(i=0;i<NUM_AXIES;++i) {
    // enable one motor at a time
    digitalWrite(arms[i].motor_enable_pin,LOW);
    
    // drive until you hit the switch
    read_switches();
    while(arms[i].limit_switch_state == HIGH) {
      onestep(i,1);
      read_switches();
      pause(1250);
    }
    
    // move to home position
    for(long j=0;j<steps_to_zero;++j) {
      onestep(i,-1);
      pause(1250);
    }
  }
  

  position(0,0,0);
}


/**
 * setup the motor shields
 */
void setup_shields() {
  arms[0].motor_step_pin=17;
  arms[0].motor_dir_pin=16;
  arms[0].motor_enable_pin=48;

  arms[1].motor_step_pin=54;
  arms[1].motor_dir_pin=47;
  arms[1].motor_enable_pin=55;

  arms[2].motor_step_pin=57;
  arms[2].motor_dir_pin=56;
  arms[2].motor_enable_pin=62;
  
  for(int i=0;i<NUM_AXIES;++i) {  
    // set the motor pin & scale
    pinMode(arms[i].motor_step_pin,OUTPUT);
    pinMode(arms[i].motor_dir_pin,OUTPUT);
    pinMode(arms[i].motor_enable_pin,OUTPUT);
  }
}


/**
 * First thing this machine does on startup.  Runs only once.
 */
void setup() {
  Serial.begin(BAUD);  // open coms
  help();  // say hello

  setup_shields();
  setup_switches();
  feedrate(6400);  // set default speed
  motor_enable();
  position(0,0,0);
  
  ready();
}


/**
 * After setup() this machine will repeat loop() forever.
 */
void loop() {
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
    processCommand();  // do something with the command
    ready();
  }

#ifdef DEBUG_SWITCHES  
  read_switches();
#endif
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
