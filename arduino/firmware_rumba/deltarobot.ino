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
#include "Vector3.h"
#include "Joint.h"
#include "Arm.h"
#include "DeltaRobot.h"


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
DeltaRobot robot;


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

/**
 * setup the geometry of the robot for faster inverse kinematics later
 */
void deltarobot_setup() {
  Vector3 temp,n;
  int i;

  float frac=TWOPI/(float)NUM_AXIES;
  
  for(i=0;i<NUM_AXIES;++i) {
    Arm &a=robot.arms[i];
    // shoulder
    a.shoulder=Vector3(cos((float)i*frac)*CENTER_TO_SHOULDER,  
                       sin((float)i*frac)*CENTER_TO_SHOULDER,
                       CENTER_TO_FLOOR);
    a.plane_ortho=a.shoulder;
    a.plane_ortho.z=0;
    a.plane_ortho.Normalize();
    
    a.plane_normal=Vector3(-a.plane_ortho.y, a.plane_ortho.x,0);
    a.plane_normal.Normalize();
    
    // elbow
    a.elbow.pos=Vector3(cos((float)i*frac)*(CENTER_TO_SHOULDER+SHOULDER_TO_ELBOW),
                        sin((float)i*frac)*(CENTER_TO_SHOULDER+SHOULDER_TO_ELBOW),
                        CENTER_TO_FLOOR);
    // Find wrist position.
    a.elbow.relative=a.elbow.pos;
    a.wrist.relative=Vector3(cos((float)i*frac)*EFFECTOR_TO_WRIST,
                             sin((float)i*frac)*EFFECTOR_TO_WRIST,
                             0);
    a.last_step=0;
    a.new_step=0;
  }

  // Find wrist height
  robot.ee.x=0;
  robot.ee.y=0;
  float aa = CENTER_TO_SHOULDER + SHOULDER_TO_ELBOW - EFFECTOR_TO_WRIST;
  float cc = ELBOW_TO_WRIST;
  float bb = sqrt(cc*cc - aa*aa);
  robot.ee.z = robot.arms[0].shoulder.z - bb;
  
  robot.current_tool=0;
  
  update_ik();
  segment_setup();
}


/**
 * can the tool reach a given point?
 * @param test the point to test
 * @return 1=out of bounds (fail), 0=in bounds (pass)
 */
char outOfBounds(float x,float y,float z) {
  // test if the move is impossible
  if(z<0) return 1;
  
  int error=0,i;
  Vector3 w, test(x,y,z);
  float len;
  for(i=0;i<NUM_AXIES;++i) {
    Arm &arm=robot.arms[i];
    // get wrist position
    w = test + arm.wrist.relative - arm.shoulder;

    len = w.Length() - ELBOW_TO_WRIST;
    
    if(fabs(len) > SHOULDER_TO_ELBOW) return 1;
  }
  return 0;
}


/**
 * inverse kinematics for each leg.  if you know the wrist, it finds the shoulder angle(s).
 */
void update_ik() {
  update_wrist_positions();
  update_elbows();
  update_shoulder_angles();
}


/**
 * Get wrist position based on end effector position.
 */
void update_wrist_positions() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    Arm &arm=robot.arms[i];

    arm.wrist.pos = robot.ee + arm.wrist.relative;
  }
}


/**
 * Calculate the position of each elbow based on the current location of the wrists.
 */
void update_elbows() {
  float a,b,c,r1,r0,d,h;
  Vector3 r,p1,mid,wop,w,n;
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    Arm &arm=robot.arms[i];

    // get wrist position on plane of bicep
    w = arm.wrist.pos - arm.shoulder;
    
    a = w | arm.plane_normal;  // ee' distance
    wop = w - arm.plane_normal * a;
    arm.wop.pos = wop + arm.shoulder;
    
    // use intersection of circles to find two possible elbow points.
    // the two circles are the bicep (shoulder-elbow) and the forearm (elbow-arm.wop.pos)
    // the distance between circle centers is wop.Length()
    //a = (r0r0 - r1r1 + d*d ) / (2 d) 
    r1=sqrt(ELBOW_TO_WRIST*ELBOW_TO_WRIST-a*a);  // circle 1 centers on wop
    r0=SHOULDER_TO_ELBOW;  // circle 0 centers on shoulder
    d=wop.Length();
    c = ( r0 * r0 - r1 * r1 + d*d ) / ( 2*d );
    // find the midpoint
    n=wop;
    n/=d;
    mid = arm.shoulder+(n*c);
    // with c and r0 we can find h, the distance from midpoint to the intersections.
    h=sqrt(r0*r0-c*c);
    // the distance h on a line orthogonal to n and plane_normal gives us the two intersections.
    r = arm.plane_normal ^ n;
    p1 = mid - r * h;
    //p2 = mid + r * h;
    
    arm.elbow.pos=p1;
  }
}


void update_shoulder_angles() {
  Vector3 temp;
  float x,y,new_angle,nx;
  int i;
  
  for(i=0;i<NUM_AXIES;++i) {
    Arm &arm=robot.arms[i];
    
    // get the angle of each shoulder
    // use atan2 to find theta
    temp = arm.elbow.pos - arm.shoulder;

    y = temp.z;
    temp.z = 0;
    x = temp.Length();
        
    if( ( arm.elbow.relative | temp ) < 0 ) x=-x;

    new_angle=atan2(-y,x) * RAD2DEG;
    // cap the angle
    //if(new_angle>90) new_angle=90;
    //if(new_angle<-90) new_angle=-90;

    // we don't care about elbow angle, but we could find it here if we needed it.

    //Serial.print(i==0?"\tAng=":",");
    //Serial.print(new_angle,2); 
    //if(i==2) Serial.print("\n"); 

    // update servo to match the new IK data
    // 2013-05-17 http://www.marginallyclever.com/forum/viewtopic.php?f=12&t=4707&p=5103#p5091
    nx = ( (reverse==1) ? new_angle : -new_angle );
  /*
    if(nx>MAX_ANGLE) {
      Serial.println("over max");
      nx=MAX_ANGLE;
    }
    if(nx<MIN_ANGLE) {
      Serial.println("under min");
      nx=MIN_ANGLE;
    }
  */
    arm.angle=nx;
  }
}


/**
 * Touch all limit switches and then home the pen holder.  Could also use the Rostock method of touching the bed to model the surface.
 */
void deltarobot_find_home() {
  char i;

  motor_disable();

  // The arms are 24 degrees from straight horizontal when they hit the switch.
  // @TODO: This could be better customized in firmware.
  float horizontal = 24;
  long j, steps_to_zero = MICROSTEPS_PER_TURN * horizontal / 360.00;

  for(i=0;i<NUM_AXIES;++i) {
    // enable one motor at a time
    digitalWrite(robot.arms[i].motor_enable_pin,LOW);
    
    // drive until you hit the switch
    deltarobot_read_switches();
    while(robot.arms[i].limit_switch_state == HIGH) {
      motor_onestep(i,1);
      deltarobot_read_switches();
      pause(500);
    }
    
    // move to home position
    for(j=0;j<steps_to_zero;++j) {
      motor_onestep(i,-1);
      pause(500);
    }
  }

  // recalculate XYZ positions
  deltarobot_setup();
}


/**
 * moving the tool in a straight line
 * @param destination x coordinate
 * @param destination y coordinate
 * @param destination z coordinate
 */
void deltarobot_line(float x, float y, float z,float new_feed_rate) {
  x-=robot.tool_offset[robot.current_tool].x;
  y-=robot.tool_offset[robot.current_tool].y;
  z-=robot.tool_offset[robot.current_tool].z;
  
  if( outOfBounds(x, y, z) ) {
    Serial.println(F("Destination out of bounds."));
    return;
  }
/*
  Serial.print(F("pos="));
  Serial.print(x);
  Serial.print(',');
  Serial.print(y);
  Serial.print(',');
  Serial.print(z);
  Serial.print(',');
  Serial.println(new_feed_rate);
*/ 
  // how long does it take to reach destination at speed feed_rate?
  Vector3 destination(x,y,z);
  Vector3 start = robot.ee;  // keep a copy of start for later in this method
  Vector3 dp = destination - start;  // far do we have to go? 

  // we need some variables in the loop.  Declaring them outside the loop can be more efficient.
  int total_steps = ceil(dp.Length() * (float)SEGMENTS_PER_CM );
  /*
    Serial.print("LENGTH=");
    Serial.println(dp.Length());
    Serial.print("STEP/CM=");
    Serial.println(SEGMENTS_PER_CM);
    Serial.print("Steps=");
    Serial.println(total_steps);
    */
  int i;
  float f;
  // until the interpolation finishes...
  for(i=1;i<=total_steps;++i) {
    // find the point between destination and start that we've reached.
    // this is linear interpolation
    f = (float)i / (float)total_steps;
    robot.ee = dp * f + start;

#if VERBOSE > 0
    Serial.print(i);
    Serial.print('=');
    Serial.print(robot.ee.x);
    Serial.print(',');
    Serial.print(robot.ee.y);
    Serial.print(',');
    Serial.print(robot.ee.z);
#endif
    //deltarobot_where();

    // update the inverse kinematics
    update_ik();
/*
    Serial.print(i);
    Serial.print('=');
    Serial.print(robot.arms[0].angle);
    Serial.print(',');
    Serial.print(robot.arms[1].angle);
    Serial.print(',');
    Serial.println(robot.arms[2].angle);
*/
    motor_segment(robot.arms[0].angle,
                  robot.arms[1].angle,
                  robot.arms[2].angle,
                  new_feed_rate);
  }
}





//------------------------------------------------------------------------------
// This method assumes the limits have already been checked.
// This method assumes the start and end radius match.
// This method assumes arcs are not >180 degrees (PI radians)
// cx/cy - center of circle
// x/y - end position
// dir - ARC_CW or ARC_CCW to control direction of arc
void deltarobot_arc(float cx,float cy,float x,float y,float z,float dir,float new_feed_rate) {
  Vector3 offset_pos=deltarobot_get_end_plus_offset();
  
  // get radius
  float dx = offset_pos.x - cx;
  float dy = offset_pos.y - cy;
  float radius=sqrt(dx*dx+dy*dy);

  // find angle of arc (sweep)
  float angle1=atan3(dy,dx);
  float angle2=atan3(y-cy,x-cx);
  float theta=angle2-angle1;
  
  if(dir>0 && theta<0) angle2+=2*PI;
  else if(dir<0 && theta>0) angle1+=2*PI;
  
  theta=angle2-angle1;
  
  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len = abs(theta) * radius;

  int i, segments = floor( len * SEGMENTS_PER_CM );
 
  float nx, ny, nz, angle3, scale;

  for(i=0;i<segments;++i) {
    // interpolate around the arc
    scale = ((float)i)/((float)segments);
    
    angle3 = ( theta * scale ) + angle1;
    nx = cx + cos(angle3) * radius;
    ny = cy + sin(angle3) * radius;
    nz = ( z - offset_pos.z ) * scale + offset_pos.z;
    // send it to the planner
    deltarobot_line(nx,ny,nz,new_feed_rate);
  }
  
  deltarobot_line(x,y,z,new_feed_rate);
}


/**
 * Set the logical position
 * @input npx new position x
 * @input npy new position y
 */
void deltarobot_position(float npx,float npy,float npz) {
  // here is a good place to add sanity tests
  
  robot.ee = Vector3(npx,npy,npz)-robot.tool_offset[robot.current_tool];
  
  update_ik();
  
  robot.arms[0].last_step = robot.arms[0].angle * MICROSTEP_PER_DEGREE;
  robot.arms[1].last_step = robot.arms[1].angle * MICROSTEP_PER_DEGREE;
  robot.arms[2].last_step = robot.arms[2].angle * MICROSTEP_PER_DEGREE;
  
  outputsteps();
}


/**
 * print the current position, feedrate, and absolute mode.
 */
void deltarobot_where() {
  Vector3 offset_pos=deltarobot_get_end_plus_offset() ;
  output("X",offset_pos.x);
  output("Y",offset_pos.y);
  output("Z",offset_pos.z);
  output("F",feed_rate);
  Serial.println(mode_abs?"ABS":"REL");
  outputsteps();
} 


/**
 * read the limit switch states
 * @return 1 if a switch is being hit
 */
char deltarobot_read_switches() {
  char i, hit=0;
  int state;
  
  for(i=0;i<NUM_AXIES;++i) {
    state=digitalRead(robot.arms[i].limit_switch_pin);
#ifdef DEBUG_SWITCHES
    Serial.print(state);
    Serial.print('\t');
#endif
    if(robot.arms[i].limit_switch_state != state) {
      robot.arms[i].limit_switch_state = state;
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


void deltarobot_tool_offset(int axis,float x,float y,float z) {
  robot.tool_offset[axis].x=x;
  robot.tool_offset[axis].y=y;
  robot.tool_offset[axis].z=z;
}

Vector3 deltarobot_get_end_plus_offset() {
  return Vector3(robot.tool_offset[robot.current_tool].x + robot.ee.x,
                 robot.tool_offset[robot.current_tool].y + robot.ee.y,
                 robot.tool_offset[robot.current_tool].z + robot.ee.z);
}


void deltarobot_tool_change(int tool_id) {
  if(tool_id < 0) tool_id=0;
  if(tool_id > NUM_TOOLS) tool_id=NUM_TOOLS;
  robot.current_tool=tool_id;
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

