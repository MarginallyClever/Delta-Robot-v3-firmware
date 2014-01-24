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
#include "segment.h"


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
// A ring buffer of line segments.
// @TODO: process the line segments in another thread
// @TODO: optimize speed between line segments
Segment line_segments[MAX_SEGMENTS];
volatile int current_segment=0;
volatile int last_segment=0;


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------
int get_next_segment(int i) {
  return ( i + 1 ) % MAX_SEGMENTS;
}


int get_prev_segment(int i) {
  return ( i + MAX_SEGMENTS - 1 ) % MAX_SEGMENTS;
}


/**
 * Add a segment to the line buffer if there is room.
 */
void motor_prepare_segment(int n0,int n1,int n2,float new_feed_rate) {
  int next_segment = get_next_segment(last_segment);
  while( next_segment == current_segment ) {
    // the buffer is full, we are way ahead of the motion system
    delay(1);
  }

  Segment &new_seg = line_segments[last_segment];
  new_seg.a[0].step_count = n0;
  new_seg.a[1].step_count = n1;
  new_seg.a[2].step_count = n2;
  
  int i;

  Segment &old_seg = line_segments[get_prev_segment(last_segment)];
  new_seg.a[0].delta = n0 - old_seg.a[0].step_count;
  new_seg.a[1].delta = n1 - old_seg.a[1].step_count;
  new_seg.a[2].delta = n2 - old_seg.a[2].step_count;

  new_seg.steps=0;
  new_seg.feed_rate=new_feed_rate;

  for(i=0;i<NUM_AXIES;++i) {
    new_seg.a[i].over = 0;
    new_seg.a[i].dir = (new_seg.a[i].delta < 0 ? LOW:HIGH);
    new_seg.a[i].absdelta = abs(new_seg.a[i].delta);
    if( new_seg.steps < new_seg.a[i].absdelta ) {
      new_seg.steps = new_seg.a[i].absdelta;
    }
  }

  if( new_seg.steps==0 ) return;

  new_seg.steps_left = new_seg.steps;
  
#ifdef VERBOSE
  Serial.print(F("At "));  Serial.println(current_segment);
  Serial.print(F("Adding "));  Serial.println(last_segment);
  Serial.print(F("Steps= "));  Serial.println(new_seg.steps_left);
#endif

  if( current_segment==last_segment ) {
    timer_set_frequency(new_feed_rate);
  }
  
  last_segment = next_segment;
}


/**
 * Set the clock 1 timer frequency.
 * @input desired_freq_hz the desired frequency
 */
void timer_set_frequency(long desired_freq_hz) {
  // Source: https://github.com/MarginallyClever/ArduinoTimerInterrupt
  // Different clock sources can be selected for each timer independently. 
  // To calculate the timer frequency (for example 2Hz using timer1) you will need:
  
  //  CPU frequency 16Mhz for Arduino
  //  maximum timer counter value (256 for 8bit, 65536 for 16bit timer)
  int prescaler_index=-1;
  long prescalers[] = {CLOCK_FREQ /   1,
                       CLOCK_FREQ /   8,
                       CLOCK_FREQ /  64,
                       CLOCK_FREQ / 256,
                       CLOCK_FREQ /1024};
  long counter_value;
  do {
    ++prescaler_index;
    //  Divide CPU frequency through the choosen prescaler (16000000 / 256 = 62500)
    //  Divide result through the desired frequency (62500 / 2Hz = 31250)
    counter_value = prescalers[prescaler_index] / desired_freq_hz;
    //  Verify counter_value < maximum timer. if fail, choose bigger prescaler.
  } while(counter_value > MAX_COUNTER && prescaler_index<4);
  
  if( prescaler_index>=5 ) {
    Serial.println(F("Timer could not be set: Desired frequency out of bounds."));
    return;
  }
  
//  Serial.print("freq=");
//  Serial.print(desired_freq_hz);
  
  prescaler_index++;

  // disable global interrupts
  noInterrupts();
  // set entire TCCR1A register to 0
  TCCR1A = 0;
  // set entire TCCR1B register to 0
  TCCR1B = 0;
  // set the overflow clock to 0
  TCNT1  = 0;
  // set compare match register to desired timer count
  OCR1A = counter_value;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10, CS11, and CS12 bits for prescaler
  TCCR1B |= ( (( prescaler_index&0x1 )   ) << CS10);
  TCCR1B |= ( (( prescaler_index&0x2 )>>1) << CS11);
  TCCR1B |= ( (( prescaler_index&0x4 )>>2) << CS12);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  // enable global interrupts
  interrupts();
}

 
/**
 * Process all line segments in the ring buffer.  Uses bresenham's line algorithm to move all motors.
 */
ISR(TIMER1_COMPA_vect) {
   digitalWrite(13,digitalRead(13)^1);
   
  // segment buffer empty? do nothing
  if( current_segment == last_segment ) return;
  
  // Is this segment done?
  if( line_segments[current_segment].steps_left <= 0 ) {
    // Move on to next segment without wasting an interrupt tick.
    current_segment = get_next_segment(current_segment);
    if( current_segment == last_segment ) return;
  }
  
  int j;
  Segment &seg = line_segments[current_segment];
  // is this a fresh new segment?
  if( seg.steps == seg.steps_left ) {
    // set the direction pins
    for(j=0;j<NUM_AXIES;++j) {
      digitalWrite( robot.arms[j].motor_dir_pin, line_segments[current_segment].a[j].dir );
    }
    // set frequency to segment feed rate
    timer_set_frequency(seg.feed_rate);
  }

  // make a step
  --seg.steps_left;

  // move each axis
  for(j=0;j<NUM_AXIES;++j) {
    Axis &a = seg.a[j];
    
    a.over += a.absdelta;
    if(a.over >= seg.steps) {
      digitalWrite(robot.arms[j].motor_step_pin,LOW);
      a.over -= seg.steps;
      digitalWrite(robot.arms[j].motor_step_pin,HIGH);
    }
  }
}


/**
* This file is part of Stewart Platform v2.
*
* Stewart Platform v2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Stewart Platform v2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Stewart Platform v2. If not, see <http://www.gnu.org/licenses/>.
*/
