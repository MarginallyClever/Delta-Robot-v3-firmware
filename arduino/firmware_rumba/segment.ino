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
Segment *working_seg = NULL;
volatile int current_segment  = 0;
volatile int last_segment     = 0;
int step_multiplier;


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------


FORCE_INLINE int get_next_segment(int i) {
  return ( i + 1 ) & ( MAX_SEGMENTS - 1 );
}


FORCE_INLINE int get_prev_segment(int i) {
  return ( i + MAX_SEGMENTS - 1 ) & ( MAX_SEGMENTS - 1 );
}


void segment_setup() {
  current_segment=0;
  last_segment=0;
  Segment &old_seg = line_segments[get_prev_segment(last_segment)];
  old_seg.a[0].step_count=0;
  old_seg.a[1].step_count=0;
  old_seg.a[2].step_count=0;
  working_seg = NULL;
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
  
  Segment &old_seg = line_segments[get_prev_segment(last_segment)];
  new_seg.a[0].delta = n0 - old_seg.a[0].step_count;
  new_seg.a[1].delta = n1 - old_seg.a[1].step_count;
  new_seg.a[2].delta = n2 - old_seg.a[2].step_count;

  new_seg.steps_total = 0;
  new_seg.feed_rate_start =
  new_seg.feed_rate_end   = new_feed_rate;

  int i;
  for(i=0;i<NUM_AXIES;++i) {
    new_seg.a[i].over = 0;
    new_seg.a[i].dir = (new_seg.a[i].delta < 0 ? LOW:HIGH);
    new_seg.a[i].absdelta = abs(new_seg.a[i].delta);
    if( new_seg.steps_total < new_seg.a[i].absdelta ) {
      new_seg.steps_total = new_seg.a[i].absdelta;
    }
  }

  if( new_seg.steps_total == 0 ) return;
  
  new_seg.steps_taken = 0;
  
  new_seg.decel_after = 
  new_seg.accel_until = new_seg.steps_total / 2;
  
#if VERBOSE > 1
  Serial.print(F("At "));  Serial.println(current_segment);
  Serial.print(F("Adding "));  Serial.println(last_segment);
  Serial.print(F("Steps= "));  Serial.println(new_seg.steps_left);
  Serial.print(F("d0= "));  Serial.println(new_seg.a[0].delta);
  Serial.print(F("d1= "));  Serial.println(new_seg.a[1].delta);
  Serial.print(F("d2= "));  Serial.println(new_seg.a[2].delta);
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
FORCE_INLINE void timer_set_frequency(long desired_freq_hz) {
  // Source: https://github.com/MarginallyClever/ArduinoTimerInterrupt
  // Different clock sources can be selected for each timer independently. 
  // To calculate the timer frequency (for example 2Hz using timer1) you will need:
  
  if(desired_freq_hz > 20000 ) {
    step_multiplier = 4;
    desired_freq_hz >>= 2;
  } else if(desired_freq_hz > 20000 ) {
    step_multiplier = 2;
    desired_freq_hz >>= 1;
  } else {
    step_multiplier=1;
  }
  
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
#if VERBOSE > 1
    // if Serial.print is called from inside a timing thread it will probably crash the arduino.
    // Serial.print takesk too long, the interrupt will interrupt itself.
    Serial.println(F("Timer could not be set: Desired frequency out of bounds."));
#endif
    return;
  }
  
#if VERBOSE > 4
  Serial.print("freq=");
  Serial.print(desired_freq_hz);
#endif

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
  // segment buffer empty? do nothing
  if( working_seg == NULL ) {
    working_seg = segment_get_working();
    if( working_seg != NULL ) {
      // New segment!
      // set the direction pins
      digitalWrite( robot.arms[0].motor_dir_pin, working_seg->a[0].dir );
      digitalWrite( robot.arms[1].motor_dir_pin, working_seg->a[1].dir );
      digitalWrite( robot.arms[2].motor_dir_pin, working_seg->a[2].dir );
      
      // set frequency to segment feed rate
      timer_set_frequency(working_seg->feed_rate_start);
    }
  }
  
  if( working_seg != NULL ) {
    // move each axis
    for(int i=0;i<step_multiplier;++i) {
      // M0
      Axis &a0 = working_seg->a[0];
      a0.over += a0.absdelta;
      if(a0.over >= working_seg->steps_total) {
        digitalWrite(robot.arms[0].motor_step_pin,HIGH);
        a0.over -= working_seg->steps_total;
        digitalWrite(robot.arms[0].motor_step_pin,LOW);
      }
      // M1
      Axis &a1 = working_seg->a[1];
      a1.over += a1.absdelta;
      if(a1.over >= working_seg->steps_total) {
        digitalWrite(robot.arms[1].motor_step_pin,HIGH);
        a1.over -= working_seg->steps_total;
        digitalWrite(robot.arms[1].motor_step_pin,LOW);
      }
      // M2
      Axis &a2 = working_seg->a[2];
      a2.over += a2.absdelta;
      if(a2.over >= working_seg->steps_total) {
        digitalWrite(robot.arms[2].motor_step_pin,HIGH);
        a2.over -= working_seg->steps_total;
        digitalWrite(robot.arms[2].motor_step_pin,LOW);
      }
    }
    
    // make a step
    working_seg->steps_taken++;

    // accel
    if( working_seg->steps_taken <= working_seg->accel_until ) {
//      OCR1A-=200;
    } else if( working_seg->steps_taken > working_seg->decel_after ) {
//      OCR1A+=200;
    }

    // Is this segment done?
    if( working_seg->steps_taken >= working_seg->steps_total ) {
      // Move on to next segment without wasting an interrupt tick.
      working_seg = NULL;
      current_segment = get_next_segment(current_segment);
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
