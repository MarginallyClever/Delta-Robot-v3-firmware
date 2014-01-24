//------------------------------------------------------------------------------
// Delta Robot v8 - Supports RUMBA 6-axis motor controller
// dan@marginallycelver.com 2014-01-07
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/DeltaRobotv8 for more information.


#define NUM_SWITCHES  (6)

int old_state[NUM_SWITCHES];


void setup() {
  Serial.begin(57600);
  Serial.println(F("Switch test"));
  for(int i=0;i<NUM_SWITCHES;++i) {
    int j=37-i;
    pinMode(j,INPUT);
    digitalWrite(j,HIGH);
    old_state[i]=-1;
  }
}


void loop() {
  for(int i=0;i<NUM_SWITCHES;++i) {
    int j=37-i;
    int state=digitalRead(j);
    if(old_state[i] != state) {
      old_state[i] = state;
      Serial.print(j);
      Serial.print('=');
      Serial.println(state);
    }
  }
  delay(50);
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
