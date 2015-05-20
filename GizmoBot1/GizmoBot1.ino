/**
 * Arduiono code for TGD Autonomous Vehicle
 *
 * Authors: TGD AVC committee
 *
 * License: CC Share/Share-alike
 */
 
#include "TGD_Movement.h" // Set pin assignments in "TGD_Movement.h"

void setup() {
  // put your setup code here, to run once:
  init_movement();
  
  move_forward_stop(1000);     // Move forward for one second and stop.
  delay(500);                  // Pause for half a second.
  move_reverse_stop(1000);     // Move backwards for one second and stop.
  delay(500);                  // Pause for half a second.
  wheels_left();               // Turn wheels left.
  move_forward_stop(750);      // Move forward while turning left for 0.75 seconds.
  delay(500);                  // Pause for half a second.
  move_reverse_stop(750);      // Move backwards with wheels turned left for 0.75 seconds.
  delay(500);                  // Pause for half a second.
  wheels_right();              // Turn wheels right.
  move_forward_stop(750);      // Move forward while turning right for 0.75 seconds.
  delay(500);                  // Pause for half a second.
  move_reverse_stop(750);      // Move backwards with wheels turned right for 0.75 seconds.
  wheels_center();             // Turn wheels to point straight forward.
   

}

void loop() {
  // put your main code here, to run repeatedly:
  
  // check current and destination location.
  // calculate heading to get from a to b.
  // compare it to current heading.
  // check for obstacles.
  // turn vehicle to correct heading.
  // check for obstacles.
  // move forward for 1 second and stop.

}
