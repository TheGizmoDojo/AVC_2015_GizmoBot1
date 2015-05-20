#ifndef __TGD_MOVEMENT_H_
#define __TGD_MOVEMENT_H_

#define forwardPin 7
#define backPin 6
#define leftPin 5
#define rightPin 4

// Initialize movement library - eliminate floating voltages and set pins to output
void init_movement() {
  digitalWrite(forwardPin, LOW);
  digitalWrite(backPin, LOW);
  digitalWrite(leftPin, LOW);
  digitalWrite(rightPin, LOW);
  
  pinMode(forwardPin, OUTPUT);
  pinMode(backPin, OUTPUT);
  pinMode(leftPin, OUTPUT);
  pinMode(rightPin, OUTPUT);
}

// Move forward and coast
void move_forward(ms_time) {
  pinMode(forwardPin, INPUT);
  delay(ms_time);
  pinMode(forwardPin, OUTPUT);
}

// Move forward and stop
void move_forward_stop(ms_time) {
  pinMode(forwardPin, INPUT);
  delay(ms_time);
  pinMode(forwardPin, OUTPUT);
  pinMode(backPin, INPUT);
  delay(50);
  pinMode(backPin, OUTPUT);
}

// Move in reverse and coast
void move_reverse(ms_time) {
  pinMode(backPin, INPUT);
  delay(ms_time);
  pinMode(backPin, OUTPUT);
}

// Move in reverse and stop
void move_reverse_stop(ms_time) {
  pinMode(backPin, INPUT);
  delay(ms_time);
  pinMode(backPin, OUTPUT);
  pinMode(forwardPin, INPUT);
  delay(50);
  pinMode(forwardPin, OUTPUT);
}

// Turn wheels left
void wheels_left(){
  pinMode(rightPin, OUTPUT);
  pinMode(leftPin, INPUT);
}

// Turn wheels right
void wheels_right(){
  pinMode(leftPin, OUTPUT);
  pinMode(rightPin, INPUT);
}

// Center wheels
void wheels_center(){
  pinMode(rightPin, OUTPUT);
  pinMode(leftPin, OUTPUT);
}

#endif __TGD_MOVEMENT_H_
