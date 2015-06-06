#include "TGD_Movement.h"

//global variables
volatile bool speedTracker; //Tracks the pseudo-PWM direction (false means the pin is set to output (not moving), true means the pin is set to input (moving))
volatile long onTime; //pseudo-PWM on-time
volatile long offTime; //pseudo-PWM off-time

// Move forward and coast
void move_forward(long ms_time) {
  pinMode(forwardPin, INPUT);
  delay(ms_time);
  pinMode(forwardPin, OUTPUT);
}

// Move forward and stop
void move_forward_stop(long ms_time) {
  pinMode(forwardPin, INPUT);
  delay(ms_time);
  pinMode(forwardPin, OUTPUT);
  pinMode(backPin, INPUT);
  delay(50);
  pinMode(backPin, OUTPUT);
}

// Move in reverse and coast
void move_reverse(long ms_time) {
  pinMode(backPin, INPUT);
  delay(ms_time);
  pinMode(backPin, OUTPUT);
}

// Move in reverse and stop
void move_reverse_stop(long ms_time) {
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
void wheels_right() {
  pinMode(leftPin, OUTPUT);
  pinMode(rightPin, INPUT);
}

// Center wheels
void wheels_center() {
  pinMode(rightPin, OUTPUT);
  pinMode(leftPin, OUTPUT);
}

void timerIntPseudoPWM() { //will trigger every time Timer1 expires, should load the new onTime or offTime so it will expire again in a set amount of time.
  if( speedTracker ) {
    // pin currently set to input, need to set pin to input and load offTime into Timer1
    speedTracker = false;
    pinMode(forwardPin, OUTPUT);
    Timer1.setPeriod(offTime);
  } else {
    // pin currently set to output, need to set pin to output and load onTime into Timer1
    speedTracker = true;
    pinMode(forwardPin, INPUT);
    Timer1.setPeriod(onTime);    
  }
}

//NOTE: we could allow robotSpeed to be a negative number, and add code to cover reversing
void setRobotSpeed( int robotSpeed ) { //robotSpeed should be between 0 and 100 
  if( robotSpeed < 0 ) {
    robotSpeed = 0;
  } else if( robotSpeed > MAX_SPEED || robotSpeed > 100 ) {
    robotSpeed = MAX_SPEED;
  }
  
  //make sure the reverse pin is low
  pinMode(backPin, OUTPUT);
  
  if( robotSpeed == 0 ) {
    //don't run the interrupt, because it will expire every cycle and slow the system to a halt
    Timer1.detachInterrupt();
    pinMode(forwardPin, OUTPUT);
  } else if( robotSpeed == 100 ) {
    //don't run the interrupt.  This is less of an issue, but it's a little more efficient this way.
    Timer1.detachInterrupt();
    pinMode(forwardPin, INPUT);
  } else {
    //set onTime and offTime so that the interrupt can run without any other inputs.
    onTime = robotSpeed*PERIOD_TICK;
    offTime = PERIOD-onTime;
    pinMode(forwardPin, INPUT);
    speedTracker = true;
    Timer1.setPeriod(onTime);
    Timer1.attachInterrupt(timerIntPseudoPWM);
  }
}  

void speedDemo(int speed) {
  setRobotSpeed(speed);
  delay(2000);
  setRobotSpeed(0);
}

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

  speedTracker = false; //initialized to not moving
  setRobotSpeed(0); //don't move...
  Timer1.initialize(0); //probably unnecessary, but I don't want it to start up uninitialized
//  Timer1.attachInterrupt(timerIntPseudoPWM); //attach the interrupt to the correct function
}


