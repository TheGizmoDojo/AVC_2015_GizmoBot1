#include "TGD_Movement.h"

// Turn wheels left
void setWheelsLeft(){
  pinMode(rightPin, OUTPUT);
  pinMode(leftPin, INPUT);
}

// Turn wheels right
void setWheelsRight() {
  pinMode(leftPin, OUTPUT);
  pinMode(rightPin, INPUT);
}

// Don't turn wheels left, leave right wheels alone
void setWheelsNotLeft() {
  pinMode(leftPin, OUTPUT);
}

// Don't turn wheels left, leave right wheels alone
void setWheelsNotRight() {
  pinMode(rightPin, OUTPUT);
}

//Move forward
void setMovementForward() {
  pinMode(reversePin, OUTPUT);
  pinMode(forwardPin, INPUT);  
}

//Move backward
void setMovementReverse() {
  pinMode(forwardPin, OUTPUT);
  pinMode(reversePin, INPUT);
}

//Safely set reverse pin off
void setReverseOff() { 
  pinMode( reversePin, OUTPUT );
}

//Safely set forward pin off
void setForwardOff() { 
  pinMode( forwardPin, OUTPUT );
}

////NOTE: we could allow robotSpeed to be a negative number, and add code to cover reversing
void setForwardSpeed( int forwardSpeed ) { //forwardSpeed should be between 0 and 100 
  if( forwardSpeed < 0 ) {
    forwardSpeed = 0;
  } else if( forwardSpeed > MAX_SPEED || forwardSpeed > 100 ) {
    forwardSpeed = MAX_SPEED;
  }
  
  //make sure the reverse pin is low
  setReverseOff();
  
  //set onTime and offTime so that the interrupt can run without any other inputs.
  speedOnMillis = forwardSpeed*SPEED_TICK;
  speedOffMillis = SPEED_PERIOD-speedOnMillis;
  
  pseudoPWMOn = true;
  
//  Serial.print("On: ");
//  Serial.print(speedOnMillis);
//  Serial.print(" Off: ");
//  Serial.println(speedOffMillis);

}  

// Initialize movement library - eliminate floating voltages and set pins to output
void init_movement() {
  digitalWrite(forwardPin, LOW);
  digitalWrite(reversePin, LOW);
  digitalWrite(leftPin, LOW);
  digitalWrite(rightPin, LOW);
  
  pinMode(forwardPin, OUTPUT);
  pinMode(reversePin, OUTPUT);
  pinMode(leftPin, OUTPUT);
  pinMode(rightPin, OUTPUT);

//  forward = false;
  pseudoPWMOn = false;
  speedOnMillis = 0;
  speedOffMillis = SPEED_PERIOD;
  speedCount = 0;
  reverseMillis = 0;
  leftMillis = 0;
  rightMillis = 0;

  setForwardSpeed(0);

}

//non-blocking, reverse for X milliseconds
void reverseForXMillis(long reverseTime) {
  setForwardOff(); //Don't go forward
  reverseMillis = reverseTime;  
}

//non-blocking, turn for X milliseconds
void turnForXMillis(long turnMillis, bool leftTurn) {
  if( leftTurn ) {
    rightMillis = 0;
    leftMillis = turnMillis;
  } else {
    leftMillis = 0;
    rightMillis = turnMillis;
  }
}

