#ifndef __TGD_MOVEMENT_H_
#define __TGD_MOVEMENT_H_

#include <arduino.h> //Necessary for Arduino IDE > 1.00 (?)

//#defines
#define forwardPin 7
#define reversePin 6
#define leftPin 5
#define rightPin 4
#define SPEED_PERIOD 100 //0.1sec NOTE: I probably wouldn't go any faster than 0.1sec period
#define SPEED_TICK SPEED_PERIOD/100
//#define MAX_SPEED 100
#define MAX_SPEED 60 //At Erik&Diana's testing

//global variables
extern bool forward;
extern bool pseudoPWMOn;
extern int speedOnMillis;
extern int speedOffMillis;
extern int speedCount;
extern int reverseMillis;
extern int leftMillis;
extern int rightMillis;

//functions
void setWheelsLeft(); //Turn wheels left
void setWheelsNotLeft(); //Don't turn wheels left, leave right alone
void setWheelsRight(); //Turn wheels right
void setWheelsNotRight(); //Don't turn wheels right, leave left alone
void setMovementForward(); //Move forward
void setMovementReverse(); //Move backward
void setReverseOff(); //Safely set reverse off 
void setForwardOff(); //Safely set forward off 
void setForwardSpeed( int forwardSpeed ); //non-blocking, set forward speed, implemented in pseudo-PWM
void init_movement(); //Initialize movement library - eliminate floating voltages and set pins to output
void reverseForXMillis(long reverseTime); //non-blocking, reverse for X milliseconds
void turnForXMillis(long turnMillis, bool leftTurn); //non-blocking, turn for X milliseconds

#endif // __TGD_MOVEMENT_H_
