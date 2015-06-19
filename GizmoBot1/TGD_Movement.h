#ifndef __TGD_MOVEMENT_H_
#define __TGD_MOVEMENT_H_

#include <arduino.h> //Necessary for Arduino IDE > 1.00 (?)
//#include "TimerOne.h"

//#defines
#define forwardPin 7
#define reversePin 6
#define leftPin 5
#define rightPin 4
#define SPEED_PERIOD 100 //0.1sec NOTE: I probably wouldn't go any faster than 0.1sec period
#define SPEED_TICK SPEED_PERIOD/100
#define MAX_SPEED 100

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
//void move_forward(long ms_time); //Move forward and coast
//void move_forward_stop(long ms_time); //Move forward and stop
//void move_reverse(long ms_time); //Move in reverse and coast
//void move_reverse_stop(long ms_time); //Move in reverse and stop
void setWheelsLeft(); //Turn wheels left
void setWheelsNotLeft(); //Don't turn wheels left, leave right alone
void setWheelsRight(); //Turn wheels right
void setWheelsNotRight(); //Don't turn wheels right, leave left alone
//void setWheelsCenter(); //Center wheels
void setMovementForward(); //Move forward
void setMovementReverse(); //Move backward
void setReverseOff(); //Safely set reverse off 
void setForwardOff(); //Safely set forward off 
//void timerInterrupt(); //runs this function when Timer1 interrupts
void setForwardSpeed( int forwardSpeed ); //non-blocking, set forward speed, implemented in pseudo-PWM
//void speedDemo(int speed); //testing speed pseudo-pwm functionality
void init_movement(); //Initialize movement library - eliminate floating voltages and set pins to output
void reverseForXMillis(long reverseTime); //non-blocking, reverse for X milliseconds
void turnForXMillis(long turnMillis, bool leftTurn); //non-blocking, turn for X milliseconds

#endif __TGD_MOVEMENT_H_
