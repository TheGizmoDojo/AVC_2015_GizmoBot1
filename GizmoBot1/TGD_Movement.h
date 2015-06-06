#ifndef __TGD_MOVEMENT_H_
#define __TGD_MOVEMENT_H_

#include <arduino.h> //Necessary for Arduino IDE > 1.00 (?)
#include "TimerOne.h"

//#defines
#define forwardPin 7
#define backPin 6
#define leftPin 5
#define rightPin 4
#define PERIOD 100000 //0.1sec NOTE: I probably wouldn't go any faster than 0.1sec period
#define PERIOD_TICK PERIOD/100
#define MAX_SPEED 100

//functions
void move_forward(long ms_time); //Move forward and coast
void move_forward_stop(long ms_time); //Move forward and stop
void move_reverse(long ms_time); //Move in reverse and coast
void move_reverse_stop(long ms_time); //Move in reverse and stop
void wheels_left(); //Turn wheels left
void wheels_right(); //Turn wheels right
void wheels_center(); //Center wheels
void timerInterrupt(); //runs this function when Timer1 interrupts
void setRobotSpeed( int robotSpeed ); //can call this at any point during the program, non-blocking
void speedDemo(int speed); //testing speed pseudo-pwm functionality
void init_movement(); //Initialize movement library - eliminate floating voltages and set pins to output

#endif __TGD_MOVEMENT_H_
