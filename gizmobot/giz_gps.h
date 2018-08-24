/*
* GPS has ability to update at about 10hz
* I think the easist way to think about x,y from lat,lng is
* as if you are looking at zoomed in google map and:
* x_pos = E/W E=positive
* y_pos = N/S N=positive
* 
* be sure to set starting lat,lng this will be the (0,0) x,y
*
* Dependencies:
* TinyGps++ : https://github.com/mikalhart/TinyGPSPlus
*/

#ifndef __GIZ_GPS_
#define __GIZ_GPS_
#include <TinyGPS++.h>
#include "Arduino.h"
#include <math.h>
#define STARTING_LAT 40.0084230000;
#define STARTING_LNG -105.0964255000;
#define GPS_SERIAL Serial1
#define GPS_BAUD 115200

class GizGps{

public:
	double y_pos;
	double x_pos;
	double lat;
	double lng;
	double heading;
	bool is_updated=false;
	double lat1=STARTING_LAT;
	double lng1=STARTING_LNG;

  
public:
	void init();
	void update();
	void update_x_y_pos(double lat2,double lng2);
};


#endif 
