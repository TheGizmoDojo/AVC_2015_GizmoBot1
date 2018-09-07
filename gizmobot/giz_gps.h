/*
* GPS has ability to update at about 10hz
* I think the easist way to think about x,y from lat,lng is
* as if you are looking at zoomed in google map and:
* x_pos = E/W E=positive
* y_pos = N/S N=positive
//Basically clockwise radians, northern = + southern = -
* North is 0 radians E= pi/2 South = pi, West = -pi/2
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
#define GPS_SERIAL Serial3
#define GPS_BAUD 115200

class GizGps{

public:
	double y_pos_m;
	double x_pos_m;
	double lat;
	double lng;
	double heading_r;
	bool is_updated;
	double lat1;
	double lng1;
    GizGps();
	void update();
	void set_starting_point();
	void init();
	void update_x_y_pos(double lat2,double lng2);
	void get_x_y_pos_from_lat_lng(double lat2,double lng2,double *x_new,double *y_new, double *heading);



private: TinyGPSPlus gps;
  GizGps(const GizGps&) = delete;
  GizGps(GizGps&&) = delete;
  GizGps& operator=(const GizGps&) = delete;
};


#endif 
