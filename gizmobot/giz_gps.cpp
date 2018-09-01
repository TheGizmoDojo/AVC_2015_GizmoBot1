#include "giz_gps.h"

#include <TinyGPS++.h>

static const double STARTING_LAT = 40.0084230000;
static const double STARTING_LON = -105.0964255000;

GizGps::GizGps() :
    y_pos_m(0),
    x_pos_m(0),
    lat(0),
    lng(0),
    heading_r(0),
    is_updated(false),
    lat1(STARTING_LAT),
    lng1(STARTING_LON),
    gps()
{
    GPS_SERIAL.begin(GPS_BAUD);
}

void GizGps::update(){
  is_updated=false;
  while (GPS_SERIAL.available() > 0) {
      gps.encode(GPS_SERIAL.read());
  }
  if (gps.location.isUpdated()){
        is_updated=true;
        lat=gps.location.lat();
        lng=gps.location.lng();
        update_x_y_pos(lat,lng);
   }
}

void GizGps::update_x_y_pos(double lat2,double lng2){
		
	double earth_radius=6371000;
    double phi_1= lat1*(PI / 180.0);
    double phi_2= lat2*(PI / 180.0);
	double delta_phi= (lat2-lat1)*(PI/180.0);
	double delta_lamda=(lng2-lng1)*(PI/180.0);

	double a = sin(delta_phi/2) * sin(delta_phi/2) +
        cos(phi_1) * cos(phi_2) *
        sin(delta_lamda/2) * sin(delta_lamda/2);

	double c = 2 * atan2(sqrt(a), sqrt(1));

	//distance 
	double d=earth_radius * c;
	double y=sin(delta_lamda)*cos(phi_2);

	double x=(cos(phi_1)*sin(phi_2))
			-(sin(phi_1) * cos(phi_2) * cos(delta_lamda));

	const double bearing_r=atan2(y,x);

	double xx=d*sin(bearing_r);
	double yy=d*cos(bearing_r);

	x_pos_m=xx;
	y_pos_m=yy;
	heading_r=bearing_r;
}



