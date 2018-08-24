#include "giz_gps.h"

TinyGPSPlus gps;

void GizGps::init(){
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

	double bearing=atan2(y,x);

	double xx=d*sin(bearing);
	double yy=d*cos(bearing);

	x_pos=xx;
	y_pos=yy;
	heading=bearing;
}



