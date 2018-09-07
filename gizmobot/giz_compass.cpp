#include "giz_compass.h"
#include <LSM303.h>
#include <Wire.h>



LSM303 compass;
int compassMin;
int compassMax;

GizCompass::GizCompass() {

}

void GizCompass::init(){
  Serial.println(F("Initializing LSM303 compass. Please wait... "));
  Wire.begin();
  compass.init();
  compass.enableDefault();
  
  // Calibration values...
 // compass.m_min = (LSM303::vector<int16_t>){445, -1242, -1005};
 // compass.m_max = (LSM303::vector<int16_t>){1468, -184, -124};

//compass.m_min = (LSM303::vector<int16_t>){-660, -472, -396};
//compass.m_max = (LSM303::vector<int16_t>){209, 307, 479};

compass.m_min = (LSM303::vector<int16_t>){-680, -625, -428};
compass.m_max = (LSM303::vector<int16_t>){360, 433, 470};



  compassMin = 360;
  compassMax = 0;

  Serial.println(F("Compass initialized!"));

}
void GizCompass::update(){
    compass.read();
    //compass.heading((LSM303::vector<int>){0, 1, 0});
    double current_bearing  = compass.heading((LSM303::vector<int>){1,0, 0});//- 92; // Convert from -Y to +X, minus the chassis offset
 // if (current_bearing < 0) {
 //   current_bearing += 360;
 // }

    double current_bearing_r=(current_bearing * PI) / 180;
    if(current_bearing_r > PI){
        current_bearing_r= -1 * (PI - (current_bearing_r-PI)); 
    }

    //Serial.print("compass heading:");
    //Serial.println(current_bearing_r);
}
