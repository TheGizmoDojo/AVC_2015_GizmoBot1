#include "giz_gyro.h"

static char temp_heading[200];
static int c=0; 

GizGyro::GizGyro() {
    GRYO_SERIAL.begin(GRYO_BAUD);
}

void GizGyro::update(){

if(int l=GRYO_SERIAL.available()){
        for (int i = 0; i < l; i++){
           char t= GRYO_SERIAL.read();
           if(t=='\n'){
               temp_heading[c]='\0';
               heading_r=atof(temp_heading);
               c=0;
          }
      temp_heading[c]=t;
      c++;
    }
}
}
