#include "giz_compass.h"

static char temp_heading[200];
static int c=0; 

void GizCompass::init(){
    COMPASS_SERIAL.begin(COMPASS_BAUD);
}

void GizCompass::update(){

if(int l=COMPASS_SERIAL.available()){
        for (int i = 0; i < l; i++){
           char t= COMPASS_SERIAL.read();
           if(t=='\n'){
               temp_heading[c]='\0';
               heading=atof(temp_heading);
               c=0;
          }
      temp_heading[c]=t;
      c++;
    }
}
}
