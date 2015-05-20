/*
 * HMC6352.cpp
 * 
 * Copyright (c) 2009 Ruben Laguna <ruben.laguna at gmail.com>. All rights reserved.
 * 
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Wire.h>
#include "Arduino.h"
#include "HMC6352.h"


int HMC6352SlaveAddress = 0x21;
int HMC6352ReadAddress = 0x41;		//"A" in hex, A command is: "Get Data" command
int HMC6352SleepAddress = 0x53; 	//"S" in hex, S command is: "Sleep" command
int HMC6352WakeAddress = 0x57; 		//"W" in hex, W command is: "Wake" command


int HMC6352Class::GetHeading()
{
  float headingSum;

  Wire.beginTransmission(HMC6352SlaveAddress);
  Wire.write(HMC6352ReadAddress);		// The "Get Data" command
  Wire.endTransmission();
  delay(8); //6000 microseconds minimum 6 ms 

  Wire.requestFrom(HMC6352SlaveAddress, 2);
  
  //"The heading output data will be the value in tenths of degrees
  //from zero to 3599 and provided in binary format over the two bytes."
  byte MSB = Wire.read();
  byte LSB = Wire.read();

  headingSum = (MSB << 8) + LSB; //(MSB / LSB sum)
  
  return (headingSum / 10);
}


void HMC6352Class::Wake()
{
  Wire.beginTransmission(HMC6352SlaveAddress);
  Wire.write(HMC6352WakeAddress); //W wake up exit sleep mode
  Wire.endTransmission();
  
}


void HMC6352Class::Sleep()
{
  Wire.beginTransmission(HMC6352SlaveAddress);
  Wire.write(HMC6352SleepAddress); //S enter sleep mode
  Wire.endTransmission();
}

HMC6352Class HMC6352;
