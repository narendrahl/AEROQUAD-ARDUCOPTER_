/*
  AeroQuad v3.0 - March 2011
  www.AeroQuad.com
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

#include <SPI.h>
#include <GlobalDefined.h>
#include <AQMath.h>
#include <stdio.h>

#include <Gyroscope_MPU6000.h> 

unsigned long timer;

void setup() {
  
  Serial.begin(115200);
  Serial.println("Gyroscope library test (MPU6000)");

  
  initializeGyro();
  calibrateGyro();
  timer = millis();}

void loop() {
  
  if((millis() - timer) > 10) // 100Hz
  {
   
    
   timer = millis();
    measureGyro();
    
   // Serial.printf("Roll(X):%.3f ,Pitch(Y):%.3f , Yaw(Z):%.3f ,Heading: %.3f",degrees(gyroRate[XAXIS]),degrees(gyroRate[XAXIS]),degrees(gyroRate[XAXIS]),gyroHeading));
    Serial.print("Roll(X):");
    Serial.print(degrees(gyroRate[XAXIS]),2);
    //Serial.print(" Pitch(Y): ");
    //Serial.print(degrees(gyroRate[YAXIS]),2);
    //Serial.print(" Yaw(Z): ");
    //Serial.print(degrees(gyroRate[ZAXIS]),2);
    //Serial.print(" Heading: ");
    //Serial.print(degrees(gyroHeading));
    //Serial.println();
  }
}