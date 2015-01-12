/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
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

#ifndef _AEROQUAD_MAX_SONAR_RANGE_FINDER_H_
#define _AEROQUAD_MAX_SONAR_RANGE_FINDER_H_

// @see http://www.arduino.cc/playground/Main/MaxSonar

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(BOARD_aeroquad32)

#include "RangeFinder.h"

#define MB1000 0 // Maxbotix LV-MaxSonar-EZ*
#define MB1200 1 // Maxbotix XL-MaxSonar-EZ*
#define NEW1 2   // Sumeet Instruments US sensor (Sudeep)
#define ECHO 3  // ECHO RHYDOLABZs
#define SPIKE_FILTER_MARGIN 500 // mm ; changes bigger than this need two samples to take effect

#if defined(WirelessTelemetry) 
  #define SERIAL_SONAR Serial //HARSHAL :7/5
#else 
    #define SERIAL_SONAR Serial2 //HARSHAL :7/5
#endif  

struct rangeFinder {
    byte target;      // {ALTITUDE,FRONT,RIGHT,REAR,LEFT}_RANGE_FINDER_INDEX
    byte pin;
    byte triggerpin;
    byte type;
  } rangeFinders[] = {
     // Define your rangers here
     // First ranger is given priority so it should be used for altitude
     // If using more than one ranger you should connect the 'trigger' to the 'RX' pin on the ranger.
     //
  //    { ALTITUDE_RANGE_FINDER_INDEX, A1, 24, MB1200}, 
    { ALTITUDE_RANGE_FINDER_INDEX, A1, A1, ECHO},
//	  { FRONT_RANGE_FINDER_INDEX,    A2, 25, MB1000},
//	  { RIGHT_RANGE_FINDER_INDEX,    A3, 26, MB1000},
//	  { REAR_RANGE_FINDER_INDEX,     A4, 27, MB1000},
//	  { LEFT_RANGE_FINDER_INDEX,     A5, 28, MB1000}
	};

// theoretical range at AIN=VCC
short rangerScale[] = { 
  13005, // MB10xx series
  10240, // MB12xx series
  98425, // New1 series
  4.00, // ECHO Series
  };

// 50Hz cycles needed to wait for ranging
byte rangerWait[] = {
  2, // MB1000 needs 50ms i.e. wait 2 cycles (60ms)
  4, // MB1200 needs 100ms i.e. wait 5 cycles (100ms)
  1, // NEW1 needs 5ms i.e wait 1 cycle (20ms)
  1,// ECHO needs 500ms
  };

#define RANGER_COUNT ((sizeof(rangeFinders) / sizeof(struct rangeFinder)))

// last reading used for 'spike' filtter
short lastRange[RANGER_COUNT];

byte rangerWaitCycles = 0;

byte rangerSchedule = 0;

int serial_count=0;
char received_serial_bytes[6]={},serial_char=0;
float distanceInMeter=0.0;


void inititalizeRangeFinders() {
  //initialise using SERIAL_SONAR begin at 9600 baudrate
  SERIAL_SONAR.begin(9600);
  rangerWaitCycles = 10; // allow to initialize
}

void updateRangeFinders() {

  byte rangerToRead = 0;
  byte rangerToTrigger = 0;
  

  if (rangerWaitCycles) {
    rangerWaitCycles--;
    return;
  }
 
 if(SERIAL_SONAR.available()) {
   
    serial_char=SERIAL_SONAR.read(); 
    if(serial_char==13)
      serial_count=0;
    else 
     {
    if(serial_char=='E'||serial_char=='R')
     {
       distanceInMeter=4;
       serial_count++;
       //if(serial_count%4==3)
      // {Serial.print(distanceInMeter);
       // Serial.println(" ,");} 
     }
    else
    { 
      received_serial_bytes[serial_count]=serial_char; 
      serial_count++;
     if(serial_count%6==5)
     {
      received_serial_bytes[5]='\0';
      distanceInMeter= atof(received_serial_bytes)/100;
      /*Serial.print(",Data_Read= ");
      Serial.print(received_serial_bytes);
      Serial.print(" ,Distinmeter=");
      Serial.print(distanceInMeter);
      Serial.println(" ,"); */   
   }    
    
   }
  
  float range = distanceInMeter*1000;
    //Serial.print(" Range=");
	//Serial.print(range);
	 
  // Following will accept the sample if it's either withing "spike margin" of last raw reading or previous accepted reading
  // otherwise it's ignored as noise
  
  if ((abs(range - lastRange[rangerToRead]) < SPIKE_FILTER_MARGIN) ||
      (abs(range * 1000.0 - rangeFinderRange[rangeFinders[rangerToRead].target]) < SPIKE_FILTER_MARGIN)) {
    rangeFinderRange[rangeFinders[rangerToRead].target] = (float)range / 1000.0;
  }
  lastRange[rangerToRead] = range;
 
  rangerWaitCycles = rangerWait[rangeFinders[rangerToRead].type];

 
}
}
}
#endif 
#endif








