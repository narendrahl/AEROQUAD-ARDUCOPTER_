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

#include <AQMath.h>

#include <GlobalDefined.h>
#include <Baro_MS5611.h>

unsigned long timer = 0;

void setup() {

  Serial.begin(115200);
  Serial.println("Barometric sensor library test (MS5611)");

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32);     // 500khz for debugging, increase later

  initializeBaro(); 
}

void loop() {

  if((millis() - timer) > 10) // 20Hz
  {
    timer = millis();
    measureBaro();
    Serial.print("Pressure : ");
    Serial.print(pressure);
    Serial.print("Raw_altitude : ");
    Serial.print(baroRawAltitude);
	Serial.print("Temperature : ");
    Serial.print(readTemperature());
	
    
    Serial.print("altitude : ");
    Serial.print(getBaroAltitude());
    Serial.println();
  }
}
