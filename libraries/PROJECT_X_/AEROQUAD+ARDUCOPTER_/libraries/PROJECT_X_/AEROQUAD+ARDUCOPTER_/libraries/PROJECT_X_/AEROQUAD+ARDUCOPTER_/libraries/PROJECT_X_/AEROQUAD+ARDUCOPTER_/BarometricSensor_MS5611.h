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

// parts of this code were taken from AN520, an early version of fabio's library and the AQ BMP085 code

#ifndef _AQ_BAROMETRIC_SENSOR_MS5611_
#define _AQ_BAROMETRIC_SENSOR_MS5611_

#include "BarometricSensor.h"
#include <SPI.h>
#include <AQMath.h>


/* on APM v.24 MS5661_CS is PG1 (Arduino pin 40) */
#define MS5611_CS 40

//#define DEBUG_MS5611
#define CMD_MS5611_RESET 0x1E
#define CMD_MS5611_PROM_Setup 0xA0
#define CMD_MS5611_PROM_C1 0xA2
#define CMD_MS5611_PROM_C2 0xA4
#define CMD_MS5611_PROM_C3 0xA6
#define CMD_MS5611_PROM_C4 0xA8
#define CMD_MS5611_PROM_C5 0xAA
#define CMD_MS5611_PROM_C6 0xAC
#define CMD_MS5611_PROM_CRC 0xAE
#define CMD_CONVERT_D1_OSR4096 0x48   // Maximum resolution (oversampling)
#define CMD_CONVERT_D2_OSR4096 0x58   // Maximum resolution (oversampling)

























long MS5611lastRawTemperature;
long MS5611lastRawPressure;
int64_t MS5611_sens=0;
int64_t MS5611_offset=0;



float pressure			 = 0;
long rawPressure         = 0;
long rawTemperature      = 0;
byte pressureCount       = 0;
float pressureFactor     = 1/5.255;
boolean isReadPressure   = false;
float rawPressureSum     = 0;
byte rawPressureSumCount = 0;



void requestRawTemperature() {
  _spi_write(CMD_CONVERT_D2_OSR4096);}


unsigned long readRawTemperature()
{
  // see datasheet page 7 for formulas
  MS5611lastRawTemperature = MS5611readConversion(MS5611_I2C_ADDRESS);
  int64_t dT     = MS5611lastRawTemperature - (((long)MS5611Prom[5]) << 8);
  MS5611_offset  = (((int64_t)MS5611Prom[2]) << 16) + ((MS5611Prom[4] * dT) >> 7);
  MS5611_sens    = (((int64_t)MS5611Prom[1]) << 15) + ((MS5611Prom[3] * dT) >> 8);

  return MS5611lastRawTemperature;
}


float readTemperature()
{
  return ((1<<5)*2000 + (((MS5611lastRawTemperature - ((int64_t)MS5611Prom[5] << 8)) * MS5611Prom[6]) >> (23-5))) / ((1<<5) * 100.0);
}

void requestRawPressure()
{
  sendByteI2C(MS5611_I2C_ADDRESS, MS561101BA_D1_Pressure + MS561101BA_OSR_4096);
}

float readRawPressure()
{
  MS5611lastRawPressure = MS5611readConversion(MS5611_I2C_ADDRESS);

  return (((( MS5611lastRawPressure * MS5611_sens) >> 21) - MS5611_offset) >> (15-5)) / ((float)(1<<5));
}

bool baroGroundUpdateDone = false;
unsigned long baroStartTime;

void initializeBaro() {
  baroStartTime = micros();

  pressure = 0;
  baroGroundAltitude = 0;
  pressureFactor = 1/5.255;

  MS5611reset(MS5611_I2C_ADDRESS); // reset the device to populate its internal PROM registers
  delay(3); // some safety time

  if(MS5611readPROM(MS5611_I2C_ADDRESS) ) {
	  vehicleState |= BARO_DETECTED;
  }

  requestRawTemperature(); // setup up next measure() for temperature
  isReadPressure = false;
  pressureCount = 0;
  delay(10);
  measureBaroSum(); // read temperature
  delay(10);
  measureBaro(); // read pressure
  delay(10);

  measureGroundBaro();
  measureGroundBaro();

  baroAltitude = baroGroundAltitude;
}

void measureBaro() {
  measureBaroSum();
  evaluateBaroAltitude();
}

void measureBaroSum() {
  // switch between pressure and temperature measurements
  if (isReadPressure) {
    rawPressureSum += readRawPressure();
    rawPressureSumCount++;
    if (pressureCount == 20) {
      requestRawTemperature();
      pressureCount = 0;
      isReadPressure = false;
    } 
	else {
      requestRawPressure();
	}
    pressureCount++;
  } 
  else { // select must equal TEMPERATURE
    readRawTemperature();
    requestRawPressure();
    isReadPressure = true;
  }
}

bool MS5611_first_read = true;

void evaluateBaroAltitude() {

  if (rawPressureSumCount == 0) { // it may occur at init time that no pressure has been read yet!
    return;
  }

  pressure = rawPressureSum / rawPressureSumCount;

  baroRawAltitude = 44330 * (1 - pow(pressure/101325.0, pressureFactor)); // returns absolute baroAltitude in meters
  // use calculation below in case you need a smaller binary file for CPUs having just 32KB flash ROM
  // baroRawAltitude = (101325.0-pressure)/4096*346;

  if(MS5611_first_read) {
    baroAltitude = baroRawAltitude;
    MS5611_first_read = false;
  } 
  else {
    baroAltitude = filterSmooth(baroRawAltitude, baroAltitude, baroSmoothFactor);
  }

  rawPressureSum = 0.0;
  rawPressureSumCount = 0;

  // set ground altitude after a delay, so sensor has time to heat up
  const unsigned long updateDelayInSeconds = 10;
  if(!baroGroundUpdateDone && (micros()-baroStartTime) > updateDelayInSeconds*1000000) {
	  baroGroundAltitude = baroAltitude;
	  baroGroundUpdateDone = true;
  }
}
*/
#endif
