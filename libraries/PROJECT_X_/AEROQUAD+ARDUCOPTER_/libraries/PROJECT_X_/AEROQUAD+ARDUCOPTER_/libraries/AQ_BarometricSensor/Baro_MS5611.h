/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef _AQ_Baro_MS5611_
#define _AQ_Baro_MS5611_

#include "BarometricSensor.h"
#include <SPI.h>
#include <AQMath.h>
#include <Serial.h>
#include <SensorsStatus.h>


/* on APM v.24 MS5661_CS is PG1 (Arduino pin 40) */
#define MS5611_CS 40

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
   
int32_t MS5611lastRawTemperature;
int32_t MS5611lastRawPressure;
int64_t MS5611_sens=0;
int64_t MS5611_offset=0;

    float dT;
    float TEMP;
    float Off;
    float SENS;
    float P1;

    
  
    /* AP_Baro public interface: */
float pressure			 = 0;
long rawPressure         = 0;
long rawTemperature      = 0;
byte pressureCount       = 0;
float pressureFactor     = 1/5.255;
boolean isReadPressure   = false;
float rawPressureSum     = 0;
byte rawPressureSumCount = 0;

    
  
    
    //float                            _ground_temperature;
   // float                            _ground_pressure;
   // float                               _altitude;
   // uint32_t                            _last_altitude_t;
   // DerivativeFilterFloat_Size7         _climb_rate_filter;

    float                           Temp;
    float                           Press;

    //float                   _raw_press;
    //unsigned long            _raw_temp;
    // Internal calibration registers
    uint16_t                        C1,C2,C3,C4,C5,C6;
    uint32_t                           D1,D2;
    
     
    /* Asynchronous handler functions: */
    //static void                     _update();
    /* Asynchronous state: */
    //static  boolean            _updated;
    //static  uint8_t         _d1_count;
    //static  uint8_t         _d2_count;
    static  uint32_t        _s_D1, _s_D2;
    static uint8_t                  _state;
   // static uint32_t                 _timer;
    /* Gates access to asynchronous state: */
   
    //static boolean                     _sync_access;

    /* Serial wrapper functions: */
    static uint8_t                  _spi_read(uint8_t reg);
   static uint16_t                 _spi_read_16bits(uint8_t reg);
   static uint32_t                 _spi_read_adc();
    static void                     _spi_write(uint8_t reg);
  // calibrate the barometer. This must be called on startup if the
    // altitude/climb_rate/acceleration interfaces are ever used
    // the callback is a delay() like routine
      //void        calibrate();

    // get current altitude in meters relative to altitude at the time
    // of the last calibrate() call
    //float        get_altitude(void);

    // return how many pressure samples were used to obtain
    // the last pressure reading
  /*  uint8_t        get_pressure_samples(void) {
        return _pressure_samples;
    } */

    // get current climb rate in meters/s. A positive number means
    // going up
 //   float           get_climb_rate(void);

    // the ground values are only valid after calibration
 /*   float           get_ground_temperature(void) {
        return _ground_temperature;
    }
    float           get_ground_pressure(void) {
        return _ground_pressure;
    }
*/
    // get last time sample was taken
  //  uint32_t        get_last_update() { return _last_update; };


   uint8_t _spi_read(uint8_t reg)
{
    uint8_t return_value;
    uint8_t addr = reg; // | 0x80; // Set most significant bit
    digitalWrite(MS5611_CS, LOW);
    SPI.transfer(addr); // discarded
    return_value = SPI.transfer(0);
    digitalWrite(MS5611_CS, HIGH);
    return return_value;
}

uint16_t _spi_read_16bits(uint8_t reg)
{
    uint8_t byteH, byteL;
    uint16_t return_value;
    uint8_t addr = reg; // | 0x80; // Set most significant bit
    digitalWrite(MS5611_CS, LOW);
    SPI.transfer(addr); // discarded
    byteH = SPI.transfer(0);
    byteL = SPI.transfer(0);
    digitalWrite(MS5611_CS, HIGH);
    return_value = ((uint16_t)byteH<<8) | (byteL);
    return return_value;
}

uint32_t _spi_read_adc()
{
    uint8_t byteH,byteM,byteL;
    uint32_t return_value;
    uint8_t addr = 0x00;
    digitalWrite(MS5611_CS, LOW);
    SPI.transfer(addr); // discarded
    byteH = SPI.transfer(0);
    byteM = SPI.transfer(0);
    byteL = SPI.transfer(0);
    digitalWrite(MS5611_CS, HIGH);
    return_value = (((uint32_t)byteH)<<16) | (((uint32_t)byteM)<<8) | (byteL);
    return return_value;
}


void _spi_write(uint8_t reg)
{
    digitalWrite(MS5611_CS, LOW);
    SPI.transfer(reg); // discarded
    digitalWrite(MS5611_CS, HIGH);
}

// Public Methods //////////////////////////////////////////////////////////////
// SPI should be initialized externally
void requestRawTemperature()
{ _spi_write(CMD_CONVERT_D2_OSR4096);
}

unsigned long readRawTemperature() {
D2 = _spi_read_adc();
_state=1;
// we need to disable interrupts to access
// _s_D1 and _s_D2 as they are not atomic
  uint8_t oldSREG = SREG;
  cli();
  MS5611lastRawTemperature = D2;
  SREG = oldSREG;
/*  long dT     = MS5611lastRawTemperature - (((long)C5) << 8);
  MS5611_offset  = (((int64_t)C2) << 16) + ((C4 * dT) >> 7);
  MS5611_sens    = (((int64_t)C1) << 15) + ((C3 * dT) >> 8);
   //Harshal: code if temp <20 degree centigrade isnot yet ported
  */
   // Formulas from manufacturer datasheet
    // sub -20c temperature compensation is not included

    // we do the calculations using floating point
    // as this is much faster on an AVR2560, and also allows
    // us to take advantage of the averaging of D1 and D1 over
    // multiple samples, giving us more precision
    dT = D2-(((uint32_t)C5)<<8);
    TEMP = (dT * C6)/8388608;
    Off = C2 * 65536.0 + (C4 * dT) / 128;
    SENS = C1 * 32768.0 + (C3 * dT) / 256;

  /*  if (TEMP < 0) {
        // second order temperature compensation when under 20 degrees C
        float T2 = (dT*dT) / 0x80000000;
        float Aux = TEMP*TEMP;
        float OFF2 = 2.5*Aux;
        float SENS2 = 1.25*Aux;
        TEMP = TEMP - T2;
        Off = Off - OFF2;
        SENS = SENS - SENS2;
    } */

    
    Temp = (TEMP + 2000)/100;
   
    
   return MS5611lastRawTemperature;	
}
float readTemperature()
{  
  //return ((1<<5)*2000 + (((MS5611lastRawTemperature - ((int64_t)C5 << 8)) * C6) >> (23-5))) / ((1<<5) * 100.0);
return Temp;
}

void requestRawPressure()
{_spi_write(CMD_CONVERT_D1_OSR4096);
}
float readRawPressure() {
 D1 = _spi_read_adc();
_state=0;
    uint8_t oldSREG = SREG;
    cli();
	MS5611lastRawPressure = D1;
     
    SREG = oldSREG;
	P1 = (MS5611lastRawPressure*SENS/2097152 - Off)/32768;
	Press = P1;
    
	return Press;
	//return (((( MS5611lastRawPressure * MS5611_sens) >> 21) - MS5611_offset) >> (15-5)) / ((float)(1<<5));


}




bool baroGroundUpdateDone = false;
unsigned long baroStartTime;

void initializeBaro() 
{   baroStartTime = micros();
    pressure = 0;
    baroGroundAltitude = 0;
    pressureFactor = 1/5.255;
    uint32_t temp=0;
    pinMode(MS5611_CS, OUTPUT);          // Chip select Pin
    digitalWrite(MS5611_CS, HIGH);
    delay(1);

    _spi_write(CMD_MS5611_RESET);
    delay(4);

    // We read the factory calibration
    // The on-chip CRC is not used
    C1 = _spi_read_16bits(CMD_MS5611_PROM_C1);
    C2 = _spi_read_16bits(CMD_MS5611_PROM_C2);
    C3 = _spi_read_16bits(CMD_MS5611_PROM_C3);
    C4 = _spi_read_16bits(CMD_MS5611_PROM_C4);
    C5 = _spi_read_16bits(CMD_MS5611_PROM_C5);
    C6 = _spi_read_16bits(CMD_MS5611_PROM_C6);


    _state = 0;
   

    _s_D1 = 0;
    _s_D2 = 0;
   

    vehicleState |= BARO_DETECTED;//find proper condition
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
 
// Read the sensor. This is a state machine
// We read one time Temperature (state=1) and then 4 times Pressure (states 2-5)
// temperature does not change so quickly...


// Calculate Temperature and compensated Pressure in real units (Celsius degrees*100, mbar*100).




/***********************************************************************/
/*// calibrate the barometer. This must be called at least once before
// the altitude() or climb_rate() interfaces can be used
void calibrate()
{
    float ground_pressure = 0;
    float ground_temperature = 0;

    while (ground_pressure == 0) {
        readBaro();         // Get initial data from absolute pressure sensor
        ground_pressure         = get_pressure();
        ground_temperature      = get_temperature();
        delay(20);
    }
    // let the barometer settle for a full second after startup
    // the MS5611 reads quite a long way off for the first second,
    // leading to about 1m of error if we don't wait
    for (uint16_t i = 0; i < 10; i++) {
        do {
            readBaro();
        } while (!healthy);
        ground_pressure         = get_pressure();
        ground_temperature      = get_temperature();
        delay(100);
    }

    // now average over 5 values for the ground pressure and
    // temperature settings
    for (uint16_t i = 0; i < 5; i++) {
        do {
            readBaro();
        } while (!healthy);
        ground_pressure         = ground_pressure * 0.8     + get_pressure() * 0.2;
        ground_temperature      = ground_temperature * 0.8  + get_temperature() * 0.2;
        delay(100);
    }

    _ground_pressure=(ground_pressure);
    _ground_temperature=(ground_temperature / 10.0f);
}

// return current altitude estimate relative to time that calibrate()
// was called. Returns altitude in meters
// note that this relies on read() being called regularly to get new data

float get_altitude(void)
{
    float scaling, temp;

    if (_last_altitude_t == _last_update) {
        // no new information
        return _altitude;
    }

    // this has no filtering of the pressure values, use a separate
    // filter if you want a smoothed value. The AHRS driver wants
    // unsmoothed values
    scaling                                 = (float)_ground_pressure / (float)get_pressure();
    temp                                    = ((float)_ground_temperature) + 273.15f;
    _altitude = log(scaling) * temp * 29.271267f;
    _last_altitude_t = _last_update;

    // ensure the climb rate filter is updated
  //  _climb_rate_filter.update(_altitude, _last_update);

    return _altitude;
}

// return current climb_rate estimeate relative to time that calibrate()
// was called. Returns climb rate in meters/s, positive means up
// note that this relies on readBaro() being called regularly to get new data
float get_climb_rate(void)
{
    // we use a 7 point derivative filter on the climb rate. This seems
    // to produce somewhat reasonable results on real hardware
   // return _climb_rate_filter.slope() * 1.0e3;
}

/*****************************************************************************/

   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
#endif //  __AP_BARO_MS5611_H__
