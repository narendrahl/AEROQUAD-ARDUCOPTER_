/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BARO_MS5611_H__
#define __AP_BARO_MS5611_H__
//#include <Filter.h>
//#include <DerivativeFilter.h>
//#include "../AP_PeriodicProcess/AP_PeriodicProcess.h"
#include <Serial.h>
#include <SPI.h>

//#include <AP_Common.h>
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
   
   boolean healthy;
    /* AP_Baro public interface: */
    
    uint8_t         readBaro();
    float           get_pressure(); // in mbar*100 units
    float           get_temperature(); // in celsius degrees * 100 units

    int32_t         get_raw_pressure();
    int32_t         get_raw_temp();

   void            _calculate();
    
    
    uint32_t                            _last_update;
    uint8_t                             _pressure_samples;


    float                            _ground_temperature;
    float                            _ground_pressure;
    float                               _altitude;
    uint32_t                            _last_altitude_t;
   // DerivativeFilterFloat_Size7         _climb_rate_filter;

    float                           Temp;
    float                           Press;

    int32_t                         _raw_press;
    int32_t                         _raw_temp;
    // Internal calibration registers
    uint16_t                        C1,C2,C3,C4,C5,C6;
    float                           D1,D2;
    

    /* Asynchronous handler functions: */
    static void                     _update();
    /* Asynchronous state: */
    static  boolean            _updated;
    static  uint8_t         _d1_count;
    static  uint8_t         _d2_count;
    static  uint32_t        _s_D1, _s_D2;
    static uint8_t                  _state;
    static uint32_t                 _timer;
    /* Gates access to asynchronous state: */
   
    static boolean                     _sync_access;

    /* Serial wrapper functions: */
    static uint8_t                  _spi_read(uint8_t reg);
   static uint16_t                 _spi_read_16bits(uint8_t reg);
   static uint32_t                 _spi_read_adc();
    static void                     _spi_write(uint8_t reg);
  // calibrate the barometer. This must be called on startup if the
    // altitude/climb_rate/acceleration interfaces are ever used
    // the callback is a delay() like routine
      void        calibrate();

    // get current altitude in meters relative to altitude at the time
    // of the last calibrate() call
    float        get_altitude(void);

    // return how many pressure samples were used to obtain
    // the last pressure reading
    uint8_t        get_pressure_samples(void) {
        return _pressure_samples;
    }

    // get current climb rate in meters/s. A positive number means
    // going up
 //   float           get_climb_rate(void);

    // the ground values are only valid after calibration
    float           get_ground_temperature(void) {
        return _ground_temperature;
    }
    float           get_ground_pressure(void) {
        return _ground_pressure;
    }

    // get last time sample was taken
    uint32_t        get_last_update() { return _last_update; };


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
boolean initBaro()
{   uint32_t temp=0;
    
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


    //Send a command to read Temp first
    _spi_write(CMD_CONVERT_D2_OSR4096);
    _timer = micros();
    _state = 0;
    Temp=0;
    Press=0;

    _s_D1 = 0;
    _s_D2 = 0;
    _d1_count = 0;
    _d2_count = 0;

   
    
    // wait for at least one value to be read
     

   /* while (!_updated){
    if(micros()-temp>1000)
    {temp=micros();
    _update();}}  // Harshal;After alot of efforts this is ingenius solution :D
     */
     
    healthy = true;
    return true;
}


// Read the sensor. This is a state machine
// We read one time Temperature (state=1) and then 4 times Pressure (states 2-5)
// temperature does not change so quickly...
void _update()     
{
    // Throttle read rate to 100hz maximum.
    // note we use 9500us here not 10000us
    // the read rate will end up at exactly 100hz because the Periodic Timer fires at 1khz
    /*if (tnow - _timer < 9500) {
        return;
   }

    _timer = tnow; */

    if (_state == 0) {
        _s_D2 += _spi_read_adc();                                // On state 0 we read temp
        _d2_count++;
        if (_d2_count == 80) {
            // we have summed 32 values. This only happens
            // when we stop reading the barometer for a long time
            // (more than 1.2 seconds)
            _s_D2 >>= 1;
            _d2_count = 40;
        }
        _state=1;
        _spi_write(CMD_CONVERT_D2_OSR4096); // Command to read temperature
    } else {
        _s_D1 += _spi_read_adc();
        _d1_count++;
        if (_d1_count == 80) {
            // we have summed 128 values. This only happens
            // when we stop reading the barometer for a long time
            // (more than 1.2 seconds)
            _s_D1 >>= 1;
            _d1_count = 40;
        }
        _state=0;
        
        _spi_write(CMD_CONVERT_D1_OSR4096);      // Command to read pressure
        _updated = true;               // New pressure reading                                                  
        /*if (_state == 5) {
            _spi_write(CMD_CONVERT_D2_OSR4096); // Command to read temperature
            _state = 0;
        } else {
            _spi_write(CMD_CONVERT_D1_OSR4096); // Command to read pressure
        }*/
    }

}

uint8_t readBaro()
{
    boolean updated = _updated;
    if (updated) {
        uint32_t sD1, sD2;
        uint8_t d1count, d2count;
        // we need to disable interrupts to access
        // _s_D1 and _s_D2 as they are not atomic
        uint8_t oldSREG = SREG;
        cli();
        sD1 = _s_D1; _s_D1 = 0;
        sD2 = _s_D2; _s_D2 = 0;
        d1count = _d1_count; _d1_count = 0;
        d2count = _d2_count; _d2_count = 0;
        _updated = false;
        SREG = oldSREG;
        if (d1count != 0) {
            D1 = ((float)sD1) / d1count;
        }
        if (d2count != 0) {
            D2 = ((float)sD2) / d2count;
        }
        _pressure_samples = d1count;
        _raw_press = D1;
        _raw_temp = D2;
    }
    _calculate();
    if (updated) {
        _last_update = millis();
    }
    return updated ? 1 : 0;
}

// Calculate Temperature and compensated Pressure in real units (Celsius degrees*100, mbar*100).
void _calculate()
{
    float dT;
    float TEMP;
    float OFF;
    float SENS;
    float P;

    // Formulas from manufacturer datasheet
    // sub -20c temperature compensation is not included

    // we do the calculations using floating point
    // as this is much faster on an AVR2560, and also allows
    // us to take advantage of the averaging of D1 and D1 over
    // multiple samples, giving us more precision
    dT = D2-(((uint32_t)C5)<<8);
    TEMP = (dT * C6)/8388608;
    OFF = C2 * 65536.0 + (C4 * dT) / 128;
    SENS = C1 * 32768.0 + (C3 * dT) / 256;

    if (TEMP < 0) {
        // second order temperature compensation when under 20 degrees C
        float T2 = (dT*dT) / 0x80000000;
        float Aux = TEMP*TEMP;
        float OFF2 = 2.5*Aux;
        float SENS2 = 1.25*Aux;
        TEMP = TEMP - T2;
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    P = (D1*SENS/2097152 - OFF)/32768;
    Temp = TEMP + 2000;
    Press = P;
}

float get_pressure()
{
    return Press;
}

float get_temperature()
{
    // callers want the temperature in 0.1C units
    return Temp/10;
}

int32_t get_raw_pressure() {
    return _raw_press;
}

int32_t get_raw_temperature() {
    return _raw_temp;
}


/***********************************************************************/
// calibrate the barometer. This must be called at least once before
// the altitude() or climb_rate() interfaces can be used
void calibrate()
{
    float ground_pressure = 0;
    float ground_temperature = 0;

    while (ground_pressure == 0) {
         _update();
          delay(10);
         _update();
        readBaro();         // Get initial data from absolute pressure sensor
        ground_pressure         = get_pressure();
        ground_temperature      = get_temperature();
        delay(20);
   Serial.print("_ground_pressure=");
    Serial.print(ground_pressure);
    Serial.print("_ground_temperature=");
    Serial.println(ground_temperature);
     
   }
    // let the barometer settle for a full second after startup
    // the MS5611 reads quite a long way off for the first second,
    // leading to about 1m of error if we don't wait
    for (uint16_t i = 0; i < 10; i++) {
        _update();
        delay(10);
        _update();
        readBaro();
        ground_pressure         = get_pressure();
        ground_temperature      = get_temperature();
        delay(90);
  Serial.print("_ground_pressure=");
    Serial.print(ground_pressure);
    Serial.print("_ground_temperature=");
    Serial.println(ground_temperature);
      
  }

    // now average over 5 values for the ground pressure and
    // temperature settings
    for (uint16_t i = 0; i < 5; i++) {
        _update();
        delay(10);
        _update();
        readBaro();
        ground_pressure         = ground_pressure * 0.8     + get_pressure() * 0.2;
        ground_temperature      = ground_temperature * 0.8  + get_temperature() * 0.2;
        delay(90);
  Serial.print("_ground_pressure=");
    Serial.print(ground_pressure);
    Serial.print("_ground_temperature=");
    Serial.println(ground_temperature);
      
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
