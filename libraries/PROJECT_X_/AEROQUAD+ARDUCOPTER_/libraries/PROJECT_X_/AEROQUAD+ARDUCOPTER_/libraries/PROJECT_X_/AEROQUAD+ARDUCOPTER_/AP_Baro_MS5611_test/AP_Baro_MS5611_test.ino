
#include <stdint.h>
#include <Serial.h>
//#include <AP_Common.h>
//#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
//#include <I2C.h>
#include <SPI.h>
//#include <Filter.h>
//#include <AP_Buffer.h>
//#include <Arduino_Mega_ISR_Registry.h>
//#include <AP_PeriodicProcess.h>
#include "AP_Baro_MS5611.h" // ArduPilot Mega ADC Library





unsigned long timer,temp=0;

void setup()
{
    Serial.begin(115200);
    Serial.println("ArduPilot Mega MeasSense Barometer library test");

    delay(1000);

   
 //   pinMode(63, OUTPUT);
 //  digitalWrite(63, HIGH);
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV32);     // 500khz for debugging, increase later
    
    initBaro();
   //Serial.println("Baro initialised.....");
    calibrate();
   // Serial.println("Baro Calibrated.....");
    timer = millis();
}

void loop()
{  /*if(millis()-(temp)>10)
   { temp=millis();
    _update();}
    */

    if((millis() - timer) > 10UL) {
        timer = millis();
        _update();
       
       
        readBaro();
        //uint32_t read_time = micros() - timer;
        if (!healthy) {
            Serial.println("not healthy");
            return;
        }
        Serial.print("Pressure:");
        Serial.print(get_pressure());
        Serial.print(" Temperature:");
        Serial.print(get_temperature());
        Serial.print(" Altitude:");
        Serial.print(get_altitude());
        Serial.print(" climb=");
        Serial.print(get_climb_rate());
        Serial.print(" samples=");        
        Serial.println((unsigned)get_pressure_samples());
        
    }
}
