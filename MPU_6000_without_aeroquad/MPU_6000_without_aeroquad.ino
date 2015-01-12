#include "MPU6000.h"

#include <SPI.h>
void setup()
{
 
 MPU6000_Init();
 Serial.begin(115200);

}

void loop()
{
  
 MPU6000_Read();
 Serial.print("accelX=");
 Serial.print(accelX);
 Serial.print(" accelY=");
 Serial.print(accelY);
 Serial.print(" accelZ=");
 Serial.print(accelZ);
 Serial.print(" gyroX=");
 Serial.print(gyroX);
 Serial.print(" gyroY=");
 Serial.print(gyroY);
 Serial.print(" gyroZ=");
 Serial.println(gyroZ);
 
 
}
