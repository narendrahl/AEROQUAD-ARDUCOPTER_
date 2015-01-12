#include "MPU6000.h"
//#include "Platform_MPU6000.h"
void setup()
{
 MPU6000_Init();
 Serial.begin(115200);

}

void loop()
{
  
 MPU6000_Read();
 
}
