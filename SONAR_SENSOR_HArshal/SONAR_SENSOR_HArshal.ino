#include <string.h>

void setup()
{
  // start serial port at 9600 bps:
  Serial2.begin(9600);
  Serial.begin(9600);
    
}

int i=0;
char received_serial_bytes[6]={},serial_char=0;
float distanceInMeter=0.0;

void loop()
{  
   if(Serial2.available()) {
    
    serial_char=Serial2.read(); 
    if(serial_char==13)
      i=0;
    else 
     {
    if(serial_char=='E'||serial_char=='R')
     {
       distanceInMeter=4;
       i++;
       //if(i%4==3)
      // {Serial.print(distanceInMeter);
       // Serial.println(" ,");} 
     }
    else
    { 
      received_serial_bytes[i]=serial_char; 
      i++;
     if(i%6==5)
     {
      received_serial_bytes[5]='\0';
      distanceInMeter= atof(received_serial_bytes)/100;
      Serial.print(",Data_Read= ");
      Serial.print(received_serial_bytes);
      Serial.print(" ,Distinmeter=");
      Serial.print(distanceInMeter);
      Serial.println(" ,");  
   }    
    
   } 
    
      
    
 }
  
  
  //delay(50);
}
}


