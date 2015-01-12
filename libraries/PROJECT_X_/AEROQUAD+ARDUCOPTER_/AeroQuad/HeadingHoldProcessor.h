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


#ifndef _AQ_HEADING_CONTROL_PROCESSOR_H_
#define _AQ_HEADING_CONTROL_PROCESSOR_H_

#include <HeadingFusionProcessorCompFilter.h>

float setHeading          = 0;
unsigned long headingTime = micros();
int z = 0;

/**
 * processHeading
 *
 * This function will calculate the craft heading correction depending 
 * of the users command. Heading correction is process with the gyro
 * or a magnetometer
 */
void processHeading()
{
  
  
  if (headingHoldConfig == ON) {

    #if defined(HeadingMagHold)
    
/*   if(navigationState == ON)
    {
      #ifdef Follower
      heading = degrees(angleToWaypoint);
      #endif
      #ifdef Leader
      heading = degrees(trueNorthHeading);
      #endif
    }
    else
    {
      heading = degrees(trueNorthHeading);
    } */                                           // just a thougt added by indrajit....which can be tried.
      heading = degrees(trueNorthHeading);
    #else
      heading = degrees(gyroHeading);
    #endif

    // Always center relative heading around absolute heading chosen during yaw command
    // This assumes that an incorrect yaw can't be forced on the AeroQuad >180 or <-180 degrees
    // This is done so that AeroQuad does not accidentally hit transition between 0 and 360 or -180 and 180
    // AKA - THERE IS A BUG HERE - if relative heading is greater than 180 degrees, the PID will swing from negative to positive
    // Doubt that will happen as it would have to be uncommanded.

    relativeHeading = heading - setHeading;
    if (heading <= (setHeading - 180)) {
      relativeHeading += 360;
    }
    if (heading >= (setHeading + 180)) {
      relativeHeading -= 360;
    }



    // Apply heading hold only when throttle high enough to start flight
    if (receiverCommand[THROTTLE] > MINCHECK ) { 
      
      #if defined (UseGPSNavigator)
//            if ( (receiverCommand[AUX2] > 1240 && receiverCommand[AUX2] > 1750) || (receiverCommand[AUX1] < 1100 ) )
//      if ( (receiverCommand[AUX1] < 1100 ) )
             {//Serial2.print('W');
       
//        gpsYawAxisCorrection = 0;
        //Serial2.print(receiverCommand[ZAXIS]);
             }     //if(>MINCHECK)
      
        if (( (receiverCommand[ZAXIS] + gpsYawAxisCorrection) > (MIDCOMMAND + 25 )) || 
            ( (receiverCommand[ZAXIS] + gpsYawAxisCorrection) < (MIDCOMMAND - 25 ))) {
              
      /*        if(z==20)
              {
                Serial2.print('Y');
                Serial2.print(':');
                Serial2.print(degrees(trueNorthHeading));
                Serial2.print(':');
                Serial2.print(heading);
                Serial2.print(':');
                Serial2.print(relativeHeading);
                Serial2.print(':');}*/
      #else
//        if ((receiverCommand[ZAXIS] > (MIDCOMMAND + 25)) || 
//            (receiverCommand[ZAXIS] < (MIDCOMMAND - 25))) { 
      #endif
      
     // Serial2.print('R');
        
        // If commanding yaw, turn off heading hold and store latest heading
        setHeading = heading;
        headingHold = 0;
        PID[HEADING_HOLD_PID_IDX].integratedError = 0;
        headingHoldState = OFF;
        headingTime = currentTime;
      }
      else {                                               ///////////////////////      if receiverCommand[ZAXIS] > MIDCOMMAND + 25
       //Serial2.print('X');
        //Serial2.print(degrees(trueNorthHeading));
        //Serial2.print(':');
        //Serial2.print(relativeHeading);
        //Serial2.print(':');
 //       Serial2.print('N');
   //     Serial2.print(':');
        //Serial2.print(setHeading);
        
        if (relativeHeading < 0.25 && relativeHeading > -0.25) {
         //Serial2.print('Q');
          headingHold = 0;
          PID[HEADING_HOLD_PID_IDX].integratedError = 0;
        }
        else if (headingHoldState == OFF) { // quick fix to soften heading hold on new heading
             //Serial2.print("D");
          if ((currentTime - headingTime) > 500000) {
     //       Serial2.print("Z");
            headingHoldState = ON;
            headingTime = currentTime;
            setHeading = heading;
            headingHold = 0;
          }
        }   ////  else if headingHoldState == OFF ends here 
        else {
          //Serial2.print("Y");
        // No new yaw input, calculate current heading vs. desired heading heading hold
        // Relative heading is always centered around zero
            if ((receiverCommand[AUX2] > 1240 && receiverCommand[AUX2] < 1750) || (receiverCommand[AUX2] > 900 && receiverCommand[AUX2] < 1100)) { 
            //  //Serial2.print('F');
             headingHold = updatePID(0, heading, &PID[HEADING_HOLD_PID_IDX]);
           }
            else {
           
             headingHold = updatePID(0, relativeHeading, &PID[HEADING_HOLD_PID_IDX]);
    //         Serial2.print(headingHold);
  //           Serial2.print(':');
    //         Serial2.print('C');
             
            }
           //Serial2.print('J');
           
          headingTime = currentTime; // quick fix to soften heading hold, wait 100ms before applying heading hold
        }                         ///  else ends here 
      }
    }
    else {
      // minimum throttle not reached, use off settings
      //Serial2.print('M');
      setHeading = heading;
      headingHold = 0;
      PID[HEADING_HOLD_PID_IDX].integratedError = 0;
    }
  }
  // NEW SI Version
  #if defined (UseGPSNavigator) 
    float receiverSiData = (receiverCommand[ZAXIS] - receiverZero[ZAXIS] + gpsYawAxisCorrection) * (2.5 * PWM2RAD);
  #else
    float receiverSiData = (receiverCommand[ZAXIS] - receiverZero[ZAXIS]) * (2.5 * PWM2RAD);
  #endif
  

  const float commandedYaw = constrain(receiverSiData + radians(headingHold), -PI, PI);
  
//const float commandedYaw = constrain(receiverSiData, -PI, PI);
  
 
motorAxisCommandYaw = updatePID(commandedYaw, gyroRate[ZAXIS], &PID[ZAXIS_PID_IDX]);

/*if(z==20)
{
  Serial2.print('M');
  Serial2.print(':');
  
  Serial2.print(motorAxisCommandYaw);
  Serial2.print(':');
  
  Serial2.print(commandedYaw);
  Serial2.print(':');
  
  Serial2.print(receiverCommand[ZAXIS]);
  Serial2.print(':');
 
 
  z = 1;
}
else {
z = z+1;}*/

//headingHold = constrain(radians(headingHold),-PI,PI);

//motorAxisCommandYaw += updatePID(0.0 , headingHold, &PID[ZAXIS_PID_IDX]);


//Serial2.print(motorAxisCommandYaw);
/*
Serial2.print('X');
Serial2.print(degrees(trueNorthHeading));
Serial2.print(':');
Serial2.print(relativeHeading);
Serial2.print(':');
Serial2.print(headingHold);
Serial2.print(':');
Serial2.print(commandedYaw);
Serial2.println();*/

}

#endif






