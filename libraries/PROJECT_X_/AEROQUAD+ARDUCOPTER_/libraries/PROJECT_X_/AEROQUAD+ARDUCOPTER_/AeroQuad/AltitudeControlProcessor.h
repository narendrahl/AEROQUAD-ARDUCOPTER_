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

// FlightControl.pde is responsible for combining sensor measurements and
// transmitter commands into motor commands for the defined flight configuration (X, +, etc.)
//////////////////////////////////////////////////////////////////////////////
/////////////////////////// calculateFlightError /////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifndef _AQ_ALTITUDE_CONTROL_PROCESSOR_H_
#define _AQ_ALTITUDE_CONTROL_PROCESSOR_H_


#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder

#define INVALID_THROTTLE_CORRECTION -1000
#define ALTITUDE_BUMP_SPEED 0.01
#define BaroAdjustValue 0.1
int i =0 ;
float curr_alt = 0.0 , prev_alt = 0.0;
float a2= 0.0 , b2= 0.0 , c2= 0.0 , d2= 0.0 , e2 = 0.0, f2=0.0, g2=0.0, h2=0.0, i2=0.0, j2=0.0;


/**
 * processAltitudeHold
 * 
 * This function is responsible to process the throttle correction 
 * to keep the current altitude if selected by the user 
 */
void processAltitudeHold()
{


  // ****************************** Altitude Adjust *************************
  // Thanks to Honk for his work with altitude hold
  // http://aeroquad.com/showthread.php?792-Problems-with-BMP085-I2C-barometer
  // Thanks to Sherbakov for his work in Z Axis dampening
  // http://aeroquad.com/showthread.php?359-Stable-flight-logic...&p=10325&viewfull=1#post10325
//int alt_flag;
   
  if ((altitudeHoldState == ON) || (positionHoldState == ON)) {                  // Sudeep Removed this to test altitude hold 03-05
    
    int altitudeHoldThrottleCorrection = INVALID_THROTTLE_CORRECTION;
    
    
    // computer altitude error!
    #if defined AltitudeHoldRangeFinder
      if (isOnRangerRange(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX])) {
        if (sonarAltitudeToHoldTarget == INVALID_RANGE) {
          sonarAltitudeToHoldTarget = rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];
        }
        //altitudeHoldThrottleCorrection = updatePID(sonarAltitudeToHoldTarget, rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX], &PID[SONAR_ALTITUDE_HOLD_PID_IDX]);
        altitudeHoldThrottleCorrection = updatePID(sonarAltitudeToHoldTarget, rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX], &PID[BARO_ALTITUDE_HOLD_PID_IDX]);
        altitudeHoldThrottleCorrection = constrain(altitudeHoldThrottleCorrection, minThrottleAdjust, maxThrottleAdjust);
        altitudeHoldCorrectionSudeep = altitudeHoldThrottleCorrection;
      }
    #endif
    #if defined AltitudeHoldBaro
      if (altitudeHoldThrottleCorrection == INVALID_THROTTLE_CORRECTION) {
        
        if(gps_alt ==1)
        {
          a2=getBaroAltitude();
          b2=getBaroAltitude();
          c2=getBaroAltitude();
          d2=getBaroAltitude();
          e2=getBaroAltitude();
          f2=getBaroAltitude();
          g2=getBaroAltitude();
          h2=getBaroAltitude();
          i2=getBaroAltitude();
          j2=getBaroAltitude();
          gps_alt=2;
               }
          
       a2=b2;
       b2=c2;
       c2=d2;
       d2=e2;
       e2=f2;
       f2=g2;
       g2=h2;
       h2=i2;
       i2=j2;
       j2=getBaroAltitude();       
       curr_alt = (a2+b2+c2+d2+e2+f2+g2+h2+i2+j2)/10;
       if(abs(prev_alt - curr_alt) < BaroAdjustValue)
        curr_alt = prev_alt;
       else 
       prev_alt = curr_alt;
       
        altitudeHoldThrottleCorrection = updatePID(baroAltitudeToHoldTarget, curr_alt, &PID[BARO_ALTITUDE_HOLD_PID_IDX]);
        altitudeHoldThrottleCorrection = constrain(altitudeHoldThrottleCorrection, minThrottleAdjust, maxThrottleAdjust);
   //     altitudeHoldThrottleCorrection = constrain(altitudeHoldThrottleCorrection, -30, 30);
 //     Serial2.print(':');
 //     Serial2.print('A');
//      Serial2.print(altitudeHoldThrottleCorrection);
 //     Serial2.print(':');
//       
        
        //if (-0.11 < (getBaroAltitude() - baroAltitudeToHoldTarget) && (getBaroAltitude() - baroAltitudeToHoldTarget) < 0) {    // Dead band added by Sudeep
            //altitudeHoldThrottleCorrection = 0;}
        //else if ( (Altitude() - baroAltitudeToHoldTarget) < 0.11 && (getBaroAltitude() - baroAltitudeToHoldTarget) > 0) {    // Dead band added by Sudeep
            //altitudeHoldThrottleCorrection = 0;}
         //Serial2.print(altitudeHoldThrottleCorrection);   
        altitudeHoldCorrectionSudeep = altitudeHoldThrottleCorrection;          // Added a variable to check the correction Sudeep
      }
    #endif       
    if (altitudeHoldThrottleCorrection == INVALID_THROTTLE_CORRECTION) {
      throttle = receiverCommand[THROTTLE];
      return;
    }
    
    // ZDAMPENING COMPUTATIONS
    #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
       
       float zDampeningThrottleCorrection ;
       if ( abs(baroAltitudeToHoldTarget - getBaroAltitude()) < BaroAdjustValue)
        {
             
            zDampeningThrottleCorrection = 0;  
        }
        
      else {
        
      zDampeningThrottleCorrection = -updatePID(0.0, estimatedZVelocity, &PID[ZDAMPENING_PID_IDX]);
      zDampeningThrottleCorrection = constrain(zDampeningThrottleCorrection, minThrottleAdjust, maxThrottleAdjust);

      }
    #endif

    
    if (abs(altitudeHoldThrottle - receiverCommand[THROTTLE]) > altitudeHoldPanicStickMovement) {
      altitudeHoldState = ALTPANIC; // too rapid of stick movement so PANIC out of ALTHOLD
    } 
    else {
      
      if (receiverCommand[THROTTLE] > (altitudeHoldThrottle + altitudeHoldBump)) { // AKA changed to use holdThrottle + ALTBUMP - (was MAXCHECK) above 1900
        #if defined AltitudeHoldBaro
      //    baroAltitudeToHoldTarget += ALTITUDE_BUMP_SPEED;
    //      Serial2.print('C');
        #endif
        #if defined AltitudeHoldRangeFinder
          float newalt = sonarAltitudeToHoldTarget + ALTITUDE_BUMP_SPEED;
          if (isOnRangerRange(newalt)) {
            sonarAltitudeToHoldTarget = newalt;
          }
        #endif
      }
      
      if (receiverCommand[THROTTLE] < (altitudeHoldThrottle - altitudeHoldBump)) { // AKA change to use holdThorrle - ALTBUMP - (was MINCHECK) below 1100
        #if defined AltitudeHoldBaro
   //     Serial2.print('C');
    //      baroAltitudeToHoldTarget -= ALTITUDE_BUMP_SPEED;
        #endif
        #if defined AltitudeHoldRangeFinder
          float newalt = sonarAltitudeToHoldTarget - ALTITUDE_BUMP_SPEED;
          if (isOnRangerRange(newalt)) {
            sonarAltitudeToHoldTarget = newalt;
          }
        #endif
      }
    }
    //Serial2.print(getBaroAltitude());
    //Serial2.print(baroAltitudeToHoldTarget);
    //Serial2.print(zDampeningThrottleCorrection);
    throttle = altitudeHoldThrottle + altitudeHoldThrottleCorrection  + zDampeningThrottleCorrection;

/*
    Serial2.print(baroAltitudeToHoldTarget);
    Serial2.print(':');
    Serial2.print(getBaroAltitude());
    Serial2.print(':');
    Serial2.print(motorAxisCommandRoll);
    Serial2.print(':');
    Serial2.print(motorAxisCommandPitch);
    Serial2.print(':');
    Serial2.print(curr_alt);
    Serial2.print(':');
 //   Serial2.print(missionPositionToReach.longitude);
 //   Serial2.print(':');
    Serial2.print(altitudeHoldCorrectionSudeep);
    Serial2.print(':');
    Serial2.print(gpsRollAxisCorrection);
    Serial2.print(':');
    Serial2.print(gpsPitchAxisCorrection);
    Serial2.print(':');
    Serial2.print(throttle);
    Serial2.print(':');
*/



/*    if(i>50)
    {
    Serial2.print(curr_alt);
    Serial2.print(":");
    Serial2.print(baroAltitudeToHoldTarget);
    Serial2.print(":");
    Serial2.print(throttle);
    Serial2.print(":");
    Serial2.print(altitudeHoldThrottleCorrection);
    Serial2.print(":");
    Serial2.print(getBaroAltitude());
    Serial2.print(":");
    Serial2.print(currentPosition.latitude);
    Serial2.print(":");
    Serial2.print(positionHoldPointToReach.latitude);
    Serial2.print(":");
    Serial2.println();
    i=0;
    }
    i++;
*/

//    Serial2.println();


    //throttle = constrain(altitudeHoldThrottle, altitudeHoldThrottle-80, altitudeHoldThrottle+80);
  }
  else {
    throttle = receiverCommand[THROTTLE];
  }
  }

#endif

#endif // _AQ_ALTITUDE_CONTROL_PROCESSOR_H_
