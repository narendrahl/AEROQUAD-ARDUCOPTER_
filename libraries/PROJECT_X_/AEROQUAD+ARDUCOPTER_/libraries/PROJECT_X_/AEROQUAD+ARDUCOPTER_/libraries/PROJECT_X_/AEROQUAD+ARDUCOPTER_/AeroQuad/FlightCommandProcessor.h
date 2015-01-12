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

// FlightCommandProcessor is responsible for decoding transmitter stick combinations
// for setting up AeroQuad modes such as motor arming and disarming

#ifndef _AQ_FLIGHT_COMMAND_READER_
#define _AQ_FLIGHT_COMMAND_READER_

#include "AltitudeControlProcessor.h"

int FLG = 1;

#if defined (AltitudeHoldBaro) || defined (AltitudeHoldRangeFinder)
  boolean isPositionHoldEnabledByUser() {
    #if defined UseGPSNavigator
    
    if(receiverCommand[AUX1] > 1240 && receiverCommand[AUX1] < 1250)
        {
          return true; }
     if((receiverCommand[AUX2] > 1240 && receiverCommand[AUX2] < 1750) || (receiverCommand[AUX2] > 900 && receiverCommand[AUX2] < 1100)) 
      {
       return true;}
      if(receiverCommand[AUX1] > 900 && receiverCommand[AUX1] < 1100) {         
        return true;
      }
      return false;
    #else
      if ((receiverCommand[AUX1] > 1240 && receiverCommand[AUX1] < 1250) || (receiverCommand[AUX1] > 900 && receiverCommand[AUX1] < 1100)) {
        return true;
      }
      return false;
    #endif
  }
#endif

#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
  void processAltitudeHoldStateFromReceiverCommand() 
  { 
    if (isPositionHoldEnabledByUser()) {
          
      if (altitudeHoldState != ALTPANIC ) {  // check for special condition with manditory override of Altitude hold

        if (!isAltitudeHoldInitialized) {
                 
          #if defined AltitudeHoldBaro
            //baroAltitudeToHoldTarget = getBaroAltitude();
            //baroAltitudeToHoldTarget = 2.5;
            PID[BARO_ALTITUDE_HOLD_PID_IDX].integratedError = 0;
            PID[BARO_ALTITUDE_HOLD_PID_IDX].lastError = baroAltitudeToHoldTarget;
          #endif
          #if defined AltitudeHoldRangeFinder
            sonarAltitudeToHoldTarget = rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];
            PID[BARO_ALTITUDE_HOLD_PID_IDX].integratedError = 0;
            PID[BARO_ALTITUDE_HOLD_PID_IDX].lastError = sonarAltitudeToHoldTarget;
            //PID[SONAR_ALTITUDE_HOLD_PID_IDX].integratedError = 0;
            //PID[SONAR_ALTITUDE_HOLD_PID_IDX].lastError = sonarAltitudeToHoldTarget;
          #endif
     //     Serial2.print('T');
          altitudeHoldThrottle = receiverCommand[THROTTLE] + 10;  //added by indrajit 24 sep 14
          isAltitudeHoldInitialized = true;
        } ///  if(!isAltitudeHoldInitialized) Ends here ... 
        
        altitudeHoldState = ON;
       // headingHoldState = ON;       
        float Temp_barovalue= getBaroAltitude();
        //baroAltitudeToHoldTarget = getBaroAltitude();
        
        if (previousaltitudeHoldState == 0)//Added for alitutude hold (30 Sept)
        {
          //Serial2.print('X');
           baroAltitudeToHoldTarget = Temp_barovalue + 0.5;
        //baroAltitudeToHoldTarget = -1;
        altitudeHoldThrottle = receiverCommand[THROTTLE];
        
        previousaltitudeHoldState=1;
        
/*      float BaroValues = 0.0;
      for(int i =0 ; i <20; i++)
       {
         BaroValues += BaroValues + getBaroAltitude();
       }
      baroAltitudeToHoldTarget = BaroValues/20; 
*/        
     /// if (previousaltitudeHoldState ==0 )
       
        }
        
      }
    } ///   if(isPositionHoldEnabledByUser()) Ends here ....  
    else {
      //Serial2.print("Y");
      isAltitudeHoldInitialized = false;
      altitudeHoldState = OFF;
      //headingHoldState = OFF;
      previousaltitudeHoldState = 0;
      baroAltitudeToHoldTarget = 5;
    }
    
  }   /////  processAltitudeHoldStateFromReceiverCommand  Ends here ...
#endif


#if defined (AutoLanding)
  void processAutoLandingStateFromReceiverCommand() {
    if (receiverCommand[AUX3] < 1750) {
      if (altitudeHoldState != ALTPANIC ) {  // check for special condition with manditory override of Altitude hold
        if (isAutoLandingInitialized) {
          autoLandingState = BARO_AUTO_DESCENT_STATE;
          #if defined AltitudeHoldBaro
            baroAltitudeToHoldTarget = getBaroAltitude();
            PID[BARO_ALTITUDE_HOLD_PID_IDX].integratedError = 0;
            PID[BARO_ALTITUDE_HOLD_PID_IDX].lastError = baroAltitudeToHoldTarget;
          #endif
          #if defined AltitudeHoldRangeFinder
            sonarAltitudeToHoldTarget = rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];
            PID[SONAR_ALTITUDE_HOLD_PID_IDX].integratedError = 0;
            PID[SONAR_ALTITUDE_HOLD_PID_IDX].lastError = sonarAltitudeToHoldTarget;
          #endif
          altitudeHoldThrottle = receiverCommand[THROTTLE];
          isAutoLandingInitialized = true;
        }
        altitudeHoldState = ON;
      }
    }
    else {
      autoLandingState = OFF;
      autoLandingThrottleCorrection = 0;
      isAutoLandingInitialized = false;
      #if defined (UseGPSNavigator)
        if ((receiverCommand[AUX1] > 1750) && (receiverCommand[AUX2] > 1750)) {
          altitudeHoldState = OFF;
          isAltitudeHoldInitialized = false;
        }
      #else
        if (receiverCommand[AUX1] > 1750) {
          altitudeHoldState = OFF;
          isAltitudeHoldInitialized = false;
        }
      #endif
    }
  }
#endif


#if defined (UseGPSNavigator)
  void processGpsNavigationStateFromReceiverCommand() {
    // Init home command
    if (motorArmed == OFF && 
        receiverCommand[THROTTLE] < MINCHECKthrottle+10 && receiverCommand[ZAXIS] < MINCHECK &&
        receiverCommand[YAXIS] > MAXCHECK && receiverCommand[XAXIS] > MAXCHECK &&
        haveAGpsLock()) {
  
      homePosition.latitude = currentPosition.latitude;
      homePosition.longitude = currentPosition.longitude;
      //homePosition.altitude = DEFAULT_HOME_ALTITUDE;
     homePosition.altitude = baroAltitudeToHoldTarget;    }                // Sudeep added for consistency wrt to position hold
      //homePosition.altitude = rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];  }


    if ((receiverCommand[AUX2] > 1240 && receiverCommand[AUX2] < 1750) || (receiverCommand[AUX2] > 900 && receiverCommand[AUX2] < 1100)) {  // Enter in execute mission state, if none, go back home, override the position hold
      if (!isGpsNavigationInitialized) {
        gpsRollAxisCorrection = 0;
        gpsPitchAxisCorrection = 0;
        gpsYawAxisCorrection = 0;
        isGpsNavigationInitialized = true;
      }
      
 //    Serial2.print('N');
      positionHoldState = OFF;         // disable the position hold while navigating
      previousPositionHoldState = 0;   // Sudeep Added 21-04

      isPositionHoldInitialized = false;
      //headingHoldState = ON; //for heading hold
      navigationState = ON;
      
    /*  if (FLG == )
      {
        Serial2.print('Y');
       baroAltitudeToHoldTarget = getBaroAltitude()+ 0.5;
       FLG = 2; 
      }*/
      
      
      altitudeHoldState = ON;
//      baroAltitudeToHoldTarget = curr_alt;
     
    }
       else if ((receiverCommand[AUX1] > 1240 && receiverCommand[AUX1] < 1250)||(receiverCommand[AUX1] > 900 && receiverCommand[AUX1] < 1100))  {  // Enter in position hold state
   //  else if (receiverCommand[AUX1] < 1100))  {  // Enter in position hold state
      if (!isPositionHoldInitialized) {
        gpsRollAxisCorrection = 0;
        gpsPitchAxisCorrection = 0;
        gpsYawAxisCorrection = 0;
  
        //positionHoldPointToReach.latitude = currentPosition.latitude;            // Sudeep changed it. Condition taken care of elsewhere
        //positionHoldPointToReach.longitude = currentPosition.longitude;
        //positionHoldPointToReach.altitude = getBaroAltitude();
        isPositionHoldInitialized = true;
      }
  
      isGpsNavigationInitialized = false;  // disable navigation
      navigationState = OFF;
      
      positionHoldState = ON;
      //positionHoldState = OFF;
      
      if (previousPositionHoldState == 0)
      {
    //    Serial2.print('X');
        positionHoldPointToReach.latitude = currentPosition.latitude;                 // from GPS
        positionHoldPointToReach.longitude = currentPosition.longitude;
        positionHoldPointToReach.altitude = getBaroAltitude();
//        baroAltitudeToHoldTarget = getBaroAltitude();                                 // Disabled by indrajit as already taken care in altitute hold.
        previousPositionHoldState = 1;
        gps_alt = 1;
      }
      
    }
    else { 
      
      // Navigation and position hold are disabled
      positionHoldState = OFF;
      previousPositionHoldState = 0;
      isPositionHoldInitialized = false;
      //headingHoldState = OFF;
      navigationState = OFF;
      isGpsNavigationInitialized = false;
      if ((receiverCommand[AUX1] > 1220 && receiverCommand[AUX1] < 1250) || (receiverCommand[AUX1] > 900 && receiverCommand[AUX1] < 1100))
      {altitudeHoldState = ON;}
      else 
      {altitudeHoldState = OFF;}
      gpsRollAxisCorrection = 0;
      gpsPitchAxisCorrection = 0;
      gpsYawAxisCorrection = 0;
    }
  }
#endif




void processZeroThrottleFunctionFromReceiverCommand() {
  // Disarm motors (left stick lower left corner)
  if (receiverCommand[ZAXIS] < MINCHECK && motorArmed == ON) {
    commandAllMotors(MINCOMMAND);
    motorArmed = OFF;
    inFlight = false;
    digitalWrite(LED_Red, HIGH);
    #ifdef OSD
      notifyOSD(OSD_CENTER|OSD_WARN, "MOTORS UNARMED");
    #endif

    #if defined BattMonitorAutoDescent
      batteryMonitorAlarmCounter = 0;
      batteryMonitorStartThrottle = 0;
      batteyMonitorThrottleCorrection = 0.0;
    #endif
  }    
 
  // Zero Gyro and Accel sensors (left stick lower left, right stick lower right corner)
  if ((receiverCommand[ZAXIS] < MINCHECK) && (receiverCommand[XAXIS] > MAXCHECK) && (receiverCommand[YAXIS] < MINCHECK)) {
    calibrateGyro();
    computeAccelBias();
    storeSensorsZeroToEEPROM();
    calibrateKinematics();
    zeroIntegralError();
    pulseMotors(3);
  }   

  // Arm motors (left stick lower right corner)
  if (receiverCommand[ZAXIS] > MAXCHECK && motorArmed == OFF && safetyCheck == ON) {
 
    #ifdef OSD_SYSTEM_MENU
      if (menuOwnsSticks) {
        return;
      }
    #endif

    /*for (byte motor = 0; motor < LASTMOTOR; motor++) {
      motorCommand[motor] = MINTHROTTLE-100;//Minthrottle-100 to stop motor rotations just after arming
    } */
    motorArmed = ON;

    digitalWrite(LED_Red, LOW);
    #ifdef OSD
      notifyOSD(OSD_CENTER|OSD_WARN, "!MOTORS ARMED!");
    #endif  

    zeroIntegralError();

  }
  // Prevents accidental arming of motor output if no transmitter command received
  if (receiverCommand[ZAXIS] > MINCHECK) {
    safetyCheck = ON; 
  }
}




/**
 * readPilotCommands
 * 
 * This function is responsible to read receiver
 * and process command from the users
 */
void readPilotCommands() {

  readReceiver(); 
  

  // Testing w/o Rx part (Comment it when using for flight testing)
//  if (countSudeep == 30)
//  {
//  receiverCommand[AUX1] = 1050;          // Altitude hold ON w/o Rx
//  //receiverCommand[AUX2] = 1850;          // 
//  receiverCommand[MODE] = 1800;          // Attitude Mode
//  motorArmed == ON;
//  receiverCommand[THROTTLE] = 1500;  
//  }
//  else
//  {
//    receiverCommand[AUX1] = 1850;          // Altitude hold OFF w/o Rx
//    receiverCommand[MODE] = 1800;          // Attitude Mode
//    receiverCommand[AUX2] = 1050;          // Navigation mode OFF
//    motorArmed == ON;
//    receiverCommand[THROTTLE] = 1500;  
//  }
  
  // Sudeep slope addition
  //receiverCommand[THROTTLE] = 0.6*receiverCommand[THROTTLE] + 800;    //Hobbyking Tx
  //receiverCommand[THROTTLE] = 0.846*receiverCommand[THROTTLE] + 280;    //9XR Tx
  //receiverCommand[THROTTLE] = 0.65*receiverCommand[THROTTLE] + 700;    //9XR Tx
  if (receiverCommand[THROTTLE] < MINCHECKthrottle+5) {
    processZeroThrottleFunctionFromReceiverCommand();
  }

  if (!inFlight) {
    if (motorArmed == ON && receiverCommand[THROTTLE] > minArmedThrottle) {
      inFlight = true;
    }
  }

    // Check Mode switch for Acro or Stable
    if (receiverCommand[MODE] > 1500) {
        flightMode = ATTITUDE_FLIGHT_MODE;
    }
    else {
        flightMode = RATE_FLIGHT_MODE;
    }
    
    if (previousFlightMode != flightMode) {
      zeroIntegralError();
      previousFlightMode = flightMode;
    }


  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    processAltitudeHoldStateFromReceiverCommand();
    #endif
  
  #if defined (AutoLanding)
    processAutoLandingStateFromReceiverCommand();
  #endif

  #if defined (UseGPSNavigator)
    processGpsNavigationStateFromReceiverCommand();
  #endif
}

#endif // _AQ_FLIGHT_COMMAND_READER_

