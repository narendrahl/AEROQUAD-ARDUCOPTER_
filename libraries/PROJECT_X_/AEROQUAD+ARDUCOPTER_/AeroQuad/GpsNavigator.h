/*
  AeroQuad v3.0 - Febuary 2012
  www.AeroQuad.com
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
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


#ifndef _AQ_Navigator_H_
#define _AQ_Navigator_H_

#include <HeadingFusionProcessorCompFilter.h>
#include "AltitudeControlProcessor.h"
extern void setDeclinationLocation(long int, long int);
// little wait to have a more precise fix of the current position since it's called after the first gps fix
#define MIN_NB_GPS_READ_TO_INIT_HOME 15  

int k=0, H=0;
int FLAG = 0;
float curr_gps_alt = 0.0;

unsigned long previousFixTime = 0;

boolean haveNewGpsPosition() {
//  return false;
  return ((haveAGpsLock()) && (previousFixTime != getGpsFixTime()));
}

void clearNewGpsPosition() {
  previousFixTime = getGpsFixTime();
}
  
boolean isHomeBaseInitialized() {
  return homePosition.latitude != GPS_INVALID_ANGLE;
}

void initHomeBase() {
  //Serial.println('home');
  if (haveNewGpsPosition()) {
    clearNewGpsPosition();
    if (countToInitHome < MIN_NB_GPS_READ_TO_INIT_HOME) {
      countToInitHome++;
    }
    else {
      homePosition.latitude = currentPosition.latitude;
      homePosition.longitude = currentPosition.longitude;
      homePosition.altitude = DEFAULT_HOME_ALTITUDE;  
      // Set the magnetometer declination when we get the home position set
      setDeclinationLocation(currentPosition.latitude,currentPosition.longitude);
      // Set reference location for Equirectangular projection used for coordinates
      setProjectionLocation(currentPosition);
      

      #if defined UseGPSNavigator
        evaluateMissionPositionToReach();
      #else
        missionPositionToReach.latitude = homePosition.latitude;
        missionPositionToReach.longitude = homePosition.longitude;
        missionPositionToReach.altitude = homePosition.altitude;
      #endif
    }  
  }
}


#if defined UseGPSNavigator

  /*
    Because we are using lat and lon to do our distance errors here's a quick chart:
    100 	= 1m
    1000 	= 11m	 = 36 feet
    1800 	= 19.80m = 60 feet
    3000 	= 33m
    10000       = 111m
  */
  
  #define MIN_DISTANCE_TO_REACHED 100

//  #define MAX_POSITION_HOLD_CRAFT_ANGLE_CORRECTION 300.0
  #define MAX_POSITION_HOLD_CRAFT_ANGLE_CORRECTION 130.0  // Changed by indrajit
  #define POSITION_HOLD_SPEED 5.0  
//  #define MAX_NAVIGATION_ANGLE_CORRECTION 300.0
  #define MAX_NAVIGATION_ANGLE_CORRECTION 130.0          //changed by indrajit
  #define NAVIGATION_SPEED 5.0//30.0 
  
  #define MAX_YAW_AXIS_CORRECTION 200.0  
    
  long readingDelay = 0;  
  long estimatedDelay = 0;
  unsigned long previousEstimationTime = 0;
  unsigned long previousReadingTime = 0;
  int latitudeMovement = 0;
  int longitudeMovement = 0;
  GeodeticPosition previousPosition = GPS_INVALID_POSITION;
  GeodeticPosition estimatedPosition = GPS_INVALID_POSITION;
  GeodeticPosition estimatedPreviousPosition = GPS_INVALID_POSITION;
  float currentSpeedRoll = 0.0; 
  float currentSpeedPitch = 0.0;
  
  float distanceToDestinationX = 0.0;         
  float distanceToDestinationY = 0.0;          
  float angleToWaypoint = 0.0;
  
  float maxSpeedToDestination = POSITION_HOLD_SPEED;
  float maxCraftAngleCorrection = MAX_POSITION_HOLD_CRAFT_ANGLE_CORRECTION;
  
  #if defined AltitudeHoldRangeFinder
    boolean altitudeProximityAlert = false;
    byte altitudeProximityAlertSecurityCounter = 0;
  #endif


  /** 
   * @return true if there is a mission to execute
   */
    boolean haveMission() {
    return missionNbPoint != 0;
  }



  /** 
   * Compute the distance to the destination, point to reach
   * @result is distanceToDestination
   */
  void computeDistanceToDestination(GeodeticPosition destination) {
    
/*
    
    Serial2.print(degrees(angleToWaypoint));
    Serial2.print(':');
    Serial2.print(degrees(trueNorthHeading));
    Serial2.print(':');
    Serial2.print(destination.longitude);
    Serial2.print(':');
    Serial2.print(destination.latitude);
    Serial2.print(':');
    Serial2.print(estimatedPosition.longitude);
    Serial2.print(':');
    Serial2.print(estimatedPosition.latitude);
    Serial2.print(':');
    Serial2.print(cosLatitude);
    Serial2.print(':');
    Serial2.print(gpsYawAxisCorrection);
    Serial2.print(':');
  */     
    // Sudeep Changed missionPositionToReach to destination here....
    distanceToDestinationX = (float)(destination.longitude - estimatedPosition.longitude) * cosLatitude * 1.113195;
    distanceToDestinationY = (float)(destination.latitude  - estimatedPosition.latitude) * 1.113195;
    distanceToDestination  = sqrt(sq(distanceToDestinationY) + sq(distanceToDestinationX));

  }





  /**
   * Evalutate the position to reach depending of the state of the mission 
   */
  void evaluateMissionPositionToReach() {
    
 /*  
      if(k==10)
    {
    Serial2.print(baroAltitudeToHoldTarget);
    Serial2.print(':');
    Serial2.print(getBaroAltitude());
    Serial2.print(':');
    Serial2.print(distanceToDestination);
    Serial2.print(':');
    Serial2.print(distanceToDestinationX);
    Serial2.print(':');
    Serial2.print(distanceToDestinationY);
    Serial2.print(':');
    Serial2.print(positionHoldPointToReach.latitude);
    Serial2.print(':');
    Serial2.print(positionHoldPointToReach.longitude);
    Serial2.print(':');
    Serial2.print(missionPositionToReach.latitude);
    Serial2.print(':');
    Serial2.print(missionPositionToReach.longitude);
    Serial2.print(':');
    Serial2.print(degrees(trueNorthHeading));
    Serial2.print(':');
    Serial2.print(angleToWaypoint);
    Serial2.print(':');
    Serial2.print(gpsRollAxisCorrection);
    Serial2.print(':');
    Serial2.print(gpsPitchAxisCorrection);
    Serial2.print(':');
    Serial2.print(angleToWaypoint);
    Serial2.println();
    k=1;}
    else{
      k=k+1;}*/
    if (waypointIndex == -1) { // if mission have not been started
      waypointIndex++;
      
    }
    
    if (waypointIndex < MAX_WAYPOINTS && distanceToDestination < MIN_DISTANCE_TO_REACHED) {
      waypointIndex++;                            // Added by indrajit 
       #ifdef LeaderFollower                          // Added by indrajit
           #ifdef Leader                               // Added by indrajit      
               waypointIndex--;                          // Added by indrajit
           #endif                                          // Added by indrajit
           #ifdef Follower                                     // Added by indrajit
               waypointIndex--;                               // Added by indrajit
           #endif                                                // Added by indrajit
        #endif                                       // Added by indrajit
    }
    
    if (waypointIndex >= MAX_WAYPOINTS || waypoint[waypointIndex].altitude == GPS_INVALID_ALTITUDE) { // if mission is completed, last step is to go home, 2147483647 == invalid altitude
      
      /*     positionHoldPointToReach.latitude = currentPosition.latitude;          // Sudeep added to make sure that the position hold point is defined
           positionHoldPointToReach.longitude = currentPosition.longitude;
            //  positionHoldPointToReach.altitude = rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];   
           positionHoldPointToReach.altitude =getBaroAltitude();
           baroAltitudeToHoldTarget = positionHoldPointToReach.altitude;
        */
      //#else
      missionPositionToReach.latitude = homePosition.latitude;
      missionPositionToReach.longitude = homePosition.longitude;
      missionPositionToReach.altitude = baroAltitudeToHoldTarget;
      //#endif
      
      }
    else
     {
       #ifdef LeaderFollower        
        #ifdef Follower
          missionPositionToReach.latitude = waypointLF.latitude;
          missionPositionToReach.longitude = waypointLF.longitude;
          missionPositionToReach.altitude = baroAltitudeToHoldTarget; ///curr_gps_alt;//waypoint[waypointIndex].altitude;
        #endif
       #else
      missionPositionToReach.latitude = waypoint[waypointIndex].latitude;
      missionPositionToReach.longitude = waypoint[waypointIndex].longitude;
      missionPositionToReach.altitude = baroAltitudeToHoldTarget;//waypoint[waypointIndex].altitude;
        #endif
      }
    if (missionPositionToReach.altitude > 5) {
        missionPositionToReach.altitude = 5; // fix max altitude to 2 km
      }         
                  
     
        #ifdef LeaderFollower        
          #ifdef Follower
             
          // navigationState = OFF;
         //  positionHoldState = OFF;
          missionPositionToReach.latitude = waypointLF.latitude;
          missionPositionToReach.longitude = waypointLF.longitude;
          missionPositionToReach.altitude = baroAltitudeToHoldTarget;//waypoint[waypointIndex].altitude;
           
        #endif
      #endif
      
       computeDistanceToDestination(missionPositionToReach);
      if (missionPositionToReach.altitude > 5) {
        missionPositionToReach.altitude = 5; // fix max altitude to 2 km
      }
    }


  void computeNewPosition() {
    
    unsigned long time = micros();
    readingDelay = time - previousReadingTime;
    previousReadingTime = time;
    estimatedDelay = time - previousEstimationTime;
    previousEstimationTime = time;
    
    latitudeMovement = currentPosition.latitude - previousPosition.latitude;
    longitudeMovement = currentPosition.longitude - previousPosition.longitude;
    
    previousPosition.latitude  = currentPosition.latitude;
    previousPosition.longitude = currentPosition.longitude;
    previousPosition.altitude  = currentPosition.altitude;
    
    estimatedPreviousPosition.latitude = estimatedPosition.latitude;
    estimatedPreviousPosition.longitude = estimatedPosition.longitude;
    
    estimatedPosition.latitude  = currentPosition.latitude;
    estimatedPosition.longitude = currentPosition.longitude;
    estimatedPosition.altitude  = currentPosition.altitude;
    
    #ifdef LeaderFollower
        #ifdef Leader
          waypointLF.latitude =currentPosition.latitude;
          waypointLF.longitude =currentPosition.longitude;
          waypointLF.altitude =currentPosition.altitude;
        #endif  
    #endif
       
}
  
  void computeEstimatedPosition() {
    
    unsigned long time = micros();
    estimatedDelay = time - previousEstimationTime;
    previousEstimationTime = time;
    
    estimatedPreviousPosition.latitude = estimatedPosition.latitude;
    estimatedPreviousPosition.longitude = estimatedPosition.longitude;
    
    estimatedPosition.latitude += (latitudeMovement / (readingDelay / estimatedDelay));
    estimatedPosition.longitude += (longitudeMovement / (readingDelay / estimatedDelay));
  }




  /**
   * Compute the current craft speed in cm per sec
   * @result are currentSpeedPitch and currentSpeedRoll
   */
  void computeCurrentSpeed() {
  
    float currentSpeedX = (float)(estimatedPosition.longitude - estimatedPreviousPosition.longitude) * cosLatitude * 1.113195;
    float currentSpeedY = (float)(estimatedPosition.latitude - estimatedPreviousPosition.latitude) * 1.113195;
    float currentSpeed = sqrt(sq(currentSpeedY) + sq(currentSpeedX));
    
    currentSpeedX = currentSpeedX * (100000 / estimatedDelay); // normalized to about 5hz
    currentSpeedY = currentSpeedY * (100000 / estimatedDelay); 
    currentSpeed = currentSpeed * (100000 / estimatedDelay); 
  
    float tmp = degrees(atan2(currentSpeedX, currentSpeedY));
    if (tmp < 0) {
      tmp += 360; 
    }

    float courseRads = radians(tmp);
    currentSpeedRoll = (sin(courseRads-trueNorthHeading)*currentSpeed); 
    currentSpeedPitch = (cos(courseRads-trueNorthHeading)*currentSpeed);
  }
    
  /**
   * compute craft angle in roll/pitch to adopt to navigate to the point to reach
   * @result are gpsRollAxisCorrection and gpsPitchAxisCorrection use in flight control processor
   */
  void computeRollPitchCraftAxisCorrection() {
    
    angleToWaypoint = atan2(distanceToDestinationX, distanceToDestinationY)-trueNorthHeading;
    float tmpsin = sin(angleToWaypoint);
    float tmpcos = cos(angleToWaypoint);
   //  tmpsin = sin(atan2(distanceToDestinationX, distanceToDestinationY));
   //  tmpcos = cos(atan2(distanceToDestinationX, distanceToDestinationY));
    float rollSpeedDesired = ((maxSpeedToDestination*tmpsin)*(float)distanceToDestination)/1000; 
    float pitchSpeedDesired = ((maxSpeedToDestination*tmpcos)*(float)distanceToDestination)/1000;
    rollSpeedDesired = constrain(rollSpeedDesired, -maxSpeedToDestination, maxSpeedToDestination);
    pitchSpeedDesired = constrain(pitchSpeedDesired, -maxSpeedToDestination, maxSpeedToDestination);
    
    int tempGpsRollAxisCorrection = updatePID(rollSpeedDesired, currentSpeedRoll, &PID[GPSROLL_PID_IDX]);
    int tempGpsPitchAxisCorrection = updatePID(pitchSpeedDesired, currentSpeedPitch, &PID[GPSPITCH_PID_IDX]);

    if (tempGpsRollAxisCorrection >= gpsRollAxisCorrection) {
      gpsRollAxisCorrection += 1;
    }
    else {
      gpsRollAxisCorrection -= 1;
    }
    if (tempGpsPitchAxisCorrection >= gpsPitchAxisCorrection) {
      gpsPitchAxisCorrection += 1;  
    }
    else {
      gpsPitchAxisCorrection -= 1;
    }
    
    gpsRollAxisCorrection = constrain(gpsRollAxisCorrection, -maxCraftAngleCorrection, maxCraftAngleCorrection);
    gpsPitchAxisCorrection = constrain(gpsPitchAxisCorrection, -maxCraftAngleCorrection, maxCraftAngleCorrection);
    

    
    
//    Serial.print(gpsData.sats);Serial.print(" ");Serial.print(distanceToDestination);Serial.print(" ");
//    Serial.print(rollSpeedDesired);Serial.print(",");Serial.print(pitchSpeedDesired);Serial.print(" ");
//    Serial.print(currentSpeedRoll);Serial.print(",");Serial.print(currentSpeedPitch);Serial.print(" ");
//    Serial.print(gpsRollAxisCorrection);Serial.print(",");Serial.print(gpsPitchAxisCorrection);Serial.print(" ");
//    Serial.println();
  }
  
  
  /**
   * Evaluate altitude to reach, if we use the range finder, we use it as altitude proximity alert
   * to increase the current point to reach altitude
   */
  void evaluateAltitudeCorrection() {
//    #if defined AltitudeHoldRangeFinder
//      // if this is true, we are too near the ground to perform navigation, then, make current alt hold target +2m
//      if (sonarAltitudeToHoldTarget != INVALID_RANGE) { 
//        if (!altitudeProximityAlert) {
//          sonarAltitudeToHoldTarget += 2;
//          missionPositionToReach.altitude += 2;
//          altitudeProximityAlert = true;
//        }
//      }
//
//      if (altitudeProximityAlert && altitudeProximityAlertSecurityCounter <= 10) {
//        altitudeProximityAlertSecurityCounter++;
//      }
//      else {
//        altitudeProximityAlertSecurityCounter = 0;
//        altitudeProximityAlert = false;
//      }
//
//    #endif
//    #if defined AltitudeHoldRangeFinder
//      if (isOnRangerRange(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX])) {
//        sonarAltitudeToHoldTarget += 0.05;
//      }
//    #endif
      /*
      if ( FLAG == 0)
      {
      baroAltitudeToHoldTarget = getBaroAltitude();
      FLAG = 1;
      }*/

//     baroAltitudeToHoldTarget = missionPositionToReach.altitude;
    //sonarAltitudeToHoldTarget = missionPositionToReach.altitude;
  }
  
  /**
   * In navigation mode, we want the craft headed to the target, so this will 
   * compute the heading correction to have
   */
  void computeHeadingCorrection() {
    
    float correctionAngle = angleToWaypoint;
    if (correctionAngle > PI) {
      correctionAngle = fmod(correctionAngle,PI) - PI;
    }
    //correctionAngle = correctionAngle*360/PI;
    gpsYawAxisCorrection = -updatePID(0.0, correctionAngle, &PID[GPSYAW_PID_IDX]);
    gpsYawAxisCorrection = constrain(gpsYawAxisCorrection, -MAX_YAW_AXIS_CORRECTION, MAX_YAW_AXIS_CORRECTION);
  }

  /**
   * Process position hold
   */
  void processPositionHold() {
      if (haveNewGpsPosition()) {
//      Serial2.print('X');
      computeNewPosition();
      clearNewGpsPosition();
    }
    else {
  //    Serial2.print('Y');      
      computeEstimatedPosition();
    }    
    
  //  baroAltitudeToHoldTarget = curr_gps_alt; // commented by indrajit for position hold test ...condition already taken care elsewhere.
  //baroAltitudeToHoldTarget = positionHoldPointToReach.altitude;  // commented by indrajit
    //sonarAltitudeToHoldTarget = positionHoldPointToReach.altitude;
    computeCurrentSpeed();
    
    computeDistanceToDestination(positionHoldPointToReach);
    

    
    // evaluate the flight behavior to adopt
    maxSpeedToDestination = POSITION_HOLD_SPEED;
    maxCraftAngleCorrection = MAX_POSITION_HOLD_CRAFT_ANGLE_CORRECTION;

    computeRollPitchCraftAxisCorrection();

    gpsYawAxisCorrection = 0;  
/*    Serial2.print(curr_gps_alt);
    Serial2.print(':');
    Serial2.print(baroAltitudeToHoldTarget);
    Serial2.print(':');
    Serial2.print(positionHoldPointToReach.altitude);
    Serial2.print(':');
*/    
  }
  
    /** 
   * Process navigation
   */
  void processNavigation() {
  // Serial2.print('K');
//Serial2.print(waypointIndex);

  #ifdef Follower
  /*
    Serial2.print(baroAltitudeToHoldTarget);
    Serial2.print(':');
    Serial2.print(getBaroAltitude());
    Serial2.print(':');
    Serial2.print(distanceToDestination);
    Serial2.print(':');
    Serial2.print(distanceToDestinationX);
    Serial2.print(':');
    Serial2.print(distanceToDestinationY);
    Serial2.print(':');
    Serial2.print(motorAxisCommandRoll);
    Serial2.print(':');
    Serial2.print(motorAxisCommandPitch);
    Serial2.print(':');
    Serial2.print(motorAxisCommandYaw);
    Serial2.print(':');
    Serial2.print(curr_alt);
    Serial2.print(':');
    Serial2.print(altitudeHoldCorrectionSudeep);
    Serial2.print(':');
    Serial2.print(degrees(angleToWaypoint));
    Serial2.print(':');
    Serial2.print(gpsRollAxisCorrection);
    Serial2.print(':');
    Serial2.print(gpsPitchAxisCorrection);
    Serial2.print(':');
    Serial2.print(throttle);
    Serial2.print(':');
    Serial2.print(degrees(trueNorthHeading));
    Serial2.print(':');
    Serial2.print(waypointLF.latitude);
    Serial2.print(':');
    Serial2.print(waypointLF.longitude);
    Serial2.print(':');
    */
  //  Serial2.print(currentPosition.latitude);
  //  Serial2.print(':');
  //  Serial2.print(currentPosition.longitude);
  //  Serial2.print(':');
  //  Serial2.print(waypointIndex);
  //  Serial2.print(':');
    //  Serial2.print(navigationState);
    //  Serial2.print(':');
    //  Serial2.print(positionHoldState);
    //
   //  Serial2.print(':');
#endif
  
    if (distanceToDestination < MIN_DISTANCE_TO_REACHED) {
     // Serial2.print('H');
      H =1;
      positionHoldPointToReach.latitude = currentPosition.latitude;          // Sudeep added to make sure that the position hold point is defined
      positionHoldPointToReach.longitude = currentPosition.longitude;
    //  positionHoldPointToReach.altitude = rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];   
      positionHoldPointToReach.altitude = baroAltitudeToHoldTarget;//curr_alt;//getBaroAltitude();
      processPositionHold();
      evaluateMissionPositionToReach();
      return;
    }
    else if (haveNewGpsPosition()) {
//      Serial2.print('N');
      computeNewPosition();
      clearNewGpsPosition();
    }
    else {
      computeEstimatedPosition();
      
      //return;
    }
    
    computeCurrentSpeed();
    ///Serial2.print('K');  
    // evaluate if we need to switch to another mission possition point
    evaluateMissionPositionToReach();

   
    computeDistanceToDestination(missionPositionToReach);

    maxSpeedToDestination = NAVIGATION_SPEED;
    maxCraftAngleCorrection = MAX_NAVIGATION_ANGLE_CORRECTION;

    computeRollPitchCraftAxisCorrection();
    
    evaluateAltitudeCorrection();    

    computeHeadingCorrection();
  }

  
  /**
   * Compute everything need to make adjustment to the craft attitude to go to the point to reach
   */
  void processGpsNavigation() {  //// 
//  Serial2.print(receiverCommand[AUX2]);
  //  Serial2.print(waypoint[0].latitude);

    if (haveAGpsLock()) {
      
  //  Serial2.print(navigationState);
      #ifdef LeaderFollower       
            #ifdef Leader
              navigationState = OFF;
            #endif 
     
            #ifdef Follower
       /*     Serial2.print(waypointIndex);
            Serial2.print(':');
            Serial2.print(MAX_WAYPOINTS);
            Serial2.print(':');*/
            
            if ((waypointIndex >= MAX_WAYPOINTS || waypoint[waypointIndex].altitude == GPS_INVALID_ALTITUDE) && distanceToDestination < MIN_DISTANCE_TO_REACHED)
          //if (receiverCommand[AUX2] > 1240 && receiverCommand[AUX2] < 1750)  
               { // if mission is completed, last step is to go home, 2147483647 == invalid altitude 
                 
        /*         if( FLAG == 0 )
                  {
                   waypointLF.longitude = gpsData.lon;
                   waypointLF.latitude  = gpsData.lat;
                   baroAltitudeToHoldTarget = getBaroAltitude();
                   FLAG = 1;
                    
                  }
          */        
         //      Serial2.print(waypoint[waypointIndex].altitude);
               navigationState = OFF;
               positionHoldState = ON;
               }
           #endif
      #endif
      
      if (navigationState == ON) {
        
        processNavigation();
       
      }
      else if (positionHoldState == ON ) {
        processPositionHold();
      }
    }
  }
#endif  // #define UseGPSNavigator


#endif











