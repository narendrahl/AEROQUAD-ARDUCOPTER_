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


#ifndef _AEROQUAD_MOTORS_APM_H_
#define _AEROQUAD_MOTORS_APM_H_

#include "Arduino.h"
#include <APM_RC.h>
#include "Motors.h"

//#define PWM_COUNTER_PERIOD (F_CPU/PWM_PRESCALER/PWM_FREQUENCY)
void initializeMotors(NB_Motors numbers) {
  commandAllMotors(1000);  
  for(uint8_t ch=0;ch<numbers;ch++)
  enable_out(ch);
}

void writeMotors() {
   
  writeMotorCommand(0,motorCommand[MOTOR1]);
  writeMotorCommand(1,motorCommand[MOTOR2]);
  writeMotorCommand(2,motorCommand[MOTOR3]);
  writeMotorCommand(3,motorCommand[MOTOR4]);
 // if ((numberOfMotors == SIX_Motors) || (numberOfMotors == EIGHT_Motors)) {  // for 6-8 motors
  writeMotorCommand(4,motorCommand[MOTOR5]);
  writeMotorCommand(5,motorCommand[MOTOR6]);//}
  //if (numberOfMotors == EIGHT_Motors){  // for 8 motors
  writeMotorCommand(6,motorCommand[MOTOR7]);
  writeMotorCommand(7,motorCommand[MOTOR8]);//}
  //Harshal: 9,and 10 include if camera servos are required at 50 Hz
  //writeMotorCommand(9,motorCommand[MOTOR9]);
  //writeMotorCommand(10,motorCommand[MOTOR10]);
  force_Out0_Out1();
  force_Out2_Out3();
  force_Out6_Out7();
}

void commandAllMotors(int command) {
  writeMotorCommand(0,command);
  writeMotorCommand(1,command);
  writeMotorCommand(2,command);
  writeMotorCommand(3,command);
  //if ((numberOfMotors == SIX_Motors) || (numberOfMotors == EIGHT_Motors)) {  // for 6-8 motors
  writeMotorCommand(4,command);
  writeMotorCommand(5,command);// }
  //if (numberOfMotors == EIGHT_Motors){  // for 8 motors
  writeMotorCommand(6,command); 
  writeMotorCommand(7,command);//}
  //Harshal: 9,and 10 include if camera servos are required at 50 Hz
  //writeMotorCommand(9,command);
  //writeMotorCommand(10,command);
  
  force_Out0_Out1();
  force_Out2_Out3();
  force_Out6_Out7();
}
  


#endif