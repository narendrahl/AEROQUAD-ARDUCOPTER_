/*
  AeroQuad v3.0 - May 2011
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

// parts of the init sequence were taken from AP_InertialSensor_MPU6000.h

#ifndef _AEROQUAD_PLATFORM_MPU6000_H_
#define _AEROQUAD_PLATFORM_MPU6000_H_



#include "Arduino.h"
#include <SensorsStatus.h>
#define MPU6000_CHIP_SELECT_PIN 53  // MPU6000 CHIP SELECT

// global variables
 byte MPU6000_newdata;
 
// MPU 6000 registers
#define MPUREG_WHOAMI			0x75
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_FIFO_EN			0x23
#define MPUREG_INT_PIN_CFG		0x37
#define MPUREG_INT_ENABLE		0x38
#define MPUREG_INT_STATUS		0x3A
#define MPUREG_ACCEL_XOUT_H		0x3B
#define MPUREG_ACCEL_XOUT_L		0x3C
#define MPUREG_ACCEL_YOUT_H		0x3D
#define MPUREG_ACCEL_YOUT_L		0x3E
#define MPUREG_ACCEL_ZOUT_H		0x3F
#define MPUREG_ACCEL_ZOUT_L		0x40
#define MPUREG_TEMP_OUT_H		0x41
#define MPUREG_TEMP_OUT_L		0x42
#define MPUREG_GYRO_XOUT_H		0x43
#define MPUREG_GYRO_XOUT_L		0x44
#define MPUREG_GYRO_YOUT_H		0x45
#define MPUREG_GYRO_YOUT_L		0x46
#define MPUREG_GYRO_ZOUT_H		0x47
#define MPUREG_GYRO_ZOUT_L		0x48
#define MPUREG_USER_CTRL		0x6A
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C
#define MPUREG_FIFO_COUNTH		0x72
#define MPUREG_FIFO_COUNTL		0x73
#define MPUREG_FIFO_R_W			0x74


// Configuration bits
#define BIT_SLEEP				0x40
#define BIT_H_RESET				0x80
#define BITS_CLKSEL				0x07
#define MPU_CLK_SEL_PLLGYROX	0x01
#define MPU_CLK_SEL_PLLGYROZ	0x03
#define MPU_EXT_SYNC_GYROX		0x02
#define BITS_FS_250DPS          0x00
#define BITS_FS_500DPS          0x08
#define BITS_FS_1000DPS         0x10
#define BITS_FS_2000DPS         0x18
#define BITS_FS_MASK            0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR    0x10
#define BIT_RAW_RDY_EN			0x01
#define BIT_I2C_IF_DIS          0x10
#define BIT_INT_STATUS_DATA		0x01


typedef struct {
  short x;
  short y;
  short z;
} tAxis;

union uMPU6000 {
  unsigned char rawByte[];
  unsigned short rawWord[];
  struct {
	tAxis accel;
	short temperature;
	tAxis gyro;
  } data;
} MPU6000;


// MPU6000 SPI functions
byte MPU6000_SPI_read(byte reg)
{
  byte dump;
  byte return_value;
  byte addr = reg | 0x80; // Set most significant bit
  digitalWrite(MPU6000_CHIP_SELECT_PIN, LOW);
  dump = SPI.transfer(addr);
  return_value = SPI.transfer(0);
  digitalWrite(MPU6000_CHIP_SELECT_PIN, HIGH);
  return(return_value);
}

void MPU6000_SPI_write(byte reg, byte data)
{
  byte dump;
  digitalWrite(MPU6000_CHIP_SELECT_PIN, LOW);
  dump = SPI.transfer(reg);
  dump = SPI.transfer(data);
  digitalWrite(MPU6000_CHIP_SELECT_PIN, HIGH);
}

// MPU6000 INTERRUPT ON INT0
void MPU6000_data_int()
{
  MPU6000_newdata++;
}



bool initializeMPU6000SensorsDone = false;
void initializeMPU6000Sensors()
{
  if(initializeMPU6000SensorsDone) {
	return;
  }
  initializeMPU6000SensorsDone = true;
   pinMode(40, OUTPUT);
   digitalWrite(40, HIGH);//To avoid barometer holding SPI line.
   // MPU6000 chip select setup
   pinMode(MPU6000_CHIP_SELECT_PIN, OUTPUT);
   digitalWrite(MPU6000_CHIP_SELECT_PIN, HIGH);

  unsigned char val;

  val = MPU6000_SPI_read(MPUREG_WHOAMI);
  if((val&0x7E) == 0x68) {
	vehicleState |= GYRO_DETECTED;
	vehicleState |= ACCEL_DETECTED;
  } 
  else {
	return;
  }

  // Chip reset
  MPU6000_SPI_write(MPUREG_PWR_MGMT_1, BIT_H_RESET);
  delay(100);  // Startup time delay

  // Wake Up device and select GyroZ clock (better performance)
  MPU6000_SPI_write(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
  MPU6000_SPI_write(MPUREG_PWR_MGMT_2, 0);//????

  // Disable I2C bus (recommended on datasheet)
    MPU6000_SPI_write(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
    delay(1);
    
  // SAMPLE RATE
    // MPU6000_SPI_write(MPUREG_SMPLRT_DIV,0x00);     // Sample rate = 1kHz
   MPU6000_SPI_write(MPUREG_SMPLRT_DIV,0x04);     // Sample rate = 200Hz    Fsample= 1Khz/(4+1) = 200Hz     
   // MPU6000_SPI_write(MPUREG_SMPLRT_DIV,19);     // Sample rate = 50Hz    Fsample= 1Khz/(19+1) = 50Hz     
    delay(1);
   

  // FS & DLPF   FS=1000�/s, DLPF = 42Hz (low pass filter)
  MPU6000_SPI_write(MPUREG_CONFIG, BITS_DLPF_CFG_42HZ);
  delay(1);
  MPU6000_SPI_write(MPUREG_GYRO_CONFIG,BITS_FS_1000DPS);  // Gyro scale 1000�/s
  delay(1);
  MPU6000_SPI_write(MPUREG_ACCEL_CONFIG,0x08);   // Accel scale +-4g (4096LSB/g)
delay(1);
   // INT CFG => Interrupt on Data Ready
    MPU6000_SPI_write(MPUREG_INT_ENABLE,BIT_RAW_RDY_EN);         // INT: Raw data ready
    delay(1);
    MPU6000_SPI_write(MPUREG_INT_PIN_CFG,BIT_INT_ANYRD_2CLEAR);  // INT: Clear on any read
    delay(1);
    // Oscillator set
    // MPU6000_SPI_write(MPUREG_PWR_MGMT_1,MPU_CLK_SEL_PLLGYROZ);
    delay(1);
  
    // MPU_INT is connected to INT 0. Enable interrupt on INT0
    attachInterrupt(0,MPU6000_data_int,RISING);

 
}


void MPU6000SwapData(unsigned char *data, int datalen)
{
  datalen /= 2;
  while(datalen--) {
    unsigned char t = data[0];
    data[0] = data[1];
    data[1] = t;
    data += 2;
  }
}



void readMPU6000Accel()
{
  int byte_H;
  int byte_L;
  
   // Read AccelX
    byte_H = MPU6000_SPI_read(MPUREG_ACCEL_YOUT_H);
    byte_L = MPU6000_SPI_read(MPUREG_ACCEL_YOUT_L);
    MPU6000.data.accel.x = 1*(byte_H<<8)| byte_L;//X=MPU_x
       
    // Read AccelY
    byte_H = MPU6000_SPI_read(MPUREG_ACCEL_XOUT_H);
    byte_L = MPU6000_SPI_read(MPUREG_ACCEL_XOUT_L);
    MPU6000.data.accel.y = 1*(byte_H<<8)| byte_L;//Y=-1*MPU_y
    // Read AccelZ
    byte_H = MPU6000_SPI_read(MPUREG_ACCEL_ZOUT_H);
    byte_L = MPU6000_SPI_read(MPUREG_ACCEL_ZOUT_L);
    MPU6000.data.accel.z =-1*(byte_H<<8)| byte_L;//Z=-1*MPU_z
  
}

void readMPU6000Gyro()
{
   int byte_H;
   int byte_L;
   int sign[3]={1,-1,1}; 
    // Read Pitch
    byte_H = MPU6000_SPI_read(MPUREG_GYRO_YOUT_H);
    byte_L = MPU6000_SPI_read(MPUREG_GYRO_YOUT_L);
    MPU6000.data.gyro.x = sign[0]*((byte_H<<8)| byte_L);
    
	// Read Roll
    byte_H = MPU6000_SPI_read(MPUREG_GYRO_XOUT_H);
    byte_L = MPU6000_SPI_read(MPUREG_GYRO_XOUT_L);
    MPU6000.data.gyro.y = sign[1]*((byte_H<<8)| byte_L);
    // Read Yaw
    byte_H = MPU6000_SPI_read(MPUREG_GYRO_ZOUT_H);
    byte_L = MPU6000_SPI_read(MPUREG_GYRO_ZOUT_L);
    MPU6000.data.gyro.z = sign[2]*((byte_H<<8)| byte_L);
 	
}
void readMPU6000Temp()
{
   int byte_H;
  int byte_L;
//Read Temp
    byte_H = MPU6000_SPI_read(MPUREG_TEMP_OUT_H);
    byte_L = MPU6000_SPI_read(MPUREG_TEMP_OUT_L);
    MPU6000.data.temperature = (byte_H<<8)| byte_L; 
   

}
#endif
