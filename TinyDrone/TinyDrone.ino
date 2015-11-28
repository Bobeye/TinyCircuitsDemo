#define	BLACK           0x00
#define	BLUE            0xE0
#define	RED             0x03
#define	GREEN           0x1C
#define	DGREEN           0x0C
#define YELLOW          0x1F
#define WHITE           0xFF
#define ALPHA           0xFE
#define	BROWN           0x32


#include <TinyScreen.h>
#include <SPI.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h" 
#include "CalLib.h"
#include <EEPROM.h>


int MFLDirPin = 2;      // Motor direction connected to digital pin 2
int MFLSpeedPin = 3;    // Motor speed connected to digital pin 3
int MBLDirPin = 4;      // Motor direction connected to digital pin 2
int MBLSpeedPin = 5;    // Motor speed connected to digital pin 3
int MFRDirPin = 7;      // Motor direction connected to digital pin 2
int MFRSpeedPin = 6;    // Motor speed connected to digital pin 3
int MBRDirPin = 8;      // Motor direction connected to digital pin 2
int MBRSpeedPin = 9;    // Motor speed connected to digital pin 3
int MotorSleepPin = A3;      // Motor sleep to analog pin 3



void setup()
{
  pinMode(MFLDirPin, OUTPUT);       // sets the pin as output
  pinMode(MFLSpeedPin, OUTPUT);     // sets the pin as output
  pinMode(MBLDirPin, OUTPUT);       // sets the pin as output
  pinMode(MBLSpeedPin, OUTPUT);     // sets the pin as output
  pinMode(MFRDirPin, OUTPUT);       // sets the pin as output
  pinMode(MFRSpeedPin, OUTPUT);     // sets the pin as output
  pinMode(MBRDirPin, OUTPUT);       // sets the pin as output
  pinMode(MBRSpeedPin, OUTPUT);     // sets the pin as output
  pinMode(MotorSleepPin , OUTPUT);     // sets the pin as output

  digitalWrite(motorDirPin, LOW);     // sets the default dir to be forward
  digitalWrite(motorSpeedPin, LOW);   // sets the default speed to be off
  digitalWrite(motorSleepPin , HIGH);   // sets the sleep mode to be off
}

void loop()
{
  // Set the motor direction to forward
  digitalWrite(motorDirPin, LOW);     

  // Ramp the motor speed up
  analogWrite(motorSpeedPin, 0);    // Min speed forward (motor off)
  delay(50);  
  analogWrite(motorSpeedPin, 63);   
  delay(50);  
  analogWrite(motorSpeedPin, 127);   
  delay(50); 
  analogWrite(motorSpeedPin, 191);   
  delay(50); 
  analogWrite(motorSpeedPin, 255);   // Max speed forward
  delay(50); 

  // Ramp the motor speed down
  analogWrite(motorSpeedPin, 191);   
  delay(50);  
  analogWrite(motorSpeedPin, 127);   
  delay(50);  
  analogWrite(motorSpeedPin, 63);   
  delay(50); 
  analogWrite(motorSpeedPin, 0);     // Min speed forward (motor off)
  delay(50); 


  // Set the motor direction to reverse
  digitalWrite(motorDirPin, HIGH);   

  // Ramp the motor speed up
  analogWrite(motorSpeedPin, 255);    // Min speed reverse (motor off)
  delay(50); 
  analogWrite(motorSpeedPin, 191);   
  delay(50);  
  analogWrite(motorSpeedPin, 127);   
  delay(50);  
  analogWrite(motorSpeedPin, 63);   
  delay(50); 
  analogWrite(motorSpeedPin, 0);      // Max speed reverse
  delay(50); 

  // Ramp the motor speed down
  analogWrite(motorSpeedPin, 63);   
  delay(50);  
  analogWrite(motorSpeedPin, 127);   
  delay(50); 
  analogWrite(motorSpeedPin, 191);   
  delay(50); 
  analogWrite(motorSpeedPin, 255);    // Min speed reverse (motor off)
  delay(50); 
}



////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Arduino
//
//  Copyright (c) 2014, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.



RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object

//  DISPLAY_INTERVAL sets the rate at which results are displayed

#define DISPLAY_INTERVAL  300                         // interval between pose displays

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200

unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;

void setup()
{
  int errcode;
  
  Serial.begin(SERIAL_PORT_SPEED);
  Wire.begin();
  imu = RTIMU::createIMU(&settings);                        // create the imu object
  
  Serial.print("ArduinoIMU starting using device "); Serial.println(imu->IMUName());
  if ((errcode = imu->IMUInit()) < 0) {
    Serial.print("Failed to init IMU: "); Serial.println(errcode);
  }
  
  if (imu->getCalibrationValid())
    Serial.println("Using compass calibration");
  else
    Serial.println("No valid compass calibration data");

  lastDisplay = lastRate = millis();
  sampleCount = 0;
}

void loop()
{  
  unsigned long now = millis();
  unsigned long delta;
  
  if (imu->IMURead()) {                                // get the latest data if ready yet
    fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
    sampleCount++;
    if ((delta = now - lastRate) >= 1000) {
      Serial.print("Sample rate: "); Serial.print(sampleCount);
      if (imu->IMUGyroBiasValid())
        Serial.println(", gyro bias valid");
      else
        Serial.println(", calculating gyro bias - don't move IMU!!");
        
      sampleCount = 0;
      lastRate = now;
    }
    if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
      lastDisplay = now;
//      RTMath::display("Gyro:", (RTVector3&)imu->getGyro());                // gyro data
//      RTMath::display("Accel:", (RTVector3&)imu->getAccel());              // accel data
//      RTMath::display("Mag:", (RTVector3&)imu->getCompass());              // compass data
      RTMath::displayDegrees("Pose:", (RTVector3&)fusion.getFusionPose()); // fused output
      Serial.println();
    }
  }
}


