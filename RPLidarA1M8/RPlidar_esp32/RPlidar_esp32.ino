/*
  Sketch to read and print current smallest distance to object at any angle.
*/
#include <Wire.h>

#include <RPLidar.h>
#include "driver/timer.h"

#define RPLIDAR_MOTOR 13 // The PWM pin for control the speed of RPLIDAR's motor (pwm_motor_lidar).

RPLidar lidar;
                      
void setup() {
  Serial.begin(115200);   // PC communication
  Serial2.begin(115200);  // For RPLidar
  lidar.begin(Serial2);
  pinMode(RPLIDAR_MOTOR, OUTPUT);  // set pin modes
}

float minDistance = 100000;
float angleAtMinDist = 0;

void loop() {
  if (IS_OK(lidar.waitPoint())) { // If Lidar detected, data ready
    float distance = lidar.getCurrentPoint().distance;  // Get distance
    float angle = lidar.getCurrentPoint().angle;  // Get angle
    
    if (lidar.getCurrentPoint().startBit) {
       // a new scan, display the previous data...
       printData(angleAtMinDist, minDistance);
       minDistance = 100000;
       angleAtMinDist = 0;
    } else {
       if ( distance > 0 &&  distance < minDistance) {  // If point is closer than current closest
          minDistance = distance; //
          angleAtMinDist = angle; // Replace minimum values with current values.
       }
    }
  }
  else {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    // Try to detect RPLIDAR
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // Detected
       lidar.startScan();
       analogWrite(RPLIDAR_MOTOR, 255);
       delay(1000);
    }
  }
}

void printData(float angle, float distance)
{
  Serial.print("dist: ");
  Serial.print(distance);
  Serial.print("    angle: ");
  Serial.println(angle);
}