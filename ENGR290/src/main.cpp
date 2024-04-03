/*=================================================================================================
                                                Includes
=================================================================================================*/
#include <Arduino.h>
#include "Servo.h"
#include <Wire.h>
#include "MPU6050_light.h"
/*=================================================================================================
                                                Defines
=================================================================================================*/
#define TRIG_PIN 13
#define ECHO_PIN 3
#define SERVO_PIN 6
#define FAN_PIN 4
/*=================================================================================================
                                                Variables
=================================================================================================*/
Servo servo;
int distance;
long duration;
MPU6050 mpu(Wire);
 
 /*=================================================================================================
                                                Prototypes
=================================================================================================*/
/*=================================================================================================
                                                Setup
=================================================================================================*/
void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Wire.begin();
  byte status = mpu.begin();
  while(status!=0){ }//stop if IMU is not found
  servo.attach(SERVO_PIN);
  delay(1000);
  mpu.calcOffsets(true,true);
  Wire.setWireTimeout(3000, true);

}
/*=================================================================================================
                                                Loop
=================================================================================================*/
void loop() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(100);
  mpu.update();
  Serial.print("Angle X: ");
  Serial.println(mpu.getAngleX());
  Serial.print("Angle Y: ");
  Serial.println(mpu.getAngleY());
  Serial.print("Angle Z: ");
  Serial.println(mpu.getAngleZ());
  Serial.println(" ");
  delay(100);
}
/*=================================================================================================
                                                Interrupts
=================================================================================================*/

/*=================================================================================================
                                                Functions
=================================================================================================*/
