
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
#define FAN_PIN 9

int delay_time = 50;
int angle_1 = 65; 


// PID GAINS
const float pidAngle[3] = { 2.1, 0.005, 0.01 };


// set points,this is the desired angle

float angleSetpoint = 0.1;

// inputs
float inputAngle = 0.0;

// outpus
float outputAngle = 0.0;

// time
unsigned long dt = 0;
const unsigned long timeStep = 500;  // Time step in milliseconds
unsigned long prevTime;
unsigned long timer;  // global timer
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
  analogWrite(FAN_PIN, 255);
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);


  mpu.update();
  Serial.print("Angle X: ");
  Serial.println(mpu.getAngleX());
  Serial.print("Angle Y: ");
  Serial.println(mpu.getAngleY());
  Serial.print("Angle Z: ");
  Serial.println(mpu.getAngleZ());
  Serial.println(" ");
  delay(100);
  float angle = mpu.getAngleZ();
  int control_pid =constrain(angle_pid(angleSetpoint, angle), 10,110);
  Serial.print("angle control ");
  Serial.println(control_pid);
  controller(control_pid);
  delay(100);
}

// controller 
void controller(int angle) {
  servo.write(angle);
  delay(delay_time);
    analogWrite(FAN_PIN, 255);
}


// pid angle 

int angle_pid(float setpoint, float input) {
  static float error;
  static float prevError;
  static float integral;
  static float derivative;

  float err = setpoint - input;
  integral += err;
  derivative = err - prevError;

  float pid =
    pidAngle[0] * err + pidAngle[1] * integral + pidAngle[2] * derivative;
  prevError = err;
  pid * -1;
  return angle_1 - pid;
}

/*=================================================================================================
                                                Interrupts
=================================================================================================*/

/*=================================================================================================
                                                Functions
=================================================================================================*/
