
/*=================================================================================================
                                                Includes
=================================================================================================*/
#include "MPU6050_light.h"
#include "Servo.h"
#include <Arduino.h>
#include <Wire.h>
/*=================================================================================================
                                                Defines
=================================================================================================*/
#define TRIG_PIN 13
#define ECHO_PIN 3
#define SERVO_PIN 6
#define FAN_PIN 9

int delay_time = 50;
int angle_1 = 90;
int rot_angle = 45;

// // PID GAINS
// const float pidAngle[3] = {2.0, 0.005, 0.015};

// // set points,this is the desired angle
// float angleSetpoint = 0.0;

// // inputs
// float inputAngle = 0.0;
// float dt = 0.0;
// // outpus
// float outputAngle = 0.0;

// // time
// const unsigned long timeStep = 500; // Time step in milliseconds
// unsigned long prevTime;
// unsigned long timer; // global timer
// /*=================================================================================================
//                                                 Variables
// =================================================================================================*/
// Servo servo;
// float distance;
// int measurement_count = 10;

// MPU6050 mpu(Wire);
// static float integrationStored = 0, integralSaturation = 1;
// float valueLast, velocity, errorLast, currentTime = 0;

// enum State { STRAIGHT, TURNING_LEFT,TURNING_RIGHT };
// State currentState;
// bool canTurn;

// /*=================================================================================================
//                                                 Prototypes
// =================================================================================================*/

// void controller(int);
// float update(float, float, float);
// float measureDistance(int);
// void printIMUData();
// /*=================================================================================================
//                                                 Setup
// =================================================================================================*/
// void setup() {
//   Serial.begin(9600);
//   pinMode(TRIG_PIN, OUTPUT);
//   pinMode(ECHO_PIN, INPUT);
//   Wire.begin();
//   byte status = mpu.begin();    
//   while (status != 0) {} // stop if IMU is not found
//   servo.attach(SERVO_PIN);
//   servo.write(angle_1);
//   delay(1000);

  
//   mpu.calcOffsets(true, true);
//   currentState = STRAIGHT;
//   Wire.setWireTimeout(3000, true);
// }
// /*=================================================================================================
//                                                 Loop
// =================================================================================================*/
// void loop() {
//   dt = millis() - currentTime;
//   distance = measureDistance(measurement_count);
//   mpu.update();
//   printIMUData();
//   float angle = mpu.getAngleZ();
//   switch (currentState) {
//   case STRAIGHT:
//     int control_pid = angle_1 - update(dt, angle, angleSetpoint);
//     Serial.print("angle control ");
//     Serial.println(control_pid);
//     controller(control_pid);
//     break;

//    case TURNING_LEFT:

//    break;
//    default:
//      break;
//   }

//   currentTime = millis();
  
// }
// /*=================================================================================================
//                                                 Interrupts
// =================================================================================================*/

/*=================================================================================================
                                                Functions
=================================================================================================*/
// print IMU data
void printIMUData() {
  Serial.print("Angle X: ");
  Serial.println(mpu.getAngleX());
  Serial.print("Angle Y: ");
  Serial.println(mpu.getAngleY());
  Serial.print("Angle Z: ");
  Serial.println(mpu.getAngleZ());
  Serial.println(" ");
  delay(100);
}

// void turnLeft(){

// }
// bool canTurn(float distance){
//   return distance<10;
// }

// float measureDistance(int measurement_count_) {
//   float duration_ = 0;
//   float distance_=0;
//   int good_measurement_count = 0;
//   for (int i = 0; i < measurement_count_; i++) {
//     digitalWrite(TRIG_PIN, LOW);
//     delayMicroseconds(2);
//     digitalWrite(TRIG_PIN, HIGH);
//     delayMicroseconds(10);
//     digitalWrite(TRIG_PIN, LOW);
//     float pulse = pulseIn(ECHO_PIN, HIGH);

//     if (pulse*0.017<2000){
//       good_measurement_count++;
//        duration_ += pulse;
//     }  
//   }
//   duration_ = duration_ / good_measurement_count;
//   distance_ = duration_ * 0.017;
//   return distance_;
// }

// controller
void controller(int angle) {
  servo.write(angle);
  delay(delay_time);
  analogWrite(FAN_PIN, 255);
}

// float update(float dt, float currentValue, float targetValue) {

//   float error = targetValue - currentValue;

//   // calculate P term
//   float P = pidAngle[0] * error;

//   // calculate I term
//   integrationStored = constrain(integrationStored + (error * dt),
//                                 -integralSaturation, integralSaturation);
//   float I = pidAngle[1] * integrationStored;

//   // calculate both D terms
//   float errorRateOfChange = (error - errorLast) / dt;
//   errorLast = error;

//   float valueRateOfChange = (currentValue - valueLast) / dt;
//   valueLast = currentValue;
//   velocity = valueRateOfChange;

//   // choose D term to use
//   float deriveMeasure = 0;

//   float D = pidAngle[2] * deriveMeasure;

//   float result = P + I + D;

//   return constrain(result, -max_spin, max_spin);
// }