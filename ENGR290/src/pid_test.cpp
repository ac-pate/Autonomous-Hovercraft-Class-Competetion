
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

float angleSetpoint = 0.0;

// inputs
float inputAngle = 0.0;
float dt=0.0;
// outpus
float outputAngle = 0.0;

// time
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
static float integrationStored=0,integralSaturation=1;
float valueLast,velocity,errorLast,currentTime=0;

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
  while (status != 0) {}  //stop if IMU is not found
  servo.attach(SERVO_PIN);
  servo.write(75);
  delay(1000);
  mpu.calcOffsets(true, true);
  Wire.setWireTimeout(3000, true);
  
}
/*=================================================================================================
                                                Loop
=================================================================================================*/
void loop() {
  dt=millis()-currentTime;
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
  int control_pid = 75-update(dt,angle,angleSetpoint);
  Serial.print("angle control ");
  Serial.println(control_pid);
  controller(control_pid);
  
  currentTime=millis();
}

// controller
void controller(int angle) {
  servo.write(angle);
  delay(delay_time);
  analogWrite(FAN_PIN, 255);
}

float update(float dt, float currentValue, float targetValue) {
  
  float error = targetValue - currentValue;

  //calculate P term
  float P = pidAngle[0] * error;

  //calculate I term
  integrationStored = constrain(integrationStored + (error * dt), -integralSaturation, integralSaturation);
  float I = pidAngle[1] * integrationStored;

  //calculate both D terms
  float errorRateOfChange = (error - errorLast) / dt;
  errorLast = error;

  float valueRateOfChange = (currentValue - valueLast) / dt;
  valueLast = currentValue;
  velocity = valueRateOfChange;

  //choose D term to use
  float deriveMeasure = 0;

  float D = pidAngle[2] * deriveMeasure;

  float result = P + I + D;

  return constrain(result, -45, 45);
}
// pid angle

int angle_pid(float setpoint, float input) {
  static float error;
  static float prevError;
  static float integral;
  static float derivative;
  static float dt;

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
