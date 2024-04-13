
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
#define FAN_PIN 5


const int angle_1 = 90;
const int max_spin = 45;
const int stop_distance = 80; // in cm

// PID GAINS
const float pidAngle[3] = {2.0, 0.005, 0.015};

// set points,this is the desired angleZ
float angleSetpoint = 0.0;

// inputs
float inputAngle = 0.0;
float dt = 0.0;
// outpus
float outputAngle = 0.0;

// time
const unsigned long timeStep = 500; // Time step in milliseconds
unsigned long prevTime;
unsigned long timer; // global timer

int delay_time = 50; // delay to spin the servo motor


/*=================================================================================================
                                                Variables
=================================================================================================*/

Servo servo;
float distance;
int measurement_count = 10;

MPU6050 mpu(Wire);
static float integrationStored = 0, integralSaturation = 1;
float valueLast, velocity, errorLast, currentTime = 0;

enum State { STRAIGHT, TURNING };
State currentState;

/*=================================================================================================
                                                Prototypes
=================================================================================================*/

void controller(int);
float update(float, float, float);
float measureDistance(int);
void printIMUData();
void stopforWall();
/*=================================================================================================
                                                Setup
=================================================================================================*/
void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Wire.begin();

  byte status = mpu.begin();    
  while (status != 0) {
    Serial.println("IMU not found");
  } // stop if IMU is not found

  servo.attach(SERVO_PIN);
  servo.write(angle_1);     // Set the servo to 90
  delay(1000);


  mpu.calcOffsets(true, true);
  currentState = STRAIGHT;
  Wire.setWireTimeout(3000, true);

  analogWrite(FAN_PIN, 230);
}
/*=================================================================================================
                                                Loop
=================================================================================================*/
void loop() {

  dt = millis() - currentTime;
  distance = measureDistance(measurement_count);
  Serial.println(distance);
  Serial.println(" ");

  mpu.update();
  float angleZ = mpu.getAngleZ();

  switch (currentState) {

  case STRAIGHT:

    analogWrite(FAN_PIN, 255);
    stopforWall();


    int control_pid = angle_1 - update(dt, angleZ, angleSetpoint); // initially (dt,mpu angel, 0.0)
    Serial.print("angle control ");
    Serial.println(control_pid);

    controller(control_pid);

    printIMUData();
    break;

  case TURNING:

  // LOGIC:
  // 1. spin left, spin right, find where to turn
  // 2. spin 45 in the direction of the turn and start the fan 
  // 3. angle is 45 till IMU senses 180 degree change from the straight angle
  // 4. once 180, turn back to 90 and keep going straight.
  
  Serial.println("Iam here in  TURNING");
  break;
  default:
    break;
  }

  currentTime = millis();
  
}
/*=================================================================================================
                                                Interrupts
=================================================================================================*/

/*=================================================================================================
                                                Functions
=================================================================================================*/

float measureDistance(int measurement_count_) {
  float duration_ = 0;
  float distance_=0;
  int good_measurement_count = 0;
  for (int i = 0; i < measurement_count_; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    float pulse = pulseIn(ECHO_PIN, HIGH);

    if (pulse*0.017<2000){
      good_measurement_count++;
       duration_ += pulse;
    }  
  }
  duration_ = duration_ / good_measurement_count;
  distance_ = duration_ * 0.017;
  return distance_;
}


void stopforWall() {
  unsigned long startTime = millis(); //timer used for time passes since we start 
  bool wallDetected = false; // Flag  if the wall is detected'

// magic number used here for 2 seconds as 2000
  while (!wallDetected && millis() - startTime < 2000) { // if the wall is detected and the timer is not 2 seconds
    // Measure distance
    distance = measureDistance(measurement_count);

    // Check if wall is detected
    if (distance < stop_distance) {
      wallDetected = true;
    }
  }
}

// controller
void controller(int angle) {
  servo.write(angle);
  delay(delay_time);
  analogWrite(FAN_PIN, 255); 
}

float update(float dt, float currentValue, float targetValue) {

  float error = targetValue - currentValue;

  // calculate P term
  float P = pidAngle[0] * error;

  // calculate I term
  integrationStored = constrain(integrationStored + (error * dt),
                                -integralSaturation, integralSaturation);
  float I = pidAngle[1] * integrationStored;

  // calculate both D terms
  float errorRateOfChange = (error - errorLast) / dt;
  errorLast = error;

  float valueRateOfChange = (currentValue - valueLast) / dt;
  valueLast = currentValue;
  velocity = valueRateOfChange;

  // choose D term to use
  float deriveMeasure = 0;

  float D = pidAngle[2] * deriveMeasure;

  float result = P + I + D;

  return constrain(result, -max_spin, max_spin);
}


// print IMU data
void printIMUData() {
  // Serial.print("Angle X: ");
  // Serial.println(mpu.getAngleX());
  // Serial.print("Angle Y: ");
  // Serial.println(mpu.getAngleY());
  Serial.print("Angle Z: ");
  Serial.println(mpu.getAngleZ());
  Serial.println(" ");
  delay(100);
}