
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
#define FAN_PIN 5

int delay_time = 50;

int currAngle = 90; /// something to store the the position
const int angle_1 = 90;
const int max_spin = max_spin
;


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
long duration;
float distance; // store the distance from sensor
float dis;
const int poop_measurements = 10; // number to average
MPU6050 mpu(Wire);
static float integrationStored=0,integralSaturation=1;
float valueLast,velocity,errorLast,currentTime=0;

/*=================================================================================================
                                                Prototypes
=================================================================================================*/

void controller(int);
float update(float, float, float);
float us_distance();
void printIMU();
/*=================================================================================================
                                                Setup
=================================================================================================*/
void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(FAN_PIN, OUTPUT);
  Wire.begin();
  byte status = mpu.begin();
  while (status != 0) {}  //stop if IMU is not found
  servo.attach(SERVO_PIN);
  servo.write(angle_1);
  delay(1000);
  mpu.calcOffsets(true, true);
  Wire.setWireTimeout(3000, true);
  
  digitalWrite(FAN_PIN, HIGH);
}
/*=================================================================================================
                                                Loop
=================================================================================================*/

void loop() {

  static enum { STRAIGHT, TURNING, STRAIGHT2 } state = STRAIGHT; // Initialize state variable
  int turn_count = 0;

  dt = millis() - currentTime;
  dis = us_distance();
   
    

    switch(state) {

    case STRAIGHT: {
      
    if (dis < 80){
      state = TURNING;
      
      Serial.println("Iam exiting STRAIGHT");
      break;
    }
        servo.write(angle_1);  
 
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
  int control_pid = angle_1-update(dt,angle,angleSetpoint);      // here its angle - update 
  Serial.print("angle control ");
  Serial.println(control_pid);
  controller(control_pid);
  
  currentTime=millis();
    }
      break;



    case TURNING: {
        
          Serial.print("Iam herein TURNING ");

        digitalWrite(FAN_PIN, LOW);
        delay(3000);  
        servo.write(angle_1-max_spin
        );  
        delay(1000);  
        digitalWrite(FAN_PIN, HIGH);
        delay(5000);  
        servo.write(angle_1);             
        delay(1500);  
        servo.write(angle_1-max_spin
        );  
        delay(1500);  
        servo.write(angle_1);  
        turn_count++; // Update turn_count after the turning procedure
      
      // Transition to straight2 state when turning is complete
      state = (turn_count > 0) ? STRAIGHT2 : TURNING;
    }
      break;

      case STRAIGHT2: {
        servo.write(angle_1);  
        mpu.update();
        Serial.print("Angle X: ");
        Serial.println(mpu.getAngleX());
        Serial.print("Angle Y: ");
        Serial.println(mpu.getAngleY());
        Serial.print("Angle Z: ");
        Serial.println(mpu.getAngleZ());
        Serial.println(" ");
        delay(100);

        float angle2 = mpu.getAngleZ(); //// angle from 180/360
        int control_pid2 = angle_1-update(dt,angle2,angleSetpoint);  // here its angle - update 
        Serial.print("angle control ");
        Serial.println(control_pid2);
        controller(control_pid2);
        
        currentTime=millis();
      break;
      }
  }
  // print the value to Serial Monitor
  
  


  
    
  


}


/*=================================================================================================
                                                Interrupts
=================================================================================================*/

/*=================================================================================================
                                                Functions
=================================================================================================*/

float us_distance(){
  for (int i = 0; i < poop_measurements; i++) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // measure duration of pulse from ECHO pin
  duration += pulseIn(ECHO_PIN, HIGH); ///// Duration = duration + pulsein
  }
  
  // Averaging poopMeasuremts
  duration = duration/poop_measurements; 

  // calculate the distance
  return distance = duration * 0.017 ;
  

  // return distance;

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

  return constrain(result, -max_spin, max_spin);
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

void printIMU(){
    // all the debug printing messages of IMU and PID controller here.
}