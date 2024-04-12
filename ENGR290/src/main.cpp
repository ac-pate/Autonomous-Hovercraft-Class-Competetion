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

MPU6050 mpu(Wire);
Servo servo;

/*=================================================================================================
                                                Variables
=================================================================================================*/
long duration;
float distance; // store the distance from sensor
const int poop_measurements = 10; // number to average
 
int currAngle = 90; /// something to store the the position
const int angle_1 = 90;
const int max_spin = 45;

 /*=================================================================================================
                                                Prototypes
=================================================================================================*/

void turnRight(int );
// void sweep();
int us_distance();

/*=================================================================================================
                                                Setup
=================================================================================================*/
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  byte status = mpu.begin();
  while(status!=0){ }//stop if IMU is not found

  servo.attach(SERVO_PIN);
  delay(100);

  mpu.calcOffsets(true,true);
  Wire.setWireTimeout(3000, true);

 

}
/*=================================================================================================
                                                Loop
=================================================================================================*/
void loop() {
  
  us_distance();
  // sweep();
  turnRight(45);
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


int us_distance(){
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
  
  
  // print the value to Serial Monitor
  Serial.print("distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // return distance;

}

void turnRight(int input_angle){
  
        analogWrite(9, 255);  
    servo.write(angle_1);  
    delay(3000);  
     analogWrite(9, 0);  
    delay(2000);  
    servo.write(angle_1-input_angle);  
    delay(1000);  
     analogWrite(9, 255);  
             // tell servo to go to position in variable 'pos'
    delay(5000);  

    servo.write(angle_1);             // tell servo to go to position in variable 'pos'
    delay(1500);  

    servo.write(angle_1-input_angle);  
    delay(1500);  
    servo.write(angle_1);   
}

// /// @sweep is the function we  ///
// void sweep(){
//   for (pos = angle_1-45; pos >= angle_1+45; pos -= 1) { // goes from 180 degrees to 0 degrees
//     servo.write(pos);              // tell servo to go to position in variable 'pos'
//     delay(15);  
//     // Serial.println(pos);                     // waits 15 ms for the servo to reach the position
    
//   }
//   delay(1000);
//   for (pos = angle_1+45; pos >= angle_1-45; pos -= 1) { // goes from 180 degrees to 0 degrees
//     servo.write(pos);              // tell servo to go to position in variable 'pos'
//     delay(15);  
//     // Serial.println(pos);                     // waits 15 ms for the servo to reach the position
    
//   }
// }

// void sweepRight(){
//   for (currAngle = pos; currAngle <= angle_1 + max_spin; currAngle += 1) { // goes from 0 degrees to 180 degrees
//     // in steps of 1 degree
//     int pos = angle_1;    // variable to store the servo position
//     pos = currAngle;    //Store the current angle into the pos
//     servo.write(currAngle);              // tell servo to go to position in variable 'pos'
//     delay(15); 
//     // Serial.println( pos);   
//     if (currAngle == angle_1 + max_spin){   // angle_1+45 limit reached
//       return true;                   // waits 15 ms for the servo to reach the position
//     }
//   }
// }
