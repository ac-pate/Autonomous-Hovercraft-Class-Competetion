#include "Servo.h"
#include <Arduino.h>
#define TRIG_PIN 13 ///P13 on the board
#define ECHO_PIN 3
#define SERVO_PIN 6
#define FAN_PIN 9

long duration;
float distance; // store the distance from sensor
int poop_measurements = 10; // number to average

Servo servo;

void setup() {
  // begin serial port
  Serial.begin (115200);
  
  // Configure the trigger and echo pins to output mode
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT); 

  servo.attach(SERVO_PIN);
  servo.write(90);


}

void loop() {
  analogWrite(FAN_PIN, 255);
  
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
  distance = duration * 0.017 ;
  
  
  // print the value to Serial Monitor
  Serial.print("distance: ");
  Serial.print(distance);
  Serial.println(" cm");

}


