#include "Servo.h"
#include <Arduino.h>
#define TRIG_PIN 13
#define ECHO_PIN 3
#define SERVO_PIN 6
#define FAN_PIN 9

float distance; // store the distance from sensor
Servo servo;
void setup() {
  // begin serial port
  Serial.begin (9600);
  
  // Configure the trigger and echo pins to output mode
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT); 

  servo.attach(SERVO_PIN);
  servo.write(90);


}

void loop() {
  analogWrite(FAN_PIN, 255);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);   // 10 microsecond pulse
  digitalWrite(TRIG_PIN, LOW);

  // measure duration of pulse from ECHO pin
  float duration_us = pulseIn(ECHO_PIN, HIGH);

  // calculate the distance
  distance = 0.017 * duration_us;

  // print the value to Serial Monitor
  Serial.print("distance: ");
  Serial.print(distance);
  Serial.print(" cm");
  Serial.print("    pulse: ");
  Serial.print(duration_us * 1000); // because the pulse is in microseconds and we need it in miliseconds
  Serial.println(" ms");
  if(distance<30){
    servo.write(60);
  }
  else{
    servo.write(90);
  }
 delay(10);
  digitalWrite(TRIG_PIN, LOW);
}


