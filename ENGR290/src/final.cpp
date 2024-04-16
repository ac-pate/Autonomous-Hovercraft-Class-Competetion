
/*Including control with distance */
#include "Servo.h"
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define SERVO_PIN 6
#define FAN_PIN 5
#define TRIG_PIN 13
#define ECHO_PIN 3





#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13       // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
unsigned long dt;
unsigned long previousTime;
unsigned long startTime;
unsigned long timer;  // global timer
float currentTime = 0;
float errorLast = 0;
float angleSetpoint = 0.0;
float distanceSetpoint = 50;
const int right_turning_delay = 20000;  //
const int left_turning_delay = 3000;


float stop_distance = 70;
float turn_ditance = 60;
const float pidAngle[3] = { 2.3, 0.05, 0.01 };
const float pidDistance[3] = { 3, 0.5, 1 };

// turn counter to keep track of imu state
int turn_count = 1;


Servo servo;
enum State { STRAIGHT,
             TURNING_LEFT,
             TURNING_RIGHT,
             SEARCH };
State currentState;
const int centerAngle = 102;
const int rotAngle = 40;
bool turned = false;

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif


  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);




  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();


  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  servo.attach(SERVO_PIN);
  delay(10);
  servo.write(centerAngle);
  delay(100);
  analogWrite(FAN_PIN, 255);
  currentState = STRAIGHT;
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
   solve_maze();

  //IMUAngle();
  // Serial.println(" ");
}


void solve_maze() {
  switch (currentState) {
    case STRAIGHT:
      {
        forward();
        break;
      }
    case SEARCH:
      {
        int delay_time = 350;
        servo.write(centerAngle - rotAngle);
        delay(delay_time);
        int disLeft = getCornerMeasurements();
        servo.write(centerAngle);
        delay(delay_time);
        int disCenter = getCornerMeasurements();
        servo.write(centerAngle + rotAngle);
        delay(delay_time);
        int disRight = getCornerMeasurements();


        Serial.print("SEARCH: \t");
        Serial.print(disLeft);
        Serial.print("\t");
        Serial.print(disCenter);
        Serial.print("\t");
        Serial.println(disRight);

        if (disCenter >= 60) {
          // add a big comment ; NEEED TO CHANGE to custome
          currentState = STRAIGHT;
          break;
        }

        if (disLeft > disRight) {
          currentState = TURNING_LEFT;
          break;
        }

        if (disRight > disLeft) {
          currentState = TURNING_RIGHT;
          break;
        }
      }

    case TURNING_RIGHT:
      {
        turnRight();
        break;
      }
    case TURNING_LEFT:
      {
        turnLeft();
        break;
      }
  }
}

void forward() {
  servo.write(centerAngle);
  delay(100);
  startTime = millis();
  while (1) {
    control_fan(255);
    //get time
    previousTime = currentTime;
    currentTime = millis();
    dt = currentTime - previousTime;

    float curr_angle = IMUAngle();

    if(turned){
        if(curr_angle>0){
            curr_angle-=180;
        }
        else{
            curr_angle+=180;
        }
    }

    // get measurements
    // wait for 2 second before taking us sensor measurements
    if (currentTime - startTime < 2000) {
      float angle = centerAngle + update(dt, curr_angle, angleSetpoint);
      control_angle(angle);
      continue;
    }

    float curr_distance = measureDistance();
    Serial.print("DISTANCE: ");Serial.println(curr_distance);
    float angle = centerAngle + update(dt, curr_angle, angleSetpoint);

    if (curr_distance < stop_distance) {
      Serial.print("Breaking out to SEARCH from FOWARD ");
      Serial.println(curr_distance);
      currentState = SEARCH;
      break;
    }
    // get control values
    Serial.print("FOWARD : \tAngle : ");
    Serial.print(curr_angle);
    Serial.print("\tControl A: ");
    Serial.print(angle);
    Serial.print("\tDistance : ");
    Serial.println(curr_distance);
    control_angle(angle);
  }

  // kill fan
  control_fan(200);

  return;
}

void turnRight() {
  startTime = millis();
  static float lastValue = 0;
  while (1) {
    float angle = IMUAngle();
    float rateOfChange = (angle - lastValue) / 20.0;  // assumed time step

    if (millis() - startTime > right_turning_delay && rateOfChange < 1.1) {
      Serial.print("Breaking out to search  from RIGHT, TIMEOUT");
      currentState = SEARCH;
      return;
    }

    if (angle < 0) {//to change if needed
      switchTurned();
      currentState = STRAIGHT;
      Serial.print("Breaking out to search  from RIGHT to FOWARD, TIMEOUT");

      turn_count++;
      return;
    }
    Serial.print("RIGHT");
    Serial.println(angle);
    analogWrite(FAN_PIN, 255);
    servo.write(centerAngle + 52);
    lastValue = angle;
  }
}
void turnLeft() {
  startTime = millis();
  static float lastValue = 0;

  while (1) {
    float angle = IMUAngle();
    float rateOfChange = (angle - lastValue) / 20.0;  // assumed time step


    if (millis() - startTime > right_turning_delay && rateOfChange < 1.1) {
      Serial.print("Breaking out to search  from LEFT, TIMEOUT");
      currentState = SEARCH;
      return;
    }
    if (angle > 0) {
      switchTurned();
      currentState = STRAIGHT;
      Serial.print("Breaking out to search  from LEFT to FOWARD");
      turn_count++;
      return;
    }
    Serial.print("LEFT :  ");
    Serial.println(angle);
    analogWrite(FAN_PIN, 255);
    servo.write(centerAngle - rotAngle);

    // update the last value
    lastValue = angle;
  }
  return;
}

float IMUAngle() {
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
  return (ypr[0] * 180 / M_PI);
  float rotated_angle;


}



float measureDistance() {
  const int WINDOW_SIZE = 5;
  float sum = 0;

  for (int i = 0; i < WINDOW_SIZE; i++) {
    // Trigger the ultrasonic sensor
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    float pulse = pulseIn(ECHO_PIN, HIGH);
    float duration_ = pulse;
    float distance_ = duration_ * 0.017;
    sum += distance_;
  }
  // Calculate the moving average
  float movingAverage = sum / WINDOW_SIZE;


  return movingAverage;
}



void switchTurned() {
  if (turned) {
    turned = false;
  } else {
    turned = true;
  }
}
float getCornerMeasurements() {
  const int WINDOW_SIZE = 100;
  // static float distances[WINDOW_SIZE] = { 0 };
  float sum = 0;

  for (int i = 0; i < WINDOW_SIZE; i++) {
    // Trigger the ultrasonic sensor
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    float pulse = pulseIn(ECHO_PIN, HIGH);
    float duration_ = pulse;
    float distance_ = duration_ * 0.017;
    sum += distance_;
  }





  // Calculate the moving average
  float movingAverage = sum / WINDOW_SIZE;


  return movingAverage;
}

float update(float dt, float currentValue, float targetValue) {
  static float integrationStored = 0, integralSaturation = 100, valueLast;
  float velocity;

  // adjust error if
  float error;
  // if the set point
  error = targetValue - currentValue;
  

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

  float D = pidAngle[2] * valueRateOfChange;

  float result = P;

  Serial.print("Error : ");
  Serial.print(error);
  Serial.print("\tSetPoiny ");
  Serial.println(targetValue);
  Serial.print("\tdt: ");
  Serial.println(dt);
  Serial.print("\tP: ");
  Serial.print(P);
  Serial.print("\tI: ");
  Serial.print(I);
  Serial.print("\tD: ");
  Serial.println(D);

  return constrain(result, -40, 40);
}

float distanceControl(float dt, float currentValue, float targetValue) {

  float error = abs(targetValue - currentValue);
  static float integrationStored = 0, integralSaturation = 2, erroLast = 0, valueLast = 0;
  float velocity;



  // calculate P term
  float P = pidDistance[0] * error;

  // calculate I term
  integrationStored = constrain(integrationStored + (error * dt),
                                -integralSaturation, integralSaturation);
  float I = pidDistance[1] * integrationStored;

  // calculate both D terms
  float errorRateOfChange = (error - errorLast) / dt;
  errorLast = error;

  float valueRateOfChange = (currentValue - valueLast) / dt;
  valueLast = currentValue;
  velocity = valueRateOfChange;

  // choose D term to use

  float D = pidDistance[2] * valueRateOfChange;

  float result = P + I + D;

  return constrain(result, 0, 55);
}


void control_angle(int angle) {
  servo.write(angle);
  delay(10);
}
void control_fan(int speed) {
  analogWrite(FAN_PIN, speed);
}


void controller(int angle) {
  servo.write(angle);
  delay(10);
  analogWrite(FAN_PIN, 255);
}