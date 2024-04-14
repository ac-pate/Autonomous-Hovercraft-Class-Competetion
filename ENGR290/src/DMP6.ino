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
float dt = 0.0;
unsigned long timer; // global timer
float currentTime=0;
float errorLast=0;
float angleSetpoint = 0.0;
const float pidAngle[3] = {2.3, 0.005, 0.01};
static float integrationStored = 0, integralSaturation = 1;
float velocity;
float valueLast;
Servo servo;
enum State { STRAIGHT, TURNING_LEFT, TURNING_RIGHT,TURNING};
State currentState;
const int centerAngle=90;
const int rotAngle=40;
bool turned=false;

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
  servo.write(90);
  analogWrite(FAN_PIN,255);
  currentState=STRAIGHT;
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  dt = millis() - currentTime;
float dis=measureDistance();
Serial.print("Distamce: ");
Serial.println(dis);
 if (dis<80&&dis>0){
  currentState=TURNING;
 }


 switch(currentState){
  case STRAIGHT:{
    analogWrite(FAN_PIN,255);
    float curr_angle=IMUAngle();
    Serial.println(curr_angle);
    if(!turned){
      angleSetpoint=0;
    }
    else{
      if(curr_angle>0){
        angleSetpoint=180;
      }
      else{
        angleSetpoint = -180;
      }
    }
    int control_pid = centerAngle + update(dt, curr_angle, angleSetpoint);
    controller(control_pid);
    break;
  }
  case TURNING: {
      analogWrite(FAN_PIN,0);
      servo.write(centerAngle);
      delay(1000);
      //check right
      servo.write(centerAngle+rotAngle);
      delay(2000);
      int disRight=measureDistance();
      Serial.print("Distance Right: ");
      Serial.println(disRight);
      delay(1000);
      //check left
      servo.write(centerAngle-rotAngle);
      delay(2000);
      int disLeft=measureDistance();
      Serial.print("Distance Left: ");
      Serial.println(disLeft);
      delay(1000);
      if (disRight>disLeft){
        currentState=TURNING_RIGHT;

      }
      else{
        currentState=TURNING_LEFT;
      }
      turned=true;
      //to refactor

       
  }
  case TURNING_RIGHT:{
    turnRight();
  }
  case TURNING_LEFT:{
    turnLeft();
  }
    
 }
 currentTime = millis();
}

float IMUAngle(){
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
  return (ypr[0]*180/M_PI);
}
void turnRight(){
  analogWrite(FAN_PIN, 255);
  servo.write(centerAngle + rotAngle);
  delay(5000);
  currentState = STRAIGHT;
  
}
void turnLeft(){
  analogWrite(FAN_PIN, 255);
  servo.write(centerAngle - rotAngle);
  delay(7000);
  currentState = STRAIGHT;
 
}

float measureDistance() {
  float duration_ = 0;
  float distance_=0;
  int good_measurement_count = 0;
  
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    float pulse = pulseIn(ECHO_PIN, HIGH);
    duration_ = pulse;
  distance_ = duration_ * 0.017;
  
  return distance_;
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

  return constrain(result, -45, 45);
}
void controller(int angle) {
  servo.write(angle);
  delay(25);
  analogWrite(FAN_PIN, 255);
}