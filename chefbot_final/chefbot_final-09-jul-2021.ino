#include <CmdMessenger.h>

/*
  #  Chefbot_ROS_Interface.ino
  #
  #  Copyright 2015 Lentin Joseph <qboticslabs@gmail.com>
  #  Website : www.qboticslabs.com , www.lentinjoseph.com
  #  This program is free software; you can redistribute it and/or modify
  #  it under the terms of the GNU General Public License as published by
  #  the Free Software Foundation; either version 2 of the License, or
  #  (at your option) any later version.
  #
  #  This program is distributed in the hope that it will be useful,
  #  but WITHOUT ANY WARRANTY; without even the implied warranty of
  #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  #  GNU General Public License for more details.
  #
  #  
  #  9 July 2021 switch to use mpu6050 ver 0.3.0 (a more tidy verison) and i love it.
*/

#include <Timer.h>
Timer tm;

float vel[2];

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp / .h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//Creating MPU6050 Object
MPU6050 mpu(0x68);
//Messenger object
CmdMessenger cmdMessenger = CmdMessenger(Serial);

//DMP options
//Set true if DMP init was successful

bool dmpReady = false;
//Holds actual interrupt status byte from MPU
uint8_t mpuIntStatus;
//return status after each device operation
uint8_t devStatus;
//Expected DMP paclet size
uint16_t packetSize;
//count of all bytes currently in FIFO
uint16_t fifoCount;
//FIFO storate buffer
uint8_t fifoBuffer[64];

#define OUTPUT_READABLE_QUATERNION

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void(*resetFunc)(void) = 0; //declare reset function at address 0

bool bPubEncoder = true;

// Left encoder
#define Left_Encoder_PinA 2
#define Left_Encoder_PinB 5

volatile long Left_Encoder_Ticks = 0;
volatile bool LeftEncoderBSet;
volatile int32_t lEnc, rEnc;

//Right Encoder
#define Right_Encoder_PinA 3
#define Right_Encoder_PinB 4
volatile long Right_Encoder_Ticks = 0;
volatile bool RightEncoderBSet;

//Motor Pin definition
//Left Motor pins
#define A_1 7
#define B_1 8
//PWM 1 pin number
#define PWM_1 6

//Right Motor
#define A_2 10
#define B_2 11
//PWM 2 pin number
#define PWM_2 9

//Motor speed from PC
//Motor left and right speed
float motor_left_speed = 0;
float motor_right_speed = 0;

void SetupEncoders()
{
  // Quadrature encoders
  // Left encoder
  pinMode(Left_Encoder_PinA, INPUT_PULLUP);      // sets pin A as input
  pinMode(Left_Encoder_PinB, INPUT_PULLUP);      // sets pin B as input
  //Attaching interrupt in Left_Enc_PinA.
  attachInterrupt(digitalPinToInterrupt(Left_Encoder_PinA), do_Left_Encoder, RISING);

  // Right encoder
  pinMode(Right_Encoder_PinA, INPUT_PULLUP);      // sets pin A as input
  pinMode(Right_Encoder_PinB, INPUT_PULLUP);      // sets pin B as input
  //Attaching interrupt in Right_Enc_PinA.
  attachInterrupt(digitalPinToInterrupt(Right_Encoder_PinA), do_Right_Encoder, RISING);
  rEnc = lEnc = 0;
}

void SetupMotors()
{
  //Left motor
  pinMode(A_1, OUTPUT);
  pinMode(B_1, OUTPUT);

  //Right Motor
  pinMode(A_2, OUTPUT);
  pinMode(B_2, OUTPUT);
}

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void Setup_MPU6050()
{
   // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Wire.setWireTimeout(3000, true); //timeout value in uSec
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(18, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  // while (Serial.available() && Serial.read()); // empty buffer
  // while (!Serial.available());                 // wait for data
  // while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // mpu.setXAccelOffset(-3761);
  // mpu.setYAccelOffset(339);
  mpu.setZAccelOffset(615);

  mpu.setXGyroOffset(139);
  mpu.setYGyroOffset(88);
  mpu.setZGyroOffset(-12);

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
    Serial.print(digitalPinToInterrupt(18));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(18), dmpDataReady, RISING);
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
  // pinMode(LED_PIN, OUTPUT);
}

//Setup serial, encoders, ultrasonic, MPU6050 and Reset functions
void setup()
{ 
  //Init Serial port with 115200 baud rate
  Serial.begin(57600);
  while (!Serial);

  //Timer1.initialize(200000); // 200ms
  //Timer1.attachInterrupt(isrTimerOne);

  //Set up Messenger
  // cmdMessenger.attach('s', OnSetSpeed);
  delay(1000);

  SetupEncoders();
  tm.every(200, isrTimerOne);
  SetupMotors();
  Setup_MPU6050();  
}

void loop()
{
  // tm.update();
  // Read_From_Serial();
  // Process incoming serial data, and perform callbacks
  // cmdMessenger.feedinSerialData();

  // Update_Time();
  // Update_Ultra_Sonic();
  //Update_Battery();

  if (bPubEncoder) {
    Update_Encoders();
    bPubEncoder = false;
  }

  //Update motor values with corresponding speed and send speed values through serial port
  Update_Motors();
  Serial.println("after motor");
  Update_MPU6050();
  delay(5);
  Serial.println("after imu");
}

void OnSetSpeed() {
  // not that launchpad sends wheel_speed in left first and then right orders.
  motor_left_speed = cmdMessenger.readFloatArg();
  motor_right_speed = cmdMessenger.readFloatArg();
}

//OnMssg Complete function definition
void OnMssageCompleted()
{
  const char reset[] = "r";
  const char set_speed[] = "s";

  // if (cmdMessenger.checkString(reset))
  {
    Serial.println("Reset Done");
    resetFunc();
  }
}

void isrTimerOne() {
  vel[0] = float(rEnc / 990.0); // temp[0] * 5.0;
  vel[1] = float(lEnc / 990.0); // temp[1] * 5.0;
  rEnc = lEnc = 0;
  Serial.print("v");
  Serial.print("\t");
  Serial.print(vel[0]);
  Serial.print("\t");
  Serial.print(vel[1]);

  // add filler to make it 9 bytes for pyseial in_waiting >= 9
  Serial.print("\t");
  Serial.print(0);
  Serial.print("\t");
  Serial.print(0);
  Serial.print("\n");
  Serial.flush();
}

void do_Left_Encoder()
{
  bPubEncoder = true;
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  LeftEncoderBSet = digitalRead(Left_Encoder_PinB);   // read the input pin
  if (LeftEncoderBSet) {
    Left_Encoder_Ticks--;
    lEnc--;
  }
  else {
    Left_Encoder_Ticks++;
    lEnc++;
  }
  // Left_Encoder_Ticks -= LeftEncoderBSet ? -1 : +1;
}

void do_Right_Encoder()
{
  bPubEncoder = true;
  RightEncoderBSet = digitalRead(Right_Encoder_PinB);   // read the input pin
  if (RightEncoderBSet) {
    Right_Encoder_Ticks++;
    rEnc++;
  }
  else {
    Right_Encoder_Ticks--;
    rEnc--;
  }
  // Right_Encoder_Ticks += RightEncoderBSet ? -1 : +1;
}

void Update_Motors()
{
  moveRightMotor(motor_right_speed);
  moveLeftMotor(motor_left_speed);
}

void Update_Encoders()
{
  Serial.print("e");
  Serial.print("\t");
  Serial.print(Left_Encoder_Ticks);
  Serial.print("\t");
  Serial.print(Right_Encoder_Ticks);

  // add filler to make it 9 bytes for pyseial in_waiting >= 9
  Serial.print("\t");
  Serial.print(0);
  Serial.print("\t");
  Serial.print(0);
  Serial.print("\n");
  //Serial.flush();
}

void Update_MPU6050() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet

    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    Serial.print("i"); Serial.print("\t");
    Serial.print(q.x); Serial.print("\t");
    Serial.print(q.y); Serial.print("\t");
    Serial.print(q.z); Serial.print("\t");
    Serial.print(q.w);
    Serial.print("\n");
  }
}

void moveLeftMotor(float out)
{
  digitalWrite(A_1, out > 0);
  digitalWrite(B_1, out < 0);
  analogWrite(PWM_1, abs(out));
}

void moveRightMotor(float out)
{
  digitalWrite(A_2, out > 0);
  digitalWrite(B_2, out < 0);
  analogWrite(PWM_2, abs(out));
}
