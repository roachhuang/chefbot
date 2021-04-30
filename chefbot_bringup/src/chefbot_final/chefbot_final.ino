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
  #  You should have received a copy of the GNU General Public License
  #  along with this program; if not, write to the Free Software
  #  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
  #  MA 02110-1301, USA.
  #
  #  Some of the portion is adapted from I2C lib example code for MPU 6050
*/

#include <Timer.h>
Timer tm;

// #include <SoftwareSerial.h>
// SoftwareSerial mySerial(A8, A9); // RX, TX

float vel[2];

//MPU 6050 Interfacing libraries

//Library to communicate with I2C devices
#include "Wire.h"
//I2C communication library for MPU6050
// #include "I2Cdev.h"
//MPU6050 interfacing library
#include "MPU6050_6Axis_MotionApps20.h"
//Processing incoming serial data
#include <Messenger.h>
//Contain definition of maximum limits of various data type
// #include <limits.h>

//Creating MPU6050 Object
MPU6050 accelgyro(0x68);
//Messenger object
Messenger Messenger_Handler = Messenger();

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

//orientation/motion vars
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;

float euler[3];
float ypr[3];

void(*resetFunc)(void) = 0; //declare reset function at address 0

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Ultrasonic pins definition
const int echo = 9, Trig = 10;
long duration, cm;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Battery level monitor for future upgrade
#define BATTERY_SENSE_PIN PC_4

float battery_level = 12;

//Reset pin for resetting Tiva C, if this PIN set high, Tiva C will reset

#define RESET_PIN PB_2

//Time  update variables

unsigned long LastUpdateMicrosecs = 0;
unsigned long LastUpdateMillisecs = 0;
unsigned long CurrentMicrosecs = 0;
unsigned long MicrosecsSinceLastUpdate = 0;
float SecondsSinceLastUpdate = 0;

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

void SetupUltrasonic()
{
  pinMode(Trig, OUTPUT);
  pinMode(echo, INPUT);
}

void Setup_MPU6050()
{
  Wire.begin();
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  //Initialize DMP in MPU 6050
  Setup_MPU6050_DMP();
}

void Setup_MPU6050_DMP()
{
  //DMP Initialization
  devStatus = accelgyro.dmpInitialize();

  accelgyro.setXGyroOffset(220);
  accelgyro.setXGyroOffset(76);
  accelgyro.setXGyroOffset(-85);
  accelgyro.setXGyroOffset(1788);

  if (devStatus == 0) {
    accelgyro.setDMPEnabled(true);

    pinMode(18, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(18), dmpDataReady, RISING);
    // attachInterrupt(PUSH2, dmpDataReady, RISING);
    mpuIntStatus = accelgyro.getIntStatus();

    dmpReady = true;
    packetSize = accelgyro.dmpGetFIFOPacketSize();
  } else {
    ;
  }
}

//Setup serial, encoders, ultrasonic, MPU6050 and Reset functions
void setup()
{
  //Init Serial port with 115200 baud rate
  Serial.begin(115200);
  Serial.flush();

  //Timer1.initialize(200000); // 200ms
  //Timer1.attachInterrupt(isrTimerOne);

  //Set up Messenger
  Messenger_Handler.attach(OnMssageCompleted);
  delay(1000);

  SetupEncoders();
  tm.every(200, isrTimerOne);
  SetupMotors();
  Setup_MPU6050();
  //SetupUltrasonic();
  //SetupReset();
}

void loop()
{
  tm.update();
  //Read from Serial port
  Read_From_Serial();
  // Update_Time();
  // Update_Ultra_Sonic();
  //Update_Battery();
  
  if (bPubEncoder){     
    Update_Encoders();
    bPubEncoder = false;
  }
  
  //Update motor values with corresponding speed and send speed values through serial port
  Update_Motors();

  Update_MPU6050();
}

void Read_From_Serial()
{
  while (Serial.available() > 0)
  
  {
    Messenger_Handler.process(Serial.read());
  }
}

//OnMssg Complete function definition
void OnMssageCompleted()
{
  const char reset[] = "r";
  const char set_speed[] = "s";

  if (Messenger_Handler.checkString(reset))
  {
    Serial.println("Reset Done");
    resetFunc();
  }
  if (Messenger_Handler.checkString(set_speed))
  {
    //This will set the speed
    Set_Speed();   
  }
}

void isrTimerOne() {
  vel[0] = float(rEnc / 901.0); // temp[0] * 5.0;
  vel[1] = float(lEnc / 901.0); // temp[1] * 5.0;
  rEnc = lEnc = 0;
  Serial.print("v");
  Serial.print("\t");
  Serial.print(vel[0]);
  Serial.print("\t");
  Serial.print(vel[1]);
  Serial.print("\n");
}

void do_Left_Encoder()
{
   bPubEncoder = true;
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  LeftEncoderBSet = digitalRead(Left_Encoder_PinB);   // read the input pin
  if (LeftEncoderBSet) {
    Left_Encoder_Ticks++;
    lEnc++;
  }
  else {
    Left_Encoder_Ticks--;
    lEnc--;
  }
  // Left_Encoder_Ticks -= LeftEncoderBSet ? -1 : +1;
}

void do_Right_Encoder()
{
  bPubEncoder = true;
  RightEncoderBSet = digitalRead(Right_Encoder_PinB);   // read the input pin
  if (RightEncoderBSet) {
    Right_Encoder_Ticks--;
    rEnc--;
  }
  else {
    Right_Encoder_Ticks++;
    rEnc++;
  }
  // Right_Encoder_Ticks += RightEncoderBSet ? -1 : +1;
}

void Set_Speed()
{
  // not that launchpad sends wheel_speed in left first and then right orders.
  motor_left_speed = Messenger_Handler.readLong();
  motor_right_speed = Messenger_Handler.readLong();
}

void Update_Motors()
{
  moveRightMotor(motor_right_speed);
  moveLeftMotor(motor_left_speed);
  /* launchpad doesn't take care of 's'
  Serial.print("s");
  Serial.print("\t");
  Serial.print(motor_left_speed);
  Serial.print("\t");
  Serial.print(motor_right_speed);
  Serial.print("\n");
  Serial.flush();
  */
}

void Update_Encoders()
{  
  Serial.print("e");
  Serial.print("\t");
  Serial.print(Left_Encoder_Ticks);
  Serial.print("\t");
  Serial.print(Right_Encoder_Ticks);
  Serial.print("\n"); 
  //Serial.flush();
}

void Update_Ultra_Sonic()
{
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  // The echo pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  duration = pulseIn(echo, HIGH);
  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);

  //Sending through serial port
  Serial.print("u");
  Serial.print("\t");
  Serial.print(cm);
  Serial.print("\n");
}

void Update_MPU6050()
{
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  ///Update values from DMP for getting rotation vector
  Update_MPU6050_DMP();
}

void Update_MPU6050_DMP()
{
  //DMP Processing
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {
    ;
  }

  mpuInterrupt = false;
  mpuIntStatus = accelgyro.getIntStatus();
  //get current FIFO count
  fifoCount = accelgyro.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount > 512) {
    // reset so we can continue cleanly
    accelgyro.resetFIFO();
  }
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = accelgyro.getFIFOCount();

    // read a packet from FIFO
    accelgyro.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);

    Serial.print("i"); Serial.print("\t");
    Serial.print(q.x); Serial.print("\t");
    Serial.print(q.y); Serial.print("\t");
    Serial.print(q.z); Serial.print("\t");
    Serial.print(q.w);
    Serial.print("\n");

#endif

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);
    accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetAccel(&aa, fifoBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);
    accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetAccel(&aa, fifoBuffer);
    accelgyro.dmpGetGravity(&gravity, &q);
    accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    accelgyro.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
    // display quaternion values in InvenSense Teapot demo format:
    teapotPacket[2] = fifoBuffer[0];
    teapotPacket[3] = fifoBuffer[1];
    teapotPacket[4] = fifoBuffer[4];
    teapotPacket[5] = fifoBuffer[5];
    teapotPacket[6] = fifoBuffer[8];
    teapotPacket[7] = fifoBuffer[9];
    teapotPacket[8] = fifoBuffer[12];
    teapotPacket[9] = fifoBuffer[13];
    Serial.write(teapotPacket, 14);
    teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif
  }
}

void Update_Time()
{
  CurrentMicrosecs = micros();
  LastUpdateMillisecs = millis();
  MicrosecsSinceLastUpdate = CurrentMicrosecs - LastUpdateMicrosecs;
  if (MicrosecsSinceLastUpdate < 0)
  {
    // MicrosecsSinceLastUpdate = INT_MIN - LastUpdateMicrosecs + CurrentMicrosecs;
  }
  LastUpdateMicrosecs = CurrentMicrosecs;
  SecondsSinceLastUpdate = MicrosecsSinceLastUpdate / 1000000.0;

  Serial.print("t");
  Serial.print("\t");
  Serial.print(LastUpdateMicrosecs);
  Serial.print("\t");
  Serial.print(SecondsSinceLastUpdate);
  Serial.print("\n");
}

//Conversion of microsecond to centimeter
long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

void Update_Battery()
{
  //battery_level = analogRead(PC_4);

  Serial.print("b");
  Serial.print("\t");
  Serial.print(battery_level);
  Serial.print("\n");

}

void moveLeftMotor(float out)
{
  digitalWrite(A_1, out < 0);
  digitalWrite(B_1, out > 0);
  analogWrite(PWM_1, abs(out));
}

void moveRightMotor(float out)
{
  digitalWrite(A_2, out < 0);
  digitalWrite(B_2, out > 0);
  analogWrite(PWM_2, abs(out));
}
