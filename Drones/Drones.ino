//----------------------------License-----------------------------------------------//
/*
  MIT License

  Copyright (c) 2016 Weili An

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

//-----------------------------Code----------------------------------------------//
//Core library
#include <MPU6050_6Axis_MotionApps20.h>
#include <I2Cdev.h>
#include <SoftwareSerial.h>
#include "Soft_serial_reader.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
// MPU6050_6Axis_MotionApps20.h and I2Cdev.h can get from Jeff Rowberg's github: https://github.com/jrowberg/i2cdevlib
// SoftwareSerial.h is used to read data from GPS without interrupt with the Xbee
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
//Specify the I2C address of MPU6050, default I2C adress is 0x60
MPU6050 mpu;
//uncomment the following define in order to get the data you want
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_YAWPITCHROLL
//Uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration components with gravity removed.
#define OUTPUT_READABLE_REALACCEL
//Define pins
#define INTERRUPT_PIN 2
#define LED_PIN 12
#define MOTOR_1_PIN 3
#define MOTOR_2_PIN 5
#define MOTOR_3_PIN 6
#define MOTOR_4_PIN 9
#define SOFT_SERIAL_TX 11
#define SOFT_SERIAL_RX 12
#define MAX_BUFFER_SIZE 150
#define START_CHAR '$'
#define TERMINATOR '*'
#define GPS_START_PATTERN "GPRMC"
//Define varibles
int throttle_stick = 0;
int yaw_stick = 0;
int pitch_stick = 0;
int roll_stick = 0;
int motor_1_output;
int motor_2_output;
int motor_3_output;
int motor_4_output;
int array_length = 1; //To store array_length
bool blinkState = false;
bool dmpReady = false;
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
/*double gps_coordinate[2]; // first one is latitude(positive for northern, negative for southern), second one is longtitude(positive for eastern, negative for western)
double gps_height;
double gps_UTCtime;*/
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
char* xbeebuffer;
// Initialize buffer for controller
SoftwareSerial gpsSerial (SOFT_SERIAL_RX, SOFT_SERIAL_TX);







//----------------------------Setup Code-----------------------------------------------//
void setup() {
  //Define all motor pins as output
  pinMode(MOTOR_1_PIN, OUTPUT);
  pinMode(MOTOR_2_PIN, OUTPUT);
  pinMode(MOTOR_3_PIN, OUTPUT);
  pinMode(MOTOR_4_PIN, OUTPUT);
  //Define softSerial pins' mode
  pinMode(SOFT_SERIAL_RX, INPUT);
  pinMode(SOFT_SERIAL_TX, OUTPUT);
  Serial.begin(9600);
  gpsSerial.begin(9600);
  // Begin both serials
}

//----------------------------Main Loop-----------------------------------------------//
void loop() {
  // put your main code here, to run repeatedly:
  char* gpsbuffer;
  char* gpstime;
  gpsSerial.listen();
  /* Testing message
    Initialize buffer for gps data
    Serial.println("Try to communicate");
    Serial.print("Buffer size: ");
    Serial.println(gpsSerial.available());
    Serial.print("First byte in buffer: ");
    Serial.println(gpsSerial.peek());
    Serial.println(gpsSerial.available());
    Serial.println(gpsSerial.read());
    Serial.println("Waiting for buffer to be filled");*/

  gpsbuffer = dynamic_array_serial(gpsSerial, MAX_BUFFER_SIZE , '$', '*');//TTry to read data from gpsSerial
  //Serial.println("Succeed in loading buffer with data");
  //Serial.println(array_length);
  if(gpsbuffer[0]!=NULL){
  for (int i; i < array_length - 1; i++) {//Print all data in gpsbuffer
    Serial.print(gpsbuffer[i]);
  }
  Serial.println();
  gpstime=substring_reader(gpsbuffer,MAX_BUFFER_SIZE,',',',');
  //Serial.println(gpstime[0]);//Test function substring_reader
  //if(gpstime== NULL) Serial.println("NULL"); 
  //Serial.println();
  }
  array_length = 1;
  
  free(gpsbuffer);//Free memory that given to temp containter
  delay(1000);//Waiting for bytes to fill IO buffer
}
