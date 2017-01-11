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
//Basic lib
#include <SoftwareSerial.h>
#include "Soft_serial_reader.h"
#include "String_process.h"
#include "MPU6050_reader.h"
//Define pins
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
  mpuInitialization();
}

//----------------------------Main Loop-----------------------------------------------//
void loop() {
  mpuReader();
  /*
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
    Serial.println("Waiting for buffer to be filled");*

  gpsbuffer = dynamic_array_serial(gpsSerial, MAX_BUFFER_SIZE , '$', '*');//TTry to read data from gpsSerial
  //Serial.println("Succeed in loading buffer with data");
  //Serial.println(array_length);
  if(gpsbuffer[0]!=NULL){
  for (int i; i < array_length - 1; i++) {//Print all data in gpsbuffer
    Serial.print(gpsbuffer[i]);
  }
  Serial.println();
  gpstime=substring_reader(gpsbuffer,MAX_BUFFER_SIZE,',',',');
  chars_serial_printer(gpstime);//Print char that shouldn't be printed
  //Serial.println(gpstime[0]);//Test function substring_reader
  //if(gpstime== NULL) Serial.println("NULL"); 
  //Serial.println();
  }
  array_length = 1;
  
  free(gpsbuffer);//Free memory that given to temp containter
  free(gpstime);
  delay(1000);//Waiting for bytes to fill IO buffer
  */
}
