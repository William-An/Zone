//----------------------------License-----------------------------------------------//
/*
  MIT License

  Copyright (c) 2016 - 2017 Weili An

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
#include "BMP085.h"
#include "Soft_serial_reader.h"
#include "String_process.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
// MPU6050_6Axis_MotionApps20.h and I2Cdev.h can get from Jeff Rowberg's github: https://github.com/jrowberg/i2cdevlib
// SoftwareSerial.h is used to read data from GPS without interrupt with the Xbee
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// MPU
MPU6050 mpu;
#define INTERRUPT_PIN 2
#define OUTPUT_READABLE_YAWPITCHROLL
#define XGYRO_OFFSET 220
#define YGYRO_OFFSET 76
#define ZGYRO_OFFSET -85
#define ZACCEL_OFFSET 1788
bool blinkState = false;
bool dmpReady = false;
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// BMP
BMP085 barometer;
float temperture;
float pressure;
float altitude;

// GPS
#define SOFT_SERIAL_TX 11
#define SOFT_SERIAL_RX 12
#define MAX_BUFFER_SIZE 150
#define START_CHAR '$'
#define TERMINATOR '*'
#define GPS_START_PATTERN "GPRMC"
int array_length = 1; //To store array_length
/*double gps_coordinate[2]; // first one is latitude(positive for northern, negative for southern), second one is longtitude(positive for eastern, negative for western)
double gps_height;
double gps_UTCtime;*/

// Motor
#define MOTOR_1_PIN 3
#define MOTOR_2_PIN 5
#define MOTOR_3_PIN 6
#define MOTOR_4_PIN 9
int throttle_stick = 0;
int yaw_stick = 0;
int pitch_stick = 0;
int roll_stick = 0;
int motor_1_output;
int motor_2_output;
int motor_3_output;
int motor_4_output;

// Xbee
// Initialize buffer for controller
char* xbeebuffer;

// Timer
long timer;

// 
// Define software serial
SoftwareSerial gpsSerial (SOFT_SERIAL_RX, SOFT_SERIAL_TX);

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}




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

  	// Begin communication
  	Serial.begin(9600);
  	gpsSerial.begin(9600);
  	Wire.begin();
  	Wire.setClock(400000);

  	// Initialize sensors
  	// Need some log output
  	mpu.Initialize();
  	pinMode(INTERRUPT_PIN,INPUT);
  	barometer.Initialize();
  	// Testing connection

  	// Empty buffer
  	while(Serial.available()&&Serial.read());
  	while(!Serial.read());
  	while(Serial.available()&&Serial.read());
  	devStatus=mpu.dmpInitialize(); // Update dmp status

  	// Set offset
  	mpu.setXGyroOffset(XGYRO_OFFSET);
  	mpu.setYGyroOffset(YGYRO_OFFSET);
  	mpu.setZGyroOffset(ZGYRO_OFFSET);
  	mpu.setZAccelOffset()ZACCEL_OFFSET;

  	// Check mpu status
  	if(devStatus == 0){
  		mpu.setDMPEnabled(true);
  		// Set interrupt
  		attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  		mpuIntStatus = mpu.getIntStatus();
  		dmpReady = true;
  		// get expected DMP packet size for later comparison
    	packetSize = mpu.dmpGetFIFOPacketSize();
  	}else{
  		// 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
  	}

  	timer = micros();
}

//----------------------------Main Loop-----------------------------------------------//
void loop() {
	// ERROR Printer
	// MPU
	if (!dmpReady) Serial.println("MPU Failed");



  	// 1 Hz interrupt
  	// GPS Update
  	if (timer-micros() > 1000){
  		char* gpsbuffer;
  		char* gpstime;
  		// Open softserial
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
	
  		gpsbuffer = dynamic_array_serial(gpsSerial, MAX_BUFFER_SIZE , '$', '*');// Try to read data from gpsSerial
  		//Serial.println("Succeed in loading buffer with data");
  		//Serial.println(array_length);
  		if(gpsbuffer[0]!=NULL){
  		for (int i; i < array_length - 1; i++) {//Print all data in gpsbuffer
  	  		Serial.print(gpsbuffer[i]);
  		}
  		Serial.println();
  		gpstime=substring_reader(gpsbuffer,array_length,',',',');
  		chars_printer(gpstime);//Print char that shouldn't be printed
  		//Serial.println(gpstime[0]);//Test function substring_reader
  		//if(gpstime== NULL) Serial.println("NULL"); 
  		//Serial.println();
  		}
  		array_length = 1;
  
  		free(gpsbuffer);//Free memory that given to temp containter
  		free(gpstime);
  		//elay(1000);//Waiting for bytes to fill IO buffer
  		timer = micros(); // Update timer
  		}

  	// 10 Hz interrupt

  	// 100 Hz interrupt
  	// MPU Xbee
  	if(timer - micros() > 10){
  		while (!mpuInterrupt && fifoCount < packetSize) {
  			// BMP
  			barometer.setControl(BMP085_MODE_TEMPERATURE);
  			timer = micros();
  			// Wait for conversion
  			while(micros() - timer < barometer.getMeasureDelayMicroseconds());
  			temperture = barometer.getTempertureC(); // In degree Celsius
  			barometer.setControl(BMP085_MODE_PRESSURE_3);
  			while(micros() - timer < barometer.getMeasureDelayMicroseconds());
  			pressure = barometer.getPressure();
  			altitude = barometer.getAltitude(pressure);
  			// Debug output
  			Serial.print(micros());
  			Serial.print("\tT/P/A \t");
  			Serial.print(temperture);
  			Serial.print("\t");
  			Serial.print(pressure);
  			Serial.print("\t");
  			Serial.println(altitude);

  			timer = micros();
    	}

    	// MPU 
    	mpuIntStatus = false;
    	mpuIntStatus = mpu.getIntStatus();
    	fifoCount = mpu.getFIFOCount();
    	// Check for overflow
    	if((mpuIntStatus & 0x10) || fifoCount == 1024){
    		// Reset buffer
    		mpu.resetFIFO();
    		Serial.println("MPU FIFO overflow!");
    	// otherwise, check for DMP data ready interrupt (this should happen frequently)
    	}else if(mpuIntStatus & 0x02){
    		// wait for correct available data length, should be a VERY short wait
    		while(fifoCount < packetSizec) fifoCount = mpu.getFIFOCount();
    		// read a packet from FIFO
        	mpu.getFIFOBytes(fifoBuffer, packetSize);
        	// track FIFO count here in case there is > 1 packet available
        	// (this lets us immediately read more without waiting for an interrupt)
        	fifoCount -= packetSize;
        	#ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            /*
            Serial.print(micros());
            Serial.print("\typr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);*/
        	#endif
    	}
    	timer = micros();
  	}

  	// Other stuff?
  	// Control 
}
