//Core library
#include <MPU6050_6Axis_MotionApps20.h>
#include <I2Cdev.h>
#include <SoftwareSerial.h>
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
// MPU6050_6Axis_MotionApps20.h and I2Cdev.h can get from Jeff Rowberg's github: https://github.com/jrowberg/i2cdevlib
// SoftwareSerial.h is used to read data from GPS without interrupt with the Xbee
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
//Specify the I2C address of MPU6050, default I2C adress is 0x68
MPU6050 mpu;
//uncomment the following define in order to get the data you want
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_YAWPITCHROLL
//Uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration components with gravity removed.
#define OUTPUT_READABLE_REALACCEL
#define INTERRUPT_PIN 2
#define LED_PIN 12
#define MOTOR_1_PIN 3
#define MOTOR_2_PIN 5
#define MOTOR_3_PIN 6
#define MOTOR_4_PIN 9
#define SOFT_SERIAL_TX 11
#define SOFT_SERIAL_RX 12 
int throttle_stick = 0;
int yaw_stick = 0;
int pitch_stick = 0;
int roll_stick = 0;
int motor_1_output;
int motor_2_output;
int motor_3_output;
int motor_4_output;
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
double gps_coordinate[2]; // first one is latitude(positive for northern, negative for southern), second one is longtitude(positive for eastern, negative for western)
double gps_height;
double gps_UTCtime;
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
char xbeebuffer[10];
// Initialize buffer for controller
SoftwareSerial gpsSerial (SOFT_SERIAL_RX,SOFT_SERIAL_TX);






void parse_gps(char buffer[70]);
void setup() {
	pinMode(MOTOR_1_PIN, OUTPUT);
	pinMode(MOTOR_2_PIN, OUTPUT);
	pinMode(MOTOR_3_PIN, OUTPUT);
	pinMode(MOTOR_4_PIN, OUTPUT);
	// Define all motor pins as output
	pinMode(SOFT_SERIAL_RX, INPUT);
	pinMode(SOFT_SERIAL_TX, OUTPUT);
	// Define softSerial pins' mode 
	Serial.begin(9600);
	gpsSerial.begin(9600);
	// Begin both serials
}

void loop() {
  // put your main code here, to run repeatedly:
  char gpsbuffer[70];
// Initialize buffer for gps data
 	if (gpsSerial.available() > 0){
    if(gpsSerial.read()=='$'){
  	gpsSerial.readBytesUntil("*",gpsbuffer,70);
    }
    }
  for(int i;i<70;i++){
  	Serial.print(gpsbuffer[i]);
   //gpsbuffer[i]=" ";
  }
  Serial.println();
//	For Testing only

  delay(1000);
}
void parse_gps(char buffer[70]){

}
