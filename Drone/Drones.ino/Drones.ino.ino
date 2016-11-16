//Core library
#include <MPU6050_6Axis_MotionApps20.h>
#include <I2Cdev.h>
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
//Specify the I2C address of MPU6050, default I2C adress is 0x68
MPU6050 mpu;
/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */
//uncomment the following define in order to get the information you want
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_YAWPITCHROLL
//Uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration components with gravity removed.
#define OUTPUT_READABLE_REALACCEL







void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
