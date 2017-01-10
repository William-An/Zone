#include "Arduino.h"
#include "MPU6050_reader.h"
void dmpDataReady() {
    mpuInterrupt = true;
}
void mpuinitialization(){
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    
}