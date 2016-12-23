#ifndef SOFT_SERIAL_READER
#define SOFT_SERIAL_READER
#include "Arduino.h"
#include "String_process.h"
#include <SoftwareSerial.h>
#define GPS_START_PATTERN "GPRMC"
char* dynamic_array_serial(SoftwareSerial, int, char, char);//serial, maximum storge, start_char, terminator
#endif