#ifndef SOFT_SERIAL_READER
#define SOFT_SERIAL_READER
#include "Arduino.h"
#include <SoftwareSerial.h>
#define GPS_START_PATTERN "GPRMC"
#define MAX_BUFFER_SIZE 150
char* dynamic_array_serial(SoftwareSerial, int, char, char);//serial, maximum storge, start_char, terminator
char* substring_reader(char*,int,char,char);//Src, maximum, start_char, terminator
boolean string_pattern_checker(char*,char*);//string, pattern
#endif