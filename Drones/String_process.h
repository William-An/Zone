#ifndef STRING_PROCESS
#define STRING_PROCESS
#include "Arduino.h"
#define MAX_BUFFER_SIZE 150
char* substring_reader(char*,int,char,char);//Src, maximum, start_char, terminator
void chars_serial_printer(char*);
boolean string_pattern_checker(char*,char*);//string, pattern
#endif