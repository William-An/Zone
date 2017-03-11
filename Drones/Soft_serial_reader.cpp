#include "Arduino.h"
#include "Soft_serial_reader.h"
#include "String_process.h"
#include <SoftwareSerial.h>
/*
GPRMC Structure
Where:
    RMC          Recommended Minimum sentence C
    123519       Fix taken at 12:35:19 UTC
    A            Status A=active or V=Void.
    4807.038,N   Latitude 48 deg 07.038' N
    01131.000,E  Longitude 11 deg 31.000' E
    022.4        Speed over the ground in knots
    084.4        Track angle in degrees True
    230394       Date - 23rd of March 1994
    003.1,W      Magnetic Variation
    *6A          The checksum data, always begins with *
From http://www.gpsinformation.org/dale/nmea.htm 
*/
char* dynamic_array_serial(SoftwareSerial src, int max_length, char start_char, char terminator) {
  src.listen();//Open serial
  while (src.available() <= 10); //Waiting for bytes to fill buffer, set 10 in order to allow more bytes loaded in buffer
  while (src.read() != start_char); //Waiting for the start char
  extern int array_length;//Use global varible to store length
  //Serial.println("Successful");
  //Serial.println(array_length); //Testing
  if (src.available() > 0) {
    //Serial.println("Checking IO buffer");
    //Serial.println(src.peek());
    //Serial.println("Initializing buffer");
    char*  max_array = (char*)malloc(sizeof(char) * max_length);
    for (int i = 0; i < max_length; i++) {
      max_array[i] = NULL;
    }
    //Serial.println("Start reading");
    while (src.available() > 0) {
      char temp = src.read();
      delay(20);//This 20ms delay is necessary in order to allow buffer being fill by data from serial
      if (temp != terminator) {
        max_array[array_length - 1] = temp;
        array_length++;
        //Serial.println(max_array[array_length-1]);
      }
      else {
        //Serial.print("Array length: ");
        //Serial.println(array_length);
        char* result = (char*)malloc(sizeof(char) * array_length);
        for (int i = 0; i < max_length; i++) {
          if (max_array[i] == NULL) {
            free(max_array);//free memory
            //Serial.println("End listening");
            //Serial.println(result[0]);
            if(string_pattern_checker(result,GPS_START_PATTERN)){//Need to add parameter in dynamic_array_function to subplace this GPS_START_PATTERN
              return result;
            }
            else return NULL;//Not the data we want? Return NULL!
          }
          else {
            result[i] = max_array[i];
            //Serial.print(max_array[i]);
            //Serial.print(":");
            //Serial.println(result[i]);
          }
        }
      }
    }
  }
  else return NULL;//Src has no data waiting to read? Return NULL!
}







