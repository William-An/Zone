#include "Arduino.h"
#include "String_process.h"

boolean string_pattern_checker(char* string, char* pattern){//Check if one string contain specific pattern
  for(int i=0;i<MAX_BUFFER_SIZE;i++){
    if (pattern[i]==NULL) return true;
    if (pattern[i]!=string[i]) return false;
  }
}

char* substring_reader(char* src,int max_length,char start_char,char terminator){//Maybe I can use this function to subplace the "while (src.available() > 0)" loop in dynamic_array_serial
  char* temp=(char*)malloc(sizeof(char) * max_length);
  int counter=0;
  int array_length=0;
  while(src[counter]!=start_char) counter++;//Wait until find the start_char
  counter++;//Can remove this line by using advanced approach?
  if(src[counter]==NULL) return NULL;//Counldn't find the start_char? Return NULL!
  while(src[counter]!=terminator){//Fill the acceptable chars into temp containter
    temp[array_length]=src[counter];
    counter++;
    array_length++;
  }
  //Serial.println(src);
  if(array_length==0) return NULL;//Can remove this line by using advanced approach?
  char* result=(char*)malloc(sizeof(char) * array_length);//Initialize result pointer
  for(int i=0;i<array_length;i++) result[i]=temp[i];
  Serial.println(array_length);
  free(temp);//Free memory spend by temp containter
  return result;
}

void chars_printer(char* src){
  int i=0;
  while(src[i]!=NULL) {
    Serial.print(src[i]);
    i++;
  }
  Serial.println();
}
