#include <Arduino.h>

int pinRed = 9;
int pinGreen = 10;
int pinBlue = 11;

void setup() 
{
  pinMode(pinRed, OUTPUT);
  pinMode(pinGreen, OUTPUT);
  pinMode(pinBlue, OUTPUT);
}

void loop() 
{
//初始红
analogWrite(pinRed,255);
//黄
for(int i=0;i<255;i++){
  analogWrite(pinGreen,i);
  delay(10);
}
//绿
for(int i=255;i>0;i--){
  analogWrite(pinRed,i);
  delay(10);
}
//青
for(int i=0;i<255;i++){
  analogWrite(pinBlue,i);
  delay(10);
}
//蓝
for(int i=255;i>0;i--){
  analogWrite(pinGreen,i);
  delay(10);
}
//紫
for(int i=0;i<255;i++){
  analogWrite(pinRed,i);
  delay(10);
}
//红
for(int i=255;i>0;i--){
  analogWrite(pinBlue,i);
  delay(10);
}
}