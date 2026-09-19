#include <Arduino.h>

int pinRed = 9;
int pinGreen = 10;
int pinBlue = 11;

void setup() {
  pinMode(pinRed, OUTPUT);
  pinMode(pinGreen, OUTPUT);
  pinMode(pinBlue, OUTPUT);

}

void loop() {
    //红
    digitalWrite(pinRed, HIGH);
    delay(1000);
    //绿
    digitalWrite(pinRed, LOW);
    digitalWrite(pinGreen, HIGH);
    delay(1000);
    //蓝
    digitalWrite(pinGreen, LOW);
    digitalWrite(pinBlue, HIGH);
    delay(1000);
    //黄
    digitalWrite(pinRed, HIGH);
    digitalWrite(pinGreen, HIGH);
    digitalWrite(pinBlue, LOW);
    delay(1000);
    //品
    digitalWrite(pinGreen, LOW);
    digitalWrite(pinBlue, HIGH);
    delay(1000);
    //青
    digitalWrite(pinRed, LOW);
    digitalWrite(pinGreen, HIGH);
    delay(1000);
    //白
    digitalWrite(pinRed, HIGH);
    digitalWrite(pinBlue, HIGH);
    delay(1000);
    digitalWrite(pinRed, LOW);
    digitalWrite(pinGreen, LOW);
    digitalWrite(pinBlue, LOW);
}