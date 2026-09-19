#include <Arduino.h>
int num = 0;
void setup() {

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, INPUT_PULLUP);    // 加1
  pinMode(10, INPUT_PULLUP);   // 减1

}

void loop() {

  // 加1
  if (digitalRead(9) == LOW) {
    delay(20);
    if (digitalRead(9) == LOW) {
      num++;
      if (num > 9) {
        num = 0;
      }
      while (digitalRead(9) == LOW) {
      }
    }
  }

  // 减1
  if (digitalRead(10) == LOW) {
    delay(20);
    if (digitalRead(10) == LOW) {
      num--;
      if (num < 0) {
        num = 9;
      }
      while (digitalRead(10) == LOW) {
      }
    }
  }

  // 初始化
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);

  // 显示0
  if (num == 0) {
    digitalWrite(3, HIGH);
    digitalWrite(4, HIGH);
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(7, HIGH);
    digitalWrite(8, HIGH);
  }

  // 显示1
  if (num == 1) {
    digitalWrite(5, HIGH);
    digitalWrite(8, HIGH);
  }

  // 显示2
  if (num == 2) {
    digitalWrite(2, HIGH);
    digitalWrite(4, HIGH);
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(7, HIGH);
  }

  // 显示3
  if (num == 3) {
    digitalWrite(2, HIGH);
    digitalWrite(4, HIGH);
    digitalWrite(5, HIGH);
    digitalWrite(7, HIGH);
    digitalWrite(8, HIGH);
  }

  // 显示4
  if (num == 4) {
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
    digitalWrite(5, HIGH);
    digitalWrite(8, HIGH);
  }

  // 显示5
  if (num == 5) {
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
    digitalWrite(4, HIGH);
    digitalWrite(7, HIGH);
    digitalWrite(8, HIGH);
  }

  // 显示6
  if (num == 6) {
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
    digitalWrite(4, HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(7, HIGH);
    digitalWrite(8, HIGH);
  }

  // 显示7
  if (num == 7) {
    digitalWrite(4, HIGH);
    digitalWrite(5, HIGH);
    digitalWrite(8, HIGH);
  }

  // 显示8
  if (num == 8) {
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
    digitalWrite(4, HIGH);
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
    digitalWrite(7, HIGH);
    digitalWrite(8, HIGH);
  }

  // 显示9
  if (num == 9) {
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
    digitalWrite(4, HIGH);
    digitalWrite(5, HIGH);
    digitalWrite(7, HIGH);
    digitalWrite(8, HIGH);
  }
}