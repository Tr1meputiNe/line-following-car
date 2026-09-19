#include <Arduino.h>

void setup()
{
    Serial.begin(9600);
    pinMode(9, OUTPUT);
}

void loop()
{
    //读取模拟输入
    int x = analogRead(A0);
    //换算电压值
    float v = x * (5.140 / 1023.0);
    //将模拟输入映射到0-255用于led亮度控制
    int led = map(x, 0, 1023, 0, 255);
    analogWrite(9, led);
    //串口输出电压值
    Serial.print("电压为：");
    Serial.print(v, 3);
    Serial.println("V");
    delay(500);
}