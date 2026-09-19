#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display(128, 64, &Wire, -1);

int buttonPin = 2;
int lastButtonState = HIGH;
bool buttonPressed = false;
unsigned long pressStart = 0;
unsigned long pressTime = 0;
unsigned long lastTime = 0;

// 编译时的当前时间，上传后由 millis() 每秒更新
int hour = (__TIME__[0] - '0') * 10 + (__TIME__[1] - '0');
int minute = (__TIME__[3] - '0') * 10 + (__TIME__[4] - '0');
int second = (__TIME__[6] - '0') * 10 + (__TIME__[7] - '0');

void setup() {
    pinMode(buttonPin, INPUT_PULLUP);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
}

void loop() {
    int buttonState = digitalRead(buttonPin);

    // 每秒更新时间
    if (millis() - lastTime >= 1000) {
        lastTime += 1000;
        second++;
        if (second >= 60) {
            second = 0;
            minute++;
        }
        if (minute >= 60) {
            minute = 0;
            hour++;
        }
        if (hour >= 24) {
            hour = 0;
        }
    }

    // 按键去抖
    if (buttonState != lastButtonState) {
        delay(20);
        buttonState = digitalRead(buttonPin);
    }

    // 按下时开始计时，松开时保存按键时长
    if (buttonState == LOW && !buttonPressed) {
        pressStart = millis();
        buttonPressed = true;
    }
    if (buttonState == HIGH && buttonPressed) {
        pressTime = millis() - pressStart;
        buttonPressed = false;
    }
    lastButtonState = buttonState;

    // OLED 显示
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Name: Lin Xinjie");
    display.println("ID: 202536210121");
    display.print("Time: ");
    if (hour < 10) display.print("0");
    display.print(hour);
    display.print(":");
    if (minute < 10) display.print("0");
    display.print(minute);
    display.print(":");
    if (second < 10) display.print("0");
    display.println(second);
    display.print("Press: ");
    if (buttonPressed) {
        display.print(millis() - pressStart);
    } else {
        display.print(pressTime);
    }
    display.println(" ms");
    display.display();

    delay(30);
}
