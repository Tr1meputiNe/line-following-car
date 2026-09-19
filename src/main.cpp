/* =====================================================================
 *  光电循迹小车  ——  主程序
 *
 *  硬件：Arduino UNO + MX1508 电机驱动 + 2 路光电对管 + 0.96" OLED(SSD1306)
 *
 *  运行流程：
 *     上电        -> OLED 显示 "Press START"，停在这里等按键
 *     按启动键    -> 倒计时 3-2-1
 *     发车        -> 直行冲出出发区，压到起始线的那一刻开始计时
 *     循迹        -> 沿黑线跑一圈
 *     再压起始线  -> 停车，OLED 显示成绩
 *
 *  要改接线：只改"引脚"那一段
 *  要调参数：只改"参数"那一段
 *
 *  两个传感器是"夹住黑线"的摆法：车正的时候黑线从两个传感器中间穿过，
 *  两个都读到白底；哪一边读到黑线，就说明线偏到哪一边，往那一边转。
 * ===================================================================== */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================== 引脚 ==============================
// 【实测】扩展板丝印标的左右和实际是反的，下表按实测写死，和丝印不一致是正常的。
// 左右以"人站在车后、车头朝前"为准。
// 验证方法：烧 bringup，敲 q（让左轮变慢）看哪个物理轮子慢；
//           敲 r，手指挡车的左侧传感器，看哪一列数字变。
int leftSensorPin  = A0;
int rightSensorPin = A1;

// MX1508：实测 MOTOR-B(D9/D10) 驱动左轮，MOTOR-A(D5/D6) 驱动右轮
// 哪一轮方向反了，就交换该轮 Forward / Back 的两个引脚号
int leftForward  = 9;
int leftBack     = 10;
int rightForward = 5;
int rightBack    = 6;

int buzzer      = 4;    // 扩展板 BUZ（用 bringup 的 z 命令扫出来的）
int startButton = 7;    // S1，板上有外部下拉电阻：松开 LOW，按下 HIGH
int ledRun      = 11;   // LED2
int ledStat     = 12;   // LED1

// ============================== 参数 ==============================
// ---- 黑白判断 ----
// 【实测】黑线读数 915~949，白底读数 705~762，取中间值 840。
// 余量：黑线最低 915 比 840 高 75，白底最高 762 比 840 低 78。
int threshold = 840;
int blackHigh = 1;       // 黑线读数大填 1；若压黑线读到小数字，改成 0

// ---- 速度（0~255）----
int speedFast = 130;     // 直行速度（两个轮子一样）
int speedTurn = 135;     // 转弯时【外侧】轮速度
int speedBack = 60;      // 转弯时【内侧】轮【反转】速度
// 外侧轮往前冲 + 内侧轮往后倒，车身几乎是原地转，这是双轮差速能做出来最狠的转向。
// 两个数越大转得越急，也越容易冲过头来回摆；车在直线上画龙就把它们往小调。

// ---- 左右轮补偿 ----
// 车往左偏 = 左轮偏慢 -> trimLeft 填正数（先试 5、10、15）
// 车往右偏 = 右轮偏慢 -> trimLeft 填负数（先试 -5、-10、-15）
int trimLeft = 0;
int fastLeft = speedFast + trimLeft;   // 左轮直行速度，自动算出来
int turnLeft = speedTurn + trimLeft;   // 左轮转弯外侧速度，自动算出来

int startKickMs = 150;   // 起步先满速"踢"多久（毫秒）。起不来就加到 300
// 直流电机静止时阻力比转动时大得多，直接给循迹速度常常原地不转。

// ---- 计时和起停 ----
// 注意：Uno 上 int 最大只有 32767，180000 装不下，毫秒一律用 unsigned long
unsigned long startLineMs = 80;      // 双黑连续保持这么久才算"起始线"
unsigned long lapMinMs    = 5000;    // 跑够这么久才允许停车（防止刚出发就误判）
unsigned long lapMaxMs    = 180000;  // 3 分钟限时
unsigned long launchMaxMs = 5000;    // 出发后最多找多久起始线，找不到就停
unsigned long oledMs      = 100;     // 循迹时每隔多久刷新一次屏幕
// 【起始线怎么定的】起始线宽 3cm，车速约 20~25cm/s，压线 120~150ms，取 80ms 留余量。
// 跑完一圈停不下来 -> 降到 60；弯道提前停车 -> 加到 110。

// ---- 调试开关（都调好之后改成 0）----
int debugLaunch = 1;   // 1 = 发车阶段把传感器读数打到串口
int debugOled   = 1;   // 1 = 在 OLED 上显示调试信息（跑完停屏"验尸"）

// [重要] 下面所有字符串都写成 F("...")。AVR 上普通字符串占 RAM，F() 把它放进 Flash。
// 原因：128x64 的 OLED 需要 malloc 1024 字节显存，UNO 只有 2048 字节 RAM，
//       字符串占多了 malloc 就会失败，屏幕一片黑，而编译器不会报任何警告。
Adafruit_SSD1306 oled(128, 64, &Wire, -1);

// ============================== setup ==============================
void setup() {
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);

  pinMode(leftForward, OUTPUT);
  pinMode(leftBack, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBack, OUTPUT);

  pinMode(startButton, INPUT);   // 板上已有下拉，不启用内部上拉
  pinMode(ledRun, OUTPUT);
  pinMode(ledStat, OUTPUT);
  pinMode(buzzer, OUTPUT);

  // 初始化屏幕之前先确保两个电机都停住
  analogWrite(leftForward, 0);
  analogWrite(leftBack, 0);
  analogWrite(rightForward, 0);
  analogWrite(rightBack, 0);

  Serial.begin(9600);

  // MCUSR 是 AVR 记录上一次复位原因的寄存器，用来判断单片机有没有重启过
  int resetCause = MCUSR;
  MCUSR = 0;                     // 读完清掉，下次开机才是新的原因
  if (debugLaunch == 1) {
    Serial.print(F("reset cause = 0x"));
    Serial.println(resetCause, HEX);
    Serial.println(F("  bit0 上电  bit1 复位键  bit2 电压过低  bit3 看门狗"));
  }

  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);   // 0x3C 是屏幕的 I2C 地址
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);         // SSD1306_WHITE 就是"点亮这个点"
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println(F("Line Car Ready"));
  oled.print(F("threshold = "));
  oled.println(threshold);
  oled.display();
}

// ============================== loop ==============================
void loop() {
  // =================================================================
  // 第 1 步  待机，等启动键按下
  // =================================================================
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println(F("Line Following Car"));
  oled.println(F(""));
  oled.println(F("Press START"));
  oled.setCursor(0, 40);
  oled.print(F("th = "));
  oled.println(threshold);
  oled.display();

  digitalWrite(ledRun, LOW);
  digitalWrite(ledStat, LOW);

  while (digitalRead(startButton) == LOW) {   // 停在这里，直到读到 HIGH（按下）
    delay(10);
  }
  delay(30);                                  // 简易消抖：30ms 后再确认一次
  if (digitalRead(startButton) == LOW) {
    return;                                   // 刚才是抖动，回开头重新等
  }
  while (digitalRead(startButton) == HIGH) {  // 等手指松开，免得一次按压算两次
    delay(10);
  }

  // =================================================================
  // 第 2 步  倒计时 3-2-1
  // =================================================================
  for (int i = 3; i >= 1; i = i - 1) {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.println(F("Ready..."));
    oled.setTextSize(4);
    oled.setCursor(50, 24);
    oled.print(i);
    oled.display();

    tone(buzzer, 900);    // 倒计时短响
    delay(150);
    noTone(buzzer);
    delay(850);
  }

  // =================================================================
  // 第 3 步  发车：直行冲向起始线（这一段不计时）
  // =================================================================
  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setCursor(24, 24);
  oled.print(F("GO !"));
  oled.display();

  digitalWrite(ledRun, HIGH);
  digitalWrite(ledStat, HIGH);

  // 起步先给满速 255"踢一脚"突破齿轮静摩擦，再降回正常的循迹速度
  analogWrite(leftBack, 0);
  analogWrite(rightBack, 0);
  analogWrite(leftForward, 255);
  analogWrite(rightForward, 255);
  delay(startKickMs);
  analogWrite(leftForward, fastLeft);
  analogWrite(rightForward, speedFast);

  unsigned long launchStart = millis();   // 发车时刻，用来做超时保护
  unsigned long dbgTime     = 0;          // 串口调试输出用的计时
  int launchFailed = 0;                   // 1 = 超时没找到起始线

  while (true) {
    int leftValue  = analogRead(leftSensorPin);
    int rightValue = analogRead(rightSensorPin);

    int leftBlack  = 0;      // 1 = 这个传感器压到黑线了
    int rightBlack = 0;
    if (blackHigh == 1) {
      if (leftValue  > threshold) leftBlack  = 1;
      if (rightValue > threshold) rightBlack = 1;
    } else {
      if (leftValue  < threshold) leftBlack  = 1;
      if (rightValue < threshold) rightBlack = 1;
    }

    if (debugLaunch == 1 && millis() - dbgTime > 200) {
      dbgTime = millis();
      Serial.print(millis() - launchStart);
      Serial.print(F("ms  L="));
      Serial.print(leftValue);
      Serial.print(F(" R="));
      Serial.println(rightValue);
    }

    // 出发区是直线，除了起始线不会有别的黑线，看到双黑就是它。
    // 这里【不能】再 delay 确认：车快的时候 150ms 已经冲过整条线了，
    // 再检查一次反而读到白底，结果永远找不到起始线，5 秒后超时停车。
    if (leftBlack == 1 && rightBlack == 1 && millis() - launchStart > 200) {
      break;
    }
    if (millis() - launchStart > launchMaxMs) {
      launchFailed = 1;
      break;
    }
  }

  if (launchFailed == 1) {
    Serial.println(F("LAUNCH FAILED: 没看到双黑，检查起始线和发车位置"));
  }

  // =================================================================
  // 第 4 步  循迹跑一圈
  // =================================================================
  unsigned long startTime = millis();   // ★ 计时从压到起始线这一刻开始

  int finished = 0;                     // 1 = 正常跑完一圈
  unsigned long oledTime      = 0;      // 上一次刷新屏幕的时刻
  unsigned long bothBlackTime = 0;      // 两个传感器"同时开始压黑线"的时刻

  // 调试统计：每刷新一次屏幕采样一次（10 次/秒），跑完显示在屏幕上验尸
  int cntWW = 0;    // 两个都白的次数
  int cntLB = 0;    // 左黑的次数
  int cntRB = 0;    // 右黑的次数
  int cntBB = 0;    // 双黑的次数
  int minL = 1023;  // 左传感器读到过的最小读数
  int maxL = 0;     // 左传感器读到过的最大读数
  int minR = 1023;  // 右传感器读到过的最小读数
  int maxR = 0;     // 右传感器读到过的最大读数

  if (launchFailed == 0) {
    while (true) {
      // ---------- ① 读传感器，判断黑白 ----------
      int leftValue  = analogRead(leftSensorPin);
      int rightValue = analogRead(rightSensorPin);

      int leftBlack  = 0;
      int rightBlack = 0;
      if (blackHigh == 1) {
        if (leftValue  > threshold) leftBlack  = 1;
        if (rightValue > threshold) rightBlack = 1;
      } else {
        if (leftValue  < threshold) leftBlack  = 1;
        if (rightValue < threshold) rightBlack = 1;
      }

      // ---------- ② 判圈 ----------
      // 两个传感器同时压黑线，并且已经跑够 lapMinMs，才算跑完一圈。
      if (leftBlack == 1 && rightBlack == 1) {
        if (bothBlackTime == 0) {
          bothBlackTime = millis();     // 刚刚同时压上，记下时刻
        }
        if (millis() - bothBlackTime > startLineMs && millis() - startTime > lapMinMs) {
          finished = 1;
          break;
        }
      } else {
        bothBlackTime = 0;              // 只要不是双黑，就重新计时
      }

      // ---------- ③ 转向 ----------
      // 哪一边读到黑线，线就在哪一边，就往哪一边转。
      if (leftBlack == 1 && rightBlack == 0) {
        // 线在左边 -> 向左转：左轮倒转、右轮往前冲
        analogWrite(leftForward, 0);
        analogWrite(leftBack, speedBack);
        analogWrite(rightForward, speedTurn);
        analogWrite(rightBack, 0);
      }
      else if (leftBlack == 0 && rightBlack == 1) {
        // 线在右边 -> 向右转：右轮倒转、左轮往前冲
        analogWrite(leftForward, turnLeft);
        analogWrite(leftBack, 0);
        analogWrite(rightForward, 0);
        analogWrite(rightBack, speedBack);
      }
      else {
        // 两个都白 = 车正；两个都黑 = 起始线。两种都直行。
        analogWrite(leftForward, fastLeft);
        analogWrite(leftBack, 0);
        analogWrite(rightForward, speedFast);
        analogWrite(rightBack, 0);
      }

      // ---------- ④ 3 分钟超时保护 ----------
      if (millis() - startTime > lapMaxMs) {
        finished = 0;
        break;
      }

      // ---------- ⑤ 每 oledMs 刷新一次屏幕 ----------
      if (millis() - oledTime > oledMs) {
        oledTime = millis();

        unsigned long passed = millis() - startTime;
        int totalSecond = passed / 1000;
        int minute      = totalSecond / 60;
        int second      = totalSecond % 60;      // % 是取余数
        int hundredth   = (passed % 1000) / 10;

        // 顺便采样一次，给跑完之后的"验尸"用
        if (leftBlack == 1 && rightBlack == 1) cntBB = cntBB + 1;
        else if (leftBlack == 1)               cntLB = cntLB + 1;
        else if (rightBlack == 1)              cntRB = cntRB + 1;
        else                                   cntWW = cntWW + 1;
        if (leftValue  < minL) minL = leftValue;
        if (leftValue  > maxL) maxL = leftValue;
        if (rightValue < minR) minR = rightValue;
        if (rightValue > maxR) maxR = rightValue;

        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setCursor(0, 0);
        oled.println(F("Running"));

        // 大号字显示 分:秒.百分秒
        oled.setTextSize(2);
        oled.setCursor(0, 20);
        if (minute < 10) oled.print(F("0"));     // 补 0，让数字宽度固定，看着不跳
        oled.print(minute);
        oled.print(F(":"));
        if (second < 10) oled.print(F("0"));
        oled.print(second);
        oled.print(F("."));
        if (hundredth < 10) oled.print(F("0"));
        oled.print(hundredth);

        // 3 分钟进度条
        int bar = (int)((millis() - startTime) * 100 / lapMaxMs);
        if (bar > 100) bar = 100;
        oled.drawRect(0, 54, 104, 10, SSD1306_WHITE);   // 先画外面的方框
        oled.fillRect(2, 56, bar, 6, SSD1306_WHITE);    // 再用实心块表示进度

        // 调试行：实时显示两个传感器的读数和当前判断
        if (debugOled == 1) {
          oled.setTextSize(1);
          oled.setCursor(0, 42);
          oled.print(F("L="));
          oled.print(leftValue);
          oled.print(F(" R="));
          oled.print(rightValue);
          oled.print(' ');
          if (leftBlack == 1 && rightBlack == 1) oled.print(F("BB"));
          else if (leftBlack == 1)               oled.print(F("LB"));
          else if (rightBlack == 1)              oled.print(F("RB"));
          else                                   oled.print(F("WW"));
        }

        oled.display();
      }
    }
  }

  // =================================================================
  // 第 5 步  停车，显示成绩
  // =================================================================
  analogWrite(leftForward, 0);
  analogWrite(leftBack, 0);
  analogWrite(rightForward, 0);
  analogWrite(rightBack, 0);
  digitalWrite(ledRun, LOW);
  digitalWrite(ledStat, LOW);

  unsigned long passed = millis() - startTime;
  int totalSecond = passed / 1000;
  int minute      = totalSecond / 60;
  int second      = totalSecond % 60;
  int hundredth   = (passed % 1000) / 10;

  // 电机已经停了：跑完高音提示，超时或发车失败低音提示
  if (finished == 1) {
    tone(buzzer, 1500);
  } else {
    tone(buzzer, 400);
  }
  delay(300);
  noTone(buzzer);

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  if (finished == 1) {
    oled.println(F("FINISHED!  time:"));
  } else {
    oled.println(F("STOPPED / TIMEOUT"));
  }
  oled.setTextSize(2);
  oled.setCursor(0, 16);
  if (minute < 10) oled.print(F("0"));
  oled.print(minute);
  oled.print(F(":"));
  if (second < 10) oled.print(F("0"));
  oled.print(second);
  oled.print(F("."));
  if (hundredth < 10) oled.print(F("0"));
  oled.print(hundredth);
  oled.setTextSize(1);

  if (debugOled == 1) {
    // ---- 调试"验尸"画面 ----
    // L / R 后面是这一趟里左 / 右传感器读到过的【最小-最大】读数。
    //   如果最大值一直停在白底的水平（比如 760），说明那个传感器整趟都没见过黑线。
    // 最后一行是四种情况各出现多少次：
    //   W=两个都白(直行)  L=左黑(左转)  R=右黑(右转)  B=双黑(起始线)
    oled.setCursor(0, 32);
    oled.print(F("L "));
    oled.print(minL);
    oled.print('-');
    oled.print(maxL);
    oled.setCursor(0, 42);
    oled.print(F("R "));
    oled.print(minR);
    oled.print('-');
    oled.print(maxR);
    oled.setCursor(0, 52);
    oled.print(F("W"));
    oled.print(cntWW);
    oled.print(F("/L"));
    oled.print(cntLB);
    oled.print(F("/R"));
    oled.print(cntRB);
    oled.print(F("/B"));
    oled.print(cntBB);
  } else {
    oled.setCursor(0, 48);
    oled.println(F("press START again"));
  }
  oled.display();

  // 串口里也打印一份，方便记录成绩
  Serial.print(F("time = "));
  Serial.print(minute);
  Serial.print(F(":"));
  Serial.print(second);
  Serial.print(F("."));
  Serial.println(hundredth);

  if (debugOled == 1) {
    // 调试画面停 20 秒，方便走过去看读数；按一下启动键可以提前结束
    unsigned long watchStart = millis();
    while (millis() - watchStart < 20000) {
      if (digitalRead(startButton) == HIGH) {
        break;
      }
      delay(10);
    }
    while (digitalRead(startButton) == HIGH) {   // 等手指松开
      delay(10);
    }
  } else {
    delay(5000);   // 成绩画面保持 5 秒，然后回到开头重新等按键
  }
}
