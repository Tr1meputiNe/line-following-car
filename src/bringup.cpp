/* =====================================================================
 *  分步测试程序（bringup）—— 装车之前先用它把每个部件单独试一遍
 *
 *  怎么用：
 *     pio run -e bringup -t upload        // 烧录这个测试程序
 *     pio device monitor                  // 打开串口（9600）
 *     然后在监视器里输入下面这些字母，回车
 *
 *      m = 两个轮子一起前进
 *      n = 两个轮子一起后退
 *      q = 左转（左轮慢、右轮快）
 *      w = 右转（左轮快、右轮慢）
 *      l = 慢速 100
 *      h = 快速 220
 *      s = 停车
 *      o = 屏幕测试（画字和一个方块）
 *      t = 蜂鸣器响 + 两个灯一起闪
 *      r = 打印 20 组传感器读数（左 A1、右 A0）
 *      b = 打印启动键 S1 的状态（按下为 1）
 *      f = 循迹测试模式
 *      v = 电机压力测试 + 自动电压监控（不用万用表）
 *      z = 蜂鸣器引脚扫描
 *
 *  跑完测试要烧回正式程序： pio run -e uno -t upload
 * ===================================================================== */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------------------- 引脚（和主程序一致） ----------------------------
// 【实测结论】扩展板丝印标的左右和实际是反的，这里按实测写死，和丝印不一致是正常的。
// 实测方法：bringup 敲 q（代码让"左轮"变慢），看哪个物理轮子慢；
//           敲 r，手指挡车的左侧传感器，看哪一列数字变。
// 左右以"人站在车后、车头朝前"为准。
int leftSensorPin  = A0;   // 实测：车左侧的传感器接在 A0
int rightSensorPin = A1;   // 实测：车右侧的传感器接在 A1

// MX1508：换了驱动模块之后实测，MOTOR-B(D9/D10) 驱动左轮，MOTOR-A(D5/D6) 驱动右轮
// 某轮前进方向不对，就在两个程序里交换该轮 Forward/Back 的引脚值
// 换了新的驱动板之后这张映射要重新用 bringup 的 q 命令验一遍
int leftForward  = 9;
int leftBack     = 10;
int rightForward = 5;
int rightBack    = 6;

// 扩展板 S1 有外部下拉电阻，松开为 LOW，按下为 HIGH
int buzzer      = 4;    // 扩展板 BUZ 实测接在 D4（用 z 命令扫描出来的）
int startButton = 7;
int ledRun      = 11;   // LED2
int ledStat     = 12;   // LED1；S2 未连接

// [重要] 下面所有字符串都写成 F("...") 的形式。
// AVR 上普通字符串会占用宝贵的 RAM，F() 把它存到 Flash 里，不占 RAM。
// 原因：128x64 的 OLED 需要 1024 字节显存，而 UNO 只有 2048 字节 RAM。
//       字符串占太多 RAM 会导致 oled.begin() 里的 malloc 失败，屏幕一片黑。
int speedNow = 130;    // 当前速度
int trimLeft = 0;      // 左轮补偿，和 src/main.cpp 保持一致（往左偏填正数）
int speedTurn = 135;   // 转弯时外侧轮速度，和 src/main.cpp 保持一致
int speedBack = 60;    // 转弯时内侧轮反转速度，和 src/main.cpp 保持一致
int followMode = 0;    // 1 = 循迹测试模式开着
int startKickMs = 150; // 起步踢一脚的时长（毫秒），和 src/main.cpp 保持一致

// 下面两个只是给 r 命令做"黑/白"显示用的，要和 src/main.cpp 里保持一致
int threshold = 840;   // 黑白分界值
int blackHigh = 1;     // 黑线读数大填 1，黑线读数小填 0

Adafruit_SSD1306 oled(128, 64, &Wire, -1);

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

  // 初始化屏幕之前，先确保两个电机都停止
  analogWrite(leftForward, 0);
  analogWrite(leftBack, 0);
  analogWrite(rightForward, 0);
  analogWrite(rightBack, 0);

  Serial.begin(9600);

  // ---- 开机时报告"复位原因"。用来判断"电机突然停"是不是单片机重启了 ----
  // MCUSR 是 AVR 的一个寄存器，里面记录着上一次复位是哪一种原因
  Serial.print(F("reset cause MCUSR = 0x"));
  Serial.println(MCUSR, HEX);
  Serial.println(F("  bit0 PORF=上电   bit1 EXTRF=复位键/串口   bit2 BORF=电压掉太低   bit3 WDRF=看门狗"));
  MCUSR = 0;             // 读完清掉，下次开机才是新的原因

  Wire.begin();          // 手动打开 I2C 总线，下面才能扫描

  // ---- 扫描 I2C 总线，看看屏幕到底在不在 ----
  // Wire.beginTransmission(地址) 意思是"我要跟这个地址说话"
  // Wire.endTransmission() 返回 0，表示这个地址上有设备应答
  Serial.println(F("Scanning I2C bus..."));
  int foundCount = 0;
  for (int addr = 1; addr < 127; addr = addr + 1) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  found device at 0x"));
      Serial.println(addr, HEX);
      foundCount = foundCount + 1;
    }
  }
  if (foundCount == 0) {
    Serial.println(F("  NOTHING FOUND -> 检查 VCC / GND / SDA(A4) / SCL(A5)"));
  }

  // ---- 初始化屏幕，成功失败都打到串口 ----
  int oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (oledOk == 0) {
    Serial.println(F("0x3C failed, retry 0x3D ..."));
    oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  }
  if (oledOk == 1) {
    Serial.println(F("OLED begin OK"));
  } else {
    Serial.println(F("OLED begin FAILED -> 屏幕没应答"));
  }

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println(F("BRING-UP TEST"));
  oled.println(F("send a letter"));
  oled.println(F("from serial"));
  oled.display();

  Serial.println(F("=== bringup test ready ==="));
  Serial.println(F("m=forward n=back s=stop q=left w=right"));
  Serial.println(F("l=100 h=220 o=oled t=beep+led r=sensor b=button"));
  Serial.println(F("f=follow z=sweep v=VCC test x=fwd/rev stress"));
  Serial.println(F("r 打印 20 组左 A1、右 A0 读数，用它找阈值"));
}

void loop() {
  // ---------------- 处理串口发过来的一个字母 ----------------
  if (Serial.available() > 0) {
    char c = Serial.read();     // 读一个字符

    if (c == 'm') {
      // 先撤掉原来的 PWM，再切换方向
      analogWrite(leftForward, 0);
      analogWrite(leftBack, 0);
      analogWrite(rightForward, 0);
      analogWrite(rightBack, 0);
      delay(20);
      // 起步踢一脚：先满速 255 再降回 speedNow，突破静摩擦
      analogWrite(leftForward, 255);
      analogWrite(rightForward, 255);
      delay(startKickMs);
      analogWrite(leftForward, speedNow + trimLeft);
      analogWrite(rightForward, speedNow);
      Serial.print(F("forward, speed = "));
      Serial.println(speedNow);
    }
    else if (c == 'n') {
      // 先撤掉原来的 PWM，再切换方向
      analogWrite(leftForward, 0);
      analogWrite(leftBack, 0);
      analogWrite(rightForward, 0);
      analogWrite(rightBack, 0);
      delay(20);
      analogWrite(leftBack, speedNow);
      analogWrite(rightBack, speedNow);
      Serial.println(F("backward"));
    }
    else if (c == 'q') {
      // 先撤掉原来的 PWM，再切换方向
      analogWrite(leftForward, 0);
      analogWrite(leftBack, 0);
      analogWrite(rightForward, 0);
      analogWrite(rightBack, 0);
      delay(20);
      analogWrite(leftForward, 70);
      analogWrite(rightForward, speedNow);
      Serial.println(F("turn left"));
    }
    else if (c == 'w') {
      // 先撤掉原来的 PWM，再切换方向
      analogWrite(leftForward, 0);
      analogWrite(leftBack, 0);
      analogWrite(rightForward, 0);
      analogWrite(rightBack, 0);
      delay(20);
      analogWrite(leftForward, speedNow + trimLeft);
      analogWrite(rightForward, 70);
      Serial.println(F("turn right"));
    }
    else if (c == 's') {
      analogWrite(leftForward, 0);
      analogWrite(leftBack, 0);
      analogWrite(rightForward, 0);
      analogWrite(rightBack, 0);
      Serial.println(F("stop"));
    }
    else if (c == 'l') {
      speedNow = 100;
      Serial.println(F("speed = 100"));
    }
    else if (c == 'h') {
      speedNow = 220;
      Serial.println(F("speed = 220"));
    }
    else if (c == 'o') {
      oled.clearDisplay();
      oled.setTextSize(2);
      oled.setCursor(0, 0);
      oled.println(F("OLED OK"));
      oled.setTextSize(1);
      oled.setCursor(0, 30);
      oled.println(F("ABCDEFG abcdefg"));
      oled.println(F("1234567890"));
      oled.drawRect(0, 50, 100, 12, SSD1306_WHITE);
      oled.fillRect(0, 50, 50, 12, SSD1306_WHITE);
      oled.display();
      Serial.println(F("oled test done"));
    }
    else if (c == 't') {
      tone(buzzer, 1200);
      digitalWrite(ledRun, HIGH);
      digitalWrite(ledStat, HIGH);
      delay(400);
      noTone(buzzer);
      digitalWrite(ledRun, LOW);
      digitalWrite(ledStat, LOW);
      delay(200);
      tone(buzzer, 1200);
      digitalWrite(ledRun, HIGH);
      digitalWrite(ledStat, HIGH);
      delay(400);
      noTone(buzzer);
      digitalWrite(ledRun, LOW);
      digitalWrite(ledStat, LOW);
      Serial.println(F("beep + led done"));
    }
    else if (c == 'b') {
      Serial.print(F("start button = "));
      Serial.println(digitalRead(startButton));
      Serial.println(F("(0 = 没按, 1 = 按下；S2 未连接)"));
    }
    else if (c == 'f') {
      // 【循迹测试】直接按循迹逻辑跑，不做倒计时、不做起始线检测。
      // 用来在赛道上慢慢调 speedNow / speedSlow / trimLeft。
      if (followMode == 0) {
        followMode = 1;
        Serial.println(F("follow mode ON  (press f again to stop)"));
      } else {
        followMode = 0;
        analogWrite(leftForward, 0);
        analogWrite(rightForward, 0);
        Serial.println(F("follow mode OFF"));
      }
    }
    else if (c == 'v') {
      // 【电机压力测试 + 自动电压监控】
      // 让两个电机一直转，同时每 200ms 打印一次芯片自己的供电电压。
      // 电压不用万用表量 —— 用芯片内部一个 1.1V 的基准电压反推出来。
      // 按任意键停止。
      Serial.println(F("=== motor stress + VCC monitor ==="));
      Serial.println(F("(A2 接一根线到 MX1508 的 VM 就能同时看 VM 电压)"));
      Serial.println(F("motors ON. press any key to stop"));

      analogWrite(leftBack, 0);
      analogWrite(rightBack, 0);
      analogWrite(leftForward, 255);
      analogWrite(rightForward, 255);
      delay(startKickMs);
      analogWrite(leftForward, speedNow + trimLeft);
      analogWrite(rightForward, speedNow);

      unsigned long t0 = millis();
      unsigned long last = 0;
      while (Serial.available() == 0) {
        if (millis() - last > 200) {
          last = millis();

          // --- 用内部 1.1V 基准反推 VCC，不需要万用表 ---
          // ADMUX 是选择"量哪个通道"的寄存器，这行的意思是"改成量内部的 1.1V 基准"
          ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
          delay(2);                          // 等基准电压稳定
          ADCSRA |= _BV(ADSC);               // 启动一次转换
          while (ADCSRA & _BV(ADSC)) { }     // 等转换完成
          long raw = ADCL;                   // 读结果（低字节）
          raw |= ADCH << 8;                  // 补上高字节
          // 换算：VCC = 1.1V * 1023 / raw
          int vcc = (int)(1125300L / raw);

          // --- 顺便量一下 MX1508 的 VM（要从 VM 接一根线到 A2）---
          analogRead(A2);                     // 刚切换过通道，丢掉第一次读数
          int vmRaw = analogRead(A2);
          int vm = (int)((long)vmRaw * vcc / 1023);

          Serial.print(millis() - t0);
          Serial.print(F("ms  VCC="));
          Serial.print(vcc);
          Serial.print(F("mV   VM(A2)="));
          Serial.print(vm);
          Serial.print(F("mV   raw="));
          Serial.println(raw);
        }
      }
      Serial.read();                          // 吃掉那个按键
      analogWrite(leftForward, 0);
      analogWrite(rightForward, 0);
      Serial.println(F("motors OFF"));
    }
    else if (c == 'x') {
      // 【正反转交替压力测试 + 电压监控】
      // 每 2 秒切换一次前进/反转 —— "换向"是电机电流最大的动作，
      // 也是最接近实际"转弯"工况的测试。同时打印芯片自己的供电电压。
      // 车可以放在地上（会原地前后晃），也可以架起来。按任意键停止。
      Serial.println(F("=== forward/reverse stress + VCC monitor ==="));
      Serial.println(F("(A2 接一根线到 MX1508 的 VM 就能同时看 VM 电压)"));
      Serial.println(F("switching direction every 2s. press any key to stop"));

      unsigned long t0 = millis();
      unsigned long last = 0;
      unsigned long flip = 0;
      int forward = 1;

      while (Serial.available() == 0) {
        if (millis() - last > 200) {
          last = millis();

          // 每 2 秒换一次方向
          if (millis() - flip > 2000) {
            flip = millis();
            if (forward == 1) {
              forward = 0;
              Serial.println(F("  --> REVERSE"));
            } else {
              forward = 1;
              Serial.println(F("  --> FORWARD"));
            }
            // 换向：先把两个输入都拉低等一下，再给新方向
            analogWrite(leftForward, 0);
            analogWrite(leftBack, 0);
            analogWrite(rightForward, 0);
            analogWrite(rightBack, 0);
            delay(50);
            if (forward == 1) {
              analogWrite(leftForward, speedNow + trimLeft);
              analogWrite(rightForward, speedNow);
            } else {
              analogWrite(leftBack, speedNow + trimLeft);
              analogWrite(rightBack, speedNow);
            }
          }

          // --- 用内部 1.1V 基准反推 VCC ---
          ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
          delay(2);
          ADCSRA |= _BV(ADSC);
          while (ADCSRA & _BV(ADSC)) { }
          long raw = ADCL;
          raw |= ADCH << 8;
          int vcc = (int)(1125300L / raw);

          // --- 顺便量一下 MX1508 的 VM（要从 VM 接一根线到 A2）---
          analogRead(A2);
          int vmRaw = analogRead(A2);
          int vm = (int)((long)vmRaw * vcc / 1023);

          Serial.print(millis() - t0);
          Serial.print(F("ms  VCC="));
          Serial.print(vcc);
          Serial.print(F("mV   VM(A2)="));
          Serial.print(vm);
          Serial.print(F("mV   raw="));
          Serial.print(raw);
          Serial.print(F("   dir="));
          Serial.println(forward);
        }
      }
      Serial.read();
      analogWrite(leftForward, 0);
      analogWrite(leftBack, 0);
      analogWrite(rightForward, 0);
      analogWrite(rightBack, 0);
      Serial.println(F("motors OFF"));
    }
    else if (c == 'z') {
      // 【蜂鸣器引脚扫描】挨个引脚输出方波，用耳朵听出蜂鸣器到底接在哪个脚
      // 不用 tone()，改成手动翻转电平，这样任何数字脚都能试
      // 注意：D0/D1 是串口，绝对不能碰，所以从 D2 开始
      Serial.println(F("--- buzzer pin sweep ---"));
      Serial.println(F("each pin beeps ~0.3s, listen carefully"));
      Serial.println(F("D5 D6 D9 D10 will twitch the motors - raise the car!"));
      for (int pin = 2; pin <= 13; pin = pin + 1) {
        pinMode(pin, OUTPUT);
        Serial.print(F("  testing D"));
        Serial.println(pin);
        // delayMicroseconds(250) = 等 250 微秒。一高一低是一个周期，约 2kHz
        for (int i = 0; i < 600; i = i + 1) {
          digitalWrite(pin, HIGH);
          delayMicroseconds(250);
          digitalWrite(pin, LOW);
          delayMicroseconds(250);
        }
        delay(300);   // 停一下，方便分辨是哪个脚在响
      }
      Serial.println(F("--- sweep done. which D pin beeped? ---"));
      Serial.println(F("if none beeped -> buzzer 没接好或坏了"));
    }
    else if (c == 'r') {
      // 打印 20 组读数，并且按 threshold / blackHigh 标出 B(黑线) 或 W(白底)
      Serial.println(F("--- left(A1) right(A0) ---"));
      Serial.println(F("--- B=black line  W=white ---"));
      for (int i = 0; i < 20; i = i + 1) {
        int lv = analogRead(leftSensorPin);
        int rv = analogRead(rightSensorPin);
        Serial.print(lv);
        Serial.print(F("  "));
        Serial.print(rv);
        Serial.print(F("   "));
        if (blackHigh == 1) {
          if (lv > threshold) Serial.print('B'); else Serial.print('W');
          Serial.print(' ');
          if (rv > threshold) Serial.println('B'); else Serial.println('W');
        } else {
          if (lv < threshold) Serial.print('B'); else Serial.print('W');
          Serial.print(' ');
          if (rv < threshold) Serial.println('B'); else Serial.println('W');
        }
        delay(250);
      }
      Serial.println(F("--- slide the car across the line and watch B/W ---"));
    }
    else {
      Serial.println(F("unknown letter"));
    }
  }

  // ================= 循迹测试：和主程序一模一样的判断逻辑 =================
  // 唯一的区别是不做起始线检测，适合在赛道上反复试参数
  if (followMode == 1) {
    // 和主程序完全一样的结构：先判断，再决定 4 个值，最后一次性写出去
    int lv = analogRead(leftSensorPin);
    int rv = analogRead(rightSensorPin);

    int lb = 0;
    int rb = 0;
    if (blackHigh == 1) {
      if (lv > threshold) lb = 1;
      if (rv > threshold) rb = 1;
    } else {
      if (lv < threshold) lb = 1;
      if (rv < threshold) rb = 1;
    }

    // 先按"直行"填好 4 个值
    int leftGo   = speedNow + trimLeft;
    int leftRev  = 0;
    int rightGo  = speedNow;
    int rightRev = 0;

    if (lb == 1 && rb == 0) {
      // 线在车左边 -> 车偏右 -> 向左转（左轮反转、右轮前进）
      leftGo  = 0;
      leftRev = speedBack;
      rightGo = speedTurn;
    }
    else if (lb == 0 && rb == 1) {
      // 线在车右边 -> 车偏左 -> 向右转（右轮反转、左轮前进）
      leftGo   = speedTurn + trimLeft;
      rightGo  = 0;
      rightRev = speedBack;
    }
    // 两个都白 = 车正 / 两个都黑 = 起始线，都是直行

    analogWrite(leftForward,  leftGo);
    analogWrite(leftBack,     leftRev);
    analogWrite(rightForward, rightGo);
    analogWrite(rightBack,    rightRev);
  }
}
