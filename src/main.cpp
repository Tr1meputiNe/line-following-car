/* =====================================================================
 *  光电循迹小车  ——  主程序
 *
 *  硬件：Arduino UNO + MX1508 电机驱动 + 2 路光电对管 + 0.96" OLED(SSD1306)
 *
 *  运行流程：
 *     上电        -> OLED 显示 "Press START"，停在这里等按键
 *     按启动键    -> 倒计时 3-2-1
 *     起步        -> 满速踢一脚，然后【立刻开始循迹】，计时也从这一刻开始
 *     循迹        -> 沿黑线跑一圈
 *     压到起始线  -> 两个传感器同时压黑线并保持一会儿 -> 停车，显示成绩
 *
 *  要改接线：只改"引脚"那一段
 *  要调参数：只改"参数"那一段
 *
 *  【为什么不设"发车阶段"】
 *  以前的写法是：先盲冲，等两个传感器【同时】压到起始线才进入循迹。
 *  但两个传感器同时压到同一条黑带，要求黑带比两个传感器的间距还宽，
 *  而且车横向只能偏 2~3 毫米 —— 实际几乎不可能满足。
 *  结果是车永远进不了循迹，一直盲冲 5 秒后停住，看起来就是"压到线也不拐"。
 *  现在改成起步就直接循迹，"双黑"只用来当停车信号。
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

// ---- 速度（0~255，就是 PWM 占空比）----
// 【为什么要压这么低】电机电源取自 Arduino 的 5V 引脚，那颗自恢复保险丝的"保持电流"是 500mA。
// 总电流超过它，保险丝就发热、阻值暴涨，5V 轨塌到 2.2V（实测），单片机欠压复位。
// 而电机电流基本和占空比成正比，所以【占空比就是唯一的软件调节手段】。
//
// 别靠猜来定这个数：烧 bringup，把车架空（轮子悬空），敲 y 跑"占空比扫描"，
// 它会从 20 一档一档加到 255，每档打印那一档见到的最低电压。
// 看到哪一档电压开始往下掉，主程序就取那一档的 60% 左右（悬空比实际轻，要留余量）。
int speedFast = 60;      // 直行速度（两个轮子一样）
int speedTurn = 65;      // 转弯时【外侧】轮速度
int speedBack = 20;      // 转弯时【内侧】轮【反转】速度；改成 0 = 内侧轮只停住不倒转，最省电
// 外侧轮往前冲 + 内侧轮往后倒，车身几乎是原地转，这是双轮差速能做出来最狠的转向。
// 两个数越大转得越急，也越容易冲过头来回摆；车在直线上画龙就把它们往小调。

// ---- 左右轮补偿 ----
// 车往左偏 = 左轮偏慢 -> trimLeft 填正数（先试 5、10、15）
// 车往右偏 = 右轮偏慢 -> trimLeft 填负数（先试 -5、-10、-15）
// 【注意】D5/D6 走 Timer0（约 976Hz），D9/D10 走 Timer1（约 490Hz），
//         左右轮 PWM 频率天生不同，同一占空比下转速不一样，所以这个补偿要认真调。
int trimLeft = 0;
int fastLeft = speedFast + trimLeft;   // 左轮直行速度，自动算出来
int turnLeft = speedTurn + trimLeft;   // 左轮转弯外侧速度，自动算出来

int speedKick  = 120;    // 软起动升到的最高速度
int startKickMs = 100;   // 升到顶之后再保持多久（毫秒）
int rampStep   = 5;      // 每档加多少
int rampStepMs = 20;     // 每档停多少毫秒
// 直流电机静止时是堵转，电流最大。一步跳到高占空比会把 5V 拉到 AVR 的欠压阈值
// （2.7V）以下 -> 复位 -> setup() 把电机清零 -> 车"冲一步就停"。
// 【实测】一步跳到 200 时起步最低电压 2.85V，跳到 130 时稳态 2.30V（读法偏高约 7%）。
// 所以改成"软起动"：5、10、15…… 一档一档加上去，每档 20ms。
// 电机在"刚好转得起来"的那个占空比上脱离静摩擦，那一刻的电流比一步跳上去小得多。
// 【注意】speedKick 是软起动的峰值，它本身也受保险丝限制，所以从 200 降到 120。
//   如果降到车起不动了，说明保险丝允许的峰值已经低于"能转起来"的门槛 —— 软件无解。
//
// 如果软起动之后还是复位，就说明供电彻底不够了，只能改硬件：
//   电机电源单独走电池盒 -> MX1508 的 VM，Arduino 单独走 USB，两边共地（README 第九节）。

// ---- 计时和起停 ----
// 注意：Uno 上 int 最大只有 32767，180000 装不下，毫秒一律用 unsigned long
unsigned long startLineMs = 80;      // 双黑连续保持这么久，才算"压到起始线"
unsigned long lapMinMs    = 5000;    // 起步后至少跑这么久才允许停车
unsigned long lapMaxMs    = 180000;  // 3 分钟限时
unsigned long oledMs      = 100;     // 循迹时每隔多久刷新一次屏幕

// lapMinMs 是防止"刚起步就误停"的保险：
//   车摆在起始线【前面】时，起步一两秒就会压过一次起始线，
//   没有这个保险就会立刻停住。跑了这么久之后就都算数了。
//   如果你的车是摆在【起始线后面】的引导线上，可以降到 2000。
//
// startLineMs 和起始线宽度直接相关：起始线 3cm、车速约 20~25cm/s，
//   压线 120~150ms，所以取 80ms 留余量。
//   跑完一圈停不下来 -> 降到 60；弯道提前停车 -> 加到 110。

// ---- 调试开关（都调好之后改成 0）----
int debugLaunch = 1;   // 1 = 开机时把复位原因打到串口
int debugOled   = 1;   // 1 = 在 OLED 上显示调试信息（跑完停屏"验尸"）

// 上一次复位的原因。注意：Uno 的 bootloader（optiboot）会在跳到程序之前把 MCUSR 清掉，
// 所以这里读出来基本永远是 0，只能用来看串口，不能当真。真正有用的是下面那个 vccMinMv。
int resetCause = 0;

// 【诊断用】运行期间见过的最低供电电压（毫伏），分两个窗口记，都放在 .noinit 段。
// .noinit 的意思是这个变量【复位时不会被清零】，所以值能跨过一次复位留下来。
// 车"冲一步就停"之后再回到待机画面，屏幕上就会显示：
//   vccMinMv  起步瞬间（软起动 + 0.4 秒内）的最低电压   -> 看电机启动冲击有多狠
//   vccRunMv  踢完之后稳态的最低电压                    -> 看正常跑起来之后供电够不够
// 比值参考：这个读法比真实值高约 7%（真实 5.0V 会显示成 5.36V）。
//   显示 2.4~2.9V -> 实际约 2.2~2.7V，已经到 AVR 欠压阈值(2.7V)以下，是欠压复位
//   显示 4.9~5.4V -> 电压没塌，复位是别的原因
int vccMinMv __attribute__((section(".noinit")));
int vccRunMv __attribute__((section(".noinit")));

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

  // MCUSR 是 AVR 记录上一次复位原因的寄存器，用来判断单片机有没有重启过。
  //   bit0 PORF 上电复位   bit1 EXTRF 复位键/外部复位
  //   bit2 BORF 电压过低   bit3 WDRF 看门狗
  resetCause = MCUSR;
  MCUSR = 0;                     // 读完清掉，下次开机才是新的原因
  if (debugLaunch == 1) {
    Serial.print(F("reset cause = 0x"));
    Serial.println(resetCause, HEX);
    Serial.println(F("  注意：Uno 的 bootloader 通常会把它清成 0，读不到东西"));
  }

  // 这两个在 .noinit 段，上电时是内存里的随机值。不在合理范围就当成没有记录。
  if (vccMinMv < 2000 || vccMinMv > 6000) {
    vccMinMv = 6000;
  }
  if (vccRunMv < 2000 || vccRunMv > 6000) {
    vccRunMv = 6000;
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
  oled.setCursor(0, 32);
  oled.print(F("th = "));
  oled.println(threshold);
  // 上一次【运行期间】见过的最低供电电压：  起步瞬间 / 稳态
  //   前面那个 2.4~2.9V -> 电机启动把 5V 拉塌了，是欠压复位（实际电压比显示低约 7%）
  //   后面那个也很低   -> 不是启动冲击的问题，是正常跑起来供电就不够
  // 显示的是"毫伏/1000"直接换算，比真实值高约 7%（真实 5.0V 会显示成 5.36V），看相对变化。
  oled.setCursor(0, 48);
  oled.print(F("minV "));
  oled.print(vccMinMv / 1000);
  oled.print('.');
  oled.print((vccMinMv % 1000) / 10);
  oled.print('/');
  oled.print(vccRunMv / 1000);
  oled.print('.');
  oled.print((vccRunMv % 1000) / 10);
  oled.print(F("V"));
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
  // 第 3 步  起步，然后立刻开始循迹
  // =================================================================
  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setCursor(24, 24);
  oled.print(F("GO !"));
  oled.display();

  digitalWrite(ledRun, HIGH);
  digitalWrite(ledStat, HIGH);

  // 软起动：从 0 一档一档升到 speedKick，而不是一步跳上去。
  // 电机静止时是堵转、电流最大；慢慢升压能让它在"刚好转得起来"的电压上脱离静摩擦，
  // 那一刻的电流比一步给到 200 小得多，供电才不会被拉塌。
  vccMinMv = 6000;          // 清掉上一次记录，重新测这一趟
  vccRunMv = 6000;
  analogWrite(leftBack, 0);
  analogWrite(rightBack, 0);
  for (int s = rampStep; s <= speedKick; s = s + rampStep) {
    analogWrite(leftForward, s);
    analogWrite(rightForward, s);
    delay(rampStepMs);
  }
  delay(startKickMs);       // 升到顶之后再保持一下，确保真的转起来了
  analogWrite(leftForward, fastLeft);
  analogWrite(rightForward, speedFast);

  unsigned long startTime = millis();   // ★ 计时从起步这一刻开始

  // =================================================================
  // 第 4 步  循迹跑一圈，第一次压到起始线就停车
  // =================================================================
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

    // ---------- 供电电压监视（只在起步后 3 秒内做）----------
    // 电机启动的一瞬间电压会塌，塌陷只持续几毫秒，10Hz 采样根本抓不到，
    // 所以在这段窗口里【每次循环都测】。分两个窗口记：
    //   vccMinMv  全程最低（主要是软起动那段）
    //   vccRunMv  1 秒之后的稳态最低（看正常跑起来供电够不够）
    // 这两个变量在 .noinit 段，单片机复位后值还在，回到待机画面就能看到。
    if (millis() - startTime < 3000) {
      // 用 AVR 内部的 1.1V 基准反推 VCC：ADMUX 切到内部基准通道
      ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
      ADCSRA |= _BV(ADSC);                        // 第一次转换丢弃（刚换通道还没稳定）
      while (bit_is_set(ADCSRA, ADSC)) { }
      ADCSRA |= _BV(ADSC);
      while (bit_is_set(ADCSRA, ADSC)) { }
      int raw = ADC;
      if (raw > 0) {
        int mv = 1125300L / raw;                  // 1125300 = 1.1V × 1023 × 1000
        if (mv < vccMinMv) vccMinMv = mv;
        if (millis() - startTime > 1000) {
          if (mv < vccRunMv) vccRunMv = mv;
        }
      }
    }

    // ---------- ② 踩到起始线就停车 ----------
    // 两个传感器同时压黑线，并连续保持 startLineMs，同时已经跑够 lapMinMs，
    // 才算"回到起始线"。
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

  // 电机已经停了：跑完高音提示，超时低音提示
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
