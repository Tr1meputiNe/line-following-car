
/* =====================================================================
 *  光电循迹小车  ——  主程序（单文件版）
 *
 *  硬件：Arduino UNO + MX1508 电机驱动 + 2 路光电对管 + 0.96" OLED(SSD1306)
 *
 *  运行流程：
 *    1. 上电后 OLED 显示 "Press START"，程序停在这里等按键
 *    2. 按下启动键 -> OLED 倒计时 3-2-1
 *    3. 小车直行开出出发区，压到起始线的那一刻开始计时
 *    4. 沿着黑线循迹前进，OLED 上实时刷新用时
 *    5. 再次压到起始线 -> 停车 -> OLED 显示最终成绩
 *
 *  要改接线：只改下面"引脚"那一段
 *  要调参数：只改下面"参数"那一段
 * ===================================================================== */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================== 引脚 ==============================
// 光电对管的模拟输出接 A0 / A1
// 【实测结论】扩展板丝印标的左右和实际是反的，这里按实测写死，和丝印不一致是正常的。
// 实测方法：bringup 敲 q（代码让"左轮"变慢），看哪个物理轮子慢；
//           敲 r，手指挡车的左侧传感器，看哪一列数字变。
// 左右以"人站在车后、车头朝前"为准。
int leftSensorPin  = A0;   // 左传感器 OUT
int rightSensorPin = A1;   // 右传感器 OUT

// MX1508：IN1/IN2 控制 MOTOR-A，IN3/IN4 控制 MOTOR-B
// 新接线实测：MOTOR-A → 左轮，MOTOR-B → 右轮。
// 哪一轮【方向反了】，就交换该轮 Forward / Back 这两个引脚号（不要改接线）。
int leftForward  = 6;    // IN1
int leftBack     = 7;    // IN2
int rightForward = 8;    // IN3
int rightBack    = 9;    // IN4

// 扩展板 S1 有外部下拉电阻，松开为 LOW，按下为 HIGH
int buzzer      = 10;   // 扩展板只有一个 buz 脚，新接在 D10
int startButton = 2;    // S1 新接在 D2
int ledRun      = 11;   // LED2
int ledStat     = 12;   // LED1；S2 未连接

// ============================== 参数 ==============================
// ---- 按你的赛道情况调这几个数字 ----
// 黑白分界值。【已实测】黑线读数 915~949，白底读数 705~762，取中间值 840
// 余量：黑线最低 915 比 840 高 75；白底最高 762 比 840 低 78
int threshold = 840;
int blackHigh = 1;      // 【已实测】黑线读数大 -> 填 1。若压黑线读到小数字，改成 0
// ---- 速度（0~255，就是 PWM 占空比）----
// 【本次：在原值基础上整体 +10】140 / 165 / 110。
// 调参记录（备查）：
//   130/155/100  回滚回来的原值，实测跑通过一圈
//   110/130/80   整体降 20%，试过
//   120/142/90   取两者的中位数，试过
//   140/165/110  本次：原值 +10（整体快一档）
// 以后要调就继续"在前后两次之间取中位数"，每试一次区间减半，几次就收敛。
// 【提醒】占空比越大，电机电流越大。实测占空比 130 时 5V 轨被拉到 2.30V；
//   这次比那时还高 10，如果一跑就复位，先把 speedFast 降回来。
int speedFast = 130;    // 直行速度（两个轮子一样）
int speedTurn = 155;    // 转弯时【外侧】轮速度，越大转得越急
int speedBack = 20;    // 转弯时【内侧】轮【反转】速度，越大转得越急
// 外侧轮往前冲 + 内侧轮往后倒，车身几乎是原地转，这是双轮差速能做出最狠的转向。
// 【注意】换向（正转↔反转）是电流冲击最大的动作：电机要穿过零点、重新克服静摩擦，
//   那一瞬间的堵转电流是平时的好几倍，会让供电电压瞬间塌下去。
//   如果你的供电是 USB（Uno 上有 500mA 保险丝），转向时可能会中途停住。
//   真出现这种情况，把 speedBack 改成 0：内侧轮会变成"停住"（MX1508 两个输入
//   都是 0 = 刹车），电流冲击小了，但转向力度也会明显变弱。
//   根治办法是给电机单独的电池供电，见 README。

// 左右轮补偿。车往左偏 = 左轮偏慢 -> trimLeft 填正数（先试 5、10、15）
//              车往右偏 = 右轮偏慢 -> trimLeft 填负数（先试 -5、-10、-15）
// 建议范围 -30 ~ +30，别太大
int trimLeft = 0;
int fastLeft = speedFast + trimLeft;   // 左轮直行速度，自动算出来，不用手改
int turnLeft = speedTurn + trimLeft;   // 左轮转弯外侧速度，自动算出来，不用手改
int startKickMs = 150;  // 起步时先给满速"踢"多久（毫秒）。还起不来就加到 400
// 循迹时每隔多久刷新一次 OLED（毫秒）。刷一次屏要往 I2C 发 1024 字节，约 23 毫秒，
// 这期间电机收不到新指令。数值越大，屏幕刷新越慢，但这段"控制盲区"越小。
unsigned long oledMs = 100;
int debugLaunch = 1;    // 1 = 发车阶段把传感器读数打到串口（调好后改回 0）

// ---- 计时和起停相关 ----
// 注意：Uno 上 int 最大只能到 32767，180000 装不下，所以这几个要用 unsigned long
//
// 【判圈为什么不能再要求"双黑持续一段时间"】
// 起始线是【垂直于赛道、横跨整条赛道】的一条黑带，两个传感器装在同一根传感器条上
// （纵向位置几乎相同），所以车横穿起始线时，两个传感器是【几乎同时】压上、同时离开的。
// 旧代码要求双黑【连续保持 startLineMs = 80 毫秒】才算数，但起始线只有 1.5~2cm 厚：
//     压线时间 = 线厚 ÷ 车速 = 15~20mm ÷ 200~250mm/s = 60~80ms
// 正好卡在 80ms 门槛上、大多数时候够不到 -> 判圈永远不触发 -> 跑完一圈停不下来。
// 所以现在【不再要求持续时间】，只要求两个传感器在 startLineWindowMs 毫秒内都压到过黑线。
unsigned long startLineWindowMs = 150;  // 两个传感器在这段时间内都压到过黑线就算回到起始线
// 【怎么调】正常过线时两个传感器几乎同时压到，时间差只有几毫秒（传感器条装歪了也就几十毫秒），
//   所以这个窗口不用大。它的作用只是"别把两次相隔很久的单边压线凑成一次判圈"。
//   跑完一圈停不下来 -> 加大（250、350）；弯道中途误停 -> 减小（100、80）。
unsigned long lockoutMs    = 300;  // 数到一条黑线后先锁这么久，防止同一条线被反复数到
unsigned long lapMinMs     = 5000;  // 第 1 条黑线之后至少跑这么久，第 2 条黑线才算"跑完一圈"
unsigned long lapMaxMs     = 180000;// 3 分钟限时，超时就停车（从起步那一刻算起）

// [重要] 下面所有字符串都写成 F("...")。AVR 上普通字符串占 RAM，F() 把它放进 Flash。
// 原因：128x64 的 OLED 需要 malloc 1024 字节显存，UNO 只有 2048 字节 RAM，
//       字符串占多了 malloc 就会失败，屏幕一片黑（编译器不会报任何警告）。
// OLED 屏幕对象。用这个库的时候必须这样写一行
Adafruit_SSD1306 oled(128, 64, &Wire, -1);

// ============================== setup ==============================
void setup() {
  // 传感器是模拟输出，设成输入
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

  Serial.begin(9600);         // 串口，用来在监视器里看数据

  // ---- 报告"复位原因"：用来判断单片机是不是重启过 ----
  // MCUSR 是 AVR 记录上次复位原因的一个寄存器
  int resetCause = MCUSR;
  MCUSR = 0;                  // 读完清掉，下次开机才是新的原因
  if (debugLaunch == 1) {
    Serial.print(F("reset cause = 0x"));
    Serial.println(resetCause, HEX);
    Serial.println(F("  bit0 PORF=上电  bit1 EXTRF=复位键  bit2 BORF=电压掉太低  bit3 WDRF=看门狗"));
  }

  // 初始化 OLED。SSD1306_SWITCHCAPVCC 是库规定的写法，0x3C 是屏幕的 I2C 地址
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);   // SSD1306_WHITE 就是"点亮这个点"
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println(F("Line Car Ready"));
  oled.print(F("threshold = "));
  oled.println(threshold);
  oled.display();
}

// ============================== loop ==============================
void loop() {
  // -----------------------------------------------------------------
  // 第一步：显示待机画面，等启动键按下
  // -----------------------------------------------------------------
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

  // 一直在这里循环，直到读到 HIGH（也就是按键被按下）
  while (digitalRead(startButton) == LOW) {
    delay(10);
  }
  // 简易消抖：等 30 毫秒再确认一次，还是 HIGH 才算真的按下
  delay(30);
  if (digitalRead(startButton) == LOW) {
    return;   // 刚才只是抖动，回到 loop 开头重新等
  }
  // 等手指松开，避免一次按压被当成两次
  while (digitalRead(startButton) == HIGH) {
    delay(10);
  }

  // -----------------------------------------------------------------
  // 第二步：倒计时 3 秒
  // -----------------------------------------------------------------
  // for 循环：i 从 3 数到 1，每轮减 1
  for (int i = 3; i >= 1; i = i - 1) {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.println(F("Ready..."));
    oled.setTextSize(4);            // 换成大号字
    oled.setCursor(50, 24);
    oled.print(i);
    oled.display();

    tone(buzzer, 900);   // 倒计时短响
    delay(150);
    noTone(buzzer);
    delay(850);
  }

  // -----------------------------------------------------------------
  // 第三步：起步，然后【立刻开始循迹】（不再有"发车阶段"）
  // -----------------------------------------------------------------
  // 旧写法是先盲冲，等两个传感器【同时】压到起始线才进入循迹。
  // 那要求起始线比传感器间距还宽、车横向只能偏 2~3mm，实际很难满足；
  // 结果是车一直进不了循迹，盲冲 5 秒后停住（屏幕一直停在 GO !）。
  // 现在起步就直接循迹，"黑线"只用来数数：
  //   第 1 次数到 -> 开始计时；第 2 次数到 -> 跑完一圈，停车。
  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setCursor(24, 24);
  oled.print(F("GO !"));
  oled.display();

  digitalWrite(ledRun, HIGH);
  digitalWrite(ledStat, HIGH);

  // MX1508：反向输入保持 LOW，在前进输入上用 PWM 调速（0~255）
  analogWrite(leftBack, 0);
  analogWrite(rightBack, 0);
  // 起步"踢一脚"：先给满速 255 突破齿轮的静摩擦
  // 直流电机静止时阻力比转动时大得多，直接给 speedFast 常常原地不转
  analogWrite(leftForward, 255);
  analogWrite(rightForward, 255);
  delay(startKickMs);

  // 再降回正常的循迹速度
  analogWrite(leftForward, fastLeft);
  analogWrite(rightForward, speedFast);

  unsigned long runStart = millis();    // 起步时刻，用来做 3 分钟超时保护

  // -----------------------------------------------------------------
  // 第四步：循迹跑一圈，数黑线
  // -----------------------------------------------------------------
  unsigned long startTime = 0;      // ★ 0 = 还没压到第一条黑线，计时还没开始

  int finished   = 0;            // 1 = 正常跑完一圈，0 = 超时
  int lineCount  = 0;            // 已经数到几条黑线：0 = 还没压到，1 = 已开始计时
  int armed      = 1;            // 1 = 可以数下一条；0 = 刚数过，锁定中
  unsigned long lockTime = 0;    // 上一次数到黑线的时刻，用来做锁定
  unsigned long oledTime     = 0;   // 上一次刷新屏幕的时刻
  unsigned long leftBlackTime  = 0; // 左边最近一次压到黑线的时刻
  unsigned long rightBlackTime = 0; // 右边最近一次压到黑线的时刻
  unsigned long passed    = 0;      // 已经过去的毫秒数
  int totalSecond = 0;
  int minute      = 0;
  int second      = 0;
  int hundredth   = 0;

  while (true) {
    // ---------- ① 读传感器，判断黑白 ----------
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

    // ---------- ② 数黑线：第 1 次开始计时，第 2 次停车 ----------
    // 判一条黑线的办法：两个传感器在 startLineWindowMs 毫秒内【都】压到过黑线。
    // 起始线横跨赛道，正常过线时两个传感器几乎同时压到，时间差只有几毫秒。
    //
    // 【为什么不能在"两个都白"的时候清空计时】
    // 起始线只有 1.5~2cm 很薄。如果两个传感器前后差一点点，就会出现
    // "左边已经过了黑线、右边还没压上"的短暂【双白】。要是在双白时把计时清零，
    // 这条线就直接漏掉了 —— 薄线最常见的就是这个坑。
    // 所以这里只记录"每一边最后一次压到黑线的时刻"，靠两者的差值判断是不是同一条线。
    if (leftBlack == 1)  leftBlackTime  = millis();
    if (rightBlack == 1) rightBlackTime = millis();

    // armed：数到一条之后锁定 lockoutMs，防止同一条线在连续几轮循环里被反复数到
    if (armed == 0 && millis() - lockTime > lockoutMs) {
      armed = 1;
    }

    if (armed == 1 && leftBlackTime != 0 && rightBlackTime != 0) {
      unsigned long gap = 0;                     // 两次压线的时间差
      if (leftBlackTime > rightBlackTime) {
        gap = leftBlackTime - rightBlackTime;
      } else {
        gap = rightBlackTime - leftBlackTime;
      }
      if (gap < startLineWindowMs) {
        armed     = 0;                           // 锁上
        lockTime  = millis();
        leftBlackTime  = 0;                      // 清掉，下一条线要重新压到才算
        rightBlackTime = 0;
        if (lineCount == 0) {
          lineCount = 1;
          startTime = millis();                  // ★ 第 1 条黑线：开始计时
        }
        else if (millis() - startTime > lapMinMs) {
          finished = 1;                          // ★ 第 2 条黑线：跑完一圈，停车
          break;
        }
      }
    }

    // ---------- ③ 决定 4 个脚各给多少 ----------
    // 先按"直行"把 4 个值都填好，下面只在需要转弯时改动其中几个。
    // 这样就不用每个分支都写一遍，也不会漏掉某个脚。
    int leftGo   = fastLeft;    // 左轮 前进值
    int leftRev  = 0;           // 左轮 反转值
    int rightGo  = speedFast;   // 右轮 前进值
    int rightRev = 0;           // 右轮 反转值

    if (leftBlack == 1 && rightBlack == 0) {
      // 只有左边压到黑线 -> 线在车的左边 -> 车偏右 -> 向左转。
      // 左轮当"内侧"往后倒，右轮当"外侧"往前冲，车身几乎是原地转。
      leftGo   = 0;
      leftRev  = speedBack;
      rightGo  = speedTurn;
    }
    else if (leftBlack == 0 && rightBlack == 1) {
      // 只有右边压到黑线 -> 线在车的右边 -> 车偏左 -> 向右转。
      leftGo   = turnLeft;
      rightGo  = 0;
      rightRev = speedBack;
    }
    // 剩下两种情况都是直行，直接用上面填好的默认值：
    //   两个都白 = 车正正压在线上（本车是"夹住线"的摆法）
    //   两个都黑 = 起始线

    // ---------- ④ 一次性把 4 个脚写出去 ----------
    // 只在这一处写，就不会出现"某个脚忘了写、残留上一次的反转"
    analogWrite(leftForward,  leftGo);
    analogWrite(leftBack,     leftRev);
    analogWrite(rightForward, rightGo);
    analogWrite(rightBack,    rightRev);

    // ---- 3 分钟超时保护 ----
    if (millis() - runStart > lapMaxMs) {
      finished = 0;
      break;
    }

    // ---- 每 100 毫秒刷新一次屏幕 ----
    if (millis() - oledTime > oledMs) {
      oledTime = millis();

      passed = 0;                            // 还没压到第一条黑线时显示 00:00.00
      if (startTime != 0) {
        passed = millis() - startTime;       // 已经过去多少毫秒
      }
      totalSecond = passed / 1000;          // 换算成整秒
      minute      = totalSecond / 60;       // 分钟
      second      = totalSecond % 60;       // 秒（% 是取余数）
      hundredth   = (passed % 1000) / 10;   // 百分秒

      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setCursor(0, 0);
      oled.print(F("Running  L"));
      oled.println(lineCount);

      // 大号字显示  分:秒.百分秒
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

      // 画 3 分钟进度条
      int bar = (int)((millis() - startTime) * 100 / lapMaxMs);
      if (bar > 100) bar = 100;
      oled.drawRect(0, 54, 104, 10, SSD1306_WHITE);   // 先画外面的方框
      oled.fillRect(2, 56, bar, 6, SSD1306_WHITE);    // 再用实心块表示进度

      oled.display();
    }
  }

  // -----------------------------------------------------------------
  // 第五步：停车，显示成绩
  // -----------------------------------------------------------------
  analogWrite(leftForward, 0);
  analogWrite(leftBack, 0);
  analogWrite(rightForward, 0);
  analogWrite(rightBack, 0);
  digitalWrite(ledRun, LOW);
  digitalWrite(ledStat, LOW);

  passed = 0;
  if (startTime != 0) {
    passed = millis() - startTime;
  }
  totalSecond = passed / 1000;
  minute      = totalSecond / 60;
  second      = totalSecond % 60;
  hundredth   = (passed % 1000) / 10;

  // 电机已经停止：完成高音提示，超时或发车失败低音提示
  if (finished == 1) {
    tone(buzzer, 1500);
  } else {
    tone(buzzer, 400);
  }
  delay(300);
  noTone(buzzer);

  // 成绩画面
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  if (finished == 1) {
    oled.println(F("FINISHED!  time:"));
  } else {
    oled.println(F("STOPPED / TIMEOUT"));
  }
  oled.setTextSize(2);
  oled.setCursor(0, 24);
  if (minute < 10) oled.print(F("0"));
  oled.print(minute);
  oled.print(F(":"));
  if (second < 10) oled.print(F("0"));
  oled.print(second);
  oled.print(F("."));
  if (hundredth < 10) oled.print(F("0"));
  oled.print(hundredth);
  oled.setTextSize(1);
  oled.setCursor(0, 48);
  oled.println(F("press START again"));
  oled.display();

  // 串口里也打印一份，方便记录成绩
  Serial.print(F("time = "));
  Serial.print(minute);
  Serial.print(F(":"));
  Serial.print(second);
  Serial.print(F("."));
  Serial.println(hundredth);

  delay(5000);   // 成绩画面保持 5 秒，然后回到开头重新等按键
}
