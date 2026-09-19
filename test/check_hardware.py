"""执行真实 .cpp 的桌面模拟检查；运行 python3 test/check_hardware.py。无需第三方库。"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
stub = r'''
#pragma once
#include <cassert>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>
#include <utility>
#define F(s) (s)
constexpr int LOW=0, HIGH=1, INPUT=0, OUTPUT=1, A0=14, A1=15;
constexpr int SSD1306_SWITCHCAPVCC=2, SSD1306_WHITE=1, HEX=16;
struct WireClass {
  void begin(){}
  void beginTransmission(int){}
  int endTransmission(){ return 4; }   // 4 = 总线上没有设备应答
};
WireClass Wire;
int pwm[20]={}, modes[20]={}, buttonReads=0, command=0, polarity=1, scenario=0;
unsigned long now=0;
int toneCalls=0, toneHz=0;
void tone(int p,int hz){assert(p==4 && hz>0); ++toneCalls; toneHz=hz;}
void noTone(int p){assert(p==4); toneHz=0;}
std::vector<std::pair<int,int>> speeds;
void pinMode(int p,int m){ modes[p]=m; }
void analogWrite(int p,int v){
  assert(p==5 || p==6 || p==9 || p==10);
  assert(v>=0 && v<=255); pwm[p]=v;
  assert(!(pwm[5] && pwm[6])); assert(!(pwm[9] && pwm[10]));
  speeds.emplace_back(pwm[9],pwm[5]);
}
void digitalWrite(int p,int v){ assert(p==11 || p==12); pwm[p]=v; }
void delay(unsigned long t){ now+=t; assert(now<200000); }
void delayMicroseconds(unsigned long){}
unsigned long millis(){ now+=10; assert(now<200000); return now; }
int digitalRead(int p){ assert(p==7); ++buttonReads; return buttonReads==2 || buttonReads==3; }
int analogRead(int p){
  assert(p==A0 || p==A1);
  bool black = now<3600 || now>=10000;
  if(now>=3600 && now<4000) black=p==A1;
  if(now>=4000 && now<4400) black=p==A0;
  if(now>=4400 && now<10000) black=false;
  if(scenario==1) black=false; // 找不到起始线
  if(scenario==2 && now>=3600) black=p==A1; // 始终单黑，跑圈超时
  return black==bool(polarity) ? 950 : 100;
}
struct Port {
  std::ostringstream out;
  void begin(int){} int available(){return command!=0;}
  int read(){int c=command; command=0; return c;}
  template<class T> void print(T v){out<<v;}
  template<class T> void println(T v){out<<v<<'\n';}
  template<class T,class U> void print(T v,U){out<<v;}
  template<class T,class U> void println(T v,U){out<<v<<'\n';}
} Serial;
struct Adafruit_SSD1306 : Port {
  Adafruit_SSD1306(int,int,WireClass*,int){}
  bool begin(int,int){return true;}
  void clearDisplay(){} void display(){} void setTextColor(int){} void setTextSize(int){}
  void setCursor(int,int){} void drawRect(int,int,int,int,int){}
  void fillRect(int,int,int,int,int){}
};
'''
common = r'''
  setup();
  assert(buzzer==4 && modes[4]==OUTPUT);
  assert(leftSensorPin==A0 && rightSensorPin==A1);
  assert(modes[7]==INPUT && modes[11]==OUTPUT && modes[12]==OUTPUT);
  assert(pwm[5]==0 && pwm[6]==0 && pwm[9]==0 && pwm[10]==0);
'''
bringup = r'''
  command='m'; loop(); assert(pwm[5]==150 && pwm[9]==150);
  command='n'; loop(); assert(pwm[6]==150 && pwm[10]==150);
  command='q'; loop(); assert(pwm[5]==70 && pwm[9]==150);   // 左转：左轮慢、右轮快
  command='w'; loop(); assert(pwm[5]==150 && pwm[9]==70);   // 右转：左轮快、右轮慢
  command='s'; loop(); assert(pwm[5]+pwm[6]+pwm[9]+pwm[10]==0);
  command='l'; loop(); command='m'; loop(); assert(pwm[5]==100 && pwm[9]==100);
  command='s'; loop();
  command='t'; loop(); assert(pwm[11]==0 && pwm[12]==0);
  assert(toneCalls==2 && toneHz==0);
  command='b'; loop(); assert(Serial.out.str().find("start button = 0")!=std::string::npos);
  command='b'; loop(); assert(Serial.out.str().find("start button = 1")!=std::string::npos);
  command='r'; loop(); command='o'; loop();
'''
main = r'''
  polarity=blackHigh=std::atoi(argv[1]); scenario=std::atoi(argv[2]);
  loop();
  assert(buttonReads>=4);
  assert(toneCalls==4 && toneHz==0);
  assert(pwm[5]+pwm[6]+pwm[9]+pwm[10]==0);
  assert(pwm[11]==0 && pwm[12]==0);
  assert(oled.out.str().find(scenario==0 ? "FINISHED!" : "STOPPED / TIMEOUT")!=std::string::npos);
  if(scenario==0){
    bool left=false,right=false;
    for(auto v:speeds){left|=v==std::make_pair(180,90); right|=v==std::make_pair(90,180);}
    assert(left && right); // 左黑：左轮 90 慢 / 右轮 180 快；右黑反之，不能接反
  }
'''
with tempfile.TemporaryDirectory(prefix='lec1-check-') as tmp:
    tmp = Path(tmp)
    for header in ('Arduino.h', 'Wire.h', 'Adafruit_GFX.h', 'Adafruit_SSD1306.h'):
        (tmp / header).write_text(stub if header == 'Arduino.h' else '#include "Arduino.h"\n')
    for name, checks in [('bringup', bringup), ('main', main)]:
        harness = tmp / (name + '.cpp')
        harness.write_text(f'#include "{root}/src/{name}.cpp"\nint main(int argc,char** argv){{\n' + common + checks + '\n}\n')
        binary = tmp / name
        subprocess.run(['c++', '-std=c++11', '-I', str(tmp), str(harness), '-o', str(binary)], check=True)
        cases = [()] if name == 'bringup' else [(str(p), str(s)) for p in (0, 1) for s in (0, 1, 2)]
        for args in cases:
            subprocess.run([str(binary), *args], check=True, timeout=10)
        print(name + ': PASS')
