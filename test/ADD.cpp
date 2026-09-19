#include <Arduino.h>

void setup() 
{
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(9600);

    int sum = 0;
    //累加
    for (int i = 1; i <= 100; i++) 
    {
        sum += i;
    }
    //输出
    Serial.print("1+2+...+100=");
    Serial.print(sum);
    if (sum % 2 == 0) 
    {
        Serial.println(",是偶数");
    } 
    else 
    {
        Serial.println(",是奇数");
    }

}

void loop(){
    
}