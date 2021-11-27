#include <EncButton.h>

#define MOTOR_ENC1 2 // interruption 0
#define MOTOR_ENC2 3 // interruption 1

EncButton<EB_TICK, 12, 13, 11> enc; // pins 11, 12, 13

volatile int count = 0;
void encoderHandler()
{
    if (digitalRead(MOTOR_ENC2) == LOW)
        count++;
    else
        count--;
}

volatile int count2 = 0;
void encoderHandler2()
{
    if (digitalRead(MOTOR_ENC1) == HIGH)
        count2++;
    else
        count2--;
}

void setup() {
    attachInterrupt(digitalPinToInterrupt(MOTOR_ENC1), encoderHandler, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_ENC2), encoderHandler2, RISING);
    Serial.begin(9600);
}

int oldCount;
int oldCount2;
void loop() {
    enc.tick();
    if (enc.press())
    {
        count = 0;
        count2 = 0;
    }

    if (oldCount != count || oldCount2 != count2)
    {
        Serial.println("0: " + String(count) + "   1: " + String(count2));
        oldCount = count;
        oldCount2 = count2;
    }
}
