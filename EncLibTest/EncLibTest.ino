#include <Encoder.h>
#include <EncButton.h>

#define MOTOR_ENC1 2 // interruption 0
#define MOTOR_ENC2 3 // interruption 1

EncButton<EB_TICK, 12, 13, 11> button; // pins 11, 12, 13
Encoder myEnc(2, 3);

void setup() {
  Serial.begin(9600);
}

long oldPosition = -999;
void loop() {
  button.tick();
  if (button.press())
  {
    oldPosition = -999;
    myEnc.readAndReset();
  }

  long newPosition = -myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }
}
