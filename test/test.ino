#include <LiquidCrystal_I2C.h>

#define RED_BUTTON 4
#define GREEN_BUTTON 7
#define YELLOW_BUTTON 8
#define ENCODER_PRESS 11
#define ENCODER1 12
#define ENCODER2 13
#define STEP_COUNT 540
#define SPEED 255

// Устанавливаем номера пинов к которым подключен дисплей
LiquidCrystal_I2C lcd(0x27, 16, 2);
volatile int16_t Count = 0;
volatile int16_t Count2 = 0;
int16_t Count3 = 0;
int16_t Count4 = 0;
bool ButtonState = 0;
bool Move = false;

bool oldState3;
bool oldState4;

void setup() {
  lcd.init();
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Andrey");
  lcd.setCursor(0, 1);
  lcd.print("Hello");
  
  Serial.begin(9600);
  pinMode(13, OUTPUT);

  attachInterrupt(0, Handler, RISING);
  attachInterrupt(1, Handler2, RISING);

  bool oldState3 = digitalRead(2);
  bool oldState4 = digitalRead(3);
}

void loop() {
  bool newButtonState = digitalRead(RED_BUTTON);
  if (newButtonState == HIGH)
  {
    if (newButtonState != ButtonState)
    {
      Start();
      ButtonState = newButtonState;
    }
  }
  else
    ButtonState = LOW;

  if (digitalRead(GREEN_BUTTON) == HIGH) {
    Count = 0;
    Count2 = 0;
    Count3 = 0;
    Count4 = 0;
  }
/*
  while (digitalRead(2) == oldState3);
  {
    Count3++;
    oldState3 = !oldState3;
  }

  while (digitalRead(3) == oldState4);
  {
    Count4++;
    oldState4 = !oldState4;
  }
*/
  
  if (Move) {
    //String output = "Time " + String(millis(), DEC) + "  Count = " + String(Count, DEC);
    //Serial.println(output);
    //lcd.setCursor(0, 0);
    //lcd.print(output);

    if (Count >= STEP_COUNT) {
      Stop();
      //lcd.setCursor(0, 0);
      //lcd.print(output);
    }
  }

  String output = String(millis(), DEC) + "  Count =  " + String(Count, DEC);
  Serial.println(output);
  output = String(millis(), DEC) + "  Count2 = " + String(Count2, DEC);
  Serial.println(output);

    if (digitalRead(ENCODER_PRESS) == HIGH)
      Serial.print("Released");
    else
      Serial.print("Pressed");

    Serial.print("  ENCODER1 = " + String(digitalRead(ENCODER1), DEC));
    Serial.println("  ENCODER2 = " + String(digitalRead(ENCODER2), DEC));
  /*
  output = String(millis(), DEC) + "  Count3 = " + String(Count3, DEC);
  Serial.println(output);
  output = String(millis(), DEC) + "  Count4 = " + String(Count4, DEC);
  Serial.println(output);
  */
}

void Start() {
  Count = 0;
  Count2 = 0;
  Move = true;
  analogWrite(6, SPEED);
}

void Stop() {
  Move = false;
  digitalWrite(6, 0);
}

void Handler() {
  Count++;
}

void Handler2() {
  Count2++;
}

class Mover
{
  public:
    int Counter;
    
    Mover(int steps, bool forward)
    {
      _steps = steps;
      _direction = forward ? 1 : -1;
    }

    void Move()
    {
      if (_finished)
        return;
      
      if (Counter < _steps / 2)
        Accelerate();
      else
        Decelerate();
    }

  private:
    const int _minSpeed = 10;
    const int _maxSpeed = 255;
    int _steps;
    int _direction;
    bool _finished;

    void Initialize(int steps, bool forward)
    {
      Counter = 0;
      _steps = steps;
      _direction = forward ? 1 : -1;
      _finished = false;
    }
    
    void Accelerate()
    {
    }

    void Decelerate()
    {
    }
};
