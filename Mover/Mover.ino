#include <LiquidCrystal_I2C.h>

#define RED_BUTTON 4
#define MIN_PWM 10;

volatile int graduationCount;
volatile int Count2;

class Mover
{
    public:
        // number of graduations for acceleration and deceleration
        // from min to max PWM and vice versa
        // Shouldn't be less than 20
        int accelerationLength = 100;

        void attach()
        {
            attachInterrupt(0, encoderHandler, RISING);
        }

        void proceed()
        {
            if (!isRunning())
                return;

            if (graduationCount >= _graduations)
            {
                stop();
                return;
            }

            makeStep();
        }

        void move(int graduations, bool forward)
        {
            if (isRunning())
                return;

            initialize(graduations, forward);
            makeStep();
        }

        void stop()
        {
            analogWrite(pins[0], 0);
            analogWrite(pins[1], 0);
            _isRunning = false;
            _isInitialized = false;
        }

        bool isRunning()
        {
            return _isRunning;
        }

        bool isStopped()
        {
            return !isRunning();
        }

    private:
        const int _minSpeed = MIN_PWM;
        const int _maxSpeed = 255;
        const int pins[2] = {6, 7};

        bool _isRunning = false;
        bool _isInitialized = false;
        int _graduations;
        byte _pinIndex;
        unsigned long _oldTime;
        int _currentSpeed;

        static void encoderHandler()
        {
            // Check whether second signal from encoder is 0
            // !!! test in backward direction!
            if (digitalRead(3) == LOW)
                graduationCount++;
        }

        void initialize(int graduations, bool forward)
        {
            graduationCount = 0;
            _graduations = graduations;
            _pinIndex = forward ? 0 : 1;
            _currentSpeed = _minSpeed;
            _isInitialized = true;
            _isRunning = true;
        }

        void makeStep()
        {
            if (!_isInitialized)
              return;

            if (graduationCount >= _graduations)
            {
                stop();
                return;
            }

            float x;
            if (graduationCount < _graduations / 2)
            {
                x = graduationCount;
            }
            else
            {
                x = _graduations - graduationCount - 1;
            }

            // Use linear function to accelerate/decelerate
            _currentSpeed = _minSpeed + x * (_maxSpeed - _minSpeed) / accelerationLength;
            // Validate
            if (_currentSpeed > _maxSpeed)
                _currentSpeed = _maxSpeed;
            else if (_currentSpeed < _minSpeed)
                _currentSpeed = _minSpeed;            

            analogWrite(pins[_pinIndex], _currentSpeed);
            //Serial.println("Current speed: " + String(_currentSpeed, DEC) + " graduationCount: " + String(graduationCount, DEC));
        }
};

LiquidCrystal_I2C lcd(0x27, 16, 2);
bool ButtonState = false;
bool started = false;
Mover mover;

void setup() {
  lcd.init();
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Andrey");
  lcd.setCursor(0, 1);
  lcd.print("Hello");
  
  Serial.begin(9600);

  mover.attach();
  attachInterrupt(1, Handler2, RISING);
}

void Handler2()
{
    if (digitalRead(2) == HIGH)
        Count2++;
    else
        Count2--;
}

int stepNumber;
int oldCommonCount;

void loop() {
  const int stepCount = 4; // 4, 8, 18, 20, 24, 30, 36, 45, 60, 72, 90, 120
  const int graduations = 1080 / stepCount;

  bool newButtonState = digitalRead(RED_BUTTON);
  if (newButtonState == HIGH)
  {
    if (newButtonState != ButtonState)
    {
      mover.move(graduations, true);
      ButtonState = newButtonState;
      Count2 = 0;
      stepNumber = 0;
      oldCommonCount = 0;
      started = true;
    }
  }
  else
    ButtonState = LOW;

  mover.proceed();
  if (started && mover.isStopped())
  {
    stepNumber++;
    Serial.println("Graduations passed: " + String(graduationCount, DEC));
    Serial.println("Step: " + String(stepNumber, DEC));
    if (stepNumber >= stepCount)
    {
      started = false;
      stepNumber = 0;
    }
    else
      mover.move(graduations, true);
  }

  if (oldCommonCount != Count2)
  {
    Serial.println("Graduations total: " + String(Count2, DEC));
    oldCommonCount = Count2;
  }
}
