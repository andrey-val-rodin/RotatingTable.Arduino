#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"
#include <EncButton.h>
#pragma GCC diagnostic pop
#include <LiquidCrystal_I2C.h>
#include <PWM.h>

#define MOTOR1 10
#define MOTOR2 9
#define MOTOR_ENC1 2
#define MOTOR_ENC2 3
#define CAMERA 5
#define SHUTTER 6
#define CAMERA_LOW HIGH
#define CAMERA_HIGH LOW
#define MIN_PWM 60
#define MAX_PWM 255
#define GRADUATIONS 4320 // number of graduations per turn
#define DEGREE (GRADUATIONS / 360)

Encoder encoder(MOTOR_ENC1, MOTOR_ENC2);

LiquidCrystal_I2C lcd(0x27, 16, 2);
EncButton<EB_TICK, 12, 13, 11> enc; // pins 11, 12, 13
EncButton<EB_TICK, 7> photoButton;  // pin 7
EncButton<EB_TICK, 4> nextButton;   // pin 4

const unsigned char stepsLength = 22;
const int16_t steps[stepsLength] =
    { 2, 4, 5, 6, 8, 9, 10, 12, 15, 18, 20, 24, 30, 36, 40, 45, 60, 72, 90, 120, 180, 360 };
char FindInSteps(int16_t numberOfSteps)
{
    for (unsigned char i = 0; i < stepsLength; i++)
    {
        if (steps[i] == numberOfSteps)
            return i;
    }

    return -1;
}

class Runner
{
    public:
        static const int delta = 5; // value to increment/decrement pwm
        
        static void runAutomatic();
        
    private:
        enum State : char
        {
            Waiting,
            Beginning,
            Delay,
            Exposure,
            Move
        };
        
        static void finalize();
        static void incrementStep();
        static void display(String top, String stepName);
};
Runner runner;

struct MenuItem
{
    String top;
    String bottom;
};

typedef void (*callback_t)();
struct MenuItemsDef
{
    static const char topItemsLength = 6;
    static MenuItem topItems[topItemsLength];

    static const char settingsItemsLength = 4;
    static MenuItem settingsItems[settingsItemsLength];

    static const callback_t handlers[1];
};

MenuItem MenuItemsDef::topItems[topItemsLength] = {
    {"%Auto",       ""},
    {"%Manual",     ""},
    {"%Nonstop",    ""},
    {"Video",       ""},
    {"Rotate 90",   ""},
    {"Settings",    ""}
};
MenuItem MenuItemsDef::settingsItems[settingsItemsLength] = {
    {"Steps",        ""},
    {"Acceleration", ""},
    {"Delay",        ""},
    {"Exposure",     ""}
};
const callback_t MenuItemsDef::handlers[1] = {
    Runner::runAutomatic
};

class PWMValidator
{
    public:
        static int validate(int pwm)
        {
            if (pwm > MAX_PWM)
                pwm = MAX_PWM;
            else if (pwm < MIN_PWM)
                pwm = MIN_PWM;

            return pwm;
        }
};

class Settings
{
    public:
        static int16_t getSteps()
        {
            return _steps;
        }

        static void setSteps(int16_t value)
        {
            _steps = value;
        }

        static char getAcceleration()
        {
            return _acceleration;
        }

        // Returns number of graduations for acceleration and deceleration from min to max PWM and vice versa.
        // Function converts current user-friendly value 1-10 to this value.
        // Note that real value shouldn't be less than 80 when GRADUATIONS = 4320.
        static int getRealAcceleration()
        {
            int result = getAcceleration();
            result = abs(result - 11); // reverse
            result *= 10;
            result += 10;
            result *= DEGREE;
            result /= 3;
            return result; // value in range from 80 to 440 when GRADUATIONS = 4320
        }

        static void setAcceleration(char value)
        {
            _acceleration = value;
        }

        static uint16_t getDelay()
        {
            return 0;
        }

        static uint16_t getExposure()
        {
            return 100;
        }

    private:
        static int16_t _steps;
        static char _acceleration;
};
int16_t Settings::_steps;
char Settings::_acceleration;

class Menu
{
    public:
        char current;

        void setItems(const MenuItem* items, char length)
        {
            _items = items;
            _length = length;
            _mode = MenuItems;
            current = 0;
        }

        void setItems(const int16_t* items, char length)
        {
            _array = items;
            _length = length;
            _mode = Array;
            current = 0;
        }

        void setItems(char upperBound, char multiplier, char offset)
        {
            _length = upperBound;
            _offset = offset;
            _multiplier = multiplier;
            _mode = Range;
            current = 0;
        }

        void display()
        {
            String top;
            unsigned char index = (unsigned char) current;
            switch (_mode)
            {
                case MenuItems:
                    top = _items[index].top;
                    if (top.startsWith("%"))
                        top = formatSteps(top);
                    printTop(top);
                    printBottom(_items[index].bottom);
                    break;

                case Array:
                    printTop(String(_array[index], DEC));
                    printBottom("");
                    break;
                
                case Range:
                    printTop(String((current + _offset) * _multiplier, DEC));
                    printBottom("");
                    break;
            }
        }

        String formatSteps(String top)
        {
            top.remove(0, 1); // remove % sign
            char strBuf[20];
            sprintf(strBuf, "%s (%d)", top.c_str(), Settings::getSteps());
            return strBuf;
        }
        
        void display(String top, String bottom)
        {
            printTop(top);
            printBottom(bottom);
        }

        void next()
        {
            current++;
            if (current >= _length)
                current = _length - 1;
        }

        void prev()
        {
            current--;
            if (current < 0)
                current = 0;
        }

    private:
        enum Mode : char
        {
            MenuItems,
            Array,
            Range
        };

        Mode _mode;
        const MenuItem* _items;
        const int16_t* _array;
        char _length;
        char _offset;
        char _multiplier;
        
        String _recentTop;
        String _recentBottom;

        void validateCurrent()
        {
            if (current < 0)
                current = 0;
            else
            {
                if (current >= _length)
                    current = _length - 1;
            }
        }
        
        void printTop(String text)
        {
            if (_recentTop != text)
            {
                _recentTop = text;
                text = fillWithSpaces(text);
                lcd.setCursor(0, 0);
                lcd.print(text);
            }
        }

        void printBottom(String text)
        {
            if (_recentBottom != text)
            {
                _recentBottom = text;
                text = fillWithSpaces(text);
                lcd.setCursor(0, 1);
                lcd.print(text);
            }
        }

        String fillWithSpaces(String text)
        {
            String result;
            result.reserve(16);
            result = text;
            while (result.length() < 16)
                result += " ";

            return result;
        }
};
Menu menu;

class Mover
{
    public:
        enum State : char
        {
            Stop,
            Move,
            Stopping,
            Correction,
            RunAcc,
            Run,
            RunDec
        };

        void tick()
        {
            switch (_state)
            {
                case Move:
                case Stopping:
                case Correction:
                    tickMove();
                    break;
                    
                case Run:
                case RunAcc:
                case RunDec:
                    tickRun();
                    break;

                default:
                    break;
            }
        }

        State getState()
        {
            return _state;
        }

        void move(int32_t graduations, int maxPWM = MAX_PWM)
        {
            if (!isStopped())
                return;

            _graduations = graduations;
            _forward = graduations > 0;
            _currentPWM = MIN_PWM;
            _maxPWM = maxPWM;
            _cumulativePos -= encoder.readAndReset();
            _state = Move;
        }

        void run(int pwm)
        {
            if (!isStopped())
                return;

            _maxPWM = abs(pwm);
            _forward = pwm > 0;
            _currentPWM = MIN_PWM;
            _cumulativePos -= encoder.readAndReset();
            _state = RunAcc;
        }

        void stop()
        {
            analogWrite(MOTOR1, 0);
            analogWrite(MOTOR2, 0);
            _state = Stop;
        }

        void softStop()
        {
            if (_state == Run)
            {
                // Calculate stop point
                float decelerationLength = Settings::getRealAcceleration();
                float graduationsToStop = (_currentPWM - MIN_PWM) * decelerationLength /
                    (MAX_PWM - MIN_PWM);
                if (!_forward)
                    graduationsToStop = -graduationsToStop;
                _graduations = getCurrentPos() + graduationsToStop;
                _state = RunDec; // deceleration state
            }
            else
                stop();
        }

        inline bool isStopped()
        {
            return _state == Stop;
        }

        // Returns direction
        inline bool isForward()
        {
            return _forward;
        }

        // Returns current PWM
        inline int getCurrentPWM()
        {
            return _currentPWM;
        }

        // Returns maximum PWM
        inline int getMaxPWM()
        {
            return _maxPWM;
        }

        void changePWM(int delta)
        {
            switch (_state)
            {
                case Move:
                    if (_currentPWM == _maxPWM)
                    {
                        _maxPWM += delta;
                        _maxPWM = PWMValidator::validate(_maxPWM);
                        _currentPWM = _maxPWM;
                        analogWrite(_forward? MOTOR1 : MOTOR2, _currentPWM);
                    }
                    break;
                    
                case Run:
                    _maxPWM += delta;
                    _maxPWM = PWMValidator::validate(_maxPWM);
                    _currentPWM = _maxPWM;
                    analogWrite(_forward? MOTOR1 : MOTOR2, _currentPWM);
                    break;

                default:
                    break;
            }
        }

        // Returns current position in graduations. Can be negative
        int32_t getCurrentPos()
        {
            int32_t pos = encoder.read();
            // Invert pos
            // clockwise movement is positive, counterclockwise movement is negative
            return -pos;
        }

        // Returns graduation count passed from starting point. Can be negative
        int32_t getAbsolutePos()
        {
            // Invert pos
            return _cumulativePos - encoder.read();
        }

        void resetAbsolutePos()
        {
            encoder.readAndReset();
            _cumulativePos = 0;
        }

    private:
        State _state = Stop;
        int32_t _graduations;
        bool _forward;
        int _maxPWM = MAX_PWM;
        int32_t _currentPos;
        int32_t _lastPos;
        int32_t _cumulativePos;
        int _currentPWM;
        unsigned long _timer;
        unsigned char _timePartCount;
        
        void tickMove()
        {
            _currentPos = getCurrentPos();
            bool reached = _forward
                ? _currentPos >= _graduations
                : _currentPos <= _graduations;
            if (reached)
            {
                switch (_state)
                {
                    case Move:
                        stop();
                        _state = Stopping;
                        _timePartCount = 0;
                        _lastPos = _currentPos;
                        _timer = millis();
                        return;

                    case Correction:
                        // finish
                        stop();
                        return;

                    default:
                        // continue
                        break;
                }
            }
            else if (_state == Move || _state == Correction)
            {
                bool firstHalf = _forward
                    ? _currentPos < _graduations / 2
                    : _currentPos > _graduations / 2;
                if (firstHalf)
                    accelerate();
                else
                    decelerate();

                analogWrite(_forward? MOTOR1 : MOTOR2, _currentPWM);
                return;
            }

            // Wait until table die
            if (_state == Stopping && millis() - _timer >= 10)
            {
                if (_currentPos == _lastPos)
                {
                    _timePartCount++;
                }
                else
                {
                    _timePartCount = 0;
                    _lastPos = _currentPos;
                }

                if (_timePartCount >= 10)
                {
                    // Table has been at rest for the last 10 times
                    makeCorrection();
                }
                else
                {
                    // continue
                    _timer = millis();
                }
            }
        }

        void accelerate()
        {
            // Use linear function to accelerate
            float x = _currentPos;
            if (!_forward)
                x = -x;
            float accelerationLength = Settings::getRealAcceleration();
            float currentPWM = MIN_PWM + x * (MAX_PWM - MIN_PWM) / accelerationLength;
            _currentPWM = validatePWM(currentPWM);
        }

        void decelerate()
        {
            // Use linear function to decelerate
            float x = _graduations - _currentPos;
            if (!_forward)
                x = -x;
            x -= getFinalDistance();
            float decelerationLength = Settings::getRealAcceleration();
            float currentPWM = MIN_PWM + x * (MAX_PWM - MIN_PWM) / decelerationLength;
            _currentPWM = validatePWM(currentPWM);
        }

        void makeCorrection()
        {
            int error = _graduations - _currentPos;
            if (error == 0)
            {
                // No correction needed, finish
                stop();
                return;
            }
            else
            {
                // Make correction
                Serial.print("Error->");

                stop();
                move(error, MIN_PWM);
                _state = Correction;
            }
        }

        // We should pass end of step with MIN_PWM
        // This function returns length of this final distance
        int getFinalDistance()
        {
            switch (Settings::getAcceleration())
            {
                case 10:
                    return _graduations > 10 * DEGREE ? 16 : 12;
                case 9:
                    return _graduations > 10 * DEGREE ? 10 : 8;
                case 8:
                    return _graduations > 10 * DEGREE ? 6 : 4;
                case 7:
                    return 2;
                default:
                    return 0;
            }
        }
        
        void tickRun()
        {
            switch (_state)
            {
                case RunAcc:
                    _currentPos = getCurrentPos();
                    accelerate();
                    analogWrite(_forward? MOTOR1 : MOTOR2, _currentPWM);
                    if (_currentPWM >= _maxPWM)
                        _state = Run;
                    break;
                    
                case RunDec:
                    _currentPos = getCurrentPos();
                    decelerate();
                    if (_currentPWM <= MIN_PWM)
                        stop();
                    else
                        analogWrite(_forward? MOTOR1 : MOTOR2, _currentPWM);
                    break;

                default:
                    break;
            }
        }

        int validatePWM(int pwm)
        {
            if (pwm > _maxPWM)
                pwm = _maxPWM;
            else if (pwm < MIN_PWM)
                pwm = MIN_PWM;

            return pwm;
        }
};
Mover mover;

int16_t stepNumber = 0;
unsigned long timer = 0;
bool isRunning = false;
void Runner::runAutomatic()
{
    const String mode = "Auto...";
    const String stepName = "photo";
    static State currentState;
    static int32_t lastGraduations;
    
    if (!mover.isStopped())
        return;

    int16_t stepCount = Settings::getSteps();
    int16_t stepGraduations = GRADUATIONS / stepCount;
    if (!isRunning)
    {
        // Starting
        stepNumber = 0;
        digitalWrite(CAMERA, CAMERA_HIGH); // prepare camera
        isRunning = true;
        lastGraduations = 0;
        mover.resetAbsolutePos();
        timer = millis();
        currentState = Beginning;
        return;
    }

    if (currentState == Beginning && millis() - timer >= Settings::getExposure())
    {
        stepNumber++;
        display(mode, stepName);
        digitalWrite(SHUTTER, CAMERA_HIGH); // make first photo
        timer = millis();
        currentState = Exposure;
        return;
    }
    
    if (currentState == Exposure && millis() - timer >= Settings::getExposure())
    {
        digitalWrite(SHUTTER, CAMERA_LOW); // release shutter
        currentState = Move;
        int32_t absolutePos = mover.getAbsolutePos();
        int error = lastGraduations - absolutePos;
        mover.resetAbsolutePos();
        lastGraduations = stepGraduations + error;
        mover.move(lastGraduations);
        return;
    }
    
    if (currentState == Move)
    {
        if (stepNumber < stepCount)
        {
            timer = millis();
            currentState = Delay;
        }
        else
        {
            finalize();
        }
        return;
    }

    if (currentState == Delay && millis() - timer >= Settings::getDelay())
    {
        stepNumber++;
        display(mode, stepName);
        digitalWrite(SHUTTER, CAMERA_HIGH); // make photo
        timer = millis();
        currentState = Exposure;
    }
}

void Runner::finalize()
{
    isRunning = false;
    mover.stop();
    stepNumber = 0;
    digitalWrite(SHUTTER, CAMERA_LOW); // release shutter
    digitalWrite(CAMERA, CAMERA_LOW); // release camera
    enc.resetState(); // reset encoder
}

void Runner::display(String top, String stepName)
{
    char strBuf[20];
    sprintf(strBuf, "%s %d (%d)", stepName.c_str(), stepNumber, Settings::getSteps());
    menu.display(top, strBuf);
}

enum State
{
    Waiting,
    Measuring,
    Measured
};

class TurnMeasurer
{
    public:
        inline State getState()
        {
            return _state;
        }

        void tick()
        {
            switch (_state)
            {
                case Waiting:
                case Measured:
                    return;
    
                case Measuring:
                    Runner::runAutomatic();
                    if (isRunning)
                        return;

                    _stop = millis();
                    _state = Measured;
            }
        }
    
        void measure(int acceleration)
        {
            Settings::setAcceleration(acceleration);
            _state = Measuring;
            _start = millis();
        }
    
        String getOutput()
        {
            float time = (_stop - _start) / 1000.0;
            return "\t" + String(time);
        }
    
        void cancel()
        {
            if (_state == Measuring)
            {
                mover.stop();
            }
    
            _state = Waiting;
        }

    private:
        State _state = Waiting;
        unsigned long _start;
        unsigned long _stop;
};

class Measurer
{
    public:
        inline State getState()
        {
            return _state;
        }

        void tick()
        {
            _measurer.tick();
            
            switch (_state)
            {
                case Measuring:
                    if (_measurer.getState() == Measured)
                    {
                        Serial.print(_measurer.getOutput());
                        _acceleration++;
                        if (_acceleration > 10)
                        {
                            _state = Measured;
                            return;
                        }
                        else
                        {
                            _measurer.measure(_acceleration);
                        }
                    }
                    return;

                default:
                    return;
            }
        }

        void measure(int fromAcc)
        {
            _acceleration = fromAcc;
            _state = Measuring;
            Serial.print(Settings::getSteps());
            _measurer.measure(_acceleration);
        }
    
        void cancel()
        {
            _measurer.cancel();
            _state = Waiting;
        }
    
    private:
        State _state = Waiting;
        TurnMeasurer _measurer;
        int _acceleration;
};

class Worker
{
    public:
        inline State getState()
        {
            return _state;
        }
        
        void tick()
        {
            if (_state == Waiting)
                return;
                
            _measurer.tick();

            if(_measurer.getState() == Measured)
            {
                _stepIndex++;
                if (_stepIndex > _stepIndexTo)
                {
                    _state = Waiting;
                    return;
                }

                Serial.println();
                Settings::setSteps(steps[_stepIndex]);
                _measurer.measure(_fromAcc);
            }
        }

        void start(int fromAcc = 1, int stepIndexFrom = 0, int stepIndexTo = stepsLength - 1)
        {
            _fromAcc = fromAcc;
            _stepIndexFrom = stepIndexFrom;
            _stepIndexTo = stepIndexTo;
            _state = Measuring;
            _stepIndex = _stepIndexFrom;
            Settings::setSteps(steps[_stepIndex]);
            Settings::setAcceleration(1);
            for (int i = fromAcc; i <= 10; i++)
            {
                Serial.print("\t");
                Serial.print(i);              
            }
            Serial.println();
            _measurer.measure(_fromAcc);
        }

        void cancel()
        {
            _measurer.cancel();
            _state = Waiting;
        }
        
    private:
        State _state = Waiting;
        Measurer _measurer;
        unsigned char _stepIndex;
        unsigned char _stepIndexFrom;
        unsigned char _stepIndexTo;
        int _fromAcc;
};
Worker worker;

void setup()
{
    pinMode(MOTOR_ENC1, INPUT);
    pinMode(MOTOR_ENC2, INPUT);
    pinMode(MOTOR1, OUTPUT);
    pinMode(MOTOR2, OUTPUT);
    pinMode(CAMERA, OUTPUT);
    pinMode(SHUTTER, OUTPUT);
    digitalWrite(SHUTTER, CAMERA_LOW); // release shutter
    digitalWrite(CAMERA, CAMERA_LOW); // release camera
    
    lcd.init();
    lcd.begin(16, 2);
    lcd.clear();
    lcd.backlight();

    photoButton.setButtonLevel(HIGH);
    nextButton.setButtonLevel(HIGH);

    SetPinFrequencySafe(MOTOR1, 15000);
    SetPinFrequencySafe(MOTOR2, 15000);

    Serial.begin(9600);
}

void loop()
{
    enc.tick();
    photoButton.tick();
    nextButton.tick();
    mover.tick();
    worker.tick();

    if (enc.press())
    {
        if (worker.getState() == Waiting)
            worker.start();
        else
            worker.cancel();
    }
}
