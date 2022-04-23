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
#define CAMERA_LOW LOW
#define CAMERA_HIGH HIGH
#define MIN_PWM 60
#define MAX_PWM 255
#define GRADUATIONS 4320 // number of graduations per turn
#define DEGREE (GRADUATIONS / 360)

// Uncomment below to enable debug output.
//#define DEBUG_MODE

Encoder encoder(MOTOR_ENC1, MOTOR_ENC2);

LiquidCrystal_I2C lcd(0x27, 16, 2);
EncButton<EB_TICK, 13, 12, 11> enc; // pins 11, 12, 13
EncButton<EB_TICK, 7> photoButton;  // pin 7
EncButton<EB_TICK, 8> nextButton;   // pin 8

uint16_t EEMEM stepsOffset;
unsigned char EEMEM accelerationOffset;
uint16_t EEMEM delayOffset;
uint16_t EEMEM exposureOffset;
uint16_t EEMEM videoPWMOffset;
float EEMEM nonstopFrequencyOffset;
unsigned char EEMEM menuIndexOffset;

const unsigned char stepsLength = 22;
const uint16_t steps[stepsLength] =
    { 2, 4, 5, 6, 8, 9, 10, 12, 15, 18, 20, 24, 30, 36, 40, 45, 60, 72, 90, 120, 180, 360 };
signed char FindInSteps(uint16_t numberOfSteps)
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
        static void runManual();
        static void runNonstop();
        static void runVideo();
        static void runRotate();
        
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
        static void display(const char* top, const char* stepName);
};
Runner runner;

struct MenuItem
{
    char top[17];
    char bottom[17];
};

typedef void (*callback_t)();
struct MenuItemsDef
{
    static const unsigned char topItemsLength = 6;
    static MenuItem topItems[topItemsLength];

    static const unsigned char settingsItemsLength = 4;
    static MenuItem settingsItems[settingsItemsLength];

    static const callback_t handlers[topItemsLength - 1];
};

MenuItem MenuItemsDef::topItems[topItemsLength] = {
    {"%Auto",         ""},
    {"%Manual",       ""},
    {"%Nonstop",      ""},
    {"Video",         ""},
    {"Rotate 90\337", ""},
    {"Settings",      ""}
};
MenuItem MenuItemsDef::settingsItems[settingsItemsLength] = {
    {"Steps",         ""},
    {"Acceleration",  ""},
    {"Delay",         ""},
    {"Exposure",      ""}
};
const callback_t MenuItemsDef::handlers[topItemsLength - 1] = {
    Runner::runAutomatic,
    Runner::runManual,
    Runner::runNonstop,
    Runner::runVideo,
    Runner::runRotate
};

class PWMValidator
{
    public:
        static unsigned char validate(int pwm)
        {
            if (pwm > MAX_PWM)
                pwm = MAX_PWM;
            else if (pwm < MIN_PWM)
                pwm = MIN_PWM;

            return pwm;
        }
};

#ifdef DEBUG_MODE
uint16_t lastLowSteps = 0;
uint16_t lastHighSteps = 0;
#endif
class Settings
{
    public:
        static constexpr float lowNonstopFrequency = 0.5;
        static constexpr float highNonstopFrequency = 3.0;
        
        static uint16_t getSteps()
        {
            return validateSteps(eeprom_read_word(&stepsOffset));
        }

        static bool checkSteps(uint16_t value)
        {
            return FindInSteps(value) >= 0;
        }

        static uint16_t validateSteps(uint16_t value)
        {
            return checkSteps(value)
                ? value
                : 24; // use default
        }

        static void setSteps(uint16_t value)
        {
            eeprom_update_word(&stepsOffset, value);
        }

        static unsigned char getAcceleration()
        {
            return validateAcceleration(eeprom_read_byte(&accelerationOffset));
        }

        // Returns number of graduations for acceleration and deceleration from min to max PWM and vice versa.
        // Function converts current user-friendly value 1-10 to this value.
        // Note that real value shouldn't be less than 80 when GRADUATIONS = 4320.
        static uint16_t getRealAcceleration()
        {
            int result = getAcceleration();
            result = abs(result - 11); // reverse
            result *= 10;
            result += 10;
            result *= DEGREE;
            result /= 3;
            return result; // value in range from 80 to 440 when GRADUATIONS = 4320
        }

        static bool checkAcceleration(unsigned char value)
        {
            return 1 <= value && value <= 10;
        }

        static unsigned char validateAcceleration(unsigned char value)
        {
            return checkAcceleration(value)
                ? value
                : 7; // use default
        }

        static void setAcceleration(unsigned char value)
        {
            eeprom_update_byte(&accelerationOffset, value);
        }

        static uint16_t getDelay()
        {
            return validateDelay(eeprom_read_word(&delayOffset));
        }

        static bool checkDelay(uint16_t value)
        {
            return value <= 5000 && value % 100 == 0;
        }

        static uint16_t validateDelay(uint16_t value)
        {
            return checkDelay(value)
                ? value
                : 0; // use default
        }

        static void setDelay(uint16_t value)
        {
            eeprom_update_word(&delayOffset, value);
        }

        static uint16_t getExposure()
        {
            return validateExposure(eeprom_read_word(&exposureOffset));
        }

        static bool checkExposure(uint16_t value)
        {
            return 100 <= value && value <= 500 && value % 100 == 0;
        }

        static uint16_t validateExposure(uint16_t value)
        {
            return checkExposure(value)
                ? value
                : 100; // use default
        }

        static void setExposure(uint16_t value)
        {
            eeprom_update_word(&exposureOffset, value);
        }

        static int16_t getVideoPWM()
        {
            return validateVideoPWM(eeprom_read_word(&videoPWMOffset));
        }

        static bool checkVideoPWM(int16_t value)
        {
            return 
                (-MAX_PWM <= value && value <= -MIN_PWM) ||
                ( MIN_PWM <= value && value <=  MAX_PWM);
        }

        static int16_t validateVideoPWM(int16_t value)
        {
            return checkVideoPWM(value)
                ? value
                : 80; // use default
        }

        static void setVideoPWM(int16_t value)
        {
            eeprom_update_word(&videoPWMOffset, value);
        }

        static float getNonstopFrequency()
        {
            return validateNonstopFrequency(eeprom_read_float(&nonstopFrequencyOffset));
        }

        static bool checkNonstopFrequency(float value)
        {
            return lowNonstopFrequency <= value && value <= highNonstopFrequency;
        }

        static float validateNonstopFrequency(float value)
        {
            return checkNonstopFrequency(value)
                ? value
                : lowNonstopFrequency; // use default
        }
        
        static unsigned char getRealNonstopPWM()
        {
            float frequency = getNonstopFrequency();
            unsigned char result = frequencyToPWM(frequency);
#ifdef DEBUG_MODE
            Serial.println("frequency from EEPROM = " + String(frequency) + "\tpwm = " + String(result));
#endif
            return PWMValidator::validate(result);
        }

        static unsigned char getLowRealNonstopPWM()
        {
            float frequency = lowNonstopFrequency;
            unsigned char result = PWMValidator::validate(frequencyToPWM(frequency));
#ifdef DEBUG_MODE
            if (lastLowSteps != getSteps())
            {
                Serial.println("low pwm = " + String(result));
                lastLowSteps = getSteps();
            }
#endif
            return result;
        }

        static unsigned char getHighRealNonstopPWM()
        {
            float frequency = highNonstopFrequency;
            unsigned char result = PWMValidator::validate(frequencyToPWM(frequency));
#ifdef DEBUG_MODE
            if (lastHighSteps != getSteps())
            {
                Serial.println("high pwm = " + String(result));
                lastHighSteps = getSteps();
            }
#endif
            return result;
        }

        static void setNonstopFrequency(float value)
        {
#ifdef DEBUG_MODE
            Serial.println("Store frequency in EEPROM = " + String(value));
#endif
            eeprom_update_float(&nonstopFrequencyOffset, value);
        }
        
        static void setNonstopPWM(unsigned char value)
        {
            float frequency = pwmToFrequency(value);
#ifdef DEBUG_MODE
            Serial.println("Store frequency = " + String(frequency));
#endif
            setNonstopFrequency(frequency);
        }

        static unsigned char getMenuIndex()
        {
            return validateMenuIndex(eeprom_read_byte(&menuIndexOffset));
        }

        static unsigned char validateMenuIndex(unsigned char value)
        {
            return value <= MenuItemsDef::topItemsLength
                ? value
                : 0; // use default
        }

        static void setMenuIndex(unsigned char value)
        {
            eeprom_update_byte(&menuIndexOffset, value);
        }

    private:
        static float getTimeOfTurn(unsigned char pwm)
        {
            static const unsigned char buff[] =
            {
                73, 69, 63, 60, 56, 52, 49, 46, 44, 41, 39, 38, 36, 35, 34, 32, 31, 30, 29, 28, 27, 26,
                25, 25, 24, 24, 24, 24, 23, 23, 22, 22, 22, 21, 21, 20, 20, 20, 19, 19, 19, 19, 18, 18,
                18, 18, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15
            };
            
            int index = pwm - MIN_PWM;
            if (index < 63)
                return buff[index];
            else if (index < 72)
                return 14;
            else if (index < 81)
                return 13;
            else if (index < 90)
                return 12;
            else if (index < 101)
                return 11;
            else if (index < 112)
                return 10;
            else if (index < 124)
                return 9;
            else if (index < 138)
                return 8;
            else
                return 7;
        }

        static float getPWMOfTurn(float time)
        {
            int pwm = MAX_PWM;
            while (getTimeOfTurn(pwm) < time && pwm > MIN_PWM)
            {
                pwm--;
            }

            return pwm;
        }

        static float pwmToFrequency(unsigned char pwm)
        {
            float steps = getSteps();
            return steps / getTimeOfTurn(pwm);
        }

        static unsigned char frequencyToPWM(float frequency)
        {
            float steps = getSteps();
            float time = steps / frequency;
            float result = getPWMOfTurn(time);
            return result + 0.5; // rounded
        }
};

class Menu
{
    public:
        unsigned char current;

        void setItems(const MenuItem* items, unsigned char length)
        {
            _items = items;
            _length = length;
            _mode = MenuItems;
            current = 0;
        }

        void setItems(const uint16_t* items, unsigned char length)
        {
            _array = items;
            _length = length;
            _mode = Array;
            current = 0;
        }

        void setItems(unsigned char upperBound, unsigned char multiplier, unsigned char offset)
        {
            _length = upperBound;
            _offset = offset;
            _multiplier = multiplier;
            _mode = Range;
            current = 0;
        }

        void display()
        {
            const char* top;
            unsigned char index = (unsigned char) current;
            switch (_mode)
            {
                case MenuItems:
                    top = _items[index].top;
                    if (top != NULL && top[0] == '%')
                        top = formatSteps(top);
                    printTop(top);
                    printBottom(_items[index].bottom);
                    break;

                case Array:
                    printTop(String(_array[index]).c_str());
                    printBottom("");
                    break;
                
                case Range:
                    printTop(String((current + _offset) * _multiplier).c_str());
                    printBottom("");
                    break;
            }
        }

        const char* formatSteps(const char* top)
        {
            top += 1; // remove % sign
            static char strBuf[20];
            sprintf(strBuf, "%s (%d)", top, Settings::getSteps());
            return strBuf;
        }
        
        void display(const char* top, const char* bottom)
        {
            printTop(top);
            printBottom(bottom);
        }

        void next()
        {
            if (current < _length - 1)
                current++;
        }

        void prev()
        {
            if (current > 0)
                current--;
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
        const uint16_t* _array;
        unsigned char _length;
        unsigned char _offset;
        unsigned char _multiplier;
        
        char _recentTop[17];
        char _recentBottom[17];

        void printTop(const char* text)
        {
            if (strcmp(_recentTop, text) != 0)
            {
                strcpy(_recentTop, text);
                lcd.setCursor(0, 0);
                lcd.print(text);

                // Clear rest of space
                for (int i = strlen(text); i <= 16; i++)
                {
                    lcd.print(' ');
                }
            }
        }

        void printBottom(const char* text)
        {
            if (strcmp(_recentBottom, text) != 0)
            {
                strcpy(_recentBottom, text);
                lcd.setCursor(0, 1);
                lcd.print(text);

                // Clear rest of space
                for (int i = strlen(text); i <= 16; i++)
                {
                    lcd.print(' ');
                }
            }
        }
};

class SettingEditor
{
    public:
        unsigned char settingNum;
        
        SettingEditor(Menu* menu)
        {
            _menu = menu;
        }
        
        void edit(unsigned char num)
        {
            settingNum = num;
            fillMenu();
            _menu->display();
        }

        void update()
        {
            unsigned char index = _menu->current;
            switch (settingNum)
            {
                case 0: // Steps
                    Settings::setSteps(steps[index]);
                    break;
                case 1: // Acceleration
                    Settings::setAcceleration(index + 1);
                    break;
                case 2: // Delay
                    Settings::setDelay(index * 100);
                    break;
                case 3: // Exposure
                    Settings::setExposure((index + 1) * 100);
                    break;
            }
        }
    
    private:
        Menu* _menu;

        void fillMenu()
        {
            switch (settingNum)
            {
                case 0: // Steps
                    _menu->setItems(steps, stepsLength);
                    _menu->current = FindInSteps(Settings::getSteps());
                    break;
                case 1: // Acceleration
                    _menu->setItems(10, 1, 1);
                    _menu->current = Settings::getAcceleration() - 1;
                    break;
                case 2: // Delay
                    _menu->setItems(51, 100, 0);
                    _menu->current = Settings::getDelay() / 100;
                    break;
                case 3: // Exposure
                    _menu->setItems(5, 100, 1);
                    _menu->current = Settings::getExposure() / 100 - 1;
                    break;
            }
        }
};

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
            _minPWM = MIN_PWM;
            _maxPWM = maxPWM;
            _cumulativePos -= encoder.readAndReset();
            _acceleration = Settings::getAcceleration();
            _realAcceleration = Settings::getRealAcceleration();
            _startTimer2 = _startTimer = millis();
            _started = false;
            _state = Move;

            analogWrite(_forward? MOTOR1 : MOTOR2, _currentPWM);
        }

        void run(int pwm)
        {
            if (!isStopped())
                return;

            _minPWM = MIN_PWM;
            _maxPWM = abs(pwm);
            _forward = pwm > 0;
            _currentPWM = MIN_PWM;
            _cumulativePos -= encoder.readAndReset();
            _acceleration = Settings::getAcceleration();
            _realAcceleration = Settings::getRealAcceleration();
            _startTimer2 = _startTimer = millis();
            _started = false;
            _state = RunAcc;

            analogWrite(_forward? MOTOR1 : MOTOR2, _currentPWM);
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
                float decelerationLength = _realAcceleration;
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
            if (!canChangePWM(delta))
                return;
                
            _maxPWM += delta;
            _maxPWM = PWMValidator::validate(_maxPWM);
            _currentPWM = _maxPWM;
            analogWrite(_forward? MOTOR1 : MOTOR2, _currentPWM);
        }

        bool canChangePWM(int delta)
        {
            // Changing is avalable only when state is Move or Run
            if (_state != Move && _state != Run)
                return false;

            // Changing is not awailable at the time of acceleration/deceleration
            if (!isUniformMotion())
                return false;

            int oldMaxPWM = _maxPWM;
            int newMaxPWM = PWMValidator::validate(_maxPWM + delta);

            // Return true if old _maxPWM will not be equal to the new one
            return newMaxPWM != oldMaxPWM;
        }

        inline bool isUniformMotion()
        {
            return (_state == Move || _state == Run) && _currentPWM == _maxPWM;
        }

        // Returns current position in graduations. Can be negative
        int32_t getCurrentPos()
        {
            int32_t pos = encoder.read();
            // Invert pos
            // clockwise movement is positive, counterclockwise movement is negative
            return -pos;
        }

        // Returns latest position obtained by Mover class.
        // This function is "easy" and does not use encoder and therefore critical section. 
        inline int32_t getLatestPos()
        {
            return _currentPos;
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
        int _minPWM;
        int _maxPWM;
        int32_t _currentPos;
        int32_t _lastPos;
        int32_t _cumulativePos;
        int _currentPWM;
        unsigned long _timer;
        unsigned long _startTimer;
        unsigned long _startTimer2;
        bool _started;
        unsigned char _timePartCount;
        int _acceleration;
        int _realAcceleration;
        
        bool start()
        {
            if (_started)
            {
                // Everything is OK, table started
                return true;
            }
           
            if (_currentPos != 0)
            {
                // Everything is OK, table started
                _started = true;
                return true;
            }

            // 100 ms is reasonable amount of time to start moving
            if (millis() - _startTimer2 >= 100)
            {
                int limit = calcHighLimitOfMinPWM();
#ifdef DEBUG_MODE
                Serial.println("High limit of _minPWM: " + String(limit));
#endif
                if (_minPWM >= limit)
                {
                    // Unable to increase _minPWM anymore
                    // If correction, wait a second, and if table is still not moving, stop it
                    if (_state == Correction && millis() - _startTimer >= 1000)
                    {
#ifdef DEBUG_MODE
                        Serial.println("Unable to start, stopping...");
#endif
                        stop();
                    }
                    return false;
                }

                _startTimer2 = millis();
                _minPWM += 1;
                if (_minPWM > limit)
                    _minPWM = limit;
                _currentPWM = _minPWM;
                analogWrite(_forward? MOTOR1 : MOTOR2, _currentPWM);
#ifdef DEBUG_MODE
                Serial.println("Increase PWM. New value: " + String(_minPWM));
#endif
            }

            return false;
        }

        int calcHighLimitOfMinPWM()
        {
            int highestLimit = min(100, _maxPWM);
            
            switch (_state)
            {
                case RunAcc:
                    // In this case we can return a large enough value
                    return highestLimit;

                case Move:
                case Correction:
                    // Calculate maximum possible value
                    {
                        float halfPoint = _graduations / 2.0;
                        float accelerationLength = _realAcceleration;
                        float value = MIN_PWM + halfPoint * (MAX_PWM - MIN_PWM) / accelerationLength;
                        int result = validatePWM(value);
                        if (result > highestLimit)
                            result = highestLimit;

                        return result;
                    }

                default:
                    return MIN_PWM;
            }
        }
        
        void tickMove()
        {
            _currentPos = getCurrentPos();
            if (!start())
                return;

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
            float accelerationLength = _realAcceleration;
            float currentPWM = _minPWM + x * (MAX_PWM - _minPWM) / accelerationLength;
            _currentPWM = validatePWM(currentPWM);
        }

        void decelerate()
        {
            // Use linear function to decelerate
            float x = _graduations - _currentPos;
            if (!_forward)
                x = -x;
            x -= getFinalDistance();
            float decelerationLength = _realAcceleration;
            float currentPWM = _minPWM + x * (MAX_PWM - _minPWM) / decelerationLength;
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
#ifdef DEBUG_MODE
                Serial.println("Error = " + String(error) + ", correction...");
#endif
                stop();
                move(error, MIN_PWM);
                _state = Correction;
            }
        }

        // We should pass end of step with minimal PWM
        // This function returns length of this final distance
        int getFinalDistance()
        {
            switch (_acceleration)
            {
                case 10:
                    return _graduations >= 10 * DEGREE ? 18 : 14;
                case 9:
                    return _graduations >= 10 * DEGREE ? 10 : 8;
                case 8:
                    return _graduations >= 10 * DEGREE ? 6 : 4;
                case 7:
                    return 2;
                default:
                    return 0;
            }
        }
        
        void tickRun()
        {
            _currentPos = getCurrentPos();
            if (!start())
                return;

            switch (_state)
            {
                case RunAcc:
                    accelerate();
                    if (_currentPWM >= _maxPWM)
                    {
                        _currentPWM = _maxPWM;
                        _state = Run;
                    }
                    analogWrite(_forward? MOTOR1 : MOTOR2, _currentPWM);
                    break;
                    
                case RunDec:
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
            else if (pwm < _minPWM)
                pwm = _minPWM;

            return pwm;
        }
};
Mover mover;

class Selector
{
    public:
        Menu menu;
        bool hold = false;

        Selector() : _editor(&menu)
        {
            menu.setItems(MenuItemsDef::topItems, MenuItemsDef::topItemsLength);
            menu.current = Settings::getMenuIndex();
        }

        void tick()
        {
            if (hold)
            {
                MenuItemsDef::handlers[menu.current]();
                return;
            }
            
            menu.display();
            
            if (enc.press())
                press();
            else if (enc.turn())
            {
                if (enc.left())
                    menu.prev();
                else if (enc.right())
                    menu.next();
            }
        }
        
    private:
        unsigned char _level = 0;
        SettingEditor _editor;

        void press()
        {
            if (_level == 0) // top menu
            {
                if (menu.current < MenuItemsDef::topItemsLength - 1)
                {
                    Settings::setMenuIndex(menu.current);
                    hold = true;
                }
                else // settings
                {
                    updateSettings();
                    menu.setItems(MenuItemsDef::settingsItems, MenuItemsDef::settingsItemsLength);
                    menu.display();
                    _level = 1;
                }
            }
            else if (_level == 1) // settings menu
            {
                _editor.edit(menu.current);
                _level = 2;
            }
            else
            {
                // Update setting
                _editor.update();
                menu.setItems(MenuItemsDef::topItems, MenuItemsDef::topItemsLength);
                menu.current = MenuItemsDef::topItemsLength - 1; // index of Settings
                menu.display();
                _level = 0;
            }
        }

        void updateSettings()
        {
            strcpy(MenuItemsDef::settingsItems[0].bottom, String(Settings::getSteps()).c_str());
            strcpy(MenuItemsDef::settingsItems[1].bottom, String(Settings::getAcceleration()).c_str());
            strcpy(MenuItemsDef::settingsItems[2].bottom, String(Settings::getDelay()).c_str());
            strcpy(MenuItemsDef::settingsItems[3].bottom, String(Settings::getExposure()).c_str());;
        }
};
Selector selector;

int16_t stepNumber = 0;
unsigned long timer = 0;
bool isRunning = false;
void Runner::runAutomatic()
{
    const char* mode = "Auto...";
    const char* stepName = "photo";
    static State currentState;
    static int32_t lastGraduations;
#ifdef DEBUG_MODE
    static int total = 0;
#endif
    
    if (enc.press())
    {
        finalize();
        return;
    }

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
#ifdef DEBUG_MODE
        total = 0;
#endif
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
#ifdef DEBUG_MODE
        total += absolutePos;
        Serial.println("stepGraduations = " + String(stepGraduations) + "  absolutePos = " +
            String(absolutePos) + "  error = " + String(error));
#endif
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
#ifdef DEBUG_MODE
            int32_t absolutePos = mover.getAbsolutePos();
            total += absolutePos;
            Serial.println("Total = " + String(total));
#endif
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

void Runner::runManual()
{
    const char* mode = "Manual...";
    const char* stepName = "step";
    static State currentState;
    static bool needToMove;
    static int32_t lastGraduations;
#ifdef DEBUG_MODE
    static int total = 0;
#endif
    
    if (enc.press())
    {
        finalize();
        return;
    }

    int16_t stepCount = Settings::getSteps();
    int stepGraduations = GRADUATIONS / stepCount;
    if (!isRunning)
    {
        // Starting
        stepNumber = 1;
        needToMove = false;
        display(mode, stepName);
        digitalWrite(CAMERA, CAMERA_HIGH); // prepare camera
        isRunning = true;
        lastGraduations = 0;
        mover.resetAbsolutePos();
        timer = millis();
        currentState = Exposure;
#ifdef DEBUG_MODE
        total = 0;
#endif
        return;
    }

    if (currentState == Exposure && millis() - timer >= Settings::getExposure())
    {
        digitalWrite(SHUTTER, CAMERA_LOW); // release shutter
        currentState = Waiting;
        if (needToMove)
        {
            needToMove = false;
            currentState = Move;
            int32_t absolutePos = mover.getAbsolutePos();
            int error = lastGraduations - absolutePos;
#ifdef DEBUG_MODE
            total += absolutePos;
            Serial.println("stepGraduations = " + String(stepGraduations) + "  absolutePos = " +
                String(absolutePos) + "  error = " + String(error));
#endif
            mover.resetAbsolutePos();
            lastGraduations = stepGraduations + error;
            mover.move(lastGraduations);
        }

        return;
    }

    if (photoButton.press() && currentState == Waiting)
    {
        digitalWrite(SHUTTER, CAMERA_HIGH); // make photo
        timer = millis();
        currentState = Exposure;
        return;
    }

    if (nextButton.press() && currentState == Waiting)
    {
        digitalWrite(SHUTTER, CAMERA_HIGH); // make photo
        timer = millis();
        currentState = Exposure;
        needToMove = true;
        return;
    }

    if (currentState == Move && mover.isStopped())
    {
        if (stepNumber < stepCount)
        {
            currentState = Waiting;
            stepNumber++;
            display(mode, stepName);
        }
        else
        {
#ifdef DEBUG_MODE
            int32_t absolutePos = mover.getAbsolutePos();
            total += absolutePos;
            Serial.println("Total = " + String(total));
#endif
            finalize();
        }
    }
}

void Runner::runNonstop()
{
    const char* mode = "Nonstop...";
    const char* stepName = "photo";
    static State currentState;
    static int nextSnapshotPos;
    static bool needToStoreNewPWM;
#ifdef DEBUG_MODE
    static int total = 0;
#endif

    if (enc.press())
    {
        finalize();
        return;
    }
    else if (enc.turn() && !mover.isStopped())
    {
        if (enc.left())
        {
            if (mover.getCurrentPWM() > Settings::getLowRealNonstopPWM())
            {
                int d = delta;
                if (mover.getCurrentPWM() - d < Settings::getLowRealNonstopPWM())
                    d = mover.getCurrentPWM() - Settings::getLowRealNonstopPWM();

                mover.changePWM(-d);
                needToStoreNewPWM = true;
            }
        }
        else if (enc.right())
        {
            if (mover.getCurrentPWM() < Settings::getHighRealNonstopPWM())
            {
                int d = delta;
                if (mover.getCurrentPWM() + d > Settings::getHighRealNonstopPWM())
                    d = Settings::getHighRealNonstopPWM() - mover.getCurrentPWM();

                mover.changePWM(d);
                needToStoreNewPWM = true;
            }
        }
    }

    int16_t stepCount = Settings::getSteps();
    int stepGraduations = GRADUATIONS / stepCount;
    if (!isRunning)
    {
        // Starting
        stepNumber = 0;
        digitalWrite(CAMERA, CAMERA_HIGH); // prepare camera
        isRunning = true;
        needToStoreNewPWM = false;
        timer = millis();
        currentState = Beginning;
#ifdef DEBUG_MODE
        total = 0;
        mover.resetAbsolutePos();
#endif
        return;
    }

    if (currentState == Beginning && millis() - timer >= Settings::getExposure())
    {
        stepNumber++;
        display(mode, stepName);
        digitalWrite(SHUTTER, CAMERA_HIGH); // make first photo
        nextSnapshotPos = stepGraduations;
        timer = millis();
        currentState = Exposure;
        mover.move(GRADUATIONS, Settings::getRealNonstopPWM());
        return;
    }

    const int releaseShutterDelay = 50;
    if (currentState == Exposure && millis() - timer >= releaseShutterDelay)
    {
        digitalWrite(SHUTTER, CAMERA_LOW); // release shutter
        currentState = Move;
    }

    if (currentState == Move && mover.getCurrentPos() >= nextSnapshotPos)
    {
        nextSnapshotPos += stepGraduations;
        stepNumber++;
        if (stepNumber > stepCount)
        {
            currentState = Waiting;
        }
        else
        {
            display(mode, stepName);
            digitalWrite(SHUTTER, CAMERA_HIGH); // make photo
            timer = millis();
            currentState = Exposure;
        }
    }

    if (isRunning && currentState != Beginning && mover.isStopped())
    {
        if (needToStoreNewPWM)
        {
#ifdef DEBUG_MODE
            Serial.println("Store new pwm = " + String(mover.getMaxPWM()));
#endif
            Settings::setNonstopPWM(mover.getMaxPWM());
            needToStoreNewPWM = false;
        }

#ifdef DEBUG_MODE
        int32_t absolutePos = mover.getAbsolutePos();
        total += absolutePos;
        Serial.println("Total = " + String(total));
#endif
        finalize();
    }
}

void Runner::runVideo()
{
    if (!isRunning)
    {
        selector.menu.display("Video...", "");
        mover.run(Settings::getVideoPWM());
        isRunning = true;
        return;
    }

    if (mover.isStopped())
    {
        finalize();
        return;
    }
    
    char direction = mover.isForward() ? 1 : -1;
    if (enc.press())
    {
        if (mover.getState() == mover.State::Run)
        {
            Settings::setVideoPWM(mover.getMaxPWM() * direction);
            mover.softStop();
        }
        else
        {
            mover.stop();
        }
    }
    else if (enc.turn() && mover.getState() == mover.State::Run)
    {
        if (enc.left())
        {
            if (direction > 0 && mover.getCurrentPWM() <= MIN_PWM)
            {
                // Change direction
                mover.stop();
                mover.run(-MIN_PWM);
                return;
            }

            mover.changePWM(-delta * direction);
        }
        else if (enc.right())
        {
            if (direction < 0 && mover.getCurrentPWM() <= MIN_PWM)
            {
                // Change direction
                mover.stop();
                mover.run(MIN_PWM);
                return;
            }

            mover.changePWM(delta * direction);
        }
    }
}

void Runner::runRotate()
{
    static State currentState;

    if (!isRunning)
    {
        selector.menu.display("Rotation...", "<-  90\337  ->");
#ifdef DEBUG_MODE
        mover.resetAbsolutePos();
#endif
        currentState = Waiting;
        isRunning = true;
    }
    
    if (enc.press())
    {
        finalize();
        return;
    }

    if (!mover.isStopped())
    {
        return;
    }
    
    if (enc.turn() && currentState == Waiting)
    {
        if (enc.left())
        {
            currentState = Move;
            mover.move(-GRADUATIONS / 4);
        }
        else if (enc.right())
        {
            currentState = Move;
            mover.move(GRADUATIONS / 4);
        }

        return;
    }
    
    if (currentState == Move)
    {
#ifdef DEBUG_MODE
        int32_t absolutePos = mover.getAbsolutePos();
        Serial.println("absolutePos = " + String(absolutePos));
#endif
        currentState = Waiting;
    }
}

void Runner::finalize()
{
    isRunning = false;
    mover.stop();
    stepNumber = 0;
    digitalWrite(SHUTTER, CAMERA_LOW); // release shutter
    digitalWrite(CAMERA, CAMERA_LOW); // release camera
    selector.hold = false; // release selector
    enc.resetState(); // reset encoder
}

void Runner::display(const char* top, const char* stepName)
{
    char strBuf[20];
    sprintf(strBuf, "%s %d (%d)", stepName, stepNumber, Settings::getSteps());
    selector.menu.display(top, strBuf);
}

void setup()
{
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

    enc.setButtonLevel(HIGH);
    photoButton.setButtonLevel(HIGH);
    nextButton.setButtonLevel(HIGH);

    SetPinFrequencySafe(MOTOR1, 15000);
    SetPinFrequencySafe(MOTOR2, 15000);

#ifdef DEBUG_MODE
    Serial.begin(9600);
#endif
}

void loop()
{
    enc.tick();
    photoButton.tick();
    nextButton.tick();
    selector.tick();
    mover.tick();
}
