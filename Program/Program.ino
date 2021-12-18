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

// Uncomment below to enable debug output.
//#define DEBUG_MODE

Encoder encoder(MOTOR_ENC1, MOTOR_ENC2);

LiquidCrystal_I2C lcd(0x27, 16, 2);
EncButton<EB_TICK, 12, 13, 11> enc; // pins 11, 12, 13
EncButton<EB_TICK, 7> photoButton;  // pin 7
EncButton<EB_TICK, 4> nextButton;   // pin 4

uint16_t EEMEM stepsOffset;
unsigned char EEMEM accelerationOffset;
uint16_t EEMEM delayOffset;
uint16_t EEMEM exposureOffset;
uint16_t EEMEM videoPWMOffset;
float EEMEM nonstopFrequencyOffset;
unsigned char EEMEM menuIndexOffset;

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

    static const callback_t handlers[topItemsLength - 1];
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
        static int validate(int pwm)
        {
            if (pwm > MAX_PWM)
                pwm = MAX_PWM;
            else if (pwm < MIN_PWM)
                pwm = MIN_PWM;

            return pwm;
        }
};

#ifdef DEBUG_MODE
int lastLowSteps = 0;
int lastHighSteps = 0;
#endif
class Settings
{
    public:
        static constexpr float lowNonstopFrequency = 0.5;
        static constexpr float highNonstopFrequency = 3.0;
        
        static int16_t getSteps()
        {
            return validateSteps(eeprom_read_word(&stepsOffset));
        }

        static int16_t validateSteps(int16_t value)
        {
            return FindInSteps(value) >= 0
                ? value
                : 24; // use default
        }

        static void setSteps(int16_t value)
        {
            eeprom_update_word(&stepsOffset, value);
        }

        static char getAcceleration()
        {
            return validateAcceleration(eeprom_read_byte(&accelerationOffset));
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

        static char validateAcceleration(char value)
        {
            return 1 <= value && value <= 10
                ? value
                : 7; // use default
        }

        static void setAcceleration(char value)
        {
            eeprom_update_byte(&accelerationOffset, value);
        }

        static uint16_t getDelay()
        {
            return validateDelay(eeprom_read_word(&delayOffset));
        }

        static uint16_t validateDelay(int16_t value)
        {
            return 0 <= value && value <= 5000 && value % 100 == 0
                ? value
                : 100; // use default
        }

        static void setDelay(int16_t value)
        {
            eeprom_update_word(&delayOffset, value);
        }

        static uint16_t getExposure()
        {
            return validateExposure(eeprom_read_word(&exposureOffset));
        }

        static uint16_t validateExposure(int16_t value)
        {
            return 100 <= value && value <= 500 && value % 100 == 0
                ? value
                : 100; // use default
        }

        static void setExposure(int16_t value)
        {
            eeprom_update_word(&exposureOffset, value);
        }

        static int16_t getVideoPWM()
        {
            return validateVideoPWM(eeprom_read_word(&videoPWMOffset));
        }

        static int16_t validateVideoPWM(int16_t value)
        {
            return 
                (-MAX_PWM <= value && value <= -MIN_PWM) ||
                ( MIN_PWM <= value && value <=  MAX_PWM)
                    ? value
                    : 100; // use default
        }

        static void setVideoPWM(int16_t value)
        {
            eeprom_update_word(&videoPWMOffset, value);
        }

        static float getNonstopFrequency()
        {
            return validateNonstopFrequency(eeprom_read_float(&nonstopFrequencyOffset));
        }

        static float validateNonstopFrequency(float value)
        {
            return lowNonstopFrequency <= value && value <= highNonstopFrequency
                ? value
                : lowNonstopFrequency; // use default
        }
        
        static int16_t getRealNonstopPWM()
        {
            float frequency = getNonstopFrequency();
            int16_t result = frequencyToPWM(frequency);
#ifdef DEBUG_MODE
            Serial.println("frequency from EEPROM = " + String(frequency) + "\tpwm = " + String(result));
#endif
            return PWMValidator::validate(result);
        }

        static int16_t getLowRealNonstopPWM()
        {
            float frequency = lowNonstopFrequency;
            int16_t result = PWMValidator::validate(frequencyToPWM(frequency));
#ifdef DEBUG_MODE
            if (lastLowSteps != getSteps())
            {
                Serial.println("low pwm = " + String(result));
                lastLowSteps = getSteps();
            }
#endif
            return result;
        }

        static int16_t getHighRealNonstopPWM()
        {
            float frequency = highNonstopFrequency;
            int16_t result = PWMValidator::validate(frequencyToPWM(frequency));
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
        
        static void setNonstopPWM(int16_t value)
        {
            float frequency = pwmToFrequency(value);
#ifdef DEBUG_MODE
            Serial.println("Store frequency = " + String(frequency));
#endif
            setNonstopFrequency(frequency);
        }

        static char getMenuIndex()
        {
            return validateMenuIndex(eeprom_read_byte(&menuIndexOffset));
        }

        static char validateMenuIndex(char value)
        {
            return 0 <= value && value <= MenuItemsDef::topItemsLength
                ? value
                : 0; // use default
        }

        static void setMenuIndex(char value)
        {
            eeprom_update_byte(&menuIndexOffset, value);
        }

    private:
        static float getTimeOfTurn(int16_t pwm)
        {
            static const char buff[] = { 97, 89, 81, 75, 69, 64, 59, 56, 52, 49, 47, 44, 42, 40, 38, 36, 35, 34, 32, 31, 30, 29, 28, 27, 26, 26, 25, 24, 24, 23, 22, 23, 23, 22, 22 };
            int index = pwm - MIN_PWM;
            if (index < 35)
                return buff[index];
            else if (index < 38)
                return 21;
            else if (index < 41)
                return 20;
            else if (index < 44)
                return 19;
            else if (index < 48)
                return 18;
            else if (index < 53)
                return 17;
            else if (index < 58)
                return 16;
            else if (index < 65)
                return 15;
            else if (index < 73)
                return 14;
            else if (index < 83)
                return 13;
            else if (index < 93)
                return 12;
            else if (index < 103)
                return 11;
            else if (index < 115)
                return 10;
            else if (index < 126)
                return 9;
            else if (index < 140)
                return 8;
            else if (index < 163)
                return 7;
            else
                return 6;
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

        static float pwmToFrequency(int16_t pwm)
        {
            float steps = getSteps();
            return steps / getTimeOfTurn(pwm);
        }

        static int16_t frequencyToPWM(float frequency)
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

class SettingEditor
{
    public:
        char settingNum;
        
        SettingEditor(Menu* menu)
        {
            _menu = menu;
        }
        
        void edit(char num)
        {
            settingNum = num;
            fillMenu();
            _menu->display();
        }

        void update()
        {
            unsigned char index = (unsigned char) _menu->current;
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

        void move(int graduations, int maxPWM = MAX_PWM)
        {
            if (!isStopped())
                return;

            _graduations = abs(graduations);
            _forward = graduations > 0;
            _currentPWM = MIN_PWM;
            _maxPWM = maxPWM;
            _accumAbsolutePos += encoder.readAndReset();
            _state = Move;
        }

        void run(int pwm)
        {
            if (!isStopped())
                return;

            _maxPWM = abs(pwm);
            _forward = pwm > 0;
            _currentPWM = MIN_PWM;
            _accumAbsolutePos += encoder.readAndReset();
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

        int getCurrentPos()
        {
            int32_t pos = encoder.read();
            return abs(pos);
        }

        // Returns graduation count passed from starting point. Can be negative
        int32_t getAbsolutePos()
        {
            return _accumAbsolutePos + encoder.read();
        }

        void resetAbsolutePos()
        {
            encoder.readAndReset();
            _accumAbsolutePos = 0;
        }

    private:
        State _state = Stop;
        int _graduations;
        bool _forward;
        int _maxPWM = MAX_PWM;
        int _currentPos;
        int _lastPos;
        int32_t _accumAbsolutePos;
        int _currentPWM;
        unsigned long _timer;
        unsigned char _timePartCount;
        
        void tickMove()
        {
            _currentPos = getCurrentPos();
            if (_currentPos >= _graduations)
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
                if (_currentPos < _graduations / 2)
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
            float accelerationLength = Settings::getRealAcceleration();
            float currentPWM = MIN_PWM + x * (MAX_PWM - MIN_PWM) / accelerationLength;
            _currentPWM = validatePWM(currentPWM);
        }

        void decelerate()
        {
            // Use linear function to decelerate
            float x = _graduations - _currentPos - getFinalDistance();
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
                if (!isForward())
                    error = -error;
#ifdef DEBUG_MODE
                Serial.println("Error = " + String(error) + ", correction...");
#endif
                stop();
                move(error, MIN_PWM);
                _state = Correction;
            }
        }

        // End of the step we should go with MIN_PWM
        // This function returns length of this final distance
        int getFinalDistance()
        {
            switch (Settings::getAcceleration())
            {
                case 10:
                    return 12;
                case 9:
                    return 8;
                case 8:
                    return 4;
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
                unsigned char index = (unsigned char) menu.current;
                MenuItemsDef::handlers[index]();
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
        char _level = 0;
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
            MenuItemsDef::settingsItems[0].bottom = String(Settings::getSteps(), DEC);
            MenuItemsDef::settingsItems[1].bottom = String(Settings::getAcceleration(), DEC);
            MenuItemsDef::settingsItems[2].bottom = String(Settings::getDelay(), DEC);
            MenuItemsDef::settingsItems[3].bottom = String(Settings::getExposure(), DEC);
        }
};
Selector selector;

int16_t stepNumber = 0;
unsigned long timer = 0;
bool isRunning = false;
void Runner::runAutomatic()
{
    const String mode = "Auto...";
    const String stepName = "photo";
    static State currentState;
    static int16_t lastGraduations;
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
        int lastError = 0;
        if (lastGraduations > 0)
        {
            int32_t absolutePos = mover.getAbsolutePos();
            lastError = lastGraduations - abs(absolutePos);
#ifdef DEBUG_MODE
            total += absolutePos;
            Serial.println("stepGraduations = " + String(stepGraduations) + "  absolutePos = " +
                String(absolutePos) + "  lastError = " + String(lastError));
#endif
        }
        mover.resetAbsolutePos();
        lastGraduations = stepGraduations + lastError;
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
    const String mode = "Manual...";
    const String stepName = "step";
    static State currentState;
    static int16_t lastGraduations;
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
        display(mode, stepName);
        digitalWrite(CAMERA, CAMERA_HIGH); // prepare camera
        isRunning = true;
        lastGraduations = 0;
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
        timer = millis();
        currentState = Move;
        int lastError = 0;
        if (lastGraduations > 0)
        {
            int32_t absolutePos = mover.getAbsolutePos();
            lastError = lastGraduations - abs(absolutePos);
#ifdef DEBUG_MODE
            total += absolutePos;
            Serial.println("stepGraduations = " + String(stepGraduations) + "  absolutePos = " +
                String(absolutePos) + "  lastError = " + String(lastError));
#endif
        }
        mover.resetAbsolutePos();
        lastGraduations = stepGraduations + lastError;
        mover.move(lastGraduations);
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
    const String mode = "Nonstop...";
    const String stepName = "photo";
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
        char direction = mover.isForward() ? 1 : -1;
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
        selector.menu.display("Rotation...", "<-  ->");
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

void Runner::display(String top, String stepName)
{
    char strBuf[20];
    sprintf(strBuf, "%s %d (%d)", stepName.c_str(), stepNumber, Settings::getSteps());
    selector.menu.display(top, strBuf);
}

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
