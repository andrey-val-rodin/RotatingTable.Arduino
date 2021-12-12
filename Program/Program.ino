#include <Encoder.h>
#include <EncButton.h>
#include <LiquidCrystal_I2C.h>
#include <PWM.h>
#include <avr/eeprom.h>

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

int16_t EEMEM stepsOffset;
char EEMEM accelerationOffset;
int16_t EEMEM delayOffset;
int16_t EEMEM exposureOffset;
int16_t EEMEM videoSpeedOffset;
float EEMEM nonstopFrequencyOffset;
char EEMEM menuIndexOffset;

const char stepsLength = 22;
const int16_t steps[stepsLength] =
    { 2, 4, 5, 6, 8, 9, 10, 12, 15, 18, 20, 24, 30, 36, 40, 45, 60, 72, 90, 120, 180, 360 };
char FindInSteps(int16_t numberOfSteps)
{
    for (char i = 0; i < stepsLength; i++)
    {
        if (steps[i] == numberOfSteps)
            return i;
    }

    return -1;
}

class Runner
{
    public:
        static const int delta = 5; // value to increment/decrement speed
        
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
            Move,
            Correction
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

    MenuItem topItems[topItemsLength] = {
        {"%Auto",       ""},
        {"%Manual",     ""},
        {"%Nonstop",    ""},
        {"Video",       ""},
        {"Rotate 90",   ""},
        {"Settings",    ""}};

    callback_t handlers[topItemsLength - 1] = {
        Runner::runAutomatic,
        Runner::runManual,
        Runner::runNonstop,
        Runner::runVideo,
        Runner::runRotate
    };

    static const char settingsItemsLength = 4;
    MenuItem settingsItems[settingsItemsLength] = {
        {"Steps",        ""},
        {"Acceleration", ""},
        {"Delay",        ""},
        {"Exposure",     ""}};
};

class SpeedValidator
{
    public:
        static int validate(int speed)
        {
            if (speed > MAX_PWM)
                speed = MAX_PWM;
            else if (speed < MIN_PWM)
                speed = MIN_PWM;

            return speed;
        }
};

#ifdef DEBUG_MODE
int lastLowSteps = 0;
int lastHighSteps = 0;
#endif
class Settings
{
    public:
        static const float lowNonstopFrequency = 0.5;
        static const float highNonstopFrequency = 3.0;
        
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

        static int16_t getDelay()
        {
            return validateDelay(eeprom_read_word(&delayOffset));
        }

        static int16_t validateDelay(int16_t value)
        {
            return 0 <= value && value <= 5000 && value % 100 == 0
                ? value
                : 100; // use default
        }

        static void setDelay(int16_t value)
        {
            eeprom_update_word(&delayOffset, value);
        }

        static int16_t getExposure()
        {
            return validateExposure(eeprom_read_word(&exposureOffset));
        }

        static int16_t validateExposure(int16_t value)
        {
            return 100 <= value && value <= 500 && value % 100 == 0
                ? value
                : 100; // use default
        }

        static void setExposure(int16_t value)
        {
            eeprom_update_word(&exposureOffset, value);
        }

        static int16_t getVideoSpeed()
        {
            return validateVideoSpeed(eeprom_read_word(&videoSpeedOffset));
        }

        static int16_t validateVideoSpeed(int16_t value)
        {
            return 
                (-MAX_PWM <= value && value <= -MIN_PWM) ||
                ( MIN_PWM <= value && value <=  MAX_PWM)
                    ? value
                    : 100; // use default
        }

        static void setVideoSpeed(int16_t value)
        {
            eeprom_update_word(&videoSpeedOffset, value);
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
        
        static int16_t getRealNonstopSpeed()
        {
            float frequency = getNonstopFrequency();
            int16_t result = frequencyToSpeed(frequency);
#ifdef DEBUG_MODE
            Serial.println("frequency from EEPROM = " + String(frequency) + "\tspeed = " + String(result));
#endif
            return SpeedValidator::validate(result);
        }

        static int16_t getLowRealNonstopSpeed()
        {
            float frequency = lowNonstopFrequency;
            int16_t result = SpeedValidator::validate(frequencyToSpeed(frequency));
#ifdef DEBUG_MODE
            if (lastLowSteps != getSteps())
            {
                Serial.println("low speed = " + String(result));
                lastLowSteps = getSteps();
            }
#endif
            return result;
        }

        static int16_t getHighRealNonstopSpeed()
        {
            float frequency = highNonstopFrequency;
            int16_t result = SpeedValidator::validate(frequencyToSpeed(frequency));
#ifdef DEBUG_MODE
            if (lastHighSteps != getSteps())
            {
                Serial.println("high speed = " + String(result));
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
        
        static void setNonstopSpeed(int16_t value)
        {
            float frequency = speedToFrequency(value);
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
        static float getTimeOfTurn(int16_t speed)
        {
            static const char buff[] = { 97, 89, 81, 75, 69, 64, 59, 56, 52, 49, 47, 44, 42, 40, 38, 36, 35, 34, 32, 31, 30, 29, 28, 27, 26, 26, 25, 24, 24, 23, 22, 23, 23, 22, 22 };
            int index = speed - MIN_PWM;
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

        static float getSpeedOfTurn(float time)
        {
            int speed = MAX_PWM;
            while (getTimeOfTurn(speed) < time && speed > MIN_PWM)
            {
                speed--;
            }

            return speed;
        }

        static float speedToFrequency(int16_t speed)
        {
            float steps = getSteps();
            return steps / getTimeOfTurn(speed);
        }

        static int16_t frequencyToSpeed(float frequency)
        {
            float steps = getSteps();
            float time = steps / frequency;
            float result = getSpeedOfTurn(time);
            return result + 0.5; // rounded
        }

        static void eeprom_update_float(int addr, float value)
        { 
            byte *x = (byte *)&value;
            for (byte i = 0; i < 4; i++ ) eeprom_update_byte(i+addr, x[i]);
        }

        static float eeprom_read_float(int addr)
        {   
            byte x[4];
            for (byte i = 0; i < 4; i++) x[i] = eeprom_read_byte(i+addr);
            float *result = (float *)&x;
            return result[0];
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
            switch (_mode)
            {
                case MenuItems:
                    top = _items[current].top;
                    if (top.startsWith("%"))
                        top = formatSteps(top);
                    printTop(top);
                    printBottom(_items[current].bottom);
                    break;

                case Array:
                    printTop(String(_array[current], DEC));
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
        MenuItem* _items;
        int16_t* _array;
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
            String result = text;
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
            switch (settingNum)
            {
                case 0: // Steps
                    Settings::setSteps(steps[_menu->current]);
                    break;
                case 1: // Acceleration
                    Settings::setAcceleration(_menu->current + 1);
                    break;
                case 2: // Delay
                    Settings::setDelay(_menu->current * 100);
                    break;
                case 3: // Exposure
                    Settings::setExposure((_menu->current + 1) * 100);
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
            RunAcc,
            Run,
            RunDec
        };

        void tick()
        {
            switch (_state)
            {
                case Move:
                    tickMove();
                    break;
                    
                case Run:
                case RunAcc:
                case RunDec:
                    tickRun();
                    break;
            }
        }

        State getState()
        {
            return _state;
        }

        void move(int graduations, int maxSpeed = MAX_PWM)
        {
            if (!isStopped())
                return;

            _graduations = abs(graduations);
            _forward = graduations > 0;
            _currentSpeed = MIN_PWM;
            _maxSpeed = maxSpeed;
            encoder.readAndReset();
            _state = Move;
        }

        void run(int speed)
        {
            if (!isStopped())
                return;

            _maxSpeed = abs(speed);
            _forward = speed > 0;
            _currentSpeed = MIN_PWM;
            encoder.readAndReset();
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
            if( _state == Run || _state == RunAcc || _state == RunDec)
            {
                // Calculate stop point
                float decelerationLength = Settings::getRealAcceleration();
                float graduationsToStop = (_currentSpeed - MIN_PWM) * decelerationLength /
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
        inline int getCurrentSpeed()
        {
            return _currentSpeed;
        }

        // Returns maximum PWM
        inline int getMaxSpeed()
        {
            return _maxSpeed;
        }

        void changeSpeed(int delta)
        {
            switch (_state)
            {
                case Move:
                    if (_currentSpeed == _maxSpeed)
                    {
                        _maxSpeed += delta;
                        _maxSpeed = SpeedValidator::validate(_maxSpeed);
                        _currentSpeed = _maxSpeed;
                        analogWrite(_forward? MOTOR1 : MOTOR2, _currentSpeed);
                    }
                    break;
                    
                case Run:
                    _maxSpeed += delta;
                    _maxSpeed = SpeedValidator::validate(_maxSpeed);
                    _currentSpeed = _maxSpeed;
                    analogWrite(_forward? MOTOR1 : MOTOR2, _currentSpeed);
                    break;
            }
        }

        // Returns graduation count passed from starting point
        int getCurrentPos()
        {
            int32_t pos = abs(encoder.read());
            return pos;
        }

    private:
        State _state = Stop;
        int _graduations;
        bool _forward;
        int _maxSpeed = MAX_PWM;
        int _currentPos;
        int _currentSpeed;

        void tickMove()
        {
            _currentPos = getCurrentPos();
            if (_currentPos >= _graduations)
            {
                stop();
                return;
            }

            if (_currentPos < _graduations / 2)
                accelerate();
            else
                decelerate();

            analogWrite(_forward? MOTOR1 : MOTOR2, _currentSpeed);
        }

        void accelerate()
        {
            // Use linear function to accelerate
            float x = _currentPos;
            float accelerationLength = Settings::getRealAcceleration();
            float currentSpeed = MIN_PWM + x * (MAX_PWM - MIN_PWM) / accelerationLength;
            _currentSpeed = validateSpeed(currentSpeed);
        }

        void decelerate()
        {
            // Use linear function to decelerate
            float x = _graduations - _currentPos - getFinalDistance();
            float decelerationLength = Settings::getRealAcceleration();
            float currentSpeed = MIN_PWM + x * (MAX_PWM - MIN_PWM) / decelerationLength;
            _currentSpeed = validateSpeed(currentSpeed);
        }

        // End of the step we should go with MIN_PWM
        // This function returns length of this final distance
        int getFinalDistance()
        {
            switch (Settings::getAcceleration())
            {
                case 10:
                    return 16;
                case 9:
                    return 12;
                case 8:
                    return 6;
                case 7:
                    return 3;
                default:
                    return 0;
            }
        }
        
        void tickRun()
        {
            switch (_state)
            {
                case RunAcc:
                    accelerate();
                    analogWrite(_forward? MOTOR1 : MOTOR2, _currentSpeed);
                    if (_currentSpeed == _maxSpeed)
                        _state = Run;
                    break;
                    
                case RunDec:
                    decelerate();
                    if (_currentSpeed <= MIN_PWM)
                        stop();
                    else
                        analogWrite(_forward? MOTOR1 : MOTOR2, _currentSpeed);
                    break;
            }
        }

        int validateSpeed(int speed)
        {
            if (speed > _maxSpeed)
                speed = _maxSpeed;
            else if (speed < MIN_PWM)
                speed = MIN_PWM;

            return speed;
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
            menu.setItems(_menuDef.topItems, _menuDef.topItemsLength);
            menu.current = Settings::getMenuIndex();
        }

        void tick()
        {
            if (hold)
            {
                _menuDef.handlers[menu.current]();
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
        MenuItemsDef _menuDef;
        char _level = 0;
        SettingEditor _editor;

        void press()
        {
            if (_level == 0) // top menu
            {
                if (menu.current < _menuDef.topItemsLength - 1)
                {
                    Settings::setMenuIndex(menu.current);
                    hold = true;
                }
                else // settings
                {
                    updateSettings();
                    menu.setItems(_menuDef.settingsItems, _menuDef.settingsItemsLength);
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
                menu.setItems(_menuDef.topItems, _menuDef.topItemsLength);
                menu.current = _menuDef.topItemsLength - 1; // index of Settings
                menu.display();
                _level = 0;
            }
        }

        void updateSettings()
        {
            _menuDef.settingsItems[0].bottom = String(Settings::getSteps(), DEC);
            _menuDef.settingsItems[1].bottom = String(Settings::getAcceleration(), DEC);
            _menuDef.settingsItems[2].bottom = String(Settings::getDelay(), DEC);
            _menuDef.settingsItems[3].bottom = String(Settings::getExposure(), DEC);
        }
};
Selector selector;

const int completeStopDelay = 50; // 50 ms to complete stop
int16_t stepNumber = 0;
unsigned long timer = 0;
bool isRunning = false;
bool needToCheckError = true;
void Runner::runAutomatic()
{
    const String mode = "Auto...";
    const String stepName = "photo";
    static State currentState;
    
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
        needToCheckError = true;
        mover.move(stepGraduations);
        return;
    }
    
    if (currentState == Move)
    {
        if (needToCheckError)
        {
            timer = millis();
            currentState = Correction;
            return;
        }

        if (stepNumber < stepCount)
        {
            timer = millis(); // set timer
            currentState = Delay;
        }
        else
        {
            finalize();
        }
        return;
    }

    if (currentState == Correction && millis() - timer >= completeStopDelay)
    {
        int16_t error = stepGraduations - mover.getCurrentPos();
        timer = millis();
        if (error == 0)
            timer -= completeStopDelay;
        else
        {
#ifdef DEBUG_MODE
            Serial.println(mover.getCurrentPos());
            Serial.println("Error = " + String(error));
#endif
            mover.move(error, MIN_PWM);
        }
        currentState = Move;
        needToCheckError = false;
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
        timer = millis();
        currentState = Exposure;
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
        needToCheckError = true;
        mover.move(stepGraduations);
        return;
    }

    if (currentState == Move && mover.isStopped())
    {
        if (needToCheckError)
        {
            timer = millis();
            currentState = Correction;
            return;
        }

        if (stepNumber < stepCount)
        {
            currentState = Waiting;
            stepNumber++;
            display(mode, stepName);
        }
        else
        {
            finalize();
        }
    }

    if (currentState == Correction && millis() - timer >= completeStopDelay)
    {
        int16_t error = stepGraduations - mover.getCurrentPos();
        if (error != 0)
        {
#ifdef DEBUG_MODE
            Serial.println(mover.getCurrentPos());
            Serial.println("Error = " + String(error));
#endif
            mover.move(error, MIN_PWM);
        }
        currentState = Move;
        needToCheckError = false;
        return;
    }
}

void Runner::runNonstop()
{
    const String mode = "Nonstop...";
    const String stepName = "photo";
    static State currentState;
    static int nextSnapshotPos;
    static bool needToStoreNewSpeed;
 
    if (enc.press())
    {
        finalize();
        return;
    }
    else if (enc.turn() && currentState != Correction)
    {
        if (enc.left())
        {
            if (mover.getCurrentSpeed() > Settings::getLowRealNonstopSpeed())
            {
                int d = delta;
                if (mover.getCurrentSpeed() - d < Settings::getLowRealNonstopSpeed())
                    d = mover.getCurrentSpeed() - Settings::getLowRealNonstopSpeed();

                mover.changeSpeed(-d);
                needToStoreNewSpeed = true;
            }
        }
        else if (enc.right())
        {
            if (mover.getCurrentSpeed() < Settings::getHighRealNonstopSpeed())
            {
                int d = delta;
                if (mover.getCurrentSpeed() + d > Settings::getHighRealNonstopSpeed())
                    d = Settings::getHighRealNonstopSpeed() - mover.getCurrentSpeed();

                mover.changeSpeed(d);
                needToStoreNewSpeed = true;
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
        needToStoreNewSpeed = false;
        timer = millis();
        currentState = Beginning;
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
        needToCheckError = true;
        mover.move(GRADUATIONS, Settings::getRealNonstopSpeed());
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

    if (isRunning && currentState != Beginning && currentState != Correction && mover.isStopped())
    {
        if (needToStoreNewSpeed)
        {
#ifdef DEBUG_MODE
            Serial.println("Store new speed = " + String(mover.getMaxSpeed()));
#endif
            Settings::setNonstopSpeed(mover.getMaxSpeed());
            needToStoreNewSpeed = false;
        }

        if (needToCheckError)
        {
            timer = millis();
            currentState = Correction;
            return;
        }
        
        finalize();
    }

    if (currentState == Correction && millis() - timer >= completeStopDelay)
    {
        int16_t error = GRADUATIONS - mover.getCurrentPos();
        if (error != 0)
        {
#ifdef DEBUG_MODE
            Serial.println(mover.getCurrentPos());
            Serial.println("Error = " + String(error));
#endif
            mover.move(error, MIN_PWM);
            currentState = Waiting;
            needToCheckError = false;
            return;
        }

        finalize();
    }
}

void Runner::runVideo()
{
    if (!isRunning)
    {
        selector.menu.display("Video...", "");
        mover.run(Settings::getVideoSpeed());
        isRunning = true;
    }

    if (mover.isStopped())
    {
        isRunning = false;
        selector.hold = false;
        return;
    }
    
    char direction = mover.isForward() ? 1 : -1;
    if (enc.press())
    {
        switch (mover.getState())
        {
            case mover.State::RunDec:
                return; // tried to stop already

            case mover.State::Run:
                Settings::setVideoSpeed(mover.getMaxSpeed() * direction);
            default:
                mover.softStop();
        }
    }
    else if (enc.turn() && mover.getState() == mover.State::Run)
    {
        char direction = mover.isForward() ? 1 : -1;
        if (enc.left())
        {
            if (direction > 0 && mover.getCurrentSpeed() <= MIN_PWM)
            {
                // Change direction
                mover.stop();
                mover.run(-MIN_PWM);
                return;
            }

            mover.changeSpeed(-delta * direction);
        }
        else if (enc.right())
        {
            if (direction < 0 && mover.getCurrentSpeed() <= MIN_PWM)
            {
                // Change direction
                mover.stop();
                mover.run(MIN_PWM);
                return;
            }

            mover.changeSpeed(delta * direction);
        }
    }
}

void Runner::runRotate()
{
    static State currentState;

    if (!isRunning)
    {
        selector.menu.display("Rotation...", "<-  ->");
        isRunning = true;
    }
    
    if (enc.press())
    {
        finalize();
        return;
    }

    if (!mover.isStopped())
    {
        enc.resetState();
        return;
    }

    if (enc.left())
    {
        currentState = Move;
        needToCheckError = true;
        mover.move(-GRADUATIONS / 4);
    }
    else if (enc.right())
    {
        currentState = Move;
        needToCheckError = true;
        mover.move(GRADUATIONS / 4);
    }

    if (currentState == Move && mover.isStopped())
    {
        if (needToCheckError)
        {
            timer = millis();
            currentState = Correction;
            return;
        }

        currentState = Waiting;
    }
    
    if (currentState == Correction && millis() - timer >= completeStopDelay)
    {
        int16_t error = GRADUATIONS / 4 - mover.getCurrentPos();
        if (!mover.isForward())
            error = -error;
            
        if (error != 0)
        {
#ifdef DEBUG_MODE
            Serial.println(mover.getCurrentPos());
            Serial.println("Error = " + String(error));
#endif
            mover.move(error, MIN_PWM);
        }
        currentState = Move;
        needToCheckError = false;
        return;
    }
}

void Runner::finalize()
{
    isRunning = false;
    mover.stop();
    stepNumber = 0;
    needToCheckError = true;
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
