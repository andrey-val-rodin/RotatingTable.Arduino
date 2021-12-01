#include <EncButton.h>
#include <LiquidCrystal_I2C.h>
#include <avr/eeprom.h>

#define MOTOR1 6
#define MOTOR2 5
#define MOTOR_ENC1 2
#define MOTOR_ENC2 3
#define CAMERA 9
#define SHUTTER 10
#define CAMERA_LOW HIGH
#define CAMERA_HIGH LOW
#define MIN_PWM 10
#define MAX_PWM 255
#define GRADUATIONS 1080

LiquidCrystal_I2C lcd(0x27, 16, 2);
EncButton<EB_TICK, 12, 13, 11> enc; // pins 11, 12, 13
EncButton<EB_TICK, 7> photoButton;  // pin 7
EncButton<EB_TICK, 4> nextButton;   // pin 4

char EEMEM stepsOffset;
char EEMEM accelerationOffset;
int16_t EEMEM delayOffset;
int16_t EEMEM exposureOffset;
int16_t EEMEM videoSpeedOffset;
int16_t EEMEM nonstopSpeedOffset;
char EEMEM menuIndexOffset;

const char stepsLength = 12;
const char lastStep = 120;
const char steps[stepsLength] = { 4, 8, 18, 20, 24, 30, 36, 45, 60, 72, 90, lastStep };
char FindInSteps(char numberOfSteps)
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
        enum State
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

    MenuItem topItems[topItemsLength] = {
        {"Automatic",   "press to start"},
        {"Manual",      "press to start"},
        {"Nonstop",     "press to start"},
        {"Video",       "press to start"},
        {"Rotate 90",   "press to rotate"},
        {"Settings",    "press to edit"}};

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

class Settings
{
    public:
        static const int16_t lowNonstopSpeed = MIN_PWM * lastStep;
        static const int16_t highNonstopSpeed = lowNonstopSpeed * 6;
        
        static char getSteps()
        {
            return verifySteps(eeprom_read_byte(&stepsOffset));
        }

        static char verifySteps(char value)
        {
            return FindInSteps(value) >= 0
                ? value
                : 24; // use default
        }

        static void setSteps(char value)
        {
            eeprom_update_byte(&stepsOffset, value);
        }

        static char getAcceleration()
        {
            return verifyAcceleration(eeprom_read_byte(&accelerationOffset));
        }

        // Returns value from 20 to 110 that can be used as a number of graduations
        // for acceleration and deceleration from min to max PWM and vice versa.
        // Function converts current user-friendly value 1-10 to this native value.
        // Note that native value shouldn't be less than 20.
        static char getNativeAcceleration()
        {
            char result = getAcceleration();
            result = abs(result - 11); // reverse
            result *= 10;
            result += 10;
            return result;
        }

        static char verifyAcceleration(char value)
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
            return verifyDelay(eeprom_read_word(&delayOffset));
        }

        static int16_t verifyDelay(int16_t value)
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
            return verifyExposure(eeprom_read_word(&exposureOffset));
        }

        static int16_t verifyExposure(int16_t value)
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
            return verifyVideoSpeed(eeprom_read_word(&videoSpeedOffset));
        }

        static int16_t verifyVideoSpeed(int16_t value)
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

        // Returns value in range from lowNonstopSpeed to highNonstopSpeed.
        static int16_t getNonstopSpeed()
        {
            return verifyNonstopSpeed(eeprom_read_word(&nonstopSpeedOffset));
        }

        static int16_t verifyNonstopSpeed(int16_t value)
        {
            return lowNonstopSpeed <= value && value <= highNonstopSpeed
                    ? value
                    : lowNonstopSpeed; // use default
        }

        // Returns speed depending on current number of steps
        static int16_t getNativeNonstopSpeed()
        {
            return SpeedValidator::validate(getNonstopSpeed() / getSteps());
        }

        static int16_t getLowNativeNonstopSpeed()
        {
            float f = (float) lowNonstopSpeed / getSteps();
            int16_t i = f;
            if (f > i)
                i++;

            return SpeedValidator::validate(i);
        }

        static int16_t getHighNativeNonstopSpeed()
        {
            return SpeedValidator::validate(highNonstopSpeed / getSteps());
        }

        static void setNonstopSpeed(int16_t value)
        {
            eeprom_update_word(&nonstopSpeedOffset, value);
        }

        static char getMenuIndex()
        {
            return verifyMenuIndex(eeprom_read_byte(&menuIndexOffset));
        }

        static char verifyMenuIndex(char value)
        {
            return 0 <= value && value <= MenuItemsDef::topItemsLength
                ? value
                : 0; // use default
        }

        static void setMenuIndex(char value)
        {
            eeprom_update_byte(&menuIndexOffset, value);
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

        void setItems(const char* items, char length)
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
            switch (_mode)
            {
                case MenuItems:
                    printTop(_items[current].top);
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
        char* _array;
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

volatile int graduationCount;
class Mover
{
    public:
        // Slightly more distance is required to decelerate than to accelerate
        // This coefficient is intended to address this problem
        float decFactor = 1.7;
        
        enum State
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
            _state = Move;
            attach();
        }

        void run(int speed)
        {
            if (!isStopped())
                return;

            _maxSpeed = abs(speed);
            _forward = speed > 0;
            _currentSpeed = MIN_PWM;
            _state = RunAcc;
            attach();
        }

        void stop()
        {
            analogWrite(MOTOR1, 0);
            analogWrite(MOTOR2, 0);
            _state = Stop;
            detach();
        }

        void softStop()
        {
            if( _state == Run || _state == RunAcc || _state == RunDec)
            {
                // Calculate stop point
                float decelerationLength = Settings::getNativeAcceleration() * decFactor;
                float graduationsToStop = (_currentSpeed - MIN_PWM) * decelerationLength /
                    (MAX_PWM - MIN_PWM);
                _graduations = getCurrentPos() + graduationsToStop;
                _state = RunDec; // deceleration state
            }
            else
                stop();
        }

        bool isStopped()
        {
            return _state == Stop;
        }

        // Returns direction
        bool isForward()
        {
            return _forward;
        }

        // Returns current PWM
        int getCurrentSpeed()
        {
            return _currentSpeed;
        }

        // Returns maximum PWM
        int getMaxSpeed()
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
            // critical section
            noInterrupts();
            int result = graduationCount;
            interrupts();

            return result;
        }

    private:
        State _state = Stop;
        int _graduations;
        bool _forward;
        int _maxSpeed = MAX_PWM;
        int _currentSpeed;

        static void forwardHandler()
        {
            if (digitalRead(MOTOR_ENC2) == LOW)
                graduationCount++;
            else
                graduationCount--;
        }

        static void backwardHandler()
        {
            if (digitalRead(MOTOR_ENC2) == HIGH)
                graduationCount++;
            else
                graduationCount--;
        }

        void attach()
        {
            graduationCount = 0;
            attachInterrupt(digitalPinToInterrupt(MOTOR_ENC1),
                _forward ? forwardHandler : backwardHandler, RISING);
        }

        void detach()
        {
            detachInterrupt(digitalPinToInterrupt(MOTOR_ENC1));
        }

        void tickMove()
        {
            int currentPos = getCurrentPos();
            if (currentPos >= _graduations)
            {
                stop();
                return;
            }

            float x;
            if (currentPos < _graduations / 2)
                accelerate();
            else
                decelerate();

            analogWrite(_forward? MOTOR1 : MOTOR2, _currentSpeed);
        }

        void accelerate()
        {
            float x = getCurrentPos();

            // Use linear function to accelerate
            int accelerationLength = Settings::getNativeAcceleration();
            _currentSpeed = MIN_PWM + x * (MAX_PWM - MIN_PWM) / accelerationLength;
            _currentSpeed = validateSpeed(_currentSpeed);
        }

        void decelerate()
        {
            float x = _graduations - getCurrentPos() - 1;

            // Use linear function to decelerate
            int decelerationLength = Settings::getNativeAcceleration() * decFactor;
            _currentSpeed = MIN_PWM + x * (MAX_PWM - MIN_PWM) / decelerationLength;
            _currentSpeed = validateSpeed(_currentSpeed);
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

char stepNumber;
unsigned long delayTimer = 0;
unsigned long exposureTimer = 0;
bool isRunning = false;
void Runner::runAutomatic()
{
    const String mode = "Automatic...";
    const String stepName = "photo";
    static State currentState;
    
    if (enc.press())
    {
        finalize();
        return;
    }

    if (!mover.isStopped())
        return;

    char stepCount = Settings::getSteps();
    int stepGraduations = GRADUATIONS / stepCount;
    if (!isRunning)
    {
        // Starting
        stepNumber = 0;
        digitalWrite(CAMERA, CAMERA_HIGH); // prepare camera
        isRunning = true;
        exposureTimer = millis();
        currentState = Beginning;
        return;
    }

    if (currentState == Beginning && millis() - exposureTimer >= Settings::getExposure())
    {
        stepNumber++;
        display(mode, stepName);
        digitalWrite(SHUTTER, CAMERA_HIGH); // make first photo
        exposureTimer = millis();
        currentState = Exposure;
        return;
    }
    
    if (currentState == Exposure && millis() - exposureTimer >= Settings::getExposure())
    {
        digitalWrite(SHUTTER, CAMERA_LOW); // release shutter
        currentState = Move;
        mover.move(stepGraduations);
        return;
    }

    if (currentState == Move)
    {
        if (stepNumber < stepCount)
        {
            delayTimer = millis(); // set timer
            currentState = Delay;
        }
        else
        {
            finalize();
        }
        return;
    }

    if (currentState == Delay && millis() - delayTimer >= Settings::getDelay())
    {
        stepNumber++;
        display(mode, stepName);
        digitalWrite(SHUTTER, CAMERA_HIGH); // make photo
        exposureTimer = millis();
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

    char stepCount = Settings::getSteps();
    int stepGraduations = GRADUATIONS / stepCount;
    if (!isRunning)
    {
        // Starting
        stepNumber = 1;
        display(mode, stepName);
        digitalWrite(CAMERA, CAMERA_HIGH); // prepare camera
        isRunning = true;
        exposureTimer = millis();
        currentState = Exposure;
        return;
    }

    if (currentState == Exposure && millis() - exposureTimer >= Settings::getExposure())
    {
        digitalWrite(SHUTTER, CAMERA_LOW); // release shutter
        currentState = Waiting;
        return;
    }

    if (photoButton.press() && currentState == Waiting)
    {
        digitalWrite(SHUTTER, CAMERA_HIGH); // make photo
        exposureTimer = millis();
        currentState = Exposure;
        return;
    }

    if (nextButton.press() && currentState == Waiting)
    {
        exposureTimer = millis();
        currentState = Move;
        mover.move(stepGraduations);
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
    static bool needToStoreNewSpeed;
 
    if (enc.press())
    {
        finalize();
        return;
    }
    else if (enc.turn())
    {
        if (enc.left())
        {
            if (mover.getCurrentSpeed() > Settings::getLowNativeNonstopSpeed())
            {
                int d = delta;
                if (mover.getCurrentSpeed() - d < Settings::getLowNativeNonstopSpeed())
                    d = mover.getCurrentSpeed() - Settings::getLowNativeNonstopSpeed();

                mover.changeSpeed(-d);
                needToStoreNewSpeed = true;
            }
        }
        else if (enc.right())
        {
            if (mover.getCurrentSpeed() < Settings::getHighNativeNonstopSpeed())
            {
                int d = delta;
                if (mover.getCurrentSpeed() + d > Settings::getHighNativeNonstopSpeed())
                    d = Settings::getHighNativeNonstopSpeed() - mover.getCurrentSpeed();
                    
                mover.changeSpeed(d);
                needToStoreNewSpeed = true;
            }
        }
    }

    char stepCount = Settings::getSteps();
    int stepGraduations = GRADUATIONS / stepCount;
    if (!isRunning)
    {
        // Starting
        stepNumber = 0;
        digitalWrite(CAMERA, CAMERA_HIGH); // prepare camera
        isRunning = true;
        needToStoreNewSpeed = false;
        exposureTimer = millis();
        currentState = Beginning;
        return;
    }

    if (currentState == Beginning && millis() - exposureTimer >= Settings::getExposure())
    {
        stepNumber++;
        display(mode, stepName);
        digitalWrite(SHUTTER, CAMERA_HIGH); // make first photo
        nextSnapshotPos = stepGraduations;
        exposureTimer = millis();
        currentState = Exposure;
        Settings::getNativeNonstopSpeed();
        mover.move(GRADUATIONS, Settings::getNativeNonstopSpeed());
        return;
    }

    if (currentState == Exposure && millis() - exposureTimer >= 50)
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
            exposureTimer = millis();
            currentState = Exposure;
        }
    }

    if (isRunning && currentState != Beginning && mover.isStopped())
    {
        if (needToStoreNewSpeed)
            Settings::setNonstopSpeed(mover.getMaxSpeed() * Settings::getSteps());
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
    if (!isRunning)
    {
        selector.menu.display("Rotation...", "<--  -->");
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
        mover.move(-GRADUATIONS / 4);
    else if (enc.right())
        mover.move(GRADUATIONS / 4);
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

    Serial.begin(9600);
}

void loop()
{
    enc.tick();
    photoButton.tick();
    nextButton.tick();
    selector.tick();
    mover.tick();
}
