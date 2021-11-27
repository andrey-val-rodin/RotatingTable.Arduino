#include <EncButton.h>
#include <LiquidCrystal_I2C.h>
#include <avr/eeprom.h>

#define MOTOR1 6
#define MOTOR2 7
#define MOTOR_ENC1 2
#define MOTOR_ENC2 3
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
char EEMEM videoSpeedOffset;
char EEMEM nonstopSpeedOffset;

const char stepsLength = 12;
const char steps[stepsLength] = { 4, 8, 18, 20, 24, 30, 36, 45, 60, 72, 90, 120 };
char FindInSteps(char numberOfSteps)
{
    for (char i = 0; i < stepsLength; i++)
    {
        if (steps[i] == numberOfSteps)
            return i;
    }

    return -1;
}
        
class Settings
{
    public:
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

        // Converts user-friendly value 1-10 to native value 20-120
        static char getNativeAcceleration()
        {
            char result = getAcceleration();
            // reverse
            result = abs(result - 11);
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
            return 0 <= value && value <= 500 && value % 100 == 0
                ? value
                : 100; // use default
        }

        static void setExposure(int16_t value)
        {
            eeprom_update_word(&exposureOffset, value);
        }

        static char getVideoSpeed()
        {
            return verifySpeed(eeprom_read_byte(&videoSpeedOffset));
        }

        static char verifySpeed(char value)
        {
            return 1 <= value && value <= 255
                ? value
                : 100; // use default
        }

       static void setVideoSpeed(char value)
        {
            eeprom_update_byte(&videoSpeedOffset, value);
        }

        static char getNonstopSpeed()
        {
            return verifySpeed(eeprom_read_byte(&nonstopSpeedOffset));
        }

        static void setNonstopSpeed(char value)
        {
            eeprom_update_byte(&nonstopSpeedOffset, value);
        }
};

struct MenuItem
{
    String top;
    String bottom;
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
                    Settings::setExposure(_menu->current * 100);
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
                    _menu->setItems(6, 100, 0);
                    _menu->current = Settings::getExposure() / 100;
                    break;
            }
        }
};

typedef void (*callback_t)();
class Selector
{
    public:
        Menu menu;
        bool hold = false;

        Selector() : _editor(&menu)
        {
        }

        void initialize(
            callback_t automatic,
            callback_t manual,
            callback_t nonstop,
            callback_t video)
        {
            _callbacks[0] = automatic;
            _callbacks[1] = manual;
            _callbacks[2] = nonstop;
            _callbacks[3] = video;

            menu.setItems(_topItems, _topItemsLength);
        }

        void tick()
        {
            if (hold)
            {
                _callbacks[menu.current]();
                return;
            }
            
            menu.display();
            
            if (enc.press())
                press();
            else if (enc.turn())
            {
                if (enc.left() && !hold)
                    menu.prev();
                else if (enc.right() && !hold)
                    menu.next();
            }
        }
        
    private:
        char _level = 0;
        SettingEditor _editor;
        callback_t _callbacks[4];

        static const char _topItemsLength = 5;
        MenuItem _topItems[_topItemsLength] = {
            {"Automatic", "press to start"},
            {"Manual",    "press to start"},
            {"Nonstop",   "press to start"},
            {"Video",     "press to start"},
            {"Settings",  "press to edit"}};

        static const char _settingsItemsLength = 4;
        MenuItem _settingsItems[_settingsItemsLength] = {
            {"Steps",        ""},
            {"Acceleration", ""},
            {"Delay",        ""},
            {"Exposure",     ""}};
        
        void press()
        {
            if (_level == 0) // top menu
            {
                if (menu.current < 4)
                {
                    hold = true;
                }
                else // settings
                {
                    updateSettings();
                    menu.setItems(_settingsItems, _settingsItemsLength);
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
                menu.setItems(_topItems, _topItemsLength);
                menu.display();
                _level = 0;
            }
        }

        void updateSettings()
        {
            _settingsItems[0].bottom = String(Settings::getSteps(), DEC);
            _settingsItems[1].bottom = String(Settings::getAcceleration(), DEC);
            _settingsItems[2].bottom = String(Settings::getDelay(), DEC);
            _settingsItems[3].bottom = String(Settings::getExposure(), DEC);
        }
};
Selector selector;

volatile int graduationCount;
class Mover
{
    public:
        // number of graduations for acceleration and deceleration
        // from min to max PWM and vice versa
        // Shouldn't be less than 20
        int accelerationLength;

        void tick()
        {
            if (isStopped())
                return;

            _tick();
        }

        void move(int graduations, bool forward)
        {
            if (isRunning())
                return;

            initialize(graduations, forward);
        }

        void stop()
        {
            analogWrite(pins[0], 0);
            analogWrite(pins[1], 0);
            _isRunning = false;
            detach();
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
        const int _maxSpeed = MAX_PWM;
        const int pins[2] = {MOTOR1, MOTOR2};

        bool _isRunning = false;
        int _graduations;
        byte _pinIndex;
        unsigned long _oldTime;
        int _currentSpeed;

        static void encoderHandler()
        {
            // Check whether second signal from encoder is 0
            // !!! test in backward direction!
            if (digitalRead(MOTOR_ENC2) == LOW)
                graduationCount++;
            else
                graduationCount--;
        }

        void attach()
        {
            attachInterrupt(digitalPinToInterrupt(MOTOR_ENC1), encoderHandler, RISING);
        }

        void detach()
        {
            detachInterrupt(digitalPinToInterrupt(MOTOR_ENC1));
        }

        void initialize(int graduations, bool forward)
        {
            graduationCount = 0;
            _graduations = graduations;
            _pinIndex = forward ? 0 : 1;
            _currentSpeed = _minSpeed;
            _isRunning = true;
            attach();
        }

        void _tick()
        {
            if (graduationCount >= _graduations)
            {
                stop();
                return;
            }

            float x;
            if (graduationCount < _graduations / 2)
                x = graduationCount;
            else
                x = _graduations - graduationCount - 1;

            // Use linear function to accelerate/decelerate
            accelerationLength = Settings::getNativeAcceleration();
            _currentSpeed = _minSpeed + x * (_maxSpeed - _minSpeed) / accelerationLength;
            // Validate
            if (_currentSpeed > _maxSpeed)
                _currentSpeed = _maxSpeed;
            else if (_currentSpeed < _minSpeed)
                _currentSpeed = _minSpeed;            

            analogWrite(pins[_pinIndex], _currentSpeed);
        }
};
Mover mover;

char stepNumber = 0;
class Runner
{
    public:
        static void runAutomatic()
        {
            mover.tick();
            
            if (enc.press())
            {
                mover.stop();
                finalize();
                return;
            }
            
            if (mover.isStopped())
            {
                char stepCount = Settings::getSteps();
                int stepGraduations = GRADUATIONS / stepCount;
                char strBuf[17];
                
                sprintf(strBuf, "step %d (%d)", stepNumber + 1, stepCount);
                selector.menu.display("Automatic...", strBuf);
        
                if (stepNumber < stepCount)
                {
                    mover.move(stepGraduations, true);
                    stepNumber++;
                }
                else
                {
                    finalize();
                }
            }
        }
        
    private:
        static void finalize()
        {
            stepNumber = 0;
            selector.hold = false; // release selector
        }
};
Runner runner;

void setup()
{
    lcd.init();
    lcd.begin(16, 2);
    lcd.clear();
    lcd.backlight();

    photoButton.setButtonLevel(HIGH);
    nextButton.setButtonLevel(HIGH);

    selector.initialize(
        runner.runAutomatic,
        [](){selector.menu.display("manual...", ""); delay(3000); selector.hold = false; },
        [](){selector.menu.display("nonstop...", ""); delay(3000); selector.hold = false; },
        [](){selector.menu.display("video...", ""); delay(3000); selector.hold = false; });

    Serial.begin(9600);
}

void loop()
{
    enc.tick();
    photoButton.tick();
    nextButton.tick();
    selector.tick();
}
