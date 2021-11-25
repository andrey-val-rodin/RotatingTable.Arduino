#include <EncButton.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

EncButton<EB_TICK, 12, 13, 11> enc;
LiquidCrystal_I2C lcd(0x27, 16, 2);

struct MenuItem
{
    String top;
    String bottom;
};

class Menu
{
    public:
        int current = 0;

        void setItems(MenuItem* items, int length)
        {
            _items = items;
            _length = length;
        }

        void display()
        {
            printTop(_items[current].top);
            printBottom(_items[current].bottom);
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
        MenuItem* _items;
        int _length;
        String _recentTop;
        String _recentBottom;
        
        void printTop(String text)
        {
            if (_recentTop != text)
            {
                _recentTop = text;
                
                while (text.length() < 16)
                    text += " ";
                lcd.setCursor(0, 0);
                lcd.print(text);
            }
        }

        void printBottom(String text)
        {
            if (_recentBottom != text)
            {
                _recentBottom = text;
                
                while (text.length() < 16)
                    text += " ";
                lcd.setCursor(0, 1);
                lcd.print(text);
            }
        }
};

byte EEMEM stepsOffset;
byte EEMEM accelerationOffset;
int16_t EEMEM delayOffset;
int16_t EEMEM exposureOffset;
byte EEMEM videoSpeedOffset;
byte EEMEM nonstopSpeedOffset;
class EEPROMManager
{
    public:
        byte getSteps()
        {
            byte result;
            EEPROM.get(stepsOffset, result);
            return result;
        }

        void setSteps(byte value)
        {
            EEPROM.put(stepsOffset, value);
        }

        byte getAcceleration()
        {
            byte result;
            EEPROM.get(accelerationOffset, result);
            return result;
        }

        void setAcceleration(byte value)
        {
            EEPROM.put(accelerationOffset, value);
        }

        int16_t getDelay()
        {
            int16_t result;
            EEPROM.get(delayOffset, result);
            return result;
        }

        void setDelay(int16_t value)
        {
            EEPROM.put(delayOffset, value);
        }

        int16_t getExposure()
        {
            int16_t result;
            EEPROM.get(exposureOffset, result);
            return result;
        }

        void setExposure(int16_t value)
        {
            EEPROM.put(exposureOffset, value);
        }

        byte getVideoSpeed()
        {
            byte result;
            EEPROM.get(videoSpeedOffset, result);
            return result;
        }

        void setVideoSpeed(byte value)
        {
            EEPROM.put(videoSpeedOffset, value);
        }

        byte getNonstopSpeed()
        {
            byte result;
            EEPROM.get(nonstopSpeedOffset, result);
            return result;
        }

        void setNonstopSpeed(byte value)
        {
            EEPROM.put(nonstopSpeedOffset, value);
        }
};

typedef void (*callback_t)();
class Selector
{
    public:
        Menu menu;
        bool hold = false;

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
        int _level = 0;
        
        callback_t _callbacks[4];

        static const int _topItemsLength = 5;
        MenuItem _topItems[_topItemsLength] = {
            {"Automatic", "press to start"},
            {"Manual",    "press to start"},
            {"Nonstop",   "press to start"},
            {"Video",     "press to start"},
            {"Settings",  "press to edit"}};

       void press()
       {
          if (_level == 0)
          {
              if (menu.current < 4)
              {
                  hold = true;
                  _callbacks[menu.current]();
              }
              else // settings
              {
              }
          }
       }
};

Selector selector;

void setup()
{
    lcd.init();
    lcd.begin(16, 2);
    lcd.clear();
    lcd.backlight();

    selector.initialize(
        [](){selector.menu.display("auto...", ""); delay(3000); selector.hold = false; },
        [](){selector.menu.display("manual...", ""); delay(3000); selector.hold = false; },
        [](){selector.menu.display("nonstop...", ""); delay(3000); selector.hold = false; },
        [](){selector.menu.display("video...", ""); delay(3000); selector.hold = false; });

    Serial.begin(9600);
}

void loop()
{
    enc.tick();
    selector.tick();
}
