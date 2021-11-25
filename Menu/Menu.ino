#include <EncButton.h>
#include <LiquidCrystal_I2C.h>

EncButton<EB_TICK, 12, 13, 11> enc;
LiquidCrystal_I2C lcd(0x27, 16, 2);

enum Menus
{
    M_FIRST     = 0,
    AUTO        = 0,
    MANUAL      = 1,
    NONSTOP     = 2,
    VIDEO       = 3,
    SETTINGS    = 4,
    M_LAST      = 4
};

struct MenuItem
{
    String top;
    String bottom;
};

class Menu
{
    public:
        int current = 0;

        Menu(MenuItem* items, int length)
        {
            _items = items;
            _length = length;
        }

        void display()
        {
            printTop(_items[current].top);
            printBottom(_items[current].bottom);
        }

        void next()
        {
            current++;
            if (current >= _length)
                current = 0;
        }

        void prev()
        {
            current--;
            if (current < 0)
                current = _length - 1;
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
                lcd.setCursor(0, 0);
                lcd.print(text);
                _recentTop = text;
            }
        }

        void printBottom(String text)
        {
            if (_recentBottom != text)
            {
                lcd.setCursor(0, 1);
                lcd.print(text);
                _recentBottom = text;
            }
        }
};

MenuItem topItems[] = {
    {"Automatic       ", "press to start  "},
    {"Manual          ", "press to start  "},
    {"Nonstop         ", "press to start  "},
    {"Video           ", "press to start  "},
    {"Settings        ", "press to edit   "}};
Menu topMenu(topItems, 5);

void setup()
{
    lcd.init();
    lcd.begin(16, 2);
    lcd.clear();
    lcd.backlight();

    Serial.begin(9600);
}

void loop()
{
    enc.tick();

    if (enc.press())
        Serial.println("Encoder pressed");

    if (enc.turn())
    {
        if (enc.left())
            topMenu.prev();
        else if (enc.right())
            topMenu.next();
    }
    topMenu.display();
}
