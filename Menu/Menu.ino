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

class Menu
{
    public:
        short int menu = M_FIRST;

        void display()
        {
            switch (menu)
            {
                case AUTO:
                    printTop("Automatic");
                    printBottom("press to start");
                    break;

                case MANUAL:
                    printTop("Manual");
                    printBottom("press to start");
                    break;
                    
                case NONSTOP:
                    printTop("Nonstop");
                    printBottom("press to start");
                    break;
                    
                case VIDEO:
                    printTop("Video");
                    printBottom("press to start");
                    break;

                case SETTINGS:
                    printTop("Settings");
                    printBottom("press to edit");
                    break;
            }
        }

        void next()
        {
            menu++;
            if (menu > M_LAST)
                menu = M_FIRST;
            Serial.println(menu);
        }

        void prev()
        {
            menu--;
            if (menu < M_FIRST)
                menu = M_LAST;
            Serial.println(menu);
        }

    private:
        String _recentTop;
        String _recentBottom;
        
        void printTop(String text)
        {
            text = FillWithSpaces(text);
            if (_recentTop != text)
            {
                lcd.setCursor(0, 0);
                lcd.print(text);
                _recentTop = text;
            }
        }

        void printBottom(String text)
        {
            text = FillWithSpaces(text);
            if (_recentBottom != text)
            {
                lcd.setCursor(0, 1);
                lcd.print(text);
                _recentBottom = text;
            }
        }

        String FillWithSpaces(String text)
        {
            String res = text;
            while (res.length() < 16)
                res += " ";

            return res;
        }
};

Menu menu;

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
            menu.prev();
        else if (enc.right())
            menu.next();
    }
    menu.display();
}
