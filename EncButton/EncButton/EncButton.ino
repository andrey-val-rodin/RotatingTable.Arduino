#include <EncButton.h>

EncButton<EB_TICK, 12, 13, 11> enc;
EncButton<EB_TICK, 4> redButton;
EncButton<EB_TICK, 7> greenButton;
EncButton<EB_TICK, 8> yellowButton;

/*
class Button
{
    public:
        Button(byte pin)
        {
            _pin = pin;
        }
        
        void tick()
        {
            bool state = digitalRead(_pin);
            if (state)
            {
                if (!_prevState)
                {
                    _pressed = true;
                    _state = true;
                    _prevState = true;
                }
            }
            else
            {
                _state = false;
                _prevState = false;
            }
        }
        
        bool press()
        {
            bool res = _pressed;
            _pressed = false;
            return res;
        }

    private:
        byte _pin;
        bool _prevState = false;
        bool _state = false;
        bool _pressed = false;
};

Button redButton(4);
Button greenButton(7);
Button yellowButton(8);
*/

void setup()
{
    Serial.begin(9600);
    redButton.setButtonLevel(HIGH);
    greenButton.setButtonLevel(HIGH);
    yellowButton.setButtonLevel(HIGH);
}

void loop()
{
    enc.tick();
    redButton.tick();
    greenButton.tick();
    yellowButton.tick();
    
    if (redButton.press())
        Serial.println("Red button pressed");
    if (greenButton.press())
        Serial.println("Green button pressed");
    if (yellowButton.press())
        Serial.println("Yellow button pressed");

    if (enc.press())
        Serial.println("Encoder pressed");
    if (enc.turn())
    {
        if (enc.left())
            Serial.println("Encoder turned left");
        if (enc.right())
            Serial.println("Encoder turned right");
    }
}
