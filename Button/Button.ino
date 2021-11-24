#include <EasyButton.h>

#define RED_BUTTON 4
#define GREEN_BUTTON 7
#define YELLOW_BUTTON 8

EasyButton redButton(4);
EasyButton greenButton(7);
EasyButton yellowButton(8);


void onRedButtonPressed() {
    Serial.println("Red button pressed");
}

void onGreenButtonPressed() {
    Serial.println("Green button pressed");
}

void onYellowButtonPressed() {
    Serial.println("Yellow button pressed");
}

void setup() {
    redButton.begin();
    greenButton.begin();
    yellowButton.begin();
    
    Serial.begin(9600);

    redButton.onPressed(onRedButtonPressed);
    greenButton.onPressed(onGreenButtonPressed);
    yellowButton.onPressed(onYellowButtonPressed);

    if (digitalRead(8) == HIGH)
        Serial.println("Yellow button pressed");
}

void loop() {
    redButton.read();
    greenButton.read();
    yellowButton.read();
}
