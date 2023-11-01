//Use FastLED library
#include <FastLED.h>
//variables can be changed here once to keep numbers uniform throughout program
#define LED_PIN     7
#define NUM_LEDS    20
int r[] = {255, 255, 255, 128, 0, 0, 0, 0, 0, 127, 255, 255};
int g[] = {0, 128, 255, 255, 255, 255, 255, 128, 0, 0, 0, 0};
int b[] = {0, 0, 0, 0, 0, 128, 255, 255, 255, 255, 255, 127};
int colorCounter = 0;
int modeCounter = 0;
CRGB leds[NUM_LEDS];

int Rvalue = 254, Gvalue = 1, Bvalue = 127;;
int Rdirection = -1, Gdirection = 1, Bdirection = -1;


void setup() {
  //setting up switch and button pins
  pinMode(10, INPUT); //motion detector
  pinMode(9, INPUT_PULLUP); //switch
  pinMode(8, INPUT_PULLUP); //button 2
  pinMode(4, INPUT_PULLUP); //button 1
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  Serial.begin(9600);
  FastLED.clear();
  FastLED.show();
  //startup function created at bottom
  startup();
}

void loop() {
  //whenever button is pressed, colorCounter will be increased by 1
  //colorCounter resets to 0 after colorCounter exceeds number of colors 
  if (digitalRead(8) == LOW) {
    colorCounter += 1;
    if (colorCounter > 11) {
      colorCounter = 0;
    }
    delay(500);
  }
  //whenever button is pressed, modeCounter will be increased by 1
  //modeCounter resets to 0 after modeCounter exceeds number of modes 
  if (digitalRead(4) == LOW) {
    modeCounter += 1;
    if (modeCounter > 4) {
      modeCounter = 0;
    }
    delay(1000);
  }
  //if switch is on, turn on LEDs
  if (digitalRead(9) == LOW) {
    //based on modeCounter, execute an LED mode. Each of these function is created at bottom.
    if (modeCounter == 0) {
      alwaysOn();
    }
    else if (modeCounter == 1) {
      motionDetection();
    }
    else if (modeCounter == 2) {
      rainbow();
    }
    else if (modeCounter == 3) {
      colorToggle();
    }
    else if (modeCounter == 4) {
      discoMode();
    }
  }
  //if switch is off, turn off LEDs
  else {
    FastLED.clear();
    FastLED.show();
  }
}

void startup() {
  for (int i = 0; i < 11; i++) {
    for (int i = 0; i < NUM_LEDS; i++) {
      FastLED.clear();
      leds[i] = CRGB ( r[colorCounter], g[colorCounter], b[colorCounter]);
      FastLED.show();
      delay(20);
    }
    colorCounter += 1;
  }
  colorCounter = 0;
}
void motionDetection() {
  if (digitalRead(10) == HIGH) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB ( r[colorCounter], g[colorCounter], b[colorCounter]);
      FastLED.show();
    }
  }
  else {
    Serial.println("Off");
    FastLED.clear();
    FastLED.show();
  }
}
void alwaysOn() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB ( r[colorCounter], g[colorCounter], b[colorCounter]);
    FastLED.show();
  }
}
void discoMode() {
  for (int i = 0; i < NUM_LEDS; i++) {
    colorCounter = random(0, 12);
    leds[i] = CRGB ( r[colorCounter], g[colorCounter], b[colorCounter]);
    FastLED.show();

  }
  delay(500);
}
void colorToggle() {
  colorCounter = random(0, 12);
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB ( r[colorCounter], g[colorCounter], b[colorCounter]);
    FastLED.show();
    delay(50);
  }
}
void rainbow () {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB ( Rvalue, Gvalue, Bvalue);
    FastLED.show();
  }
  Rvalue = Rvalue + Rdirection;   //changing values of LEDs
  Gvalue = Gvalue + Gdirection;
  Bvalue = Bvalue + Bdirection;

  //now change direction for each color if it reaches 255
  if (Rvalue >= 255 || Rvalue <= 0)
  {
    Rdirection = Rdirection * -1;
  }
  if (Gvalue >= 255 || Gvalue <= 0)
  {
    Gdirection = Rdirection * -1;
  }
  if (Bvalue >= 255 || Bvalue <= 0)
  {
    Bdirection = Bdirection * -1;
  }
  delay(3);    //give some delay so you can see the change

}
