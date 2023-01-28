#define LGFX_M5STACK
#define LGFX_USE_V1

#include <Arduino.h>
#include <M5Stack.h>
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>
#include <FastLED.h>

#define NUM_LEDS 64
#define DATA_PIN 21

CRGB leds[NUM_LEDS];

static LGFX lcd;

TaskHandle_t thp[1];

const int dataPin = 16;  /* Q7 */
const int clockPin = 17; /* CP */
const int latchPin = 26; /* PL */

int numSteps = 16;
int numParts = 4;
int nowSteps = 0;


int outPort[4] = { 22, 5, 12, 15 };

constexpr size_t NUM_SW = 72;

int nowSw[72];
int oldSw[72];
int outNow[72];

int buffer[NUM_SW];

int bpm = 120;
int oldTime;
int perNote = 16;
int perTime[4] = { 80, 80, 80, 80 };

void nextStep() {
  for (int i = 0; i < numParts; i++) {
    if (!buffer[nowSteps + (i * 16)]) {
      leds[nowSteps + (i * 16)] = CRGB(0, 0, 0);
    }
  }
  // Serial.println(nowSteps);
  oldTime = millis();
  nowSteps++;
  if (nowSteps >= numSteps) nowSteps = 0;
  for (int i = 0; i < numParts; i++) {
    if (!buffer[nowSteps + (i * 16)]) {
      leds[nowSteps + (i * 16)] = CRGB(0, 0, 2);
    }
    digitalWrite(outPort[i], buffer[nowSteps + (i * 16)]);
    // Serial.print(buffer[nowSteps + (i * 16)]);
  }
  // Serial.println("");
  FastLED.show();
}


void SwScan(void *args) {
  while (1) {
    // Step 1: Sample
    digitalWrite(latchPin, LOW);
    digitalWrite(latchPin, HIGH);

    // Step 2: Shift
    for (int i = NUM_SW - 1; i >= 0; i--) {
      nowSw[i] = !digitalRead(dataPin);
      if (nowSw[i]) {
        if (oldSw[i] != nowSw[i]) {
          oldSw[i] = nowSw[i];
          buffer[i] = !buffer[i];
          leds[i] = CRGB(buffer[i] * 10, buffer[i] * 10, 0);
        }
      } else {
        oldSw[i] = nowSw[i];
      }
      Serial.print(buffer[i]);
      digitalWrite(clockPin, HIGH);  // Shift out the next bit
      digitalWrite(clockPin, LOW);
    }
    Serial.println();
    delay(10);
  }
}

void setup() {
  M5.begin();
  Serial.begin(115200);
  lcd.init();

  pinMode(dataPin, INPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  lcd.setTextColor(YELLOW);
  lcd.setTextSize(2);
  lcd.setCursor(65, 10);
  lcd.println("M5 StepSequencer");
  lcd.setTextSize(3);
  for (int i = 0; i < 4; i++) {
    digitalWrite(outPort[i], LOW);
    pinMode(outPort[i], OUTPUT);
  }

  pinMode(15, OUTPUT);
  xTaskCreatePinnedToCore(SwScan, "SwScan", 4096, NULL, 3, &thp[0], 0);
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
}

void loop() {
  int oneStep = 60000 / bpm / (perNote / 4);
  if (millis() > oldTime + oneStep) {
    nextStep();
  }
  perTime();
}