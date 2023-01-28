#define LGFX_M5STACK
#define LGFX_USE_V1

#include <Arduino.h>
#include <M5Stack.h>
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>
#include <FastLED.h>
#include "driver/pcnt.h"

#define NUM_LEDS 64
#define DATA_PIN 21

#define PULSE_PIN_CLK 35
#define PULSE_PIN_DT 36

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
int cursor;

int buffer[NUM_SW];

int bpm = 120;
int oldbpm;
int lBpm = 40;
int hBpm = 400;

int oldTime;
int perNote = 16;
int gate[4] = { 80, 80, 80, 80 };
int oldGate[4];

int LY[5];

int play;

void nextStep() {
  oldTime = millis();
  for (int i = 0; i < numParts; i++) {
    if (!buffer[nowSteps + (i * 16)]) {
      leds[nowSteps + (i * 16)] = CRGB(0, 0, 0);
    }
  }
  // Serial.println(nowSteps);
  nowSteps++;
  if (nowSteps >= numSteps) nowSteps = 0;
  for (int i = 0; i < numParts; i++) {
    if (!buffer[nowSteps + (i * 16)]) {
      leds[nowSteps + (i * 16)] = CRGB(0, 0, 10);
    }
    digitalWrite(outPort[i], buffer[nowSteps + (i * 16)]);
    // Serial.print(buffer[nowSteps + (i * 16)]);
  }
  // Serial.println("");
  FastLED.show();
}

void stop() {
  for (int i = 0; i < numParts; i++) {
    if (!buffer[nowSteps + (i * 16)]) {
      leds[nowSteps + (i * 16)] = CRGB(0, 0, 0);
    }
  }
  FastLED.show();
  nowSteps = 16;
  play = 0;
  lcd.fillRect(260,180,40,40,RED);
}

void start() {
  play = 1;
    lcd.fillRect(260,180,40,40,BLACK);
    lcd.fillTriangle  ( 260, 180, 260, 219, 299, 199, YELLOW);
}

void SwScan(void *args) {
  while (1) {
    enc();
    //シフトレジスタから値を持ってくる
    digitalWrite(latchPin, LOW);
    digitalWrite(latchPin, HIGH);

    //ビットシフトしながら読んでいく
    for (int i = NUM_SW - 1; i >= 0; i--) {
      nowSw[i] = !digitalRead(dataPin);  //反転して今の状態に格納
      if (nowSw[i]) {                    //押されているなら
        if (oldSw[i] != nowSw[i]) {      //一つ前に押されてなかったら
          oldSw[i] = nowSw[i];
          buffer[i] = !buffer[i];  //スイッチの状態を変える
          if (i < NUM_LEDS) {
            leds[i] = CRGB(buffer[i] * 30, buffer[i] * 30, 0);
          }
          if (i == NUM_SW - 1) {
            lcd.drawRect(94, LY[cursor] - 20, 51, 36, BLACK);
            cursor++;
            if (cursor == 5) cursor = 0;
            lcd.drawRect(94, LY[cursor] - 20, 51, 36, WHITE);
          }
        }
      } else if (oldSw[i]) {                 //押されてなくて一個前が押されてたら
        oldSw[i] = nowSw[i];                 //ただ値を入れる
        if (i == NUM_SW - 1) buffer[i] = 0;  //最後のスイッチの時だけモーメンタリ動作させる
      }
      // Serial.print(buffer[i]);
      digitalWrite(clockPin, HIGH);  // 次のビットへ
      digitalWrite(clockPin, LOW);
    }
    if(!buffer[NUM_SW - 2] && play) stop();
    else if(buffer[NUM_SW - 2] && !play) start();
    // Serial.println();
    FastLED.show();
    delay(10);
  }
}


void enc() {
  int16_t count = 0;
  pcnt_get_counter_value(PCNT_UNIT_0, &count);

  if (cursor == 0) {
    bpm = bpm + count;
    if (bpm > hBpm) bpm = hBpm;
    if (bpm < lBpm) bpm = lBpm;
  }
  if (bpm != oldbpm) {
    oldbpm = bpm;
    lcd.setTextPadding(40);
    lcd.setTextDatum(textdatum_t::middle_right);
    lcd.drawNumber(bpm, 140, LY[0]);
  }

  if (cursor > 0) {
    gate[cursor - 1] = gate[cursor - 1] + count;
    if (gate[cursor - 1] > 100) gate[cursor - 1] = 100;
    if (gate[cursor - 1] < 0) gate[cursor - 1] = 0;
  }

  for (int i = 0; i < 4; i++) {
    if (gate[i] != oldGate[i]) {
      oldGate[i] = gate[i];
      lcd.setTextPadding(40);
      lcd.setTextDatum(textdatum_t::middle_right);
      lcd.drawNumber(gate[i], 140, LY[i + 1]);
    }
  }
  pcnt_counter_clear(PCNT_UNIT_0);
}

void setup() {
  M5.begin();
  Serial.begin(115200);
  lcd.init();

  pinMode(dataPin, INPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);

  //パルスカウンタ設定
  pcnt_config_t pcnt_config1 = {};
  pcnt_config1.pulse_gpio_num = PULSE_PIN_CLK;
  pcnt_config1.ctrl_gpio_num = PULSE_PIN_DT;
  pcnt_config1.lctrl_mode = PCNT_MODE_KEEP;
  pcnt_config1.hctrl_mode = PCNT_MODE_REVERSE;
  pcnt_config1.pos_mode = PCNT_COUNT_INC;
  pcnt_config1.neg_mode = PCNT_COUNT_DEC;
  pcnt_config1.counter_h_lim = 32767;
  pcnt_config1.counter_l_lim = -32768;
  pcnt_config1.unit = PCNT_UNIT_0;
  pcnt_config1.channel = PCNT_CHANNEL_0;

  pcnt_config_t pcnt_config2 = {};
  pcnt_config2.pulse_gpio_num = PULSE_PIN_DT;
  pcnt_config2.ctrl_gpio_num = PULSE_PIN_CLK;
  pcnt_config2.lctrl_mode = PCNT_MODE_REVERSE;
  pcnt_config2.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_config2.pos_mode = PCNT_COUNT_INC;
  pcnt_config2.neg_mode = PCNT_COUNT_DEC;
  pcnt_config2.counter_h_lim = 32767;
  pcnt_config2.counter_l_lim = -32768;
  pcnt_config2.unit = PCNT_UNIT_0;
  pcnt_config2.channel = PCNT_CHANNEL_1;

  pcnt_unit_config(&pcnt_config1);
  pcnt_unit_config(&pcnt_config2);

  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);
  //


  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  lcd.setFont(&fonts::Font4);
  lcd.setTextColor(0xFFFF00U, 0x000000U);
  lcd.setTextSize(1);
  lcd.setTextDatum(textdatum_t::middle_center);
  lcd.drawString("M5 StepSequencer", lcd.width() / 2, 15);

  LY[0] = 60;
  for (int i = 1; i < 5; i++) {
    LY[i] = LY[0] + (30 * (i + 1));
  }

  lcd.setTextDatum(textdatum_t::middle_left);
  lcd.drawString("BPM  :", 15, LY[0]);
  lcd.drawString("Gate1:", 15, LY[1]);
  lcd.drawString("Gate2:", 15, LY[2]);
  lcd.drawString("Gate3:", 15, LY[3]);
  lcd.drawString("Gate4:", 15, LY[4]);

  lcd.drawRect(94, LY[cursor] - 20, 51, 36, WHITE);

  for (int i = 0; i < 4; i++) {
    digitalWrite(outPort[i], LOW);
    pinMode(outPort[i], OUTPUT);
  }

  pinMode(15, OUTPUT);

  stop();

  xTaskCreatePinnedToCore(SwScan, "SwScan", 4096, NULL, 3, &thp[0], 0);
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();

}

void loop() {
  int oneStep = 60000 / (bpm * 1.00) / (perNote / 4);
  for (int i = 0; i < 4; i++) {
    int gateOff = oneStep * gate[i] / 100;
    if (millis() >= oldTime + gateOff && digitalRead(outPort[i])) {
      digitalWrite(outPort[i], LOW);
    }
  }
  if (play) {
    if (millis() > oldTime + oneStep) {
      nextStep();
    }
  }
}