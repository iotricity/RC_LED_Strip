#include <Arduino.h>
#include <FastLED.h>

#define RX_THRO        2        // Receiver Throttle channel connected to D2
#define RX_AUX         3        // Receiver Aux (or any other) channel connected to D3
#define SERVO_MIN   1100        // Minimum channel servo position in microseconds
#define SERVO_MAX   1850        // Maximum channel servo position in microseconds
#define LED_LEFT       4        // Data out for left LED strip
#define LED_RIGHT      5        // Data out for right LED strip
#define SER_DEBUG      6        // Use D6 for serial debug output when connected to GND
#define NUM_LEDS      21        // The number of LEDs in a single LED strip (left and right must be of equal length)
#define CHIPSET       WS2812B   // Type of LEDs used in LED strip (see FastLED library for other types)
#define COLOR_ORDER   GRB       // The color order for the type of LEDs used
#define BRIGHTNESS   240        // Maximum brightness of the LEDs (0-255)
#define FPS           30        // The number of updates per second for the LED strip (frames/second)
#define MAX_CURRENT  500        // The (calculated) total current limit for all LEDs together (LEDs will be dimmed if exceeded)

CRGB leds[2][NUM_LEDS];

unsigned long rxThroMicros = 0, rxThroTemp;
boolean rxThroPrevState;
unsigned long rxAuxMicros = 0, rxAuxTemp;
boolean rxAuxPrevState;

boolean ledState;
boolean serDebug;
unsigned long serDelay;
unsigned long fpsDelay;

char txtDummy[32];


void setup() {
  pinMode(RX_THRO, INPUT_PULLUP);
  pinMode(RX_AUX, INPUT_PULLUP);
  pinMode(SER_DEBUG, INPUT_PULLUP);
  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);

  delay(10);
  FastLED.addLeds<CHIPSET, LED_LEFT, COLOR_ORDER>(leds[0], NUM_LEDS).setCorrection(TypicalSMD5050);
  FastLED.addLeds<CHIPSET, LED_RIGHT, COLOR_ORDER>(leds[1], NUM_LEDS).setCorrection(TypicalSMD5050);
  FastLED.setTemperature(Halogen);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, MAX_CURRENT);
  FastLED.setBrightness(BRIGHTNESS);

  Serial.begin(115200);
  serDebug = !digitalRead(SER_DEBUG);
  if(serDebug) {
    Serial.println("Rx Monitor Started...");
  }
  attachInterrupt(digitalPinToInterrupt(RX_THRO), rxProcessThro, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RX_AUX), rxProcessAux, CHANGE);
}

void loop() {
  // Output serial debug data every 1 second if dedug is enabled (pin D6 to GND)
  if(serDebug && millis() - serDelay >= 1000) {
    serDelay = millis();
    sprintf(txtDummy, "THRO: %4d - AUX: %4d", rxThroMicros, rxAuxMicros);
    Serial.println(txtDummy);
  }

  // Update LED data
  if(millis() - fpsDelay >= 1000/FPS) {
    fpsDelay = millis();
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[0][i] = CRGB::Black;
      leds[1][i] = CRGB::Black;
    }
    // Remap servo position to LED number, show current position and blink LEDs
    if(ledState) {
      leds[0][map(rxThroMicros, SERVO_MIN, SERVO_MAX, 0, NUM_LEDS - 1)] = CRGB::Red;
    } else {
      leds[0][map(rxAuxMicros, SERVO_MIN, SERVO_MAX, 0, NUM_LEDS - 1)] = CRGB::Green;
    }
    ledState = !ledState;
    FastLED.show();
  }
}


void rxProcessThro() {
  unsigned long currMicros = micros();
  boolean currState = digitalRead(RX_THRO);
  if(currState != rxThroPrevState) {
    rxThroPrevState = currState;
    if(currState == HIGH) {
      rxThroTemp = currMicros;
    } else {
      rxThroMicros = currMicros - rxThroTemp;
    }
  }
}

void rxProcessAux() {
  unsigned long currMicros = micros();
  boolean currState = digitalRead(RX_AUX);
  if(currState != rxAuxPrevState) {
    rxAuxPrevState = currState;
    if(currState == HIGH) {
      rxAuxTemp = currMicros;
    } else {
      rxAuxMicros = currMicros - rxAuxTemp;
    }
  }
}

