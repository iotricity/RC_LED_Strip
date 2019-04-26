#include <Arduino.h>
#include <FastLED.h>            // Get this library from your Library Manager (Menu-bar -> Sketch -> Include Library -> Manage Libraries -> Search and install FastLED).

// I/O Pin definitions
#define RX_THRO        2        // Receiver Throttle channel connected to D2.
#define RX_AUX         3        // Receiver Aux (or any other) channel connected to D3.
#define LED_LEFT       4        // Data out for left LED strip D4.
#define LED_RIGHT      5        // Data out for right LED strip D5.
#define SER_DEBUG      6        // Use D6 for serial debug output when connected to GND.

#define SERVO_MIN   1100        // Minimum channel servo position in microseconds.
#define SERVO_MAX   1850        // Maximum channel servo position in microseconds.

#define NUM_LEDS      21        // The number of LEDs in a single LED strip (left and right must be of equal length).
#define CHIPSET       WS2812B   // Type of LEDs used in LED strip (see FastLED library for other types).
#define COLOR_ORDER   GRB       // The color order for the type of LEDs used.
#define BRIGHTNESS   240        // Maximum brightness of the LEDs (0-255).
#define FPS           30        // The number of updates per second for the LED strip (frames/second).
#define MAX_CURRENT  500        // The (calculated) total current limit in milli-Amps for all LEDs together (LEDs will be 
                                // dimmed if exceeded). Use an additional BEC for sufficient power if not limiting the current and
                                // change this value to the current limit of the BEC.

// Create two arrays for LED data.
CRGB leds[2][NUM_LEDS];

// Variables for PWM measurement.
unsigned long rxThroMicros = 0, rxThroTemp;
boolean rxThroPrevState;
unsigned long rxAuxMicros = 0, rxAuxTemp;
boolean rxAuxPrevState;

// Misc. variables.
boolean ledState;
boolean serDebug;
unsigned long serDelay;
unsigned long fpsDelay;

// Dummy variable for formatting debug text.
char txtDummy[32];


void setup() {
  // Configure port directions and internal resistor pullups.
  pinMode(RX_THRO, INPUT_PULLUP);
  pinMode(RX_AUX, INPUT_PULLUP);
  pinMode(SER_DEBUG, INPUT_PULLUP);
  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);

  // Configure LED arrays, color correction and current limits.
  FastLED.addLeds<CHIPSET, LED_LEFT, COLOR_ORDER>(leds[0], NUM_LEDS).setCorrection(TypicalSMD5050);
  FastLED.addLeds<CHIPSET, LED_RIGHT, COLOR_ORDER>(leds[1], NUM_LEDS).setCorrection(TypicalSMD5050);
  FastLED.setTemperature(Halogen);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, MAX_CURRENT);
  FastLED.setBrightness(BRIGHTNESS);

  // Start serial port for debugging output.
  Serial.begin(115200);
  serDebug = !digitalRead(SER_DEBUG);
  if(serDebug) {
    Serial.println("Rx Monitor Started...");
  }

  // Attach interrupt routines to detect PWM signals.
  attachInterrupt(digitalPinToInterrupt(RX_THRO), rxProcessThro, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RX_AUX), rxProcessAux, CHANGE);
}

void loop() {
  // Output serial debug data every 1 second if debug mode is enabled (pin D6 to GND).
  // Use the debug mode to determine the minimum and maximum PWM signals of the receiver. Change the 
  // values of SERVO_MIN and SERVO_MAX in the #define section to these values.
  if(serDebug) {
    if(millis() - serDelay >= 1000) {
      serDelay = millis();
      sprintf(txtDummy, "THRO: %4d - AUX: %4d", rxThroMicros, rxAuxMicros);
      Serial.println(txtDummy);
    }
  }

  // Update LED data based on set frames per second.
  if(millis() - fpsDelay >= 1000/FPS) {
    fpsDelay = millis();
    
    // Clear the arrays.
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[0][i] = CRGB::Black;
      leds[1][i] = CRGB::Black;
    }
    // Remap servo position to LED number, show current position and blink LEDs.
    if(ledState) {
      leds[0][map(rxThroMicros, SERVO_MIN, SERVO_MAX, 0, NUM_LEDS - 1)] = CRGB::Red;
      leds[1][map(rxThroMicros, SERVO_MIN, SERVO_MAX, 0, NUM_LEDS - 1)] = CRGB::Violet;
    } else {
      leds[0][map(rxAuxMicros, SERVO_MIN, SERVO_MAX, 0, NUM_LEDS - 1)] = CRGB::Green;
      leds[1][map(rxAuxMicros, SERVO_MIN, SERVO_MAX, 0, NUM_LEDS - 1)] = CRGB::Yellow;
    }
    // Toggle blinking.
    ledState = !ledState;
    // Output arrays to LED strips.
    FastLED.show();
  }
}

// Measure the PWM pulse length in microseconds for the THRO channel. On the rising edge, the timing starts. On the
// falling edge the measurement stops and the difference between the start and stop is put in the variable rxThroMicros.
void rxProcessThro() {
  unsigned long currMicros = micros();  // Capture current microseconds.
  boolean currState = digitalRead(RX_THRO);
  if(currState != rxThroPrevState) {  // Check if the state of the input signal has changed (should always be the fact).
    rxThroPrevState = currState; 
    if(currState == HIGH) {  // Check if the signal is the rising edge to reset the counter.
      rxThroTemp = currMicros;
    } else {  // If the signal is the falling edge, calculate the difference between the current microseconds and the start.
      rxThroMicros = currMicros - rxThroTemp;
    }
  }
}

// Measure the PWM pulse length in microseconds for the AUX channel. On the rising edge, the timing starts. On the
// falling edge the measurement stops and the difference between the start and stop is put in the variable rxAuxMicros.
void rxProcessAux() {
  unsigned long currMicros = micros();  // Capture current microseconds.
  boolean currState = digitalRead(RX_AUX);
  if(currState != rxAuxPrevState) {  // Check if the state of the input signal has changed (should always be the fact).
    rxAuxPrevState = currState;
    if(currState == HIGH) {  // Check if the signal is the rising edge to reset the counter.
      rxAuxTemp = currMicros;
    } else {  // If the signal is the falling edge, calculate the difference between the current microseconds and the start.
      rxAuxMicros = currMicros - rxAuxTemp;
    }
  }
}

