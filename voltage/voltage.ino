#include <FastLED.h>

const int analogPin = A0;  // Analog pin for reading voltage
const int numLeds = 4;     // Number of LEDs in the strip
const int ledPin = 6;      // Data pin for WS2812B LED strip
CRGB leds[numLeds];

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);

  // Initialize FastLED with the correct type and number of LEDs
  FastLED.addLeds<WS2812B, ledPin, GRB>(leds, numLeds);

  // Test LED strip by turning all LEDs on and off
  fill_solid(leds, numLeds, CRGB::White);
  FastLED.show();
  delay(500);
  fill_solid(leds, numLeds, CRGB::Black);
  FastLED.show();
  delay(500);
}

void loop() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(analogPin);

  // Convert the analog reading (which goes from 0 - 1023) to a voltage (9.6V - 11.4V):
  float voltage = sensorValue * (5.0 / 1200.0) * 11.03 / 0.95;

  // print out the value you read:
  Serial.println(voltage);

  // Map voltage to LED brightness
  // int brightness = map(sensorValue, 0, 1023, 0, 255);
  int brightness=40;
  // Set LED brightness and color based on voltage
  if (voltage <= 10) {
    fill_solid(leds, numLeds, CRGB::Red);
  } else if (voltage <= 10.325) {
    fill_solid(leds, numLeds, CRGB::Yellow);
  } else if (voltage <= 10.85) {
    fill_solid(leds, numLeds, CRGB::Green);
  } else if (voltage <= 11.375) {
    fill_solid(leds, numLeds, CRGB::Blue);
  } else {
    fill_solid(leds, numLeds, CRGB::White);
  }

  // Set LED brightness
  FastLED.setBrightness(brightness);

  // Show the LED strip
  FastLED.show();

  // Wait for a moment
  delay(1000);
}
