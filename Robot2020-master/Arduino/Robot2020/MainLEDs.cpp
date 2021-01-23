#include "MainLEDs.h"

const uint32_t MainLEDs::off = strip.Color(0, 0, 0);
const uint32_t MainLEDs::red = strip.Color(255, 0, 0);
const uint32_t MainLEDs::orange = strip.Color(255, 0, 31);
const uint32_t MainLEDs::yellow = strip.Color(255, 0, 127);
const uint32_t MainLEDs::green = strip.Color(0, 0, 255);
const uint32_t MainLEDs::blue = strip.Color(0, 255, 0);
const uint32_t MainLEDs::purple = strip.Color(127, 255, 0);
const uint32_t MainLEDs::white = strip.Color(255, 255, 255);

static Adafruit_NeoPixel MainLEDs::strip;

static unsigned long MainLEDs::timestamp;
static byte MainLEDs::counter;
static byte MainLEDs::prevMode;

static void MainLEDs::initialize(byte pin, byte numLEDs, byte brightness) {
  strip = Adafruit_NeoPixel(numLEDs, pin, NEO_GRB + NEO_KHZ800);
  strip.begin();
  strip.setBrightness(brightness);
  strip.show();

  counter = 0;
  timestamp = millis();
  prevMode = 0;
}

static void MainLEDs::refresh(byte mode, byte value) {
  // reset if pattern changed
  if (mode != prevMode) {
    allOff();
    counter = 0;
    timestamp = millis();
  }
  // run correct animation
  switch (mode) {
    case 0:
      allOff();
      break;
    case 1:
      chasing(green);
      break;
  }
  // send data to LED strip
  strip.show();
  // store previous mode for future comparison
  prevMode = mode;
}

static void MainLEDs::allOff() {
  for (byte i = 0; i < strip.numPixels(); i ++)
    strip.setPixelColor(i, off);
}

static void MainLEDs::chasing(uint32_t color) {
  if (millis() - timestamp >= 80) {
    timestamp = millis();
    if (!(counter < 5))
      counter = 0;
    for (byte j = counter; j <= strip.numPixels(); j += 5) {
      strip.setPixelColor(j, color);
      strip.setPixelColor(j - 1, off);
    }
    counter++;
  }
}
