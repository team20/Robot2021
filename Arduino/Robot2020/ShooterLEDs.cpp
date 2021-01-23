#include "ShooterLEDs.h"

const uint32_t ShooterLEDs::off = strip.Color(0, 0, 0);
const uint32_t ShooterLEDs::red = strip.Color(255, 0, 0);
const uint32_t ShooterLEDs::orange = strip.Color(255, 0, 31);
const uint32_t ShooterLEDs::yellow = strip.Color(255, 0, 127);
const uint32_t ShooterLEDs::green = strip.Color(0, 0, 255);
const uint32_t ShooterLEDs::blue = strip.Color(0, 255, 0);
const uint32_t ShooterLEDs::purple = strip.Color(127, 255, 0);
const uint32_t ShooterLEDs::white = strip.Color(255, 255, 255);

static Adafruit_NeoPixel ShooterLEDs::strip;

static unsigned long ShooterLEDs::timestamp;
static byte ShooterLEDs::counter;
static byte ShooterLEDs::prevMode;

static void ShooterLEDs::initialize(byte pin, byte numLEDs, byte brightness) {
  strip = Adafruit_NeoPixel(numLEDs, pin, NEO_GRB + NEO_KHZ800);
  strip.begin();
  strip.setBrightness(brightness);
  strip.show();

  counter = 0;
  timestamp = millis();
  prevMode = 0;
}

static void ShooterLEDs::refresh(byte mode, byte value) {
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
      flywheelPercent(value);
      break;
  }
  // send data to LED strip
  strip.show();
  // store previous mode for future comparison
  prevMode = mode;
}

static void ShooterLEDs::allOff() {
  for (byte i = 0; i < strip.numPixels(); i ++)
    strip.setPixelColor(i, off);
}

static void ShooterLEDs::flywheelPercent(byte percent) {
  double percentDouble = percent / 100.0;
  byte numPixelsOn = percentDouble * strip.numPixels();
  for (byte i = 0; i < strip.numPixels(); i++)
    if (i < numPixelsOn)
      strip.setPixelColor(i, strip.Color((1.0 - percentDouble) * 255, percentDouble * 255, 0));
    else
      strip.setPixelColor(i, off);
}
