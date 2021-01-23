// Robot 2020
// Arduino code for Team 20's 2020 FIRST Robotics Competition Robot

#include "I2C.h"
#include "PixyCam.h"
#include "MainLEDs.h"
#include "ShooterLEDs.h"

void setup() {
//  Serial.begin(9600);
  I2C::initialize(0x1);
  PixyCam::initialize();
  MainLEDs::initialize(4, 33, 255);
  ShooterLEDs::initialize(6, 15, 255);
}

void loop() {
  PixyCam::refresh();
  MainLEDs::refresh(I2C::getMainLEDMode(), I2C::getMainLEDValue());
  ShooterLEDs::refresh(I2C::getShooterLEDMode(), I2C::getShooterLEDValue());
  I2C::setWriteData(PixyCam::getTargetInView(), PixyCam::getXValue(), PixyCam::getDistance());
}
