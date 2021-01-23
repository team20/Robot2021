#ifndef I2C_h
#define I2C_h

#include "Arduino.h"
#include <Wire.h>

class I2C {
  public:
    static void initialize(byte address);
    static void receiveEvent();
    static void requestEvent();
    static byte getMainLEDMode();
    static byte getMainLEDValue();
    static byte getShooterLEDMode();
    static byte getShooterLEDValue();
    static void setWriteData(bool targetInView, short xValue, byte distance);
    
  private:
    // data received from RoboRio
    static byte readData[4];
    // data to send to RoboRio
    static byte writeData[5];

    static void splitValue(short value, byte startIndex, byte endIndex);
};

#endif
