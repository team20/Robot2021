#include "I2C.h"

static byte I2C::readData[4];
static byte I2C::writeData[5];

static void I2C::initialize(byte address) {
  Wire.begin(address);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  for (byte i = 0; i < sizeof(readData) / sizeof(byte); i++)
    readData[i] = 0;
  for (byte i = 0; i < sizeof(writeData) / sizeof(byte); i++)
    writeData[i] = 0;
}

static void I2C::receiveEvent() {
  for (byte i = 0; Wire.available() > 0 && i < sizeof(readData) / sizeof(byte); i++)
    readData[i] = Wire.read();
}

static void I2C::requestEvent() {
  Wire.write(writeData, sizeof(writeData) / sizeof(byte));
}

static byte I2C::getMainLEDMode() {
  return readData[0];
}

static byte I2C::getMainLEDValue() {
  return readData[1];
}

static byte I2C::getShooterLEDMode() {
  return readData[2];
}

static byte I2C::getShooterLEDValue() {
  return readData[3];
}

static void I2C::setWriteData(bool targetInView, short xValue, byte distance) {
  writeData[0] = targetInView ? 1 : 0;
  splitValue(xValue, 1, 3);
  writeData[4] = distance;
}

static void I2C::splitValue(short value, byte startIndex, byte endIndex) {
  for (byte i = startIndex; i <= endIndex; i++) {
    if (value <= 127) {
      writeData[i] = value;
      for (byte j = i + 1; j <= endIndex; j++)
        writeData[j] = 0;
      break;
    } else {
      writeData[i] = 127;
      value -= 127;
    }
  }
}
