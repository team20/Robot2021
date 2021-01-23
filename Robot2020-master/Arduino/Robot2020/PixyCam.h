#ifndef PixyCam_h
#define PixyCam_h

#include "Arduino.h"
#include <Pixy2.h>
#include <PIDLoop.h>

class PixyCam {
  public:
    static void initialize();
    static void refresh();
    static bool getTargetInView();
    static short getXValue();
    static short getDistance();
    
  private:
    // Pixy2 object
    static Pixy2 pixy;
    // servo PID loops
    static PIDLoop pan;
    static PIDLoop tilt;
    // booleans for checking if blocks are detected
    static bool targetInView;
    // x-value of center of largest block
    static short xValue;
    // distance to power cell
    static byte distance;
    
    static void reset();
};

#endif
