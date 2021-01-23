#include "PixyCam.h"

static Pixy2 PixyCam::pixy;
static PIDLoop PixyCam::pan = PIDLoop(400, 0, 400, true);
static PIDLoop PixyCam::tilt = PIDLoop(500, 0, 500, true);
static bool PixyCam::targetInView;
static short PixyCam::xValue;
static byte PixyCam::distance;

static void PixyCam::initialize() {
  pixy.init();
  pixy.changeProg("color");
  pixy.setCameraBrightness(50);
  reset();
}

static void PixyCam::refresh() {
  // get block data
  pixy.ccc.getBlocks(1);
  if (pixy.ccc.numBlocks) {
    if (!targetInView)
      targetInView = true;
    // update values
    xValue = pixy.ccc.blocks[0].m_x;
    short width = pixy.ccc.blocks[0].m_width;
    double angle = (double)width / (double)pixy.frameWidth * (PI / 3.0);
    // distance = 3.5 * sin(90 - angle / 2.0) * (1.0 / sin(angle / 2.0));
    distance = 3.5 / tan(angle / 2);
  } else if (targetInView)
    targetInView = false;
}

static void PixyCam::reset() {
  // reset servo PID loops to set camera position to center
  pan.reset();
  tilt.reset();
  pixy.setServos(500, 750);
}

static bool PixyCam::getTargetInView() {
  return targetInView;
}

static short PixyCam::getXValue() {
  return xValue;
}

static short PixyCam::getDistance() {
  return distance;
}
