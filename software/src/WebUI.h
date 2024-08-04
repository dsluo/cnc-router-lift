#include "Actuator.h"

#ifndef WEBUI_H
#define WEBUI_H

class MotionProfileUI
{
  const char *title;
  MotionProfile *profile;
  uint16_t velocityControl;
  uint16_t accelerationControl;
  uint16_t stallThresholdControl;
  uint16_t velocityThresholdControl;

public:
  MotionProfileUI(const char *title, MotionProfile *profile);

  void begin();
};

class WebUI
{
  Actuator *actuator;
  uint16_t driverVelocityControl;
  uint16_t stallValueControl;

  MotionProfileUI *runningPositiveUI;
  MotionProfileUI *runningNegativeUI;
  MotionProfileUI *homingUI;

  static void startTask(void *instance);
  void loop();
public:
  WebUI(Actuator *actuator);
  ~WebUI();

  void begin();
};

#endif
