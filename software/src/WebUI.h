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

class ActuatorUI
{
  Actuator *actuator;

  uint16_t driverVelocityControl;
  uint16_t stallValueControl;

  uint16_t stateValueControl;
  uint16_t activeProfileControl;

  uint16_t homedControl;
  uint16_t maxPositionControl;
  uint16_t minPositionControl;
  uint16_t currentPositionControl;
  uint16_t targetPositionControl;
  uint16_t velocityControl;
  uint16_t accelerationControl;

public:
  ActuatorUI(Actuator *actuator);
  void begin();
  void update();
};

class ActuatorControlUI
{
  Actuator *actuator;

  uint16_t disableControl;
  uint16_t stopControl;
  uint16_t forceStopControl;
  uint16_t homeControl;
  uint16_t moveToControl;
  uint16_t moveControl;
  uint16_t setZeroControl;
  uint16_t setCurrentPositionControl;

public:
  ActuatorControlUI(Actuator *actuator);

  void begin();
  void update();
};

class WebUI
{
  Actuator *actuator;

  ActuatorUI *actuatorUI;
  ActuatorControlUI *actuatorControlUI;
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
