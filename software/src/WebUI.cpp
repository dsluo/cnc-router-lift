#include "WebUI.h"
#include <ESPUI.h>

MotionProfileUI::MotionProfileUI(const char *title, MotionProfile *profile)
    : title(title), profile(profile) {}

void MotionProfileUI::begin()
{
  ESPUI.addControl(ControlType::Separator, title);
  ESPUI.number(
      "Velocity",
      [this](Control *sender, int event)
      {
        profile->velocity = sender->value.toInt();
      },
      ControlColor::None,
      profile->velocity,
      INT_MIN,
      INT_MAX);
  ESPUI.number(
      "Acceleration",
      [this](Control *sender, int event)
      {
        profile->acceleration = sender->value.toInt();
      },
      ControlColor::None,
      profile->acceleration,
      INT_MIN,
      INT_MAX);
  ESPUI.slider(
      "Stall Threshold",
      [this](Control *sender, int event)
      {
        profile->stallThreshold = sender->value.toInt();
      },
      ControlColor::None,
      profile->stallThreshold,
      0,
      255);
  ESPUI.slider(
      "Velocity Threshold",
      [this](Control *sender, int event)
      {
        profile->velocityThreshold = sender->value.toInt();
      },
      ControlColor::None,
      profile->velocityThreshold,
      0,
      1000);
}

WebUI::WebUI(Actuator *actuator) : actuator(actuator)
{
  runningPositiveUI = new MotionProfileUI("Running Positive Profile", actuator->runningPositiveProfile);
  runningNegativeUI = new MotionProfileUI("Running Negative Profile", actuator->runningNegativeProfile);
  homingUI = new MotionProfileUI("Homing Profile", actuator->homingProfile);
}
WebUI::~WebUI()
{
  delete runningPositiveUI;
  delete runningNegativeUI;
  delete homingUI;
}
void WebUI::startTask(void *instance)
{
  while (true)
  {
    ((WebUI *)instance)->loop();
  }
}
void WebUI::loop()
{
  ESPUI.updateLabel(driverVelocityControl, String(actuator->driver->TSTEP()));
  ESPUI.updateLabel(stallValueControl, String(actuator->driver->SG_RESULT()));
}
void WebUI::begin()
{
  driverVelocityControl = ESPUI.label("TSTEP", ControlColor::Carrot, String(actuator->driver->TSTEP()));
  stallValueControl = ESPUI.label("SG_RESULT", ControlColor::Carrot, String(actuator->driver->SG_RESULT()));

  runningPositiveUI->begin();
  runningNegativeUI->begin();
  homingUI->begin();

  ESPUI.begin("Router Lift");

  xTaskCreate(
      WebUI::startTask,
      "WebUITask",
      1024 * 4,
      this,
      1,
      NULL);
}
