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

ActuatorUI::ActuatorUI(Actuator *actuator) : actuator(actuator) {}
void ActuatorUI::begin()
{
  ESPUI.addControl(ControlType::Separator, "Driver State");
  driverVelocityControl = ESPUI.label("TSTEP - driver velocity", ControlColor::Carrot, String());
  stallValueControl = ESPUI.label("SG_RESULT - driver stall value", ControlColor::Carrot, String());

  ESPUI.addControl(ControlType::Separator, "Actuator State");
  stateValueControl = ESPUI.label("Actuator State", ControlColor::Sunflower, String());
  activeProfileControl = ESPUI.label("Active Profile", ControlColor::Sunflower, String());

  homedControl = ESPUI.label("Homed?", ControlColor::Sunflower, String());
  minPositionControl = ESPUI.label("Min Position", ControlColor::Sunflower, String());
  maxPositionControl = ESPUI.label("Max Position", ControlColor::Sunflower, String());
  currentPositionControl = ESPUI.label("Current Position", ControlColor::Sunflower, String());
  targetPositionControl = ESPUI.label("Target Position", ControlColor::Sunflower, String());
  velocityControl = ESPUI.label("Current Velocity", ControlColor::Sunflower, String());
  accelerationControl = ESPUI.label("Current Acceleration", ControlColor::Sunflower, String());
}

void ActuatorUI::update()
{
  ESPUI.updateLabel(driverVelocityControl, String(actuator->getDriverVelocity()));
  ESPUI.updateLabel(stallValueControl, String(actuator->getDriverStallValue()));

  ESPUI.updateLabel(stateValueControl, String(STATE_NAMES[actuator->getState()]));
  ESPUI.updateLabel(activeProfileControl, String(actuator->getActiveProfile() ? actuator->getActiveProfile()->displayName : "None"));

  ESPUI.updateLabel(homedControl, String(actuator->isHomed() ? "true" : "false"));
  ESPUI.updateLabel(minPositionControl, String(actuator->getMin()));
  ESPUI.updateLabel(maxPositionControl, String(actuator->getMax()));
  ESPUI.updateLabel(currentPositionControl, String(actuator->getCurrentPosition()));
  ESPUI.updateLabel(targetPositionControl, String(actuator->getTargetPosition()));
  ESPUI.updateLabel(velocityControl, String(actuator->getVelocity()));
  ESPUI.updateLabel(accelerationControl, String(actuator->getAcceleration()));
}

ActuatorControlUI::ActuatorControlUI(Actuator *actuator) : actuator(actuator) {}

void ActuatorControlUI::begin()
{
  ESPUI.addControl(ControlType::Separator, "Actuator Control");
  disableControl = ESPUI.button(
      "Enable",
      [this](Control *sender, int event)
      {
        actuator->disable();
      },
      ControlColor::Carrot,
      String());
  stopControl = ESPUI.button(
      "Stop",
      [this](Control *sender, int event)
      {
        actuator->stop();
      },
      ControlColor::Carrot,
      String());
  stopControl = ESPUI.button(
      "Force Stop",
      [this](Control *sender, int event)
      {
        actuator->forceStop();
      },
      ControlColor::Carrot,
      String());
  homeControl = ESPUI.button(
      "Home",
      [this](Control *sender, int event)
      {
        actuator->home();
      },
      ControlColor::Carrot,
      String());
  moveToControl = ESPUI.number(
      "Move Absolute",
      [this](Control *sender, int event)
      {
        actuator->moveTo(sender->value.toInt());
      },
      ControlColor::Carrot,
      actuator->getCurrentPosition(),
      actuator->getMin(),
      actuator->getMax());
  moveControl = ESPUI.number(
      "Move Relative",
      [this](Control *sender, int event)
      {
        actuator->move(sender->value.toInt());
      },
      ControlColor::Carrot,
      0,
      INT_MIN,
      INT_MAX); // todo between -TOTAL_TRAVEL and TOTAL_TRAVEL
  setZeroControl = ESPUI.button(
      "Set Zero",
      [this](Control *sender, int event)
      {
        actuator->setZero();
      },
      ControlColor::Carrot,
      String());
}

void ActuatorControlUI::update()
{
}

WebUI::WebUI(Actuator *actuator) : actuator(actuator)
{
  actuatorUI = new ActuatorUI(actuator);
  actuatorControlUI = new ActuatorControlUI(actuator);
  runningPositiveUI = new MotionProfileUI("Running Positive Profile", actuator->runningPositiveProfile);
  runningNegativeUI = new MotionProfileUI("Running Negative Profile", actuator->runningNegativeProfile);
  homingUI = new MotionProfileUI("Homing Profile", actuator->homingProfile);
}
WebUI::~WebUI()
{
  delete actuatorUI;
  delete actuatorControlUI;
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
  actuatorUI->update();
  actuatorControlUI->update();
}
void WebUI::begin()
{
  actuatorUI->begin();
  actuatorControlUI->begin();
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
