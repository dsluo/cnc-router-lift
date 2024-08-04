#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include "Actuator.h"
#include <WiFi.h>
#include <ESPUI.h>

#define ENABLE_PIN 14
#define DIRECTION_PIN 27
#define STEP_PIN 12
#define STALL_PIN 13

#define DRIVER_SERIAL Serial2
#define DRIVER_RESIST 0.11f
#define DRIVER_ADDRESS 0b00

// 200 steps / rev
// 64 microsteps / step
// 4 starts * 2 mm pitch / rev
#define MICROSTEPS 64
#define STEPS_PER_UNIT_TRAVEL MICROSTEPS * 200 / (4 * 2)
#define TOTAL_TRAVEL 200

#define RMS_CURRENT 600

MotionProfile runningPositiveProfile = MotionProfile{
    10000, 1000, 100, 250};
MotionProfile runningNegativeProfile = {10000, 1000, 100, 250};
MotionProfile homingProfile = MotionProfile{10000, 1000, 100, 250};

Actuator actuator(
    &DRIVER_SERIAL,
    DRIVER_RESIST,
    DRIVER_ADDRESS,
    RMS_CURRENT,
    MICROSTEPS,
    STEP_PIN,
    DIRECTION_PIN,
    ENABLE_PIN,
    STALL_PIN,
    STEPS_PER_UNIT_TRAVEL,
    &runningPositiveProfile,
    &runningNegativeProfile,
    &homingProfile,
    TOTAL_TRAVEL);

void networkSetup()
{
  WiFi.setHostname("router-lift");
  WiFi.begin("ssid", "password");

  do
  {
    delay(500);
    Serial.print(".");
  } while (WiFi.status() != WL_CONNECTED);

  Serial.println(WiFi.localIP());
}

int driverVelocity;
int stallValue;
void uiSetup()
{
  driverVelocity = ESPUI.label("TSTEP", ControlColor::None, String(actuator.driver->TSTEP()));
  stallValue = ESPUI.label("SG_RESULT", ControlColor::None, String(actuator.driver->SG_RESULT()));
  ESPUI.addControl(ControlType::Separator, "Runing Positive Motion Profile");
  ESPUI.number(
      "Velocity",

      [](Control *sender, int event)
      {
        runningPositiveProfile.velocity = sender->value.toInt();
      },
      ControlColor::None,
      runningPositiveProfile.velocity,
      INT_MIN,
      INT_MAX);
  ESPUI.number(
      "Acceleration",
      [](Control *sender, int event)
      {
        runningPositiveProfile.acceleration = sender->value.toInt();
      },
      ControlColor::None,
      runningPositiveProfile.acceleration,
      INT_MIN,
      INT_MAX);
  ESPUI.slider(
      "Stall Threshold",
      [](Control *sender, int event)
      {
        runningPositiveProfile.stallThreshold = sender->value.toInt();
      },
      ControlColor::None,
      runningPositiveProfile.stallThreshold,
      0,
      255);
  ESPUI.slider(
      "Velocity Threshold",
      [](Control *sender, int event)
      {
        runningPositiveProfile.velocityThreshold = sender->value.toInt();
      },
      ControlColor::None,
      runningPositiveProfile.velocityThreshold,
      0, 1000);
}

void setup()
{
  ESPUI.setVerbosity(Verbosity::VerboseJSON);
  Serial.begin(115200);
  networkSetup();
  actuator.begin();
  uiSetup();
  ESPUI.begin("hello world");
}

void loop()
{
  ESPUI.updateLabel(driverVelocity, String(actuator.driver->TSTEP()));
  ESPUI.updateLabel(stallValue, String(actuator.driver->SG_RESULT()));
}
