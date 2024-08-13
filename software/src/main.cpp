#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include <WiFi.h>

#include "Actuator.h"
#include "WebServer.h"

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

MotionProfile runningPositiveProfile = {
    "runningPositive",
    "Running Positive",
    10000,
    1000,
    100,
    250};
MotionProfile runningNegativeProfile = {
    "runningNegative",
    "Running Negative",
    10000,
    1000,
    100,
    250};
MotionProfile homingProfile = {
    "homing",
    "Homing",
    10000,
    1000,
    100,
    250};

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
    &runningPositiveProfile,
    &runningNegativeProfile,
    &homingProfile,
    Direction::NEGATIVE,
    TOTAL_TRAVEL_STEPS,
    false);

WebServer server(&actuator);

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

void setup()
{
  Serial.begin(115200);
  networkSetup();
  actuator.begin();
  server.begin();
}

void loop()
{
}
