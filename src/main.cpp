#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include "Actuator.h"

#define ENABLE_PIN 14
#define DIRECTION_PIN 27
#define STEP_PIN 12
#define STALL_PIN 13

#define DRIVER_SERIAL Serial2
#define DRIVER_RESIST 0.11f
#define DRIVER_ADDRESS 0b00

TMC2209Stepper driver(&DRIVER_SERIAL, DRIVER_RESIST, DRIVER_ADDRESS);

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

Actuator *actuator = NULL;

void setup()
{
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(STALL_PIN, INPUT);

  Serial.begin(115200);
  DRIVER_SERIAL.begin(115200);

  driver.begin();
  // idk what this does
  driver.blank_time(24);
  driver.rms_current(600);
  driver.microsteps(64);
  // disable coolstep; interferes with stallguard
  driver.semin(0);
  // disable stealthchop by setting threshold above which its enabled to zero.
  driver.TPWMTHRS(0);
  // enable UART control
  driver.pdn_disable(true);

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (stepper)
  {
    stepper->setDirectionPin(DIRECTION_PIN);
    stepper->setEnablePin(ENABLE_PIN);
    // enable when motion required.
    // disable when motion stopps.
    stepper->setAutoEnable(true);
  }

  // todo
  // actuator = new Actuator();
}

void loop()
{
}
