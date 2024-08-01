#include "Actuator.h"

Actuator::Actuator(
    TMC2209Stepper *driver,
    FastAccelStepper *stepper,
    int stallPin,
    int runningMinStallThreshold,
    int runningMinVelocityThreshold,
    int runningMaxStallThreshold,
    int runningMaxVelocityThreshold,
    int homingMinStallThreshold,
    int homingMinVelocityThreshold,
    int homingMaxStallThreshold,
    int homingMaxVelocityThreshold)
{
    this->driver = driver;
    this->stepper = stepper;
    this->stallPin = stallPin;
    this->runningMinStallThreshold = runningMinStallThreshold;
    this->runningMinVelocityThreshold = runningMinVelocityThreshold;
    this->runningMaxStallThreshold = runningMaxStallThreshold;
    this->runningMaxVelocityThreshold = runningMaxVelocityThreshold;
    this->homingMinStallThreshold = homingMinStallThreshold;
    this->homingMinVelocityThreshold = homingMinVelocityThreshold;
    this->homingMaxStallThreshold = homingMaxStallThreshold;
    this->homingMaxVelocityThreshold = homingMaxVelocityThreshold;
}

void Actuator::begin()
{
    attachInterruptArg(digitalPinToInterrupt(stallPin), Actuator::setStalled, this, RISING);

    xTaskCreatePinnedToCore(
        Actuator::ActuatorTask,
        "ActuatorTask",
        1024 * 4, // todo: find high water mark of stack usage
        this,
        3, // todo: is this the right prio?
        NULL,
        1);
}

void IRAM_ATTR Actuator::setStalled(void *instance)
{
    ((Actuator *)instance)->stalled = true;
}

void Actuator::resetStalled()
{
    stalled = false;
}