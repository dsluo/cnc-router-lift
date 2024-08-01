#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>

#ifndef ACTUATOR_H
#define ACTUATOR_H

class Actuator
{
    TMC2209Stepper *driver;
    FastAccelStepper *stepper;
    int stallPin;

    int minPos = INT_MAX;
    int maxPos = INT_MIN;
    // volatile because core 0 handles the interrupt, while core 1 handles response?
    volatile bool stalled;

    // StallGuard threshold during normal operation.

    // if driver.SG_RESULT() falls below twice this value, stallguard is triggered.
    int runningMinStallThreshold;
    // velocity above which to enable stallguard
    // velocity can be retrieved using driver.TSTEP()
    // this value is the amount of time between two microsteps,
    // thus smaller values are faster.
    int runningMinVelocityThreshold;
    int runningMaxStallThreshold;
    int runningMaxVelocityThreshold;
    int homingMinStallThreshold;
    int homingMinVelocityThreshold;
    int homingMaxStallThreshold;
    int homingMaxVelocityThreshold;

    void resetStalled();

public:
    Actuator(
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
        int homingMaxVelocityThreshold);
    static void IRAM_ATTR setStalled(void *instance);
    static void ActuatorTask(void *instance);
    void begin();
};

#endif