#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include "RunningMedian.h"

#ifndef ACTUATOR_H
#define ACTUATOR_H

#define INTERRUPT_DISABLED DISABLED
#undef DISABLED

enum State
{
    DISABLED,
    HOMING_INIT,
    HOMING_MOVING,
    HOMING_STALLED,
    STOPPED,
    FORCE_STOPPED,
    MOVING,
    MOVING_STALLED
};

char const STATE_NAMES[8][16] = {
    "DISABLED",
    "HOMING_INIT",
    "HOMING_MOVING",
    "HOMING_STALLED",
    "STOPPED",
    "FORCE_STOPPED",
    "MOVING",
    "MOVING_STALLED"};

struct MotionProfile
{

    char const *const name;
    char const *const displayName;
    int acceleration;
    int velocity;
    // if driver.SG_RESULT() falls below twice this value, stallguard is triggered.
    uint8_t stallThreshold;
    // velocity above which to enable stallguard
    // velocity can be retrieved using driver.TSTEP()
    // this value is the amount of time between two microsteps,
    // thus smaller values are faster.
    uint32_t velocityThreshold;
};

class Actuator
{
    HardwareSerial *driverSerial;
    float driverSenseResistance;
    uint8_t driverAddress;
    TMC2209Stepper *driver;

    uint16_t rmsCurrent;
    uint16_t microsteps;
    uint8_t stepPin;
    uint8_t directionPin;
    uint8_t enablePin;
    uint8_t stallPin;
    FastAccelStepperEngine engine;
    FastAccelStepper *stepper;

    // Direction homingDirection;
    uint32_t totalTravel;

    // =====================================
    MotionProfile *activeProfile;

    bool homed = false;
    int32_t minPos = INT_MIN;
    int32_t maxPos = INT_MAX;
    int32_t targetPos = 0;
    // todo: maybe volatile because core 0 handles the interrupt, while core 1 handles response?
    volatile bool stalled = false;
    State state;

    RunningMedian driverVelocity = RunningMedian(20);
    RunningMedian stallValue = RunningMedian(20);

    static void IRAM_ATTR setStalled(void *instance);
    static void startTask(void *instance);
    void loop();

    void setMotionProfile(MotionProfile *parameters);

public:
    Actuator(
        HardwareSerial *driverSerial,
        float driverSenseResistance,
        uint8_t driverAddress,
        uint16_t rmsCurrent,
        uint16_t microsteps,
        uint8_t stepPin,
        uint8_t directionPin,
        uint8_t enablePin,
        uint8_t stallPin,
        MotionProfile *const runningPositiveProfile,
        MotionProfile *const runningNegativeProfile,
        MotionProfile *const homingProfile,
        uint32_t totalTravel);
    ~Actuator();
    void begin();

    uint32_t getDriverVelocity();
    float getMedianDriverVelocity();
    uint16_t getDriverStallValue();
    float getMedianDriverStallValue();

    State getState();
    MotionProfile *getActiveProfile();
    MotionProfile *const runningPositiveProfile;
    MotionProfile *const runningNegativeProfile;
    MotionProfile *const homingProfile;
    MotionProfile *const allProfiles[3];

    bool isHomed();
    int32_t getMin();
    int32_t getMax();
    int32_t getCurrentPosition();
    int32_t getTargetPosition();
    int32_t getVelocity();
    int32_t getAcceleration();

    bool setZero();
    bool setCurrentPosition(int32_t position);

    void home();
    bool moveTo(int32_t position);
    bool move(int32_t delta);
    void stop();
    void forceStop();
    void disable();
};

#endif
