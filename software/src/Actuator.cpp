#include "Actuator.h"

Actuator::Actuator(
    TMC2209Stepper *driver,
    FastAccelStepper *stepper,
    int stallPin,
    uint32_t stepsPerUnitTravel,
    MotionProfile *runningPositiveProfile,
    MotionProfile *runningNegativeProfile,
    // Direction homingDirection,
    MotionProfile *homingProfile,
    uint32_t totalTravel)
    : driver(driver),
      stepper(stepper),
      stallPin(stallPin),
      stepsPerUnitTravel(stepsPerUnitTravel),
      runningPositiveProfile(runningPositiveProfile),
      runningNegativeProfile(runningNegativeProfile),
      //   homingDirection(homingDirection),
      homingProfile(homingProfile),
      totalTravelSteps(totalTravel / stepsPerUnitTravel)
{
}

void Actuator::begin()
{
    attachInterruptArg(digitalPinToInterrupt(stallPin), Actuator::setStalled, this, RISING);

    xTaskCreatePinnedToCore(
        Actuator::startTask,
        "ActuatorTask",
        1024 * 4, // todo: find high water mark of stack usage
        this,
        1, // todo: is this the right prio?
        NULL,
        1);
}

void IRAM_ATTR Actuator::setStalled(void *instance)
{
    ((Actuator *)instance)->stalled = true;
}

void Actuator::startTask(void *instance)
{
    while (true)
    {
        ((Actuator *)instance)->loop();
    }
}

std::optional<int32_t> Actuator::getMin()
{
    if (!homed)
        return {};
    return minPosStep / stepsPerUnitTravel;
}

std::optional<int32_t> Actuator::getMax()
{
    if (!homed)
        return {};
    return maxPosStep / stepsPerUnitTravel;
}

std::optional<int32_t> Actuator::getCurrentPosition()
{
    if (!homed)
        return {};
    return stepper->getCurrentPosition() / stepsPerUnitTravel;
}

bool Actuator::setZero()
{
    return setCurrentPosition(0);
}

bool Actuator::setCurrentPosition(int32_t position)
{
    if (!homed)
        return false;
    int posStep = position * stepsPerUnitTravel;
    int delta = stepper->getCurrentPosition() - posStep;
    minPosStep += delta;
    maxPosStep += delta;
    stepper->setCurrentPosition(posStep);
    targetPosStep = posStep;
    return true;
}

void Actuator::setMotionProfile(MotionProfile *profile)
{
    stepper->setAcceleration(profile->acceleration * stepsPerUnitTravel);
    stepper->setSpeedInHz(profile->velocity * stepsPerUnitTravel);
    driver->SGTHRS(profile->stallThreshold);
    driver->TCOOLTHRS(profile->velocityThreshold);
}

void Actuator::loop()
{
    switch (this->state)
    {
    case DISABLED:
        stepper->disableOutputs();
        homed = false;
        stalled = false;
        break;
    case HOMING_INIT:
        setMotionProfile(homingProfile);
        stepper->enableOutputs();
        stepper->moveTo(INT_MIN);
        state = HOMING_MOVING;
        break;
    case HOMING_MOVING:
        if (stalled)
        {
            state = HOMING_STALLED;
        }
        break;
    case HOMING_STALLED:
        stepper->forceStopAndNewPosition(0);
        minPosStep = stepper->getCurrentPosition();
        maxPosStep = minPosStep + totalTravelSteps;

        homed = true;
        state = STOPPED;
        break;
    case STOPPED:
        if (stepper->isRunning() || stepper->isRunningContinuously())
        {
            stepper->stopMove();
        }
        stalled = false;
        break;
    case FORCE_STOPPED:
        stepper->forceStop();
        stalled = false;
        break;
    case MOVING:
        if (stalled)
        {
            state = MOVING_STALLED;
        }
        else if (!homed || stepper->getCurrentPosition() == targetPosStep)
        {
            state = STOPPED;
        }
        else if (!stepper->isRunning())
        {
            int delta = targetPosStep - stepper->getCurrentPosition();
            MotionProfile *profile =
                delta > 0 ? runningPositiveProfile : runningNegativeProfile;
            setMotionProfile(profile);
            stepper->enableOutputs();
            stepper->moveTo(targetPosStep);
        }
        break;
    case MOVING_STALLED:
        homed = false;
        state = STOPPED;
        break;
    }
}

void Actuator::home()
{
    this->state = HOMING_INIT;
}

bool Actuator::moveTo(int32_t position)
{
    if (!homed)
        return false;
    int positionStep = position * stepsPerUnitTravel;
    if (positionStep > maxPosStep)
        positionStep = maxPosStep;
    else if (positionStep < minPosStep)
        positionStep = minPosStep;
    targetPosStep = positionStep;
    this->state = MOVING;
    return true;
}

bool Actuator::move(int32_t delta)
{
    int32_t deltaStep = delta * stepsPerUnitTravel;
    return moveTo(deltaStep + targetPosStep);
}

void Actuator::stop()
{
    this->state = STOPPED;
}

void Actuator::forceStop()
{
    this->state = FORCE_STOPPED;
}
