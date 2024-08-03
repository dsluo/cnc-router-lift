#include <Arduino.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>
#include "Actuator.h"
#include <LiquidCrystal_I2C.h>
#include <ESP32Encoder.h>

#define ENABLE_PIN 14
#define DIRECTION_PIN 27
#define STEP_PIN 12
#define STALL_PIN 13

#define ENCODER_BUTTON 36
#define ENCODER_A 39
#define ENCODER_B 34

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

MotionProfile runningPositiveProfie = MotionProfile{
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
    &runningPositiveProfie,
    &runningNegativeProfile,
    &homingProfile,
    TOTAL_TRAVEL);
;

LiquidCrystal_I2C lcd(0x27, 16, 2);
ESP32Encoder encoder;

void interfaceSetup()
{
  lcd.init();
  lcd.backlight();

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(ENCODER_A, ENCODER_B);
}

void setup()
{
  Serial.begin(115200);
  actuator.begin();
  interfaceSetup();
}

void loop()
{
  // lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(encoder.getCount() / 4);
  Serial.println(encoder.getCount() / 4);
}
