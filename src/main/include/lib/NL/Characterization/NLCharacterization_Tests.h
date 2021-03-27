// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <rev/CANSparkMax.h>
#include <frc/Encoder.h>
#include <assert.h>
#include "lib/N/NMath.h"
#include "lib/N/NType.h"
#include "lib/N/NFlags.h"
#include <networktables/NetworkTableEntry.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <units/units.h>
#include <stdio.h>

#define NB_MOTOR 4
#define NB_ENCODER 2

typedef struct TestSpecs TestSpecs;
struct TestSpecs
{
  unsigned long m_flags;
  double m_voltage;
  double m_ramp;
};

typedef struct NL_ENCODER_LOGDATA NL_ENCODER_LOGDATA;
struct NL_ENCODER_LOGDATA
{
  frc::Encoder *m_pEncoder;
  char name[16]; //NULL character included
};

typedef struct NL_CANSPARK_LOGDATA NL_CANSPARK_LOGDATA;
struct NL_CANSPARK_LOGDATA
{
  rev::CANSparkMax *m_pCanSparkMax;
  char name[16]; //NULL character included
};

class NLCharacterization_Tests
{

public:
  enum class State
  {
    Stopped = 0,
    AskForStart = 1,
    Started = 2,
    AskForStop = 3
  };
  NLCharacterization_Tests(rev::CANSparkMax *leftMotor,
                           rev::CANSparkMax *leftMotorFollower,
                           rev::CANSparkMax *rightMotor,
                           rev::CANSparkMax *rightMotorFollower,
                           frc::Encoder *externalEncoderLeft,
                           frc::Encoder *externalEncoderRight,
                           long nbTestLow,
                           double endVoltageLow,
                           long nbTestMedium,
                           double endVoltageMedium,
                           long nbTestHigh,
                           double endVoltageHigh,
                           double rampValue,
                           double rampVoltage);
  ~NLCharacterization_Tests();
  void nextTest();
  void previousTest();
  void setCurrentTest(uint8_t testId);
  void start();
  void stop();
  void fastLoop();
  State getState();
  uint8_t getCurrentTestId();
  char *getCurrentTestDescription(char *pmessage, uint size_terminated_null_char_included);
  uint getTestsCounter();
  uint areAllTestsDone();

private:
  rev::CANSparkMax *m_rightMotor;
  rev::CANSparkMax *m_rightMotorFollower;
  rev::CANSparkMax *m_leftMotor;
  rev::CANSparkMax *m_leftMotorFollower;

  frc::Encoder *m_externalEncoderRight;
  frc::Encoder *m_externalEncoderLeft;

  NL_ENCODER_LOGDATA m_encoderLogData[2] = {
      {m_externalEncoderRight, "encoderGetR"},
      {m_externalEncoderLeft, "encoderGetL"}};

  NL_CANSPARK_LOGDATA m_cansparkLogData[4]{
      {m_rightMotor, "rightMotor"},
      {m_rightMotorFollower, "rightFollower"},
      {m_leftMotor, "leftMotor"},
      {m_leftMotorFollower, "leftFollower"}};

  TestSpecs *TestData;
  State m_state = State::Stopped;
  uint8_t m_CurrentTestID = 0;
  double m_oldRamp;
  uint8_t m_nbTotalTest;

  FILE *m_LogFile;

  double m_ramp = 0;
  double m_time0;
};
