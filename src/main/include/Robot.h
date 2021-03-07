/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <Constants.h>
#include <frc/TimedRobot.h>
#if XBOX_CONTROLLER
#include <frc/XboxController.h>
#else
#include <frc/Joystick.h>
#endif
#include <frc/DoubleSolenoid.h>
#include <frc/Encoder.h>
#include <rev/CANSparkMax.h>
#include <frc/PWMSparkMax.h>
#include <frc/VictorSP.h>
#include <networktables/NetworkTableEntry.h>
#include "lib/CSVLogFile.h"
#include "lib/CustomMaths.h"
#include "Joystick.h"
#if IMU
#include <adi/ADIS16470_IMU.h>
#endif
#include <frc/LinearFilter.h>
#include <frc/PowerDistributionPanel.h>

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void DriveOld(double forward, double turn);
  void Drive(double jy, double jx);
  void DriveA(double forward, double turn);
  void DriveB();

private:
  double m_targetLeftSpeed;
  double m_targetRightSpeed;
  VA m_va_left;
  VA m_va_right;
  VA m_va_max;
  KineticToVoltage m_kv;

  CSVLogFile *m_LogFile, *m_LogFileDriving;
  nt::NetworkTableEntry m_LogFilename, m_PowerEntry, m_logGyro, m_LogFilenameDriving, m_speedY, m_speedX, m_customEntry;
  rev::CANSparkMax m_moteurDroite{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_moteurDroiteFollower{4, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_moteurGauche{2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_moteurGaucheFollower{3, rev::CANSparkMax::MotorType::kBrushless};

  frc::PowerDistributionPanel m_pdp;

  frc::Encoder m_encodeurExterneDroite{0, 1, false, frc::Encoder::k4X};
  frc::Encoder m_encodeurExterneGauche{2, 3, true, frc::Encoder::k4X};

#if IMU
  frc::ADIS16470_IMU m_imu{};
  frc::LinearFilter<double> filterX = frc::LinearFilter<double>::MovingAverage(64);
  frc::LinearFilter<double> filterY = frc::LinearFilter<double>::MovingAverage(64);
#endif

  frc::PWMSparkMax m_moteurTreuil{5};

  rev::CANSparkMax m_moteurGaucheShooter{6, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_moteurDroiteShooter{7, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANEncoder m_encodeurShooterGauche{m_moteurGaucheShooter};
  rev::CANEncoder m_encodeurShooterDroit{m_moteurDroiteShooter};

  rev::CANSparkMax m_moteurCoveyor{14, rev::CANSparkMax::MotorType::kBrushless};

  frc::VictorSP m_moteurDoigt{6}; //6

  frc::VictorSP m_moteurFeeder{3}; //3

  frc::VictorSP m_moteurIntake{2}; //2

  //frc::DoubleSolenoid m_solenoidClimber1{0, 1};

  //frc::DoubleSolenoid m_solenoidDoigt{2, 3};

  char m_invertedPrefix[8];

  bool modeClimberJF = false;
  bool doigtLeve;
  bool shooterOn = false;
  bool m_override = false;
  bool m_isLogging = false;
  double m_ramp = 0;
  double m_time0;

  int m_logState = 0;
  char m_prefix[512];

  double init_x;
  double init_y;

  /*m_IsEnabledEntry = frc::Shuffleboard::GetTab("Shooter").Add("Is Shooter enabled", false).WithWidget(frc::BuiltInWidgets::kBooleanBox).GetEntry();
    m_PowerEntry = frc::Shuffleboard::GetTab("Shooter").Add("Power", 0.0).WithWidget(frc::BuiltInWidgets::kNumberSlider).GetEntry();
    m_LogEntry = frc::Shuffleboard::GetTab("Shooter").Add("Logging", false).WithWidget(frc::BuiltInWidgets::kToggleButton).GetEntry();
    m_LogFilename = frc::Shuffleboard::GetTab("Shooter").Add("Logfile Name", "").WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
  */

  void LogData();
  double m_turnAdjustFactor = 1;

#if XBOX_CONTROLLER
  frc::XboxController m_driverController{0};
#else
  frc::Joystick m_leftHandController{0};
  frc::Joystick m_rightHandController{1};
#endif
};
