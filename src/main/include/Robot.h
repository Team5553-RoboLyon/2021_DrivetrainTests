/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Encoder.h>
#include <rev/CANSparkMax.h>
#include <frc/PWMSparkMax.h>
#include <frc/VictorSP.h>
#include <networktables/NetworkTableEntry.h>
#include "lib/CSVLogFile.h"
#include "lib/CustomMaths.h"
#include <frc/ADXRS450_Gyro.h>
#include "Joystick.h"

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

private:
  VA m_va_left;
  VA m_va_right;
  VA m_va_max;
  KineticToVoltage m_kv;

  CSVLogFile *m_LogFile, *m_LogFileDriving;
  nt::NetworkTableEntry m_LogFilename, m_PowerEntry, m_logGyro, m_LogFilenameDriving;
  rev::CANSparkMax m_moteurDroite{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_moteurDroiteFollower{4, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_moteurGauche{2, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_moteurGaucheFollower{3, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANEncoder m_encodeurDroite1{m_moteurDroite};
  rev::CANEncoder m_encodeurDroite2{m_moteurDroiteFollower};

  rev::CANEncoder m_encodeurGauche1{m_moteurGauche};
  rev::CANEncoder m_encodeurGauche2{m_moteurGaucheFollower};

  frc::Encoder m_encodeurExterneDroite{0, 1, true, frc::Encoder::k4X};
  frc::Encoder m_encodeurExterneGauche{2, 3, false, frc::Encoder::k4X};

  frc::ADXRS450_Gyro m_gyro{frc::SPI::Port::kOnboardCS0};

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

  frc::XboxController m_driverController{0};

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

  /*m_IsEnabledEntry = frc::Shuffleboard::GetTab("Shooter").Add("Is Shooter enabled", false).WithWidget(frc::BuiltInWidgets::kBooleanBox).GetEntry();
    m_PowerEntry = frc::Shuffleboard::GetTab("Shooter").Add("Power", 0.0).WithWidget(frc::BuiltInWidgets::kNumberSlider).GetEntry();
    m_LogEntry = frc::Shuffleboard::GetTab("Shooter").Add("Logging", false).WithWidget(frc::BuiltInWidgets::kToggleButton).GetEntry();
    m_LogFilename = frc::Shuffleboard::GetTab("Shooter").Add("Logfile Name", "").WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
  */

  void LogData();
};
