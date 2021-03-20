/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Constants.h"
#include "Robot.h"
#include "lib/NL/NLOdometry.h"
#include <iostream>
#include <frc/shuffleboard/Shuffleboard.h>
#include <time.h>
#include <units/units.h>

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/Path.h>
#include <wpi/SmallString.h>

double Deadband(double value, double deadband = 0.1)
{
    if (std::abs(value) < deadband)
        return 0;
    else
        return value < 0 ? (value + deadband) / (1.0 - deadband) : (value - deadband) / (1.0 - deadband);
}

void Robot::Drive(double forward, double turn)
{
    //std::cout<<forward<<std::endl;
    forward = Deadband(forward, 0.1);
    turn = Deadband(turn, 0.2);

    /*
    double c = 0.35 * (turn * 5.0 * (abs(turn) + 1) / (abs(forward) + 1));
    if (turn < 0.0) {
        m_drivetrain->Drive(forward * ((c + 1) / (1 - c)), forward);
    } else {
        m_drivetrain->Drive(forward, forward * ((1 - c) / (c + 1)));
    }*/

    double v = forward * VMAX;
    double w = turn * WMAX * m_turnAdjustFactor;

    // w = m_drivetrain->CalculateTurn(forward, w);

    double lwheel = v + (w * HALF_TRACKWIDTH);
    double rwheel = v - (w * HALF_TRACKWIDTH);

    double k;
    k = 1.0 / (NMAX(VMAX, NMAX(NABS(lwheel), NABS(rwheel))));
    lwheel *= k;
    rwheel *= k;

    //std::cout<<lwheel<<std::endl;

    m_moteurGauche.Set(lwheel);
    m_moteurGaucheFollower.Set(lwheel);
    m_moteurDroite.Set(rwheel);
    m_moteurDroiteFollower.Set(rwheel);
}

void Robot::RobotInit()
{
    m_moteurDroite.RestoreFactoryDefaults();
    m_moteurGauche.RestoreFactoryDefaults();
    m_moteurDroiteFollower.RestoreFactoryDefaults();
    m_moteurGaucheFollower.RestoreFactoryDefaults();
#ifdef TIME_RAMP
    m_moteurDroite.SetOpenLoopRampRate(TIME_RAMP);
    m_moteurGauche.SetOpenLoopRampRate(TIME_RAMP);
    m_moteurDroiteFollower.SetOpenLoopRampRate(TIME_RAMP);
    m_moteurGaucheFollower.SetOpenLoopRampRate(TIME_RAMP);
#else
    m_moteurDroite.SetOpenLoopRampRate(0);
    m_moteurGauche.SetOpenLoopRampRate(0);
    m_moteurDroiteFollower.SetOpenLoopRampRate(0);
    m_moteurGaucheFollower.SetOpenLoopRampRate(0);
#endif
#if VOLTAGE_COMPENSATION
    m_moteurDroite.EnableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);
    m_moteurGauche.EnableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);
    m_moteurDroiteFollower.EnableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);
    m_moteurGaucheFollower.EnableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);
#else
    m_moteurGauche.DisableVoltageCompensation();
    m_moteurGaucheFollower.DisableVoltageCompensation();
    m_moteurDroite.DisableVoltageCompensation();
    m_moteurDroiteFollower.DisableVoltageCompensation();
#endif
    m_moteurDroite.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_moteurGauche.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_moteurDroiteFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_moteurGaucheFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_PowerEntry = frc::Shuffleboard::GetTab("voltage").Add("Voltage", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
#if IMU
    m_speedY = frc::Shuffleboard::GetTab("voltage").Add("speedY", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    m_speedX = frc::Shuffleboard::GetTab("voltage").Add("speedX", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
#endif
    m_encodeurExterneDroite.SetReverseDirection(true);
    m_encodeurExterneGauche.SetReverseDirection(false);

    m_encodeurExterneDroite.SetDistancePerPulse(1);
    m_encodeurExterneGauche.SetDistancePerPulse(1);

    m_encodeurExterneDroite.SetSamplesToAverage(65);
    m_encodeurExterneGauche.SetSamplesToAverage(65);

    m_moteurDroite.SetInverted(false);
    m_moteurDroiteFollower.SetInverted(false);

    m_moteurGauche.SetInverted(true);
    m_moteurGaucheFollower.SetInverted(true);

    m_moteurGaucheShooter.SetInverted(true);

    m_moteurCoveyor.SetInverted(true);
    m_moteurFeeder.SetInverted(true);

    m_moteurDroite.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 2);
    m_moteurDroite.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 2);
    m_moteurDroite.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);

    m_moteurDroiteFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 2);
    m_moteurDroiteFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 2);
    m_moteurDroiteFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);

    m_moteurGauche.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 2);
    m_moteurGauche.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 2);
    m_moteurGauche.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);

    m_moteurGaucheFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 2);
    m_moteurGaucheFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 2);
    m_moteurGaucheFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);

    Robot::AddPeriodic([&]() {
        m_motorCharacterizationTests.fastLoop();
    },
                       2_ms, 1_ms);
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
    m_encodeurExterneDroite.Reset();
    m_encodeurExterneGauche.Reset();

    m_isLogging = 0;
    m_isPathFollowing = 0;

    // reset robot path
    m_dsLeftWheel = 0.0f;
    m_dsRightWheel = 0.0f;

    m_currrentSState.null();

    m_refLeftS = 0.0f;
    m_refRightS = 0.0f;
    m_prevK = 0.0f;
    m_prevS = 0.0f;

    m_errorLeft.reset();
    m_errorRight.reset();

#if IMU
    init_x = m_imu.GetAccelInstantX();
    init_y = m_imu.GetAccelInstantY();
#endif
}

void Robot::TeleopPeriodic()
{
    std::cout << "on : " << m_isLogging << std::endl;
#if IMU
    m_speedY.SetDouble(filterY.Calculate(m_imu.GetAccelInstantY() - init_y));
    m_speedX.SetDouble(filterX.Calculate(m_imu.GetAccelInstantX() - init_x));
#endif

#if XBOX_CONTROLLER
    Drive(-m_driverController.GetY(frc::GenericHID::JoystickHand::kLeftHand), m_driverController.GetX(frc::GenericHID::JoystickHand::kRightHand));
#else
    m_va_max.m_acceleration = m_PowerEntry.GetDouble(0.0f);
    Drive(-m_leftHandController.GetY(), m_rightHandController.GetZ());
    std::cout << m_encodeurExterneGauche.GetDistance() << std::endl;
#endif

#if XBOX_CONTROLLER
    if (m_driverController.GetBButtonPressed())
    {
        if (m_motorCharacterizationTests.getState() == NLCharacterization_Tests::State::Stopped)
        {
            m_motorCharacterizationTests.nextTest();
        }
    }

    if (m_driverController.GetXButtonPressed())
    {
        if (m_motorCharacterizationTests.getState() == NLCharacterization_Tests::State::Stopped)
        {
            m_motorCharacterizationTests.previousTest();
        }
    }

    if (m_driverController.GetAButtonPressed())
    {
        if (m_motorCharacterizationTests.getState() == NLCharacterization_Tests::State::Started)
        {
            m_motorCharacterizationTests.stop();
            m_motorCharacterizationTests.nextTest();
        }
        else if (m_motorCharacterizationTests.getState() == NLCharacterization_Tests::State::Stopped)
        {
            m_motorCharacterizationTests.start();
        }
    }
#else
    m_turnAdjustFactor = (m_rightHandController.GetThrottle() + 1.0) / 2.0;
    m_customEntry.SetDouble(m_turnAdjustFactor);
#endif
}

void Robot::TestInit()
{
}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
