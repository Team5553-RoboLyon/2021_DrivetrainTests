/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>
#include <frc/shuffleboard/Shuffleboard.h>
#include <time.h>
#include <units/units.h>

#define TRACKWIDTH 0.61f
#define HALF_TRACKWIDTH (TRACKWIDTH / 2.0f)

#define AMAX 5                           // Acceleration Max  au PIF .. à définir aux encodeurs
#define VMAX 3.4                         // vitesse Max  théorique (3,395472 sur JVN-DT) .. à vérifier aux encodeurs
#define WMAX ((2.0 * VMAX) / TRACKWIDTH) // vitesse angulaire Max theorique

// Flags Manipulation
#define FLAG_TOGGLE(val, flag) ((val) ^= (flag))
#define FLAG_ON(val, flag) ((val) |= (flag))
#define FLAG_OFF(val, flag) ((val) &= ~(flag))
#define ISFLAG_ON(val, flag) ((val) & (flag))                                   // !! ZERO or NON ZERO value !!! BE AWARE THAT NON ZERO DOESN'T MEAN 1 !!!
#define ISFLAG_OFF(val, flag) (!((val) & (flag)))                               // !! ZERO or NON ZERO value !!! BE AWARE THAT NON ZERO DOESN'T MEAN 1 !!!
#define FLAGS_TEST(val, bmask, flags) (((val) & (bmask)) == (flags))            // NFALSE or NTRUE
#define SET_FLAGS(val, bmask, flags) ((val) = (((val) & (~(bmask))) | (flags))) // RESET FLAGS BITS and Set them all like flags.
#define RESET_FLAGS(val, bmask)		((val)&=~(bmask)))						// Set all FLAGS BITS to ZERO

#define VOLTAGE_COMPENSATION_VALUE 10
// TEST *********************************************
#define TEST_LOWVOLTAGE_NB 10    // Nombre de tests ( subdivisions ) sur l'intervalle ]0,TEST_LOWVOLTAGE_MAX] volts						... 10 ou 20 ?
#define TEST_LOWVOLTAGE_MAX 0.15 // Volts

#define TEST_MEDIUMVOLTAGE_NB 5    // Nombre de tests ( subdivisions ) sur l'intervalle ]TEST_LOWVOLTAGE_MAX,TEST_MEDIUMVOLTAGE_MAX] volts	... 20 ou 25 ?
#define TEST_MEDIUMVOLTAGE_MAX 1.0 // Volts

#define TEST_HIGHVOLTAGE_NB 44    // Nombre de tests ( subdivisions ) sur l'intervalle ]TEST_MEDIUMVOLTAGE_MAX,TEST_HIGHVOLTAGE_MAX] volts... 12 ou 24 ?
#define TEST_HIGHVOLTAGE_MAX 12.0 // Volts

#define TEST_TOTAL_NB (TEST_LOWVOLTAGE_NB + TEST_MEDIUMVOLTAGE_NB + TEST_HIGHVOLTAGE_NB)

#define FLAG_TestSpecs_Done 1

#define TIME_RAMP 0

typedef struct TestSpecs TestSpecs;
struct TestSpecs
{
    unsigned long m_flags;
    double m_voltage;
};

TestSpecs TestData[TEST_TOTAL_NB * 2];
int CurrentTestID = 0;

nt::NetworkTableEntry Message;

void initializeTestData()
{
    int i;
    // Low Voltages
    for (i = 0; i < TEST_LOWVOLTAGE_NB; i++)
    {
        TestData[i * 2].m_voltage = TEST_LOWVOLTAGE_MAX * (double)(i + 1) / (double)TEST_LOWVOLTAGE_NB;
        TestData[i * 2].m_flags = 0;

        TestData[i * 2 + 1].m_voltage = -(TEST_LOWVOLTAGE_MAX * (double)(i + 1) / (double)TEST_LOWVOLTAGE_NB);
        TestData[i * 2 + 1].m_flags = 0;
    }
    // Medium Voltages
    for (i = 0; i < TEST_MEDIUMVOLTAGE_NB; i++)
    {
        TestData[2 * (TEST_LOWVOLTAGE_NB + i)].m_voltage = TEST_LOWVOLTAGE_MAX + (TEST_MEDIUMVOLTAGE_MAX - TEST_LOWVOLTAGE_MAX) * (double)(i + 1) / (double)TEST_MEDIUMVOLTAGE_NB;
        TestData[2 * (TEST_LOWVOLTAGE_NB + i)].m_flags = 0;

        TestData[2 * (TEST_LOWVOLTAGE_NB + i) + 1].m_voltage = -(TEST_LOWVOLTAGE_MAX + (TEST_MEDIUMVOLTAGE_MAX - TEST_LOWVOLTAGE_MAX) * (double)(i + 1) / (double)TEST_MEDIUMVOLTAGE_NB);
        TestData[2 * (TEST_LOWVOLTAGE_NB + i) + 1].m_flags = 0;
    }

    // High Voltages
    for (i = 0; i < TEST_HIGHVOLTAGE_NB; i++)
    {
        TestData[2 * (TEST_LOWVOLTAGE_NB + TEST_MEDIUMVOLTAGE_NB + i)].m_voltage = TEST_MEDIUMVOLTAGE_MAX + (TEST_HIGHVOLTAGE_MAX - TEST_MEDIUMVOLTAGE_MAX) * (double)(i + 1) / (double)TEST_HIGHVOLTAGE_NB;
        TestData[2 * (TEST_LOWVOLTAGE_NB + TEST_MEDIUMVOLTAGE_NB + i)].m_flags = 0;

        TestData[2 * (TEST_LOWVOLTAGE_NB + TEST_MEDIUMVOLTAGE_NB + i) + 1].m_voltage = -(TEST_MEDIUMVOLTAGE_MAX + (TEST_HIGHVOLTAGE_MAX - TEST_MEDIUMVOLTAGE_MAX) * (double)(i + 1) / (double)TEST_HIGHVOLTAGE_NB);
        TestData[2 * (TEST_LOWVOLTAGE_NB + TEST_MEDIUMVOLTAGE_NB + i) + 1].m_flags = 0;
    }

    Message = frc::Shuffleboard::GetTab("voltage").Add("Message", "").WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
}

void messageTestEnAttente()
{
    char txt[256];
    if (ISFLAG_OFF(TestData[CurrentTestID].m_flags, FLAG_TestSpecs_Done))
    {
        sprintf(txt, "TEST %d / %d [ %.2f Volts ]. En Attente ... Appuyer sur A pour Démarrer.", CurrentTestID, TEST_TOTAL_NB * 2, TestData[CurrentTestID].m_voltage);
        Message.SetString(txt);
    }
    else
    {
        sprintf(txt, "TEST %d / %d [ %.2f Volts ]. DEJA EFFECTUE ! ... Appuyer sur A pour Démarrer à nouveau.( Le précédent LOGFILE sera conservé.)", CurrentTestID, TEST_TOTAL_NB, TestData[CurrentTestID].m_voltage);
        Message.SetString(txt);
    }
}
void messageTestTousEffectues()
{
    char txt[256];
    sprintf(txt, "%d / %d  TESTS EFFECTUES ! ", TEST_TOTAL_NB * 2, TEST_TOTAL_NB * 2);
    Message.SetString(txt);
}
void messageTestEnCours()
{
    char txt[256];
    sprintf(txt, "TEST %d / %d [ %.2f Volts ]. En cours ... Appuyer à nouveau sur A pour arrêter.", CurrentTestID, TEST_TOTAL_NB * 2, TestData[CurrentTestID].m_voltage);
    Message.SetString(txt);
}
// TEST *********************************************

double Deadband(double value, double deadband = 0.1)
{
    if (std::abs(value) < deadband)
        return 0;
    else
        return value < 0 ? (value + deadband) / (1.0 - deadband) : (value - deadband) / (1.0 - deadband);
}

void Robot::DriveOld(double forward, double turn)
{
    if (!m_override)
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
        //std::cout<<m_encodeurDroite.GetVelocity()<<std::endl;
        //std::cout<<m_encodeurGauche.GetVelocity()<<std::endl;

        // m_drivetrain->Drive(forward + 0.5 * turn, forward - 0.5 * turn);
    }
    else
    {
        // double power = m_PowerEntry.GetDouble(0.0) / 11.5;

        // double power =TestData[CurrentTestID].m_voltage;

        //m_moteurGauche.Set(TestData[CurrentTestID].m_voltage / VOLTAGE_COMPENSATION_VALUE);
        //m_moteurDroite.Set(TestData[CurrentTestID].m_voltage / VOLTAGE_COMPENSATION_VALUE);
        m_moteurGauche.Set(TestData[CurrentTestID].m_voltage / m_moteurGauche.GetBusVoltage());
        m_moteurGaucheFollower.Set(TestData[CurrentTestID].m_voltage / m_moteurGaucheFollower.GetBusVoltage());
        m_moteurDroite.Set(TestData[CurrentTestID].m_voltage / m_moteurDroite.GetBusVoltage());
        m_moteurDroiteFollower.Set(TestData[CurrentTestID].m_voltage / m_moteurDroiteFollower.GetBusVoltage());
    }
}

void Robot::Drive(double forward, double turn)
{
    forward = Deadband(forward, 0.1);
    turn = Deadband(turn, 0.1);
    double v = forward * VMAX;
    double w = turn * WMAX * m_turnAdjustFactor;

    // w = m_drivetrain->CalculateTurn(forward, w);

    double lwheel = v + (w * HALF_TRACKWIDTH);
    double rwheel = v - (w * HALF_TRACKWIDTH);

    double k;
    k = 1.0 / (NMAX(VMAX, NMAX(NABS(lwheel), NABS(rwheel))));
    lwheel *= k;
    rwheel *= k;

    double target_left_speed = lwheel * VMAX;
    double target_right_speed = rwheel * VMAX;
    //std::cout << target_left_speed << "     " << target_right_speed << std::endl;
    //getSpeedsAndAccelerationsNew(&m_va_left, &m_va_right, &m_va_max, jx, jy);

    updateVelocityAndAcceleration(&m_va_left, &m_va_max, target_left_speed, 0.02);
    updateVelocityAndAcceleration(&m_va_right, &m_va_max, target_right_speed, 0.02);

    m_moteurGauche.Set(m_kv.getVoltage(0, &m_va_left) / m_moteurGauche.GetBusVoltage());
    //m_moteurGaucheFollower.Set(m_kv.getVoltage(1, &m_va_left) / m_moteurGaucheFollower.GetBusVoltage());
    m_moteurDroite.Set(m_kv.getVoltage(2, &m_va_right) / m_moteurDroite.GetBusVoltage());
    //m_moteurDroiteFollower.Set(m_kv.getVoltage(3, &m_va_right) / m_moteurDroiteFollower.GetBusVoltage());

    std::cout << forward << "          " << target_left_speed << "          " << m_va_left.m_speed << "   :   " << m_va_left.m_acceleration << "             " << m_kv.getVoltage(0, &m_va_left) << "      :     " << std::endl;
}
void Robot::DriveA(double forward, double turn)
{
    forward = Deadband(forward, 0.1);
    turn = Deadband(turn, 0.1);
    double v = forward * VMAX;
    double w = turn * WMAX * m_turnAdjustFactor;

    // w = m_drivetrain->CalculateTurn(forward, w);

    double lwheel = v + (w * HALF_TRACKWIDTH);
    double rwheel = v - (w * HALF_TRACKWIDTH);

    double k;
    k = 1.0 / (NMAX(VMAX, NMAX(NABS(lwheel), NABS(rwheel))));
    lwheel *= k;
    rwheel *= k;

    m_targetLeftSpeed = lwheel * VMAX;
    m_targetRightSpeed = rwheel * VMAX;
}
void Robot::DriveB()
{
    updateVelocityAndAcceleration(&m_va_left, &m_va_max, m_targetLeftSpeed, 0.005);
    updateVelocityAndAcceleration(&m_va_right, &m_va_max, m_targetRightSpeed, 0.005);

    /*m_moteurGauche.Set(m_kv.getVoltage(0, &m_va_left) / m_moteurGauche.GetBusVoltage());
    m_moteurGaucheFollower.Set(m_kv.getVoltage(1, &m_va_left) / m_moteurGaucheFollower.GetBusVoltage());
    m_moteurDroite.Set(m_kv.getVoltage(2, &m_va_right) / m_moteurDroite.GetBusVoltage());
    m_moteurDroiteFollower.Set(m_kv.getVoltage(3, &m_va_right) / m_moteurDroiteFollower.GetBusVoltage());*/

    m_moteurGauche.SetVoltage(units::volt_t(m_kv.getVoltage(2, &m_va_left)));
    //m_moteurGaucheFollower.SetVoltage(units::volt_t(m_kv.getVoltage(3, &m_va_left)));
    m_moteurDroite.SetVoltage(units::volt_t(m_kv.getVoltage(0, &m_va_right)));
    //m_moteurDroiteFollower.SetVoltage(units::volt_t(m_kv.getVoltage(1, &m_va_right)));
}

void Robot::RobotInit()
{
    m_isLogging = false;

    m_moteurDroite.RestoreFactoryDefaults();
    m_moteurGauche.RestoreFactoryDefaults();
    m_moteurDroiteFollower.RestoreFactoryDefaults();
    m_moteurGaucheFollower.RestoreFactoryDefaults();

    m_moteurDroite.SetOpenLoopRampRate(TIME_RAMP);
    m_moteurGauche.SetOpenLoopRampRate(TIME_RAMP);
    m_moteurDroiteFollower.SetOpenLoopRampRate(TIME_RAMP);
    m_moteurGaucheFollower.SetOpenLoopRampRate(TIME_RAMP);

    m_moteurDroite.EnableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);
    m_moteurGauche.EnableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);
    m_moteurDroiteFollower.EnableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);
    m_moteurGaucheFollower.EnableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);

    m_moteurGauche.DisableVoltageCompensation();
    m_moteurGaucheFollower.DisableVoltageCompensation();
    m_moteurDroite.DisableVoltageCompensation();
    m_moteurDroiteFollower.DisableVoltageCompensation();
    m_moteurDroite.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_moteurGauche.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_moteurDroiteFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_moteurGaucheFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    /*m_moteurGauche.SetClosedLoopRampRate(0.5);
    m_moteurGaucheFollower.SetClosedLoopRampRate(0.5);
    m_moteurDroite.SetClosedLoopRampRate(0.5);
    m_moteurDroiteFollower.SetClosedLoopRampRate(0.5);
    */

    m_PowerEntry = frc::Shuffleboard::GetTab("voltage").Add("Voltage", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    m_LogFilename = frc::Shuffleboard::GetTab("voltage").Add("Logfile Name", "").WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    m_customEntry = frc::Shuffleboard::GetTab("voltage").Add("Data", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
#if IMU
    m_speedY = frc::Shuffleboard::GetTab("voltage").Add("speedY", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    m_speedX = frc::Shuffleboard::GetTab("voltage").Add("speedX", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
#endif
    //frc::Shuffleboard::GetTab("voltage").Add(m_gyro).WithWidget(frc::BuiltInWidgets::kGyro);
    /*m_moteurGaucheShooter.SetClosedLoopRampRate(0.6);
    m_moteurDroiteShooter.SetClosedLoopRampRate(0.6);*/

    m_moteurDroiteFollower.Follow(m_moteurDroite);
    m_moteurGaucheFollower.Follow(m_moteurGauche);

    m_encodeurExterneDroite.SetDistancePerPulse(1);
    m_encodeurExterneGauche.SetDistancePerPulse(1);
    m_encodeurExterneDroite.SetSamplesToAverage(65);
    m_encodeurExterneGauche.SetSamplesToAverage(65);

    m_moteurDroite.SetInverted(false);
    m_moteurDroiteFollower.SetInverted(false);

    m_moteurGauche.SetInverted(true);
    m_moteurGaucheFollower.SetInverted(true);

    sprintf(m_invertedPrefix, "L%d%dR%d%d", (int)m_moteurGauche.GetInverted(), (int)m_moteurGaucheFollower.GetInverted(), (int)m_moteurDroite.GetInverted(), (int)m_moteurDroiteFollower.GetInverted());

    m_moteurGaucheShooter.SetInverted(true);

    /*m_ShooterEnabled = false;

    m_IsEnabledEntry = frc::Shuffleboard::GetTab("Shooter").Add("Is Shooter enabled", false).WithWidget(frc::BuiltInWidgets::kBooleanBox).GetEntry();
    m_PowerEntry = frc::Shuffleboard::GetTab("Shooter").Add("Power", 0.0).WithWidget(frc::BuiltInWidgets::kNumberSlider).GetEntry();
    m_LogEntry = frc::Shuffleboard::GetTab("Shooter").Add("Logging", false).WithWidget(frc::BuiltInWidgets::kToggleButton).GetEntry();
    m_LogFilename = frc::Shuffleboard::GetTab("Shooter").Add("Logfile Name", "").WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    */
    initializeTestData();

    m_moteurCoveyor.SetInverted(true);
    m_moteurFeeder.SetInverted(true);

    modeClimberJF = false;

    doigtLeve = false;
    //m_solenoidDoigt.Set(frc::DoubleSolenoid::Value::kForward);

    //Left 1 Forward
    m_kv.SetMotorCoefficients(0, 0, 3.152155758951382, 0.5846560513692277, 0.16874033425660784);
    //Left 2 Forward
    m_kv.SetMotorCoefficients(1, 0, 3.153964152576119, 0.5723628160446611, 0.16744753306274163);
    //Right 1 Forward
    m_kv.SetMotorCoefficients(2, 0, 3.0595817465468746, 0.5274566943213479, 0.17244602277001775);
    //Right 2 Forward
    m_kv.SetMotorCoefficients(3, 0, 3.0586967840650097, 0.5554384750908242, 0.17311052718403896);
    //Left 1 Backward
    m_kv.SetMotorCoefficients(0, 1, 3.118440174563925, 0.5829291200314344, -0.16580957666566665);
    //Left 2 Backward
    m_kv.SetMotorCoefficients(1, 1, 3.118960183596434, 0.5542235558754568, -0.16548790978136374);
    //Right 1 Backward
    m_kv.SetMotorCoefficients(2, 1, 3.1011616001787696, 0.4443591530083219, -0.16510492228166562);
    //Right 2 Backward
    m_kv.SetMotorCoefficients(3, 1, 3.0998353792893143, 0.4589298941678074, -0.16643629483787947);

    m_va_max.m_speed = VMAX;
    m_va_max.m_acceleration = AMAX;
    m_va_max.m_jerk = 5;

    m_va_left.m_speed = 0;
    m_va_left.m_acceleration = 0;
    m_va_right.m_speed = 0;
    m_va_right.m_acceleration = 0;

    m_moteurDroite.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 5);
    m_moteurDroite.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 5);

    m_moteurDroiteFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 5);
    m_moteurDroiteFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 5);

    m_moteurGauche.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 5);
    m_moteurGauche.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 5);

    m_moteurGaucheFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 5);
    m_moteurGaucheFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 5);

    Robot::AddPeriodic([&]() {
        DriveB();
        if (m_isLogging)
        {
            m_LogFileDriving->Log(m_targetLeftSpeed,
                                  m_va_left.m_speed,
                                  m_encodeurExterneGauche.GetDistance(),
                                  m_va_left.m_acceleration,
                                  m_kv.getVoltage(2, &m_va_left),
                                  m_moteurGauche.GetBusVoltage() * m_moteurGauche.GetAppliedOutput(),
                                  m_moteurGaucheFollower.GetBusVoltage() * m_moteurGaucheFollower.GetAppliedOutput(),
                                  m_moteurDroite.GetBusVoltage() * m_moteurDroite.GetAppliedOutput(),
                                  m_moteurDroiteFollower.GetBusVoltage() * m_moteurDroiteFollower.GetAppliedOutput(),
                                  m_pdp.GetCurrent(0),
                                  m_pdp.GetCurrent(1),
                                  m_pdp.GetCurrent(14),
                                  m_pdp.GetCurrent(15),
                                  m_moteurDroite.GetOutputCurrent(),
                                  m_moteurDroiteFollower.GetOutputCurrent(),
                                  m_moteurGauche.GetOutputCurrent(),
                                  m_moteurGaucheFollower.GetOutputCurrent(),
                                  m_moteurDroite.GetFaults(),
                                  m_moteurDroiteFollower.GetFaults(),
                                  m_moteurGauche.GetFaults(),
                                  m_moteurGaucheFollower.GetFaults());
        }
    },
                       5_ms, 5_ms);
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
    /*     filterX.Reset();
    filterY.Reset(); */

    //m_imu.ConfigCalTime(frc::ADIS16470CalibrationTime::_4s);
    //m_imu.Calibrate();
    m_LogFileDriving = new CSVLogFile("/home/lvuser/logs/freeRiding", "target", "speed", "tureSpeed", "acceleration", "voltageTheoric", " trueVoltageG1 ", "trueVoltageG2", "trueVoltageR1", "trueVoltageR2", "pdp0", "pdp1", "pdp14", "pdp15", "A1", "A2", "A3", "A4", "erreur1", "erreur2", "erreur3", "erreur4");
    m_isLogging = true;

    m_encodeurExterneDroite.Reset();
    m_encodeurExterneGauche.Reset();
#if IMU
    init_x = m_imu.GetAccelInstantX();
    init_y = m_imu.GetAccelInstantY();
#endif
}

void Robot::TeleopPeriodic()
{
    //std::cout << m_imu.GetAngle() << std::endl;
    //DriveOld(-m_driverController.GetY(frc::GenericHID::JoystickHand::kLeftHand), m_driverController.GetX(frc::GenericHID::JoystickHand::kRightHand));
#if IMU
    m_speedY.SetDouble(filterY.Calculate(m_imu.GetAccelInstantY() - init_y));
    m_speedX.SetDouble(filterX.Calculate(m_imu.GetAccelInstantX() - init_x));
#endif

#if XBOX_CONTROLLER
    Drive(-m_driverController.GetY(frc::GenericHID::JoystickHand::kLeftHand), m_driverController.GetX(frc::GenericHID::JoystickHand::kRightHand));
#else:
    m_va_max.m_acceleration = m_PowerEntry.GetDouble(0.0f);
    DriveA(-m_leftHandController.GetY(), m_rightHandController.GetZ());
    std::cout << m_encodeurExterneGauche.GetDistance() << std::endl;
#endif

    /*if (m_driverController.GetAButton())
    {
        m_moteurIntake.Set(0.7);
    }
    else
    {
        m_moteurIntake.Set(0);
    }*/
#if XBOX_CONTROLLER
    if (m_driverController.GetBButtonPressed() && (!m_override))
    {
        CurrentTestID += 1;
        if (CurrentTestID < TEST_TOTAL_NB * 2)
        {
            messageTestEnAttente();
        }
        else
        {
            messageTestTousEffectues();
        }
    }

    if (m_driverController.GetXButtonPressed() && (!m_override))
    {
        CurrentTestID -= 1;
        if (CurrentTestID < TEST_TOTAL_NB * 2)
        {
            messageTestEnAttente();
        }
        else
        {
            messageTestTousEffectues();
        }
    }

    if (m_driverController.GetAButtonPressed())
    {
        m_override = !m_override;
        //std::cout << m_override << std::endl;

        m_time0 = std::time(0);

        if (m_override)
        {
            if (CurrentTestID < TEST_TOTAL_NB * 2)
            {
                FLAG_ON(TestData[CurrentTestID].m_flags, FLAG_TestSpecs_Done);
                messageTestEnCours();

                if (TestData[CurrentTestID].m_voltage < 0)
                {
                    sprintf(m_prefix, "/home/lvuser/logs/-test_%s_%d_%.2fvolts_", m_invertedPrefix, CurrentTestID, TestData[CurrentTestID].m_voltage);
                }
                else
                {
                    sprintf(m_prefix, "/home/lvuser/logs/+test_%s_%d_%.2fvolts_", m_invertedPrefix, CurrentTestID, TestData[CurrentTestID].m_voltage);
                }
                m_logState = 1; // Préviens de démarrer le log (écriture du fichier)
            }
        }
        else
        {
            CurrentTestID += 1;
            if (CurrentTestID < TEST_TOTAL_NB * 2)
            {
                m_logState = 3;
                messageTestEnAttente();
            }
            else
            {
                m_logState = 3;
                messageTestTousEffectues();
            }
        }
    }

    if (m_driverController.GetYButtonPressed())
    {
        m_isLogging = !m_isLogging;
        //std::cout << m_override << std::endl;

        m_time0 = std::time(0);

        if (m_isLogging)
        {
            m_LogFileDriving = new CSVLogFile("/home/lvuser/logs/freeRiding", "encoderGetD", "encoderGetG", "encoderGetRawD", "encoderGetRawG", "Theorical Voltage", "BusVoltageD1", "BusVoltageD2", "BusVoltageG1", "BusVoltageG2", "AppliedOutputD1", "AppliedOutputD2", "AppliedOutputG1", "AppliedOutputG2", "currentD1", "currentD2", "currentG1", "currentG2", "rampActive");
            m_LogFilenameDriving.SetString(m_LogFileDriving->GetFileName());
            m_encodeurExterneDroite.Reset();
            m_encodeurExterneGauche.Reset();
        }
        else
        {
            delete m_LogFileDriving;
        }
    }

    if (m_isLogging)
    {
        m_LogFileDriving->Log(m_encodeurExterneDroite.Get(),
                              m_encodeurExterneGauche.Get(),
                              m_encodeurExterneDroite.GetRaw(),
                              m_encodeurExterneGauche.GetRaw(),
                              TestData[CurrentTestID].m_voltage,
                              m_moteurDroite.GetBusVoltage(),
                              m_moteurDroiteFollower.GetBusVoltage(),
                              m_moteurGauche.GetBusVoltage(),
                              m_moteurGaucheFollower.GetBusVoltage(),
                              m_moteurDroite.GetAppliedOutput(),
                              m_moteurDroiteFollower.GetAppliedOutput(),
                              m_moteurGauche.GetAppliedOutput(),
                              m_moteurGaucheFollower.GetAppliedOutput(),
                              m_moteurDroite.GetOutputCurrent(),
                              m_moteurDroiteFollower.GetOutputCurrent(),
                              m_moteurGauche.GetOutputCurrent(),
                              m_moteurGaucheFollower.GetOutputCurrent(),
                              m_ramp);
        //std::cout << mg0 << " " << mg1 << " " << md0 << " " << md1 << std::endl;
        //std::cout << m_encodeurExterneDroite.GetDistance() << std::endl;
    }

    /*if (m_driverController.GetBButtonPressed())
    {
        shooterOn = !shooterOn;
        if (shooterOn)
        {
            m_moteurDroiteShooter.Set(1);
            m_moteurGaucheShooter.Set(1);
        }
        else
        {
            m_moteurDroiteShooter.Set(0);
            m_moteurGaucheShooter.Set(0);
        }
    }*/

    if (m_driverController.GetYButton())
    {

        m_moteurCoveyor.Set(1);
        m_moteurFeeder.Set(1);
    }
    else
    {
        m_moteurCoveyor.Set(0);
        m_moteurFeeder.Set(0);
    }

    /*if (m_driverController.GetYButton())
    {
        m_moteurDoigt.Set(0.5);
    }
    else
    {
        m_moteurDoigt.Set(0);
    }*/

    if (m_driverController.GetPOV(0))
    {
        if (modeClimberJF)
        {
            m_moteurTreuil.Set(-0.3);
        }
        else
        {
            m_moteurTreuil.Set(0);
            //m_solenoidClimber1.Set(frc::DoubleSolenoid::Value::kForward);
        }
    }
    else if (m_driverController.GetPOV(90))
    {
        if (!modeClimberJF)
        {
            m_moteurTreuil.Set(0);
            //m_solenoidClimber1.Set(frc::DoubleSolenoid::Value::kReverse);
        }
    }
    else if (m_driverController.GetPOV(180))
    {
        m_moteurTreuil.Set(0.3);
    }
    else
    {
        m_moteurTreuil.Set(0);
    }
#else
    m_turnAdjustFactor = (m_rightHandController.GetThrottle() + 1.0) / 2.0;
    m_customEntry.SetDouble(m_turnAdjustFactor);
    /*if (m_rightHandController.GetRawButton(2))
    {
        reduction_factor = 1;
    }
    else
    {
        reduction_factor = 0.6;
    }*/
#endif
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
