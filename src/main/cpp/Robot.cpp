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

#define TRACKWIDTH 0.61f
#define HALF_TRACKWIDTH (TRACKWIDTH / 2.0f)

#define AMAX 7   // Acceleration Max  au PIF .. à définir aux encodeurs
#define VMAX 3.4 // vitesse Max  théorique (3,395472 sur JVN-DT) .. à vérifier aux encodeurs
#define JMAX 45
#define WMAX ((2.0 * VMAX) / TRACKWIDTH) * 0.6 // vitesse angulaire Max theorique
#define PATHNAME "0_8Great2UJ30A1_5V1"

// Flags Manipulation
/*
#define FLAG_TOGGLE(val, flag) ((val) ^= (flag))
#define FLAG_ON(val, flag) ((val) |= (flag))
#define FLAG_OFF(val, flag) ((val) &= ~(flag))
#define ISFLAG_ON(val, flag) ((val) & (flag))                                   // !! ZERO or NON ZERO value !!! BE AWARE THAT NON ZERO DOESN'T MEAN 1 !!!
#define ISFLAG_OFF(val, flag) (!((val) & (flag)))                               // !! ZERO or NON ZERO value !!! BE AWARE THAT NON ZERO DOESN'T MEAN 1 !!!
#define FLAGS_TEST(val, bmask, flags) (((val) & (bmask)) == (flags))            // NFALSE or NTRUE
#define SET_FLAGS(val, bmask, flags) ((val) = (((val) & (~(bmask))) | (flags))) // RESET FLAGS BITS and Set them all like flags.
#define RESET_FLAGS(val, bmask)		((val)&=~(bmask)))						// Set all FLAGS BITS to ZERO
*/
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

#define TIME_RAMP 0.6

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

    Message = frc::Shuffleboard::GetTab("RBL").Add("Message", "").WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
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

void Robot::DriveWithoutCharacterization(double forward, double turn)
{
    double v = forward * VMAX;
    double w = turn * WMAX * m_turnAdjustFactor;

    double lwheel = v + (w * HALF_TRACKWIDTH);
    double rwheel = v - (w * HALF_TRACKWIDTH);

    double k;
    k = 1.0 / (NMAX(VMAX, NMAX(NABS(lwheel), NABS(rwheel))));
    lwheel *= k;
    rwheel *= k;

    m_moteurGauche.Set(lwheel);
    m_moteurGaucheFollower.Set(lwheel);
    m_moteurDroite.Set(rwheel);
    m_moteurDroiteFollower.Set(rwheel);
}

void Robot::Drive(double forward, double turn)
{
    forward = Deadband(forward, 0.2);
    turn = Deadband(turn, 0.2);

    m_driveModeSelected = m_driveModeChooser.GetSelected();
    if (m_driveModeSelected == kdriveModeCharacterization)
    {
        DriveWithCharacterization(forward, turn);
    }
    else if (m_driveModeSelected == kdriveModeNoCharacterization)
    {
        DriveWithoutCharacterization(forward, turn);
    }
    else
    {
        return;
    }
}

void Robot::DriveWithCharacterization(double forward, double turn)
{
    double v = forward * VMAX;
    double w = turn * WMAX * m_turnAdjustFactor;

    double lwheel = v + (w * HALF_TRACKWIDTH);
    double rwheel = v - (w * HALF_TRACKWIDTH);

    double k;
    k = 1.0 / (NMAX(VMAX, NMAX(NABS(lwheel), NABS(rwheel))));
    lwheel *= k;
    rwheel *= k;
    double target_left_speed = lwheel * VMAX;
    double target_right_speed = rwheel * VMAX;

    updateVelocityAndAcceleration(&m_va_left, &m_va_max, target_left_speed, 0.02);
    updateVelocityAndAcceleration(&m_va_right, &m_va_max, target_right_speed, 0.02);

    m_moteurGauche.Set(m_kv.getVoltage(0, &m_va_left) / m_moteurGauche.GetBusVoltage());
    m_moteurGaucheFollower.Set(m_kv.getVoltage(1, &m_va_left) / m_moteurGauche.GetBusVoltage());
    m_moteurDroite.Set(m_kv.getVoltage(2, &m_va_right) / m_moteurDroite.GetBusVoltage());
    m_moteurDroiteFollower.Set(m_kv.getVoltage(3, &m_va_right) / m_moteurDroite.GetBusVoltage());
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

    m_PowerEntry = frc::Shuffleboard::GetTab("RBL").Add("Accélération", 7.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    m_LogFilename = frc::Shuffleboard::GetTab("RBL").Add("Logfile Name", "").WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    m_customEntry = frc::Shuffleboard::GetTab("RBL").Add("Data", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
#if IMU
    m_speedY = frc::Shuffleboard::GetTab("RBL").Add("speedY", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    m_speedX = frc::Shuffleboard::GetTab("RBL").Add("speedX", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
#endif
    frc::Shuffleboard::GetTab("RBL").Add(m_gyro).WithWidget(frc::BuiltInWidgets::kGyro);

    m_driveModeChooser.SetDefaultOption(kdriveModeNoCharacterization, kdriveModeNoCharacterization);
    m_driveModeChooser.AddOption(kdriveModeCharacterization, kdriveModeCharacterization);
    m_driveModeChooser.AddOption(kdriveModeDisabled, kdriveModeDisabled);
    frc::Shuffleboard::GetTab("RBL").Add("Mode de conduite", m_driveModeChooser).WithWidget(frc::BuiltInWidgets::kComboBoxChooser);

    m_gyro.Calibrate();
    m_gyro.Reset();
    /*m_moteurGaucheShooter.SetClosedLoopRampRate(0.6);
    m_moteurDroiteShooter.SetClosedLoopRampRate(0.6);*/
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
    m_kv.SetMotorCoefficients(0, 0, 2.913921067637356, 0.5330842773642531, 0.17048001042741223);
    //Left 2 Forward
    m_kv.SetMotorCoefficients(1, 0, 2.9199808165196677, 0.5322737374019705, 0.16821280977894304);
    //Right 1 Forward
    m_kv.SetMotorCoefficients(2, 0, 2.9746284237846483, 0.5426701407165282, 0.15729272509030423);
    //Right 2 Forward
    m_kv.SetMotorCoefficients(3, 0, 2.9761769661388064, 0.5386750620636838, 0.15697255085663597);
    //Left 1 Backward
    m_kv.SetMotorCoefficients(0, 1, 2.9173147307197733, 0.522716984981712, -0.14941576500105835);
    //Left 2 Backward
    m_kv.SetMotorCoefficients(1, 1, 2.9198355067184707, 0.5222812439428668, -0.14961511479256862);
    //Right 1 Backward
    m_kv.SetMotorCoefficients(2, 1, 2.99608505650355, 0.5196244708988396, -0.1643722172976183);
    //Right 2 Backward
    m_kv.SetMotorCoefficients(3, 1, 2.9960377905386903, 0.5336450448724275, -0.1640406680476354);

    m_va_max.m_speed = VMAX;
    m_va_max.m_acceleration = AMAX;
    m_va_max.m_jerk = JMAX;

    m_va_left.m_speed = 0;
    m_va_left.m_acceleration = 0;
    m_va_right.m_speed = 0;
    m_va_right.m_acceleration = 0;

    m_moteurDroite.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 5);
    m_moteurDroite.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 5);
    m_moteurDroite.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);

    m_moteurDroiteFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 5);
    m_moteurDroiteFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 5);
    m_moteurDroiteFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);

    m_moteurGauche.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 5);
    m_moteurGauche.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 5);
    m_moteurGauche.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);

    m_moteurGaucheFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 5);
    m_moteurGaucheFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 5);
    m_moteurGaucheFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);
}

void Robot::RobotPeriodic()
{
#if XBOX_CONTROLLER
#else
    if (m_leftHandController.GetRawButton(1) && m_rightHandController.GetRawButton(1))
    {
        std::cout << "ARRET D'URGENCE DÉCLENCHÉ !" << std::endl;
        FILE *eStoppedFile = fopen(eStoppedFilePath, "a");
        fclose(eStoppedFile);
        abort();
    }
    if (m_leftHandController.GetRawButton(11) && m_rightHandController.GetRawButton(11) && m_leftHandController.GetRawButton(12) && m_rightHandController.GetRawButton(12))
    {
        std::cout << "ARRET D'URGENCE DÉSACTIVÉ ! BE CAREFUL ! " << std::endl;
        remove(eStoppedFilePath);
    }
#endif
    struct stat buffer;
    int exist = stat(eStoppedFilePath, &buffer);
    if (exist == 0)
    {
        std::cout << "ARRET D'URGENCE ACTIF ! Pour sortir, appuyez sur les deux boutons 11 et 12 de chaque joystick." << std::endl;
        abort();
    }
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
    Drive(-m_leftHandController.GetY(), m_rightHandController.GetZ());
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
        m_isPathFollowing = !m_isPathFollowing;
        //std::cout << m_override << std::endl;

        m_time0 = std::time(0);
        if (m_isPathFollowing)
        {
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

            m_encodeurExterneDroite.Reset();
            m_encodeurExterneGauche.Reset();
        }
        if (m_isLogging)
        {
            //m_LogFileDriving = new CSVLogFile("/home/lvuser/logs/freeRiding", "encoderGetD", "encoderGetG", "encoderGetRawD", "encoderGetRawG", "Theorical Voltage", "BusVoltageD1", "BusVoltageD2", "BusVoltageG1", "BusVoltageG2", "AppliedOutputD1", "AppliedOutputD2", "AppliedOutputG1", "AppliedOutputG2", "currentD1", "currentD2", "currentG1", "currentG2", "rampActive");
            char logName[128];
            sprintf(logName, "/home/lvuser/logs/%s", PATHNAME);
            m_LogFileDriving = new CSVLogFile(logName, "encodeurG", "encodeurD", "speed", "SpeedG", "SpeedR", "AccelG", "AccelR", "ErrorG", "ErrorR", "GyroAngle");
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

        /*
	// *****************************************************    'THE' METHOD(e)
	// feed back:
	// avec les encodeurs on estime la position du robot:
	//			l = distance parcourue par la roue gauche depuis le dernier reset encodeur.
	//			r = distance parcourue par la roue droite depuis le dernier reset encodeur.
	Nf32 l = (m_encodeurExterneGauche.GetDistance()*NF32_2PI*WHEEL_RADIUS)/2048.0f;
	Nf32 r = (m_encodeurExterneDroite.GetDistance()*NF32_2PI*WHEEL_RADIUS)/2048.0f;

	//			dl et dr = distances parcourues par les roues gauche et droite depuis le dernier call.
	//			(note dl/dt = vitesse roue gauche et dr/dt = vitesse roue droite droite )
	Nf32 dl = l - m_dsLeftWheel;
	Nf32 dr = r - m_dsRightWheel;

	// mise à jour des distances totales parcourues par chaque roue.
	m_dsLeftWheel = l;
	m_dsRightWheel = r;

	// mise à jour de la position et de l'angle "estimés" du robot.
	Nf32 v = NLODOMETRY_DRIVETRAIN_V_FROM_WHEELS(dl, dr);
	Nf32 w = NLODOMETRY_DRIVETRAIN_W_FROM_WHEELS(dl, dr, TRACKWIDTH);

	// méthode simplifiée pour de petites valeurs de w où on considère que le déplacement du robot est un petit segment de droite
	m_estimatedAngle += w;
	//m_estimatedPosition.x += cos(m_estimatedAngle)*v;
	//m_estimatedPosition.y += sin(m_estimatedAngle)*v;
	//m_estimatedPosition.z = 0.0f;
	
	/*
	// méthode "complète" considérant que le déplacement du robot est un arc ( et non pas un segment de droite comme pour la méthode simplifiée )
	if (w == 0.0f) // mettre peut-être ici une tolérance if (NABS(w)< 0.000001f)
	{
		m_estimatedPosition.x	+= cos(m_estimatedAngle)*v;
		m_estimatedPosition.y	+= sin(m_estimatedAngle)*v;
		m_estimatedPosition.z	 = 0.0f;
		m_estimatedArcCenterPos  = m_estimatedPosition;
	}
	else
	{
		rcenter = v / w;
		m_estimatedArcCenterPos.x = m_estimatedPosition.x - rcenter * sin(m_estimatedAngle);//  ==m_position.x - rcenter *cos(m_angle - NF32_PI_2);
		m_estimatedArcCenterPos.y = m_estimatedPosition.y + rcenter * cos(m_estimatedAngle);//  ==m_position.y - rcenter *sin(m_angle - NF32_PI_2);
		m_estimatedArcCenterPos.z = 0.0f;

		m_estimatedAngle += w;
		m_estimatedPosition.x = m_estimatedArcCenterPos.x + rcenter * cos(m_estimatedAngle - NF32_PI_2);
		m_estimatedPosition.y = m_estimatedArcCenterPos.y + rcenter * sin(m_estimatedAngle - NF32_PI_2);
		m_estimatedPosition.z = 0.0f;
	}
	*/
        // feed forward : S(imple)State
        /*
	m_currrentSState.m_kin.m_t += 0.02f;
	m_trajectoryStatesPack.getState(&m_currrentSState, m_currrentSState.m_kin.m_t);
	
    Nf32 ds = m_currrentSState.m_kin.m_s - m_prevS;
	Nf32 k  = (m_currrentSState.m_localCurvature + m_prevK) / 2.0f;
	if (k != 0.0f)
	{
		Nf32 radius = 1 / k;
		m_refLeftS  += ds * (radius - HALF_TRACKWIDTH) / radius;
		m_refRightS += ds * (radius + HALF_TRACKWIDTH) / radius;
	}
	else
	{
		m_refLeftS	+= ds;
		m_refRightS += ds;
	}
	m_prevK = m_currrentSState.m_localCurvature;
	m_prevS = m_currrentSState.m_kin.m_s;
	
	m_errorLeft.update(m_refLeftS - m_dsLeftWheel);
	m_errorRight.update(m_refRightS - m_dsRightWheel );

	m_leftErrorVoltage = m_pid.command(&m_errorLeft);
	m_rightErrorVoltage = m_pid.command(&m_errorRight);

	// 2) avec k on a R = 1/k et avec R on a la distribution gauche droite
	if (m_currrentSState.m_localCurvature == 0.0f)
	{
        std::cout << " PID: "<< m_pid.m_kP << " "<< m_pid.m_kI <<" "<< m_pid.m_kD << std::endl;
        std::cout << " MG: "<<m_motorCharacterization[2].getVoltage(m_currrentSState.m_kin.m_v, m_currrentSState.m_kin.m_a) + m_leftErrorVoltage << std::endl;
        std::cout << " MD: "<<m_motorCharacterization[0].getVoltage(m_currrentSState.m_kin.m_v, m_currrentSState.m_kin.m_a) + m_rightErrorVoltage << std::endl;
		m_moteurGauche.SetVoltage( units::volt_t(m_motorCharacterization[2].getVoltage(m_currrentSState.m_kin.m_v, m_currrentSState.m_kin.m_a) + m_leftErrorVoltage) );
		m_moteurGaucheFollower.SetVoltage( units::volt_t(m_motorCharacterization[3].getVoltage(m_currrentSState.m_kin.m_v, m_currrentSState.m_kin.m_a) + m_leftErrorVoltage) );
     	m_moteurDroite.SetVoltage( units::volt_t(m_motorCharacterization[0].getVoltage(m_currrentSState.m_kin.m_v, m_currrentSState.m_kin.m_a) + m_rightErrorVoltage) );
		m_moteurDroiteFollower.SetVoltage( units::volt_t(m_motorCharacterization[1].getVoltage(m_currrentSState.m_kin.m_v, m_currrentSState.m_kin.m_a) + m_rightErrorVoltage) );
        
     }
	else 
	{
		r = 1.0f / m_currrentSState.m_localCurvature;
		Nf32 left = (r - HALF_TRACKWIDTH)*m_currrentSState.m_localCurvature;
		Nf32 right = (r + HALF_TRACKWIDTH)*m_currrentSState.m_localCurvature;
        std::cout << " MG: "<<m_motorCharacterization[2].getVoltage(m_currrentSState.m_kin.m_v*left, m_currrentSState.m_kin.m_a*left) + m_leftErrorVoltage << std::endl;
        std::cout << " MD: "<<m_motorCharacterization[0].getVoltage(m_currrentSState.m_kin.m_v*right, m_currrentSState.m_kin.m_a*right) + m_rightErrorVoltage << std::endl;
        std::cout << " PID: "<< m_pid.m_kP << " "<< m_pid.m_kI <<" "<< m_pid.m_kD << std::endl;
        
		m_moteurGauche.SetVoltage( units::volt_t(m_motorCharacterization[2].getVoltage(m_currrentSState.m_kin.m_v*left, m_currrentSState.m_kin.m_a*left) + m_leftErrorVoltage) );
		m_moteurGaucheFollower.SetVoltage( units::volt_t(m_motorCharacterization[3].getVoltage(m_currrentSState.m_kin.m_v*left, m_currrentSState.m_kin.m_a*left) + m_leftErrorVoltage) );
     	m_moteurDroite.SetVoltage( units::volt_t(m_motorCharacterization[0].getVoltage(m_currrentSState.m_kin.m_v*right, m_currrentSState.m_kin.m_a*right) + m_rightErrorVoltage) );
		m_moteurDroiteFollower.SetVoltage( units::volt_t(m_motorCharacterization[1].getVoltage(m_currrentSState.m_kin.m_v*right, m_currrentSState.m_kin.m_a*right) + m_rightErrorVoltage) );
    
    }*/

        /*m_LogFileDriving->Log(m_encodeurExterneDroite.Get(),
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
                              m_ramp);*/
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
