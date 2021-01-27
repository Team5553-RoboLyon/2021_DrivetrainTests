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

#define TRACKWIDTH 0.61f
#define HALF_TRACKWIDTH (TRACKWIDTH / 2.0f)

#define AMAX 5.1 // Acceleration Max  au PIF .. à définir aux encodeurs
#define VMAX 3.4 // vitesse Max  théorique (3,395472 sur JVN-DT) .. à vérifier aux encodeurs
#define WMAX                       \
    (((2.0 * VMAX) / TRACKWIDTH) / \
     1.7) // vitesse angulaire Max theorique	.. à modifier avec Garice

// Flags Manipulation
#define FLAG_TOGGLE(val, flag) ((val) ^= (flag))
#define FLAG_ON(val, flag) ((val) |= (flag))
#define FLAG_OFF(val, flag) ((val) &= ~(flag))
#define ISFLAG_ON(val, flag) ((val) & (flag))                                   // !! ZERO or NON ZERO value !!! BE AWARE THAT NON ZERO DOESN'T MEAN 1 !!!
#define ISFLAG_OFF(val, flag) (!((val) & (flag)))                               // !! ZERO or NON ZERO value !!! BE AWARE THAT NON ZERO DOESN'T MEAN 1 !!!
#define FLAGS_TEST(val, bmask, flags) (((val) & (bmask)) == (flags))            // NFALSE or NTRUE
#define SET_FLAGS(val, bmask, flags) ((val) = (((val) & (~(bmask))) | (flags))) // RESET FLAGS BITS and Set them all like flags.
#define RESET_FLAGS(val, bmask)		((val)&=~(bmask)))						// Set all FLAGS BITS to ZERO

#define VOLTAGE_COMPENSATION_VALUE 11.5
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
        forward = Deadband(forward);
        turn = Deadband(turn, 0.2);

        /*
    double c = 0.35 * (turn * 5.0 * (abs(turn) + 1) / (abs(forward) + 1));
    if (turn < 0.0) {
        m_drivetrain->Drive(forward * ((c + 1) / (1 - c)), forward);

    } else {
        m_drivetrain->Drive(forward, forward * ((1 - c) / (c + 1)));
    }*/

        double v = forward * VMAX;
        double w = turn * WMAX;

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

void Robot::Drive(double jy, double jx)
{
    jy = Deadband(jy, 0.1);
    jx = Deadband(jx, 0.2);
    std::cout << jy << "       " << jx << std::endl;

    getSpeedsAndAccelerations(&m_va_left, &m_va_right, &m_va_max, jx, jy);

    m_moteurGauche.Set(m_kv.getVoltage(0, &m_va_left) / m_moteurGauche.GetBusVoltage());
    m_moteurGaucheFollower.Set(m_kv.getVoltage(1, &m_va_left) / m_moteurGaucheFollower.GetBusVoltage());
    m_moteurDroite.Set(m_kv.getVoltage(2, &m_va_right) / m_moteurDroite.GetBusVoltage());
    m_moteurDroiteFollower.Set(m_kv.getVoltage(3, &m_va_right) / m_moteurDroiteFollower.GetBusVoltage());
}

void Robot::RobotInit()
{
    /*m_moteurDroite.RestoreFactoryDefaults();
    m_moteurGauche.RestoreFactoryDefaults();
    m_moteurDroiteFollower.RestoreFactoryDefaults();
    m_moteurGaucheFollower.RestoreFactoryDefaults();
    m_moteurGaucheShooter.RestoreFactoryDefaults();
    m_moteurDroiteShooter.RestoreFactoryDefaults();*/

    /*m_moteurDroite.SetOpenLoopRampRate(TIME_RAMP);
    m_moteurGauche.SetOpenLoopRampRate(TIME_RAMP);
    m_moteurDroiteFollower.SetOpenLoopRampRate(TIME_RAMP);
    m_moteurGaucheFollower.SetOpenLoopRampRate(TIME_RAMP);
    */
    /*m_moteurDroite.EnableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);
    m_moteurGauche.EnableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);
    m_moteurDroiteFollower.EnableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);
    m_moteurGaucheFollower.EnableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);*/

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

    m_gyro.Calibrate();

    m_PowerEntry = frc::Shuffleboard::GetTab("voltage").Add("Voltage", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    m_LogFilename = frc::Shuffleboard::GetTab("voltage").Add("Logfile Name", "").WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    frc::Shuffleboard::GetTab("voltage").Add(m_gyro).WithWidget(frc::BuiltInWidgets::kGyro);
    /*m_moteurGaucheShooter.SetClosedLoopRampRate(0.6);
    m_moteurDroiteShooter.SetClosedLoopRampRate(0.6);*/

    //m_moteurDroiteFollower.Follow(m_moteurDroite);
    //m_moteurGaucheFollower.Follow(m_moteurGauche);

    m_moteurDroite.SetInverted(false);
    m_moteurDroiteFollower.SetInverted(false);

    m_moteurGauche.SetInverted(true);
    m_moteurGaucheFollower.SetInverted(true);

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
    m_kv.SetMotorCoefficients(0, 0, 2.80842, 0.16071, 0.1467);
    //Left 2 Forward
    m_kv.SetMotorCoefficients(1, 0, 2.80833, 0.1549, 0.14571);
    //Right 1 Forward
    m_kv.SetMotorCoefficients(2, 0, 2.80423, 0.13559, 0.16904);
    //Right 2 Forward
    m_kv.SetMotorCoefficients(3, 0, 2.80524, 0.13376, 0.16744);
    //Left 1 Backward
    m_kv.SetMotorCoefficients(0, 1, 2.80397, 0.14271, -0.15557);
    //Left 2 Backward
    m_kv.SetMotorCoefficients(1, 1, 2.80367, 0.13479, -0.15524);
    //Right 1 Backward
    m_kv.SetMotorCoefficients(2, 1, 2.83487, 0.12593, -0.14335);
    //Right 2 Backward
    m_kv.SetMotorCoefficients(3, 1, 2.83517, 0.12205, -0.14276);

    m_va_max.m_speed = 3;
    m_va_max.m_acceleration = 5;

    m_va_left.m_speed = 0;
    m_va_left.m_acceleration = 0;
    m_va_right.m_speed = 0;
    m_va_right.m_acceleration = 0;
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
    m_encodeurExterneDroite.Reset();

    m_encodeurExterneGauche.Reset();
}

void Robot::TeleopPeriodic()
{
    //DriveOld(-m_driverController.GetY(frc::GenericHID::JoystickHand::kLeftHand), m_driverController.GetX(frc::GenericHID::JoystickHand::kRightHand));
    Drive(-m_driverController.GetY(frc::GenericHID::JoystickHand::kLeftHand), m_driverController.GetX(frc::GenericHID::JoystickHand::kRightHand));

    /*if (m_driverController.GetAButton())
    {
        m_moteurIntake.Set(0.7);
    }
    else
    {
        m_moteurIntake.Set(0);
    }*/

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

                char prefix[256];
                if (TestData[CurrentTestID].m_voltage < 0)
                {
                    sprintf(prefix, "/home/lvuser/logs/-test_%d_%.2fvolts_", CurrentTestID, TestData[CurrentTestID].m_voltage);
                }
                else
                {
                    sprintf(prefix, "/home/lvuser/logs/+test_%d_%.2fvolts_", CurrentTestID, TestData[CurrentTestID].m_voltage);
                }

                m_LogFile = new CSVLogFile(prefix, "Right", "Left", "neoD1", "neoD2", "neoG1", "neoG2", "gyro", "Theorical Voltage", "BusVoltageD1", "BusVoltageD2", "BusVoltageG1", "BusVoltageG2", "AppliedOutputD1", "AppliedOutputD2", "AppliedOutputG1", "AppliedOutputG2", "currentD1", "currentD2", "currentG1", "currentG2", "rampActive");
                m_LogFilename.SetString(m_LogFile->GetFileName());
                m_encodeurExterneDroite.Reset();
                m_encodeurExterneGauche.Reset();
            }
        }
        else
        {
            CurrentTestID += 1;
            if (CurrentTestID < TEST_TOTAL_NB * 2)
            {
                delete m_LogFile;
                messageTestEnAttente();
            }
            else
            {
                messageTestTousEffectues();
            }
        }
    }

    if (m_override)
    {
        if (std::time(0) - m_time0 < TIME_RAMP)
        {
            m_ramp = 1;
        }
        else
        {
            m_ramp = 0;
        }
        double md0, md1, mg0, mg1;
        md0 = m_moteurDroite.GetBusVoltage() * m_moteurDroite.GetAppliedOutput();
        md1 = m_moteurDroiteFollower.GetBusVoltage() * m_moteurDroiteFollower.GetAppliedOutput();
        mg0 = m_moteurGauche.GetBusVoltage() * m_moteurGauche.GetAppliedOutput();
        mg1 = m_moteurGaucheFollower.GetBusVoltage() * m_moteurGaucheFollower.GetAppliedOutput();

        m_LogFile->Log(m_encodeurExterneDroite.GetDistance(), m_encodeurExterneGauche.GetDistance(), m_encodeurDroite1.GetPosition(), m_encodeurDroite2.GetPosition(), m_encodeurGauche1.GetPosition(), m_encodeurGauche2.GetPosition(), m_gyro.GetAngle(), TestData[CurrentTestID].m_voltage, m_moteurDroite.GetBusVoltage(), m_moteurDroiteFollower.GetBusVoltage(), m_moteurGauche.GetBusVoltage(), m_moteurGaucheFollower.GetBusVoltage(), m_moteurDroite.GetAppliedOutput(), m_moteurDroiteFollower.GetAppliedOutput(), m_moteurGauche.GetAppliedOutput(), m_moteurGaucheFollower.GetAppliedOutput(), m_moteurDroite.GetOutputCurrent(), m_moteurDroiteFollower.GetOutputCurrent(), m_moteurGauche.GetOutputCurrent(), m_moteurGaucheFollower.GetOutputCurrent(), m_ramp);
        //std::cout << mg0 << " " << mg1 << " " << md0 << " " << md1 << std::endl;
        //std::cout << m_encodeurExterneDroite.GetDistance() << std::endl;
    }

    if (m_driverController.GetYButtonPressed())
    {
        m_isLogging = !m_isLogging;
        //std::cout << m_override << std::endl;

        m_time0 = std::time(0);

        if (m_isLogging)
        {
            m_LogFileDriving = new CSVLogFile("/home/lvuser/logs/freeRiding", "Right", "Left", "neoD1", "neoD2", "neoG1", "neoG2", "gyro", "Theorical Voltage", "BusVoltageD1", "BusVoltageD2", "BusVoltageG1", "BusVoltageG2", "AppliedOutputD1", "AppliedOutputD2", "AppliedOutputG1", "AppliedOutputG2", "currentD1", "currentD2", "currentG1", "currentG2", "rampActive");
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
        m_LogFileDriving->Log(m_encodeurExterneDroite.GetDistance(), m_encodeurExterneGauche.GetDistance(), m_encodeurDroite1.GetPosition(), m_encodeurDroite2.GetPosition(), m_encodeurGauche1.GetPosition(), m_encodeurGauche2.GetPosition(), m_gyro.GetAngle(), TestData[CurrentTestID].m_voltage, m_moteurDroite.GetBusVoltage(), m_moteurDroiteFollower.GetBusVoltage(), m_moteurGauche.GetBusVoltage(), m_moteurGaucheFollower.GetBusVoltage(), m_moteurDroite.GetAppliedOutput(), m_moteurDroiteFollower.GetAppliedOutput(), m_moteurGauche.GetAppliedOutput(), m_moteurGaucheFollower.GetAppliedOutput(), m_moteurDroite.GetOutputCurrent(), m_moteurDroiteFollower.GetOutputCurrent(), m_moteurGauche.GetOutputCurrent(), m_moteurGaucheFollower.GetOutputCurrent(), m_ramp);
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
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
