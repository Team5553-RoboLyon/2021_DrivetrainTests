// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "lib/NL/Characterization/NLCharacterization_Tests.h"

NLCharacterization_Tests::NLCharacterization_Tests(
    rev::CANSparkMax *leftMotor,
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
    double rampVoltage)
    : m_rightMotor(rightMotor),
      m_leftMotor(leftMotor),
      m_rightMotorFollower(rightMotorFollower),
      m_leftMotorFollower(leftMotorFollower),
      m_externalEncoderLeft(externalEncoderLeft),
      m_externalEncoderRight(externalEncoderRight)
{
    //---Declare NetworkTables Entries---
    m_LogFileName = frc::Shuffleboard::GetTab("voltage").Add("Logfile Name", "").WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    m_customEntry = frc::Shuffleboard::GetTab("voltage").Add("Data", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();

    //---set all state---
    int i;

    m_nbTotalTest = nbTestLow + nbTestMedium + nbTestHigh;
    TestData = (TestSpecs *)malloc(sizeof(TestSpecs) * m_nbTotalTest);
    // Low Voltages
    for (i = 0; i < nbTestLow; i++)
    {
        TestData[i * 2].m_voltage = endVoltageLow * (double)(i + 1) / (double)nbTestLow;
        TestData[i * 2].m_flags = 0;

        TestData[i * 2 + 1].m_voltage = -(endVoltageLow * (double)(i + 1) / (double)nbTestLow);
        TestData[i * 2 + 1].m_flags = 0;
    }
    // Medium Voltages
    for (i = 0; i < nbTestMedium; i++)
    {
        TestData[2 * (nbTestLow + i)].m_voltage = endVoltageLow + (endVoltageMedium - endVoltageLow) * (double)(i + 1) / (double)nbTestMedium;
        TestData[2 * (nbTestLow + i)].m_flags = 0;

        TestData[2 * (nbTestLow + i) + 1].m_voltage = -(endVoltageLow + (endVoltageMedium - endVoltageLow) * (double)(i + 1) / (double)nbTestMedium);
        TestData[2 * (nbTestLow + i) + 1].m_flags = 0;
    }

    // High Voltages
    for (i = 0; i < nbTestHigh; i++)
    {
        TestData[2 * (nbTestLow + nbTestMedium + i)].m_voltage = endVoltageMedium + (endVoltageHigh - endVoltageMedium) * (double)(i + 1) / (double)nbTestHigh;
        TestData[2 * (nbTestLow + nbTestMedium + i)].m_flags = 0;
        TestData[2 * (nbTestLow + nbTestMedium + i)].m_ramp = 0;

        TestData[2 * (nbTestLow + nbTestMedium + i) + 1].m_voltage = -(endVoltageMedium + (endVoltageHigh - endVoltageMedium) * (double)(i + 1) / (double)nbTestHigh);
        TestData[2 * (nbTestLow + nbTestMedium + i) + 1].m_flags = 0;

        if (rampVoltage >= TestData[2 * (nbTestLow + nbTestMedium + i)].m_voltage)
        {
            TestData[2 * (nbTestLow + nbTestMedium + i)].m_ramp = rampValue;
            TestData[2 * (nbTestLow + nbTestMedium + i) + 1].m_ramp = rampValue;
        }
    }
}

NLCharacterization_Tests::~NLCharacterization_Tests()
{
    free(TestData);
}

void NLCharacterization_Tests::nextTest()
{
    assert(m_state == State::Stopped);
    assert(m_CurrentTestID <= m_nbTotalTest);
    m_CurrentTestID++;
}
void NLCharacterization_Tests::previousTest()
{
    assert(m_state == State::Stopped);
    assert(m_CurrentTestID <= m_nbTotalTest);
    m_CurrentTestID--;
}
void NLCharacterization_Tests::setCurrentTest(uint8_t testNumber)
{
    assert(m_state == State::Stopped);
    m_CurrentTestID = testNumber;
}

void NLCharacterization_Tests::start()
{
    assert(m_state == State::Stopped);
    assert(m_CurrentTestID <= m_nbTotalTest);
    //setting ramp
    m_oldRamp = m_rightMotor->GetOpenLoopRampRate();
    m_rightMotor->SetOpenLoopRampRate(TestData[m_CurrentTestID].m_ramp);
    m_rightMotorFollower->SetOpenLoopRampRate(TestData[m_CurrentTestID].m_ramp);
    m_leftMotor->SetOpenLoopRampRate(TestData[m_CurrentTestID].m_ramp);
    m_leftMotorFollower->SetOpenLoopRampRate(TestData[m_CurrentTestID].m_ramp);

    //set state of test
    m_state = State::AskForStart;
}
void NLCharacterization_Tests::stop()
{
    assert(m_state == State::Started);
    //setting ramp
    m_rightMotor->SetOpenLoopRampRate(m_oldRamp);
    m_rightMotorFollower->SetOpenLoopRampRate(m_oldRamp);
    m_leftMotor->SetOpenLoopRampRate(m_oldRamp);
    m_leftMotorFollower->SetOpenLoopRampRate(m_oldRamp);

    //set state of test
    m_state = State::AskForStop;
}

NLCharacterization_Tests::State NLCharacterization_Tests::getState()
{
    return m_state;
}
void NLCharacterization_Tests::fastLoop()
{
    switch (m_state)
    {
    case State::Stopped:
        //Do nothing
        break;

    case State::AskForStart:
        char prefix[512];
        char invertedPrefix[8];
        sprintf(invertedPrefix, "L%d%dR%d%d", (int)m_leftMotor->GetInverted(), (int)m_leftMotorFollower->GetInverted(), (int)m_rightMotor->GetInverted(), (int)m_rightMotorFollower->GetInverted());

        if (TestData[m_CurrentTestID].m_voltage < 0)
        {
            sprintf(prefix, "/home/lvuser/logs/-_%s_%d_%.2fvolts_", invertedPrefix, m_CurrentTestID, TestData[m_CurrentTestID].m_voltage);
        }
        else
        {
            sprintf(prefix, "/home/lvuser/logs/+_%s_%d_%.2fvolts_", invertedPrefix, m_CurrentTestID, TestData[m_CurrentTestID].m_voltage);
        }

        m_LogFile = new CSVLogFile(prefix, "encoderGetD", "encoderGetG", "encoderGetRawD", "encoderGetRawG", "Theorical Voltage", "BusVoltageD1", "BusVoltageD2", "BusVoltageG1", "BusVoltageG2", "AppliedOutputD1", "AppliedOutputD2", "AppliedOutputG1", "AppliedOutputG2", "currentD1", "currentD2", "currentG1", "currentG2", "rampActive");
        m_LogFileName.SetString(m_LogFile->GetFileName());

        testRunning();
        BITSET(TestData[m_CurrentTestID].m_flags, 0);
        m_time0 = std::time(0);

        m_leftMotor->Set(TestData[m_CurrentTestID].m_voltage / m_leftMotor->GetBusVoltage());
        m_leftMotorFollower->Set(TestData[m_CurrentTestID].m_voltage / m_leftMotorFollower->GetBusVoltage());
        m_rightMotor->Set(TestData[m_CurrentTestID].m_voltage / m_rightMotor->GetBusVoltage());
        m_rightMotorFollower->Set(TestData[m_CurrentTestID].m_voltage / m_rightMotorFollower->GetBusVoltage());

        m_state = State::Started;
        break;

    case State::Started:
        if (TestData[m_CurrentTestID].m_voltage > 0)
        {
            assert(m_externalEncoderLeft->Get() > -2048);
            assert(m_externalEncoderRight->Get() > -2048);
        }
        else
        {
            assert(m_externalEncoderLeft->Get() < 2048);
            assert(m_externalEncoderRight->Get() < 2048);
        }
        if (std::time(0) - m_time0 < TestData[m_CurrentTestID].m_ramp)
        {
            m_ramp = TestData[m_CurrentTestID].m_ramp;
        }
        else
        {
            m_ramp = 0;
        }
        m_LogFile->Log(m_externalEncoderRight->Get(),
                       m_externalEncoderLeft->Get(),
                       m_externalEncoderRight->GetRaw(),
                       m_externalEncoderLeft->GetRaw(),
                       TestData[m_CurrentTestID].m_voltage,
                       m_rightMotor->GetBusVoltage(),
                       m_rightMotorFollower->GetBusVoltage(),
                       m_leftMotor->GetBusVoltage(),
                       m_leftMotorFollower->GetBusVoltage(),
                       m_rightMotor->GetAppliedOutput(),
                       m_rightMotorFollower->GetAppliedOutput(),
                       m_leftMotor->GetAppliedOutput(),
                       m_leftMotorFollower->GetAppliedOutput(),
                       m_rightMotor->GetOutputCurrent(),
                       m_rightMotorFollower->GetOutputCurrent(),
                       m_leftMotor->GetOutputCurrent(),
                       m_leftMotorFollower->GetOutputCurrent(),
                       TestData[m_CurrentTestID].m_ramp);
        break;

    case State::AskForStop:
        m_leftMotor->StopMotor();
        m_leftMotorFollower->StopMotor();
        m_rightMotor->StopMotor();
        m_rightMotorFollower->StopMotor();
        delete m_LogFile;
        m_state = State::Stopped;
        break;

    default:
        //Do nothing
        break;
    }
}

void NLCharacterization_Tests::waitingForTest()
{
    char txt[256];
    if (!BITGET(TestData[m_CurrentTestID].m_flags, 0))
    {
        sprintf(txt, "TEST %d / %d [ %.2f Volts || Rampe : %.2f ]. En Attente ... Appuyer sur A pour Démarrer.", m_CurrentTestID, m_nbTotalTest * 2, TestData[m_CurrentTestID].m_voltage, TestData[m_CurrentTestID].m_ramp);
        m_customEntry.SetString(txt);
    }
    else
    {
        sprintf(txt, "TEST %d / %d [ %.2f Volts || Rampe : %.2f ]. DEJA EFFECTUE ! ... Appuyer sur A pour Démarrer à nouveau.( Le précédent LOGFILE sera conservé.)", m_CurrentTestID, m_nbTotalTest, TestData[m_CurrentTestID].m_voltage, TestData[m_CurrentTestID].m_ramp);
        m_customEntry.SetString(txt);
    }
}
void NLCharacterization_Tests::allTestsDone()
{
    char txt[256];
    sprintf(txt, "%d / %d  TESTS EFFECTUES ! ", m_nbTotalTest * 2, m_nbTotalTest * 2);
    m_customEntry.SetString(txt);
}
void NLCharacterization_Tests::testRunning()
{
    char txt[256];
    sprintf(txt, "TEST %d / %d [ %.2f Volts || Rampe : %.2f ]. En cours ... Appuyer à nouveau sur A pour arrêter.", m_CurrentTestID, m_nbTotalTest * 2, TestData[m_CurrentTestID].m_voltage, TestData[m_CurrentTestID].m_ramp);
    m_customEntry.SetString(txt);
}