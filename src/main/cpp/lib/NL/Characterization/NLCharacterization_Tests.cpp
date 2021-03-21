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
    //---set all state---
    int i;
    std::cout << "Constructeur démarré" << std::endl;
    m_nbTotalTest = (nbTestLow + nbTestMedium + nbTestHigh) * 2;
    TestData = (TestSpecs *)malloc(sizeof(TestSpecs) * m_nbTotalTest);
    // Low Voltages
    for (i = 0; i < nbTestLow; i++)
    {
        TestData[i * 2].m_voltage = endVoltageLow * (double)(i + 1) / (double)nbTestLow;
        TestData[i * 2].m_flags = 0;
        TestData[i * 2].m_ramp = 0;

        TestData[i * 2 + 1].m_voltage = -(endVoltageLow * (double)(i + 1) / (double)nbTestLow);
        TestData[i * 2 + 1].m_flags = 0;
        TestData[i * 2 + 1].m_ramp = 0;

        if (rampVoltage <= TestData[2 * i].m_voltage)
        {
            TestData[2 * i].m_ramp = rampValue;
            TestData[2 * i + 1].m_ramp = rampValue;
        }
    }
    std::cout << "Low voltage effectué" << std::endl;
    // Medium Voltages
    for (i = 0; i < nbTestMedium; i++)
    {
        TestData[2 * (nbTestLow + i)].m_voltage = endVoltageLow + (endVoltageMedium - endVoltageLow) * (double)(i + 1) / (double)nbTestMedium;
        TestData[2 * (nbTestLow + i)].m_flags = 0;
        TestData[2 * (nbTestLow + i)].m_ramp = 0;

        TestData[2 * (nbTestLow + i) + 1].m_voltage = -(endVoltageLow + (endVoltageMedium - endVoltageLow) * (double)(i + 1) / (double)nbTestMedium);
        TestData[2 * (nbTestLow + i) + 1].m_flags = 0;
        TestData[2 * (nbTestLow + i) + 1].m_ramp = 0;

        if (rampVoltage <= TestData[2 * (nbTestLow + i)].m_voltage)
        {
            TestData[2 * (nbTestLow + i)].m_ramp = rampValue;
            TestData[2 * (nbTestLow + i) + 1].m_ramp = rampValue;
        }
    }
    std::cout << "Medium voltage effectué" << std::endl;
    // High Voltages
    for (i = 0; i < nbTestHigh; i++)
    {
        TestData[2 * (nbTestLow + nbTestMedium + i)].m_voltage = endVoltageMedium + (endVoltageHigh - endVoltageMedium) * (double)(i + 1) / (double)nbTestHigh;
        TestData[2 * (nbTestLow + nbTestMedium + i)].m_flags = 0;
        TestData[2 * (nbTestLow + nbTestMedium + i)].m_ramp = 0;

        TestData[2 * (nbTestLow + nbTestMedium + i) + 1].m_voltage = -(endVoltageMedium + (endVoltageHigh - endVoltageMedium) * (double)(i + 1) / (double)nbTestHigh);
        TestData[2 * (nbTestLow + nbTestMedium + i) + 1].m_flags = 0;
        TestData[2 * (nbTestLow + nbTestMedium + i) + 1].m_ramp = 0;

        if (rampVoltage <= TestData[2 * (nbTestLow + nbTestMedium + i)].m_voltage)
        {
            TestData[2 * (nbTestLow + nbTestMedium + i)].m_ramp = rampValue;
            TestData[2 * (nbTestLow + nbTestMedium + i) + 1].m_ramp = rampValue;
        }
    }
    std::cout << "High voltage effectué" << std::endl;
}

NLCharacterization_Tests::~NLCharacterization_Tests()
{
    free(TestData);
}

void NLCharacterization_Tests::nextTest()
{
    assert((m_state == State::Stopped) || (m_state == State::AskForStop));
    if (m_CurrentTestID < (m_nbTotalTest - 1))
    {
        m_CurrentTestID++;
    }
}
void NLCharacterization_Tests::previousTest()
{
    assert((m_state == State::Stopped) || (m_state == State::AskForStop));
    if (m_CurrentTestID > 0)
    {
        m_CurrentTestID--;
    }
}
void NLCharacterization_Tests::setCurrentTest(uint8_t testId)
{
    assert(m_state == State::Stopped);
    assert((testId > 0) && (testId < (m_nbTotalTest - 1)));
    m_CurrentTestID = testId;
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

    m_externalEncoderLeft->Reset();
    m_externalEncoderRight->Reset();

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

uint8_t NLCharacterization_Tests::getCurrentTestId()
{
    return m_CurrentTestID;
}

char *NLCharacterization_Tests::getCurrentTestDescription(char *pmessage, uint size_terminated_null_char_included)
{
    char desc[256];
    sprintf(desc, "TEST %d / %d [ %.2f Volts || Rampe : %.2f ]", m_CurrentTestID + 1, m_nbTotalTest, TestData[m_CurrentTestID].m_voltage, TestData[m_CurrentTestID].m_ramp);
    uint sizetocopy = (NMIN(size_terminated_null_char_included, 256) - 1);
    strncpy(pmessage, desc, sizetocopy);
    pmessage[sizetocopy + 1] = 0;
    return pmessage;
}

uint NLCharacterization_Tests::getTestsCounter()
{
    uint counter = 0;
    for (uint i = 0; i < m_nbTotalTest; i++)
    {
        if (BITGET(TestData[i].m_flags, 0))
        {
            counter++;
        }
    }
    return counter;
}

uint NLCharacterization_Tests::areAllTestsDone()
{
    uint counter = 0;
    for (uint i = 0; i < m_nbTotalTest; i++)
    {
        if (BITGET(TestData[i].m_flags, 0))
        {
            counter++;
        }
        else
        {
            break;
        }
    }
    return (counter == m_nbTotalTest) ? 1 : 0;
}

char *NLCharacterization_Tests::getCurrentFileLogName(char *pbuffer, uint size_terminated_null_char_included)
{
    if (m_LogFile)
    {
        uint sizecopied = m_LogFile->GetFileName().copy(pbuffer, size_terminated_null_char_included - 1);
        pbuffer[sizecopied] = 0;
    }
    else
    {
        uint sizetocopy = NMIN(14, size_terminated_null_char_included - 1);
        strncpy(pbuffer, "No file opened", sizetocopy);
        pbuffer[sizetocopy] = 0;
    }
    return pbuffer;
}
void NLCharacterization_Tests::fastLoop()
{
    switch (m_state)
    {
        /*case State::Stopped:
        //Do nothing
        break;*/

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
        m_LogFile = nullptr;
        m_state = State::Stopped;
        break;

    default:
        //Do nothing
        break;
    }
}