#include "lib/Utils.h"

void KineticToVoltage::SetMotorCoefficients(uint motorID, uint isBackward, double kv, double ka, double vintersept)
{
    k_lut[motorID][isBackward][0] = kv;
    k_lut[motorID][isBackward][1] = ka;
    k_lut[motorID][isBackward][2] = vintersept;
}

double KineticToVoltage::getVoltage(uint motorID, const VA *pva)
{
    int isBackward = (pva->m_speed < 0) ? 1 : 0;
    if (pva->m_speed == 0)
        return 0;
    return k_lut[motorID][isBackward][0] * pva->m_speed + k_lut[motorID][isBackward][1] * pva->m_acceleration + k_lut[motorID][isBackward][2];
}