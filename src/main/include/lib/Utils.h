#pragma once

#include <cmath>

typedef struct VA VA;
struct VA
{
  double m_speed;
  double m_acceleration;
};

class KineticToVoltage
{

  //k_lut[MoteurIndex][ForwardBackward][Kv, Ka, Kintersept]
  double k_lut[4][2][3];

public:
  void SetMotorCoefficients(uint motorID, uint isBackward, double kv, double ka, double vintersept);
  double getVoltage(uint motorID, const VA *pva);
};