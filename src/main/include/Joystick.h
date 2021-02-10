#include "lib/CustomMaths.h"
#include "lib/Utils.h"

void getSpeedsAndAccelerations(VA *pva_left, VA *pva_right, const VA *pvamax, const double jx, const double jy);
void getSpeedsAndAccelerationsNew(VA *pva_left, VA *pva_right, const VA *pvamax, const double jx, const double jy);
void updateVelocityAndAcceleration(VA *pva, const VA *pva_max, const double target_speed, const double dt);