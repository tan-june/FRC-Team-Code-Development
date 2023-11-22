#pragma once

#include "rev/CANSparkMax.h"
#include <frc/Encoder.h>
using namespace frc;

class Climber{


private:
    rev::CANSparkMax climb;
    rev::CANEncoder climbencoder;

    double encodervalue;

public:

Climber();
void ClimberDirection(double powerup);
void ZeroClimberSpeed();
double EncoderValue();

};