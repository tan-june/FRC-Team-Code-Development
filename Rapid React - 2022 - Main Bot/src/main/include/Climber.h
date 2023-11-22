#pragma once

#include "rev/CANSparkMax.h"
#include <frc/Encoder.h>
#include <Constants.h>
#include <rev/CANEncoder.h>
using namespace frc;

class Climber{


private:
    rev::CANSparkMax climber;
    rev::SparkMaxRelativeEncoder * climberencoder;
    double encodervalue;
    
public:

Climber();
void SetClimber(double rotationCount, bool firstTime, double speed);
void RunClimber(double power);
void ZeroClimberSpeed();
double EncoderValue();
void ZeroEncoder();
void SetBasedonEncoder(double encodervalue);
};