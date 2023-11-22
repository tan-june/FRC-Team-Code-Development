#pragma once

#include "rev/CANSparkMax.h"
#include <frc/Encoder.h>
#include <Constants.h>
#include <rev/CANEncoder.h>
#include <frc/motorcontrol/Spark.h>
using namespace frc;

class Climber{

Spark * climberl;
Spark * climberr;

private:
    
public:
Climber();
void SetClimber(double rotationCount, bool firstTime, double speed);
void RunClimber(double power);
void ZeroClimberSpeed();
double EncoderValue();
void ZeroEncoder();
void SetBasedonEncoder(double encodervalue);
};