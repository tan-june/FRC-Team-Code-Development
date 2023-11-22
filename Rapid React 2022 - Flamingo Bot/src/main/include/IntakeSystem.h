#pragma once
#include <ctre/Phoenix.h>
#include <frc/Solenoid.h>
#include <frc/Timer.h>
#include "rev/CANSparkMax.h"
#include <frc/Encoder.h>

#include <rev/CANEncoder.h>
#include <frc/Encoder.h>
#include <Constants.h>

class IntakeSystem{

private:

ctre::phoenix::motorcontrol::can::WPI_TalonSRX intake;
ctre::phoenix::motorcontrol::can::WPI_VictorSPX intermediary;
ctre::phoenix::motorcontrol::can::WPI_VictorSPX horizontalfeeder;
ctre::phoenix::motorcontrol::can::WPI_VictorSPX intakepivot;

public:
IntakeSystem();
void RunIntake(double power);
void RunStarfishFeeder(double power);
void RunHorizontalFeeder(double power);
void IntakePivot(double RotationCount,bool initialTimeRunning,double speed);
void SetIntakeZero();
void PivotZero();
void StarfishZero();
void HorizontalFeederZero();

};