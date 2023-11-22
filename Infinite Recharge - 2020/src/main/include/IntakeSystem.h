#pragma once
#include <ctre/Phoenix.h>
#include <frc/Solenoid.h>
#include <frc/Timer.h>
#include <frc/Encoder.h>

class IntakeSystem{

private:
ctre::phoenix::motorcontrol::can::WPI_TalonSRX *intakemotor;
ctre::phoenix::motorcontrol::can::WPI_TalonSRX *towermotor;
ctre::phoenix::motorcontrol::can::WPI_TalonSRX * intakepivot;
//frc::Encoder * towerencoder;


public:
IntakeSystem();
void RunIntake(double power);
void RunTower(double power);
void IntakePivot(double power);
void SetIntakeZero();
void SetTowerZero();
void PivotZero();
};
