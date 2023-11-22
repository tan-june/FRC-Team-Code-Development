#include "Shooter.h"


Shooter::Shooter():
    shooter(ShooterMotorNo, rev::CANSparkMaxLowLevel::MotorType::kBrushless)
{
    shooterencoder = new rev::SparkMaxRelativeEncoder(shooter.GetEncoder());
    shooterpid = new rev::SparkMaxPIDController(shooter.GetPIDController());


    shooterpid -> SetP(SHOOTER_P);
    shooterpid ->SetI(SHOOTER_I);
    shooterpid ->SetD(SHOOTER_D);
    shooterpid ->SetFF(SHOOTER_FF);
    shooterpid->SetIZone(SHOOTER_IZONE);
    shooterpid ->SetOutputRange(kMinOutput, kMaxOutput);


}

void Shooter::RunFlywheel(double power){
    shooter.Set(power);
}


void Shooter::FlywheelZero(){
    shooter.Set(0.0);
}

void Shooter::SpinFlywheelPID(double RPM){
    shooterpid->SetReference(3600, rev::CANSparkMax::ControlType::kVelocity, 0, 0.0, rev::SparkMaxPIDController::ArbFFUnits::kVoltage);
}

double Shooter::ReturnRPM(){
   double shooterrpm = shooterencoder->GetVelocity();
    return shooterrpm;
}


