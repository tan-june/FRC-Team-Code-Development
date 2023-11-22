#include "Shooter.h"

Shooter::Shooter():
    TopFlywheel(9, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
    BottomFlywheel(10, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
    TopEncoder(TopFlywheel),
    BottomEncoder(BottomFlywheel),
    TopController(TopFlywheel),
    BottomController(BottomFlywheel),
    Feeder(13),
    Hood(12),
    Turret(11),
    TurretEncoder(3, 4, false, frc::CounterBase::k4X),
    HoodEncoder(0, 1, false, frc::CounterBase::k4X),
    TurretController(),
    HoodController()
{
    //HoodEncoder.SetDistancePerPulse(1.0);
    TopFlywheel.GetEncoder();
    BottomFlywheel.GetEncoder();
    TopController.SetP(SHOOTER_P);
    TopController.SetI(SHOOTER_I);
    TopController.SetD(SHOOTER_D);
    TopController.SetFF(SHOOTER_FF);
    TopController.SetIZone(SHOOTER_IZONE);
    BottomController.SetP(SHOOTER_P);
    BottomController.SetI(SHOOTER_I);
    BottomController.SetD(SHOOTER_D);
    BottomController.SetFF(SHOOTER_FF);
    BottomController.SetIZone(SHOOTER_IZONE);
    trimTurret = 0.0;
}

void Shooter::TurnTurret(double power){
    if(power > 1.0)
        power = 1.0;
    else if(power < 1.0)
        power = -1.0;

 Turret.Set(power);

}

void Shooter::TeleAimLimelight(){
    limelight.Update();
    TurnTurret(TURRET_VISION_P * limelight.GetXOffset());
}

void Shooter::TrimTurret(double angleDegrees){




}

void TurnToPosition(double angleDegrees){

}
void Shooter::SpinFlywheelsOpenLoop(double topPower, double bottomPower){
    TopFlywheel.Set(topPower);
    BottomFlywheel.Set(bottomPower);

}

void Shooter::SpinFlywheelsPID(double topRPM, double bottomRPM){
    TopController.SetReference(topRPM, rev::ControlType::kVelocity, 0, SHOOTER_FF, rev::CANPIDController::ArbFFUnits::kVoltage);
    BottomController.SetReference(bottomRPM, rev::ControlType::kVelocity, 0, SHOOTER_FF, rev::CANPIDController::ArbFFUnits::kVoltage);
}

float Shooter::GetTopFlywheelVelocity(){
    return TopEncoder.GetVelocity();
}

float Shooter::GetBottomFlywheelVelocity(){
    return BottomEncoder.GetVelocity();
}

void Shooter::FeederWheel(double feederspeed){
    Feeder.Set(feederspeed);
}

void Shooter::MoveHood(double power){
    Hood.Set(power);
}

void Shooter::MoveHoodToPosition(int encoderDist){
    
}

float Shooter::GetHoodPosition(){
    return (float)HoodEncoder.GetRaw();
}

void Shooter::ResetHoodEncoder(){
    HoodEncoder.Reset();
}