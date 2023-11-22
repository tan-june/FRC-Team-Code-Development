#include <IntakeSystem.h>

IntakeSystem::IntakeSystem()
{
        intakemotor = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(14);
        towermotor = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(15);
        intakepivot = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(16);
        //towerencoder = new frc::Encoder(2,3, false, frc::CounterBase::EncodingType::k4X);

}
/*----------------------------------------------------------*/
void IntakeSystem::RunIntake(double power){
    intakemotor -> Set(power);
}

void IntakeSystem::SetIntakeZero(){
    intakemotor -> Set(0.0);
}
/*----------------------------------------------------------*/
void IntakeSystem::RunTower(double power){
    towermotor -> Set(power);
}


void IntakeSystem::SetTowerZero(){
    towermotor -> Set(0.0);
}

/*----------------------------------------------------------*/
void IntakeSystem::IntakePivot(double power){
    intakepivot -> Set(power);
}

void IntakeSystem::PivotZero(){
    intakepivot -> Set(0.0);
}


