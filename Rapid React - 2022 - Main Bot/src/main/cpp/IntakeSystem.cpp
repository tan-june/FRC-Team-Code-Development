#include <IntakeSystem.h>

IntakeSystem::IntakeSystem():
intake(IntakeMotorNo),
intermediary(BallFeederMotorNo),
horizontalfeeder(HorizontalMotorNo),
intakepivot(PivotMotorNo)
{
    
}


void IntakeSystem::RunIntake(double power){
    intake.Set(power);
}

void IntakeSystem::SetIntakeZero(){
    intake.Set(0.0);
}

void IntakeSystem::IntakePivot(double RotationCount,bool initialTimeRunning,double speed){
    intakepivot.Set(speed);

}
    
void IntakeSystem::PivotZero(){
    intakepivot.Set(0.0);
}

void IntakeSystem::RunStarfishFeeder(double power){
    intermediary.Set(power);
}

void IntakeSystem::StarfishZero(){
    intermediary.Set(0.0);
}


void IntakeSystem::RunHorizontalFeeder(double power){
    horizontalfeeder.Set(power);
}

void IntakeSystem::HorizontalFeederZero(){
    horizontalfeeder.Set(0.0);
}

