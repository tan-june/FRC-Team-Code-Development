#include <Climber.h>
#include <iostream>

Climber::Climber():
climber(ClimberMotorNo, rev::CANSparkMax::MotorType::kBrushless)
{
    climberencoder = new rev::SparkMaxRelativeEncoder(climber.GetEncoder());
    
}


void Climber::RunClimber(double power){
    climber.Set(power);
}

void Climber::ZeroClimberSpeed(){
    climber.Set(0.0);
}

double Climber::EncoderValue(){
    return climberencoder->GetPosition();
}

//Still working on setting specific boundaries
void Climber::SetClimber(double RotationCount,bool initialTimeRunning,double ClimberSpeed){

    if(initialTimeRunning){
    climberencoder->SetPosition(0);
    while(RotationCount != climberencoder->GetPosition()){
        climber.Set(ClimberSpeed);
        }
    }
    else{
        climberencoder->SetPosition(RotationCount);
    while(RotationCount != climberencoder->GetPosition()){
        climber.Set(-ClimberSpeed);
        }
    }
}
    

    
