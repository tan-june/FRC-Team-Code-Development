#include <Climber.h>
#include <iostream>

Climber::Climber()
{
    climberl = new Spark(ClimberLNo);
    climberr = new Spark(ClimberRNo);
}


void Climber::RunClimber(double power){
    climberl-> Set(power);
    climberr-> Set(-power);
}

void Climber::ZeroClimberSpeed(){
    climberl-> Set(0.0);
    climberr-> Set(0.0);
}

//Still working on setting specific boundaries
void Climber::SetClimber(double RotationCount,bool initialTimeRunning,double ClimberSpeed){}