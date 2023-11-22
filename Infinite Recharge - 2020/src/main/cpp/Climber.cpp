#include <Climber.h>
#include <iostream>

Climber::Climber(): 
climb(18, rev::CANSparkMax::MotorType::kBrushless),
climbencoder(climb, rev::CANEncoder::EncoderType::kHallSensor, 1)
{
    climb.GetEncoder();

}


void Climber::ClimberDirection(double powerup){
climb.Set(powerup);
}

void Climber::ZeroClimberSpeed(){
    climb.Set(0.0);
}

double Climber::EncoderValue(){

return climbencoder.GetPosition();

}
