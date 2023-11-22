/*

#include "ColorWheel.h"

ColorWheel::ColorWheel(){
wheel = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(16);
sensor = new rev::ColorSensorV3(frc::I2C::kOnboard);
  static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
  static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
  static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
  static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);

  m_colorMatcher.AddColorMatch(kBlueTarget);
  m_colorMatcher.AddColorMatch(kGreenTarget);
  m_colorMatcher.AddColorMatch(kRedTarget);
  m_colorMatcher.AddColorMatch(kYellowTarget);
  frc::Color detectedColor = sensor->GetColor();
  frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, 0.0);
  index=0;
  Prev_color;
  this->Prev_color = matchedColor;

}


static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);


void ColorWheel::SpinRight(){
  wheel -> Set(0.3);
}


void ColorWheel::SpinLeft(){
  wheel -> Set(-0.3);
}


void ColorWheel::SetZero(){
  wheel -> Set(0.0);
}



int ColorWheel::SensorDetection(int index, frc::Color matchedColor, frc::Color Prev_color){

  if(matchedColor == Prev_color)
  {
    
  }
  else
  {
    index++;
  }
  
  Prev_color = matchedColor;
  return index;
}

frc::Color ColorWheel::tocolor(frc::Color matchedColor, char* kTarget)
{
  bool state1, state2, state3, state4 = true;
  
  if(matchedColor == kRedTarget)
  {
    state1 = false;
  }
  else if(matchedColor == kBlueTarget)
  {
    state2 = false;
  }
  else if(matchedColor == kGreenTarget)
  {
    state3 = false;
  }
  else if(matchedColor == kYellowTarget)
  {
    state4 = false;
  }
  else 
  {
    state1 = true;
    state2 = true;
    state3 = true;
    state4 = true;
  }

  if(kTarget == "red" && state1)
  {
    SpinRight();
  }
  else if(kTarget == "blue" && state2)
  {
    SpinRight();
  }
  else if(kTarget == "green" && state3)
  {
    SpinRight();
  }
    else if(kTarget == "yellow" && state4)
  {
    SpinRight();
  }
}

*/