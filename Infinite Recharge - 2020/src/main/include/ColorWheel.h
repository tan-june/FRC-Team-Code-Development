#pragma once

#include <ctre/Phoenix.h>
#include <frc/util/color.h>
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>

using namespace frc;

class ColorWheel{

private:

ctre::phoenix::motorcontrol::can::WPI_TalonSRX * wheel;

rev::ColorSensorV3 * sensor;

frc::Color Prev_color;

rev::ColorMatch m_colorMatcher;





public:
ColorWheel();
void SpinRight();
void SpinLeft();
void SetZero();
int SensorDetection(int index, frc::Color matchedColor, frc::Color Prev_color);
frc::Color tocolor(frc::Color matchedColor, char* kTarget);
int index;
};