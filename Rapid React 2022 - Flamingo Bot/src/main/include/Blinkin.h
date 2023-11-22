#pragma once
#include <frc/motorcontrol/Spark.h>

using namespace frc;

class Bling{


frc::Spark * blinkin;


public:
Bling();
void BlingRed();
void BlingBlue();
void BlingGreen();
void BlingYellow();

};