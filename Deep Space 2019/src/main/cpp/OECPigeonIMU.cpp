#include "OECPigeonIMU.h"

OECPigeonIMU::OECPigeonIMU(int canPort){
    pigeonGyro = new ctre::phoenix::sensors::PigeonIMU(canPort);
}

void OECPigeonIMU::ResetAngle(){
    pigeonGyro->SetYaw(0.0);
}

void OECPigeonIMU::BootCalibrate(){
    pigeonGyro->EnterCalibrationMode(ctre::phoenix::sensors::PigeonIMU::CalibrationMode::BootTareGyroAccel);
}

double OECPigeonIMU::GetYaw(AngleUnits units){
    double ypr[3];
    pigeonGyro->GetYawPitchRoll(ypr);
    if(units == AngleUnits::degrees)
        return ypr[0];
    else
        return ypr[0]*(3.141592653589793238)/180.0;
}

double OECPigeonIMU::GetPitch(AngleUnits units){
    double ypr[3];
    pigeonGyro->GetYawPitchRoll(ypr);
    if(units == AngleUnits::degrees)
        return ypr[0];
    else
        return ypr[0]*(3.141592653589793238)/180.0;
}

double OECPigeonIMU::GetRoll(AngleUnits units){
    double ypr[3];
    pigeonGyro->GetYawPitchRoll(ypr);
    if(units == AngleUnits::degrees)
        return ypr[0];
    else
        return ypr[0]*(3.141592653589793238)/180.0;
}