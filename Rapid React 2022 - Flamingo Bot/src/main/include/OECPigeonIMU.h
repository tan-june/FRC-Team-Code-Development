#pragma once
#include <ctre/Phoenix.h>

class OECPigeonIMU{
    public:
        OECPigeonIMU(int canPort);
        void ResetAngle();
        void BootCalibrate();
        double GetYaw();
        double GetPitch();
        double GetRoll();
        
    private:
        ctre::phoenix::sensors::PigeonIMU *pigeonGyro;
        
};
