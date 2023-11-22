#pragma once
#include <frc/Counter.h>
//#include "Limelight.h"
//#include "CameraArm.h"
#include <units/time.h>
#include <hal/Types.h>
#include <hal/I2C.h>
#include <hal/I2CTypes.h>
#include <units/time.h>
#include <frc/I2C.h>


using namespace std;
class LidarLite{
    public:
        LidarLite();
        //CameraMountAimer camarm;
        void Lidarinit();
        int DistanceCM;
        
        frc::Counter lidarcounter{frc::Counter::Mode::kSemiperiod};
        // I2C I2Cprt = I2C<frc::I2C::Port i2cprt,int adress>();
        
        //void UpdateLidar();
        double GetDistance();
    private:
        //int GetDistance();
        //Counter lidarCount
        int Lidaroffset = -10;
        //units::second_t counterPeriod;
        double distance;
        double avgdistance;
        //int32_t* status;
        //uint8_t distance[2];
        //int32_t Distance_Value;
        //const uint8_t  *DistanceMeasureCommand; 
        

    

};
