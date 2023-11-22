
#include "LidarLite.h"
#include <math.h>
#include <algorithm>
#include <bitset>
#include<iostream>




LidarLite::LidarLite(){
    
    distance = 0.0;
    lidarcounter.SetUpSource(9);
    lidarcounter.SetSemiPeriodMode(true);
    
    lidarcounter.Reset();

}
double LidarLite::GetDistance(){
   
    //find out units
    distance = ((lidarcounter.GetPeriod().value()*100000.0 + Lidaroffset)/2.54);
    avgdistance*=0.9;
    avgdistance += 0.1*distance;
    return avgdistance;
}










/*
LidarLite::LidarLite(){
   
    // distance[2]; 

   
    HAL_InitializeI2C(HAL_I2C_kOnboard, status);
   
   // I2CPort.Write(0x04,0x00);
   // I2CPort.Read(0x8f,2,distance);
   //Combines 2 bits into an int
   

}


int LidarLite::GetDistance(){
    Distance_Value = (int)((distance[0] << 8) + distance[1]);
    return Distance_Value;
}
void LidarLite::UpdateLidar(){
    uint8_t* arraypointer;
    uint8_t* arraypointer2;
     
     arraypointer = &distance[0];
     arraypointer2 = &distance[1];
     //DistanceMeasureCommand = 0x04b;
     HAL_WriteI2C(HAL_I2C_kOnboard,0x00,(uint8_t*)0x04b,1);
     frc::Wait(period);
     HAL_ReadI2C(HAL_I2C_kOnboard,0x8f,arraypointer,2);
     HAL_ReadI2C(HAL_I2C_kOnboard,0x8f,arraypointer2,2);
     frc::Wait(period);
     DistanceCM = GetDistance();
   
}
   // double angle = camarm.cameraServo.GetAngle();
   // return (cos ( angle*(PI/180)))*distance;
*/
