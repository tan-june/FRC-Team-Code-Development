
#include "Usonic.h"
#include <math.h>
#include <algorithm>
#include <bitset>
#include<iostream>
#define PI 3.14159265



Usonic::Usonic(){
    
    distance = 0.0;
    Usoniccounter.SetUpSource(port);
    Usoniccounter.SetSemiPeriodMode(true);
    
    Usoniccounter.Reset();

}
double Usonic::GetDistance(){
   
    //inch
    distance = ((Usoniccounter.GetPeriod().value()*1000000.0)/147) ;
    avgdistance*=0.9;
    avgdistance += 0.1*distance;
    return avgdistance;
}










