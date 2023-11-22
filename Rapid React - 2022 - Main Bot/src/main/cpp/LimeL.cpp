#include "Limelight.h"

LimeLight::LimeLight(){
    //instance of Network Table
    Table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
}

void LimeLight::Update(){

    tv = Table->GetNumber("tv",0.0); //value to represent if target is seen
    area = Table->GetNumber("ta",0.0);
    X = Table->GetNumber("tx",0.0);
    Y = Table->GetNumber("ty",0.0);
    Height = Table->GetNumber("tvert",0);
    Width = Table->GetNumber("thor",0);

}
double LimeLight::targetIsFound(){
    return tv;
}
double LimeLight::GetArea(){
    return area;
}
double LimeLight::GetXOffset(){
    return X;
}
double LimeLight::GetYOffset(){
    return Y;
}
double LimeLight::GetHeight(){
    return Height;
}
double LimeLight::GetWidth(){
    return Width;
}
void LimeLight::SetVisionMode(bool visionMode){
    Table->PutNumber("ledMode", visionMode ? 0.0 : 1.0);
	Table->PutNumber("camMode", visionMode ? 0.0 : 1.0);
}
void LimeLight::LEDon(){
    Table->PutNumber("ledMode", 3);
}
void LimeLight::LEDoff(){
    Table->PutNumber("ledMode", 1);
}
