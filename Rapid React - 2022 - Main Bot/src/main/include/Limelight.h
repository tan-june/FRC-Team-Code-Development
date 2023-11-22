#pragma once

#include <memory>
#include <vector>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"



class LimeLight{
public:
    LimeLight();
    void Update();
    double targetIsFound(); //tv value is double not bool

    double GetArea();
//for XY centering
    double GetXOffset();
    double GetYOffset();

    double GetWidth();
    double GetHeight();
    void SetVisionMode(bool visionMode);
    double tv;
    void LEDon();
    void LEDoff();

private:
    std::shared_ptr<nt::NetworkTable> Table;
    double X;
    double Y;
    double Width;
    double Height;
    double area;
    unsigned int resolutionX;
    unsigned int resolutionY;
    unsigned int FullImgArea;
    double found; //find out what to do with this

};