#pragma once
#include <frc/Timer.h>
#include <units/time.h>

class PIDcontroller{
    public:
        double TurnCorrection(double visionValue, double deltaTime,double P, double I, double D);
        PIDcontroller();
        void ResetController();
        void SetConstants(double coefP, double coefI, double coefD, double MaximumCorrection);
        void SetIntegral(double IVal);
        double GetCorrection(double error);
         double Integral = 0.0;
    double Derivative;
    double sample;
    double LastSample = 0.0;
    private:
   
    double Turn;

    frc::Timer pidTimer;
    double GetTimeMillis();
    double isFirstRun;
    double kP;
    double kI;
    double kD;
    double maxCorrection;

    double P;
    double I;
    double D;

    double TimeMillis;

    double correction;
    double lastError;
    double lastTime;
};