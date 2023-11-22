#include "PIDcontroller.h"
#include "Constants.h"

PIDcontroller::PIDcontroller():
pidTimer()
{
    isFirstRun = true;
    P = 0.0;
    I = 0.0;
    D = 0.0;
    correction = 0.0;
    lastError = 0.0;
    lastTime = 0.0;
}
void PIDcontroller::ResetController(){
    isFirstRun = true;
    P = 0.0;
    I = 0.0;
    D = 0.0;
    correction = 0.0;
    lastError = 0.0;
    lastTime = 0.0;

    TimeMillis = 0.0;
    pidTimer.Reset();
}
void PIDcontroller::SetConstants(double coefP, double coefI, double coefD, double MaximumCorrection)
{
    kP = coefP;
    kI = coefI;
    kD = coefD;
    maxCorrection = fabs(MaximumCorrection);
}
void PIDcontroller::SetIntegral(double IVal){
    I = IVal;
}
double PIDcontroller::GetCorrection(double error)
{
    double time = GetTimeMillis();
    double IIncrement;
    // Special case for first run: performs necessary setup steps and returns 0 correction:
    if(isFirstRun)
    {
        pidTimer.Start();
        lastTime = time;
        lastError = error;
        isFirstRun = false;
        return 0.0;
    }

    P = kP * error;
    
    D = kD * ((error - lastError) / (time - lastTime));
    IIncrement = ((error + lastError) / 2) * (time - lastTime);

    //Don't add to the integral if correction is already at the max
    if(fabs(P + kI * I + D + kI * IIncrement) >= maxCorrection){
        correction = maxCorrection * fabs(P + kI * I + D + kI * IIncrement)/(P + kI * I + D + kI * IIncrement);
    }

    else{
        I += IIncrement;
        correction = P + kI * I + D;
    }

    lastTime = time;
    lastError = error;

    return correction;
}

double PIDcontroller::GetTimeMillis()
{
	return pidTimer.Get().value()*1000;
}

double PIDcontroller::TurnCorrection(double visionval, double deltaTime,double P, double I, double D){
    sample = visionval - 160; // finD out why the 160
    Integral = Integral + (deltaTime/2)*(sample+LastSample);
    Derivative = (sample-LastSample)/2;
    Turn = (P*sample) + (I*Integral) + (D*Derivative);
    sample = LastSample;
    return Turn;
}