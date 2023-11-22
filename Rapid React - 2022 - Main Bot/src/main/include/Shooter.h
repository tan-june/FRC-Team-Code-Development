#pragma once
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <frc/Solenoid.h>
#include <frc/Encoder.h>
#include <frc/DigitalInput.h>
#include "LidarLite.h"
#include "Limelight.h"
#include <frc/Timer.h>
#include "Limelight.h"
#include "Constants.h"
#include <rev/CANEncoder.h>
#include <frc/motorcontrol/PWMSparkMax.h>

class Shooter{
    private:
        rev::CANSparkMax shooter; 
        rev::SparkMaxPIDController * shooterpid;   
        rev::SparkMaxRelativeEncoder* shooterencoder;
    public:
    Shooter();
    void RunFlywheel(double power);
    void FlywheelZero();
    void SpinFlywheelPID(double RPM);
    double ReturnRPM();
  
};