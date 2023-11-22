#pragma once
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <frc/Solenoid.h>
#include <frc/Encoder.h>
#include <frc/DigitalInput.h>
#include "LidarLite.h"
#include "Limelight.h"
#include <frc/Timer.h>
#include "OECPIDController.h"
#include "Limelight.h"
#include "Constants.h"

class Shooter{
    private:
        Limelight limelight;
        rev::CANSparkMax TopFlywheel;
        rev::CANSparkMax BottomFlywheel;

        rev::CANEncoder TopEncoder;
        rev::CANEncoder BottomEncoder;

        rev::CANPIDController TopController;
        rev::CANPIDController BottomController;
        
        ctre::phoenix::motorcontrol::can::WPI_TalonSRX Feeder;
        ctre::phoenix::motorcontrol::can::WPI_TalonSRX Hood;
        ctre::phoenix::motorcontrol::can::WPI_TalonSRX Turret;
        frc::Encoder TurretEncoder;
        frc::Encoder HoodEncoder;

        OECPIDController HoodController;
        OECPIDController TurretController;

        double trimTurret;
    public:
        Shooter();
        void TurnTurret(double power);
        void TrimTurret(double angleDegrees);
        void TurnToPosition(double angleDegrees);
        void TeleAimLimelight();
        
        void MoveHood(double power);
        float GetHoodPosition();
        void ResetHoodEncoder();
        void TrimHood(double angleDegrees);
        void MoveHoodToPosition(int encoderDist);

        void SpinFlywheelsOpenLoop(double topPower, double bottomPower);
        void SpinFlywheelsPID(double topRPM, double bottomRPM);
        float GetTopFlywheelVelocity();
        float GetBottomFlywheelVelocity();
        void Fire();

    void FeederWheel(double feederspeed);

        void AutoTarget();
};