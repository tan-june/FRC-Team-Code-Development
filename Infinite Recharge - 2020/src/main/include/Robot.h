#pragma once

#include <ctre/Phoenix.h>

#include <frc/TimedRobot.h>
#include "OECJoystick.h"

#include <frc/smartdashboard/SmartDashboard.h>


//Class References Files
#include "Tankdrive.h"
#include "IntakeSystem.h"
#include "Climber.h"
#include "ColorWheel.h"
#include "Bling.h"
#include <frc/Encoder.h>
#include <frc/AnalogInput.h>
#include "LidarLite.h"
#include <Shooter.h>
#include "ColorWheel.h"
using namespace frc;

class Robot : public frc::TimedRobot {


private:

OECJoystick stick1;
OECJoystick stick2;
OECJoystick stick3;
OECJoystick driverstation;

frc::SmartDashboard * dash;

frc::AnalogInput beamBreak0;
frc::AnalogInput beamBreak1;


Tankdrive tankdrive;
IntakeSystem intake;
Climber climber;
//ColorWheel colorwheel;
Bling bling;
Shooter shooter;

LidarLite lidar;



//variable for climber speed
double climberspeed;

//variable for shooter speed
double shooterspeed;


public:
  Robot();
  void RobotInit() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;


};
