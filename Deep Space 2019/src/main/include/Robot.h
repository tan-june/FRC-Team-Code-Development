/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/WPILib.h>
#include <frc/TimedRobot.h>
#include "Tankdrive.h"
#include <ctre/Phoenix.h>
using namespace frc;

class Robot : public frc::TimedRobot {
 public:
  //Joysticks
  Joystick j_right{0};
  Joystick j_left{1};
  Joystick j_lift{2};

  //Motors
  Spark m_left{0};
  Spark m_right{1};
  Spark m_lift{2};
  //Spark m_arm{3};

  //sensitivity
  double sens;

  OECPigeonIMU *myGyro; 

  cs::UsbCamera camera{}; 

  SmartDashboard *smartDashboard;
  
  //Tankdrive
  Tankdrive *tdrive; 

  void RobotInit() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;
};
