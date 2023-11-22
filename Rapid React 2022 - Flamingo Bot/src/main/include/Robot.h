
#pragma once
#include <string>
#include <frc/TimedRobot.h>
#include <units/time.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "Limelight.h"
#include "LidarLite.h"
//#include "OECPigeonIMU.h"
#include "Constants.h"
#include <frc/Timer.h>
#include "IntakeSystem.h"
#include <frc/Joystick.h>
//#include "Usonic.h"
#include "Shooter.h"
#include "TankDrive.h"
#include "Climber.h"
#include <cameraserver/CameraServer.h>
#include <Blinkin.h>
#include <frc/AnalogInput.h>
#include <frc/DigitalInput.h>


using namespace frc;

class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 
  IntakeSystem intake;
  Tankdrive tankdrive;
  Shooter shooter;
  Climber climber;
  Joystick stick1{JoystickPort};
  Joystick stick2{JoystickPort2};
  Joystick stick3{JoystickPort3};
  Joystick driverstation{JoystickPort4};
  units::second_t delay{0.000000002};
  Bling lights;
  bool aboveThreshold;
  double getz1 ;
  double rpm1;
  
  cs::UsbCamera camera{};

 private:
  bool ClimberUp;
  bool IntakeUp;
  SmartDashboard * dash;
  DigitalInput DIOswitch0;
  DigitalInput DIOswitch1;
  DigitalInput DIOswitch2;
  DigitalInput DIOswitch3;
  frc::AnalogInput AutoPmeter; //reads 0-5 volts and multiply by 2
  //DigitalInput DIOIntakePivotLimitTop;
  //DigitalInput DIOIntakePivotLimitBott;
};
