#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>


Robot::Robot():
tankdrive(2),
DIOswitch0(0),
DIOswitch1(1),
DIOswitch2(2),
DIOswitch3(3),
AutoPmeter(0)
//DIOIntakePivotLimitTop(6),
//DIOIntakePivotLimitBott(7)
//fill in with real port values before you test
{
  dash -> init();

} 
void Robot::RobotInit(){
    camera = frc::CameraServer::StartAutomaticCapture();
    camera.SetResolution(426,240);
    camera.SetExposureAuto();
    camera.SetFPS(30);
    

}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {} 
void Robot::TeleopPeriodic()
{

	//SmartDashboard Requirements
	dash->PutNumber("Robot Speed", tankdrive.GetThrottle());
	dash->PutNumber("Climber Speed", (1 - stick3.GetZ()) / 2);
	
	//-----------------------------------------------------------------------------------------------------------------

	//Tankdrive
	tankdrive.SetThrottle(stick1.GetZ());
	tankdrive.limelight.Update();

	//LimelightAIM included drive
	if ((stick1.GetTrigger()))
	{
				tankdrive.DirectDrive(0.9,0.9);
		//tankdrive.;TeleAimLimelight(1,true);
		//tankdrive.TeleAimLimelight(.5, true);
	}
	else if (stick2.GetTrigger()){
		tankdrive.DirectDrive(-0.9,-0.9);
	}
	else
	{
		tankdrive.TeleAimLimelight(1.0, false);
		tankdrive.Drive(stick1.GetY(), stick2.GetY());
		//cam.EndMotor();
	}
//----------------------------------------------------------------------------------------------------------------
//Climber
	if (driverstation.GetRawButton(1))
	{
		climber.RunClimber((1 - stick3.GetZ()) / 2 * stick3.GetY());
	}
	//Set Climber Zero
	else
	{
		climber.ZeroClimberSpeed();
	}
//---------------------------------------------------------------------------------------------------------------------

}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
