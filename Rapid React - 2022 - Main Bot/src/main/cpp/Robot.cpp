#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>


Robot::Robot():
tankdrive(2),
DIOswitch0(0),
DIOswitch1(1),
DIOswitch2(2),
DIOswitch3(3),
DIOswitch4(4),
AutoPmeter(0),
DIOIntakePivotLimitTop(6),
DIOIntakePivotLimitBott(7)
//fill in with real port values before you test
{
  dash -> init();

} 
void Robot::RobotInit(){
    dash->PutNumber("Shooter RPM", 0.0);
    camera = frc::CameraServer::StartAutomaticCapture();
    camera.SetResolution(426,240);
    camera.SetExposureAuto();
    camera.SetFPS(30);
    

}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
	double getz = -0.55;
	double rpm = ((1-(getz)/2))*5700;
	/*
	bool switch0 = !DIOswitch0.Get(); // ball 1
	bool switch1 = !DIOswitch1.Get();// ball 2
 	bool switch2 = !DIOswitch2.Get();// ball 3
	bool switch3 = !DIOswitch3.Get();// ball 4
	bool switch4 = !DIOswitch4.Get();// ball 4
	*/
/*
	//1 Ball Auto
	
	if(switch0 == true){
		frc::Wait(2_s);
		intake.IntakePivot(0,0,0.5);
   		 frc::Wait(1_s);
		intake.IntakePivot(0,0,0.0);
		tankdrive.AutoDriveGyroUSING(18,.6,3);
		shooter.SpinFlywheelPID(rpm);
		frc::Wait(1.5_s);
		intake.RunHorizontalFeeder(-0.8);
		intake.RunStarfishFeeder(0.8);
		frc::Wait(1.5_s);
		tankdrive.AutoDriveGyroUSING(22,.6,3);
		intake.RunHorizontalFeeder(0.0);
		intake.RunStarfishFeeder(0.0);
		shooter.SpinFlywheelPID(0.0);
	}
	*/

	//2 Ball Auto
	intake.IntakePivot(0,0,0.5);
    frc::Wait(1_s);
	intake.IntakePivot(0,0,0.0);
	intake.RunIntake(0.95);
	tankdrive.AutoDriveGyroUSING(35,.5, 3);
	frc::Wait(0.8_s);
	intake.RunIntake(0.0);
	shooter.SpinFlywheelPID(rpm);
	tankdrive.AutoDriveGyroUSING(10, -.5, 3);
	intake.RunHorizontalFeeder(-0.8);
	intake.RunStarfishFeeder(0.8);
	frc::Wait(2.5_s);
	intake.RunHorizontalFeeder(0.0);
	intake.RunStarfishFeeder(0.0);
	shooter.FlywheelZero();
	tankdrive.AutoDriveGyroUSING(30.0,.2,3);
	
	
/*
	//3 Ball Auto
		if(switch2 == true){
		//insert code for ball 3
		intake.IntakePivot(0,0,0.5);
		frc::Wait(0.5_s);
		intake.IntakePivot(0,0,0.0);
		intake.RunIntake(0.95);
		tankdrive.AutoDriveGyroUSING(35,.7, 3);
		frc::Wait(0.8_s);
		intake.RunIntake(0.0);
		shooter.SpinFlywheelPID(rpm);
		tankdrive.AutoDriveGyroUSING(10, -.5, 3);
		intake.RunHorizontalFeeder(-0.8);
		intake.RunStarfishFeeder(0.8);
		intake.RunIntake(0.95);
		frc::Wait(2.3_s);
		intake.RunHorizontalFeeder(0.0);
		intake.RunStarfishFeeder(0.0);
		intake.RunIntake(0.0);
		shooter.SpinFlywheelPID(0.0);
		tankdrive.AutoTurnGyro(105.5,0.6,4);
		tankdrive.AutoDriveGyroUSING(55,.7,5);
		intake.RunIntake(0.95);	
		tankdrive.AutoDriveGyroUSING(47, 0.5, 4);
		intake.RunIntake(0.0);
		tankdrive.AutoDriveGyroUSING(9, 0.7, 3);
		tankdrive.AutoTurnGyro(-44,0.6,4);
		tankdrive.AutoDriveGyroUSING(58, -0.7, 3);
		shooter.SpinFlywheelPID(rpm);
		frc::Wait(0.5_s);
		intake.RunHorizontalFeeder(-0.8);
		intake.RunStarfishFeeder(0.8);
		frc::Wait(2_s);
		intake.RunHorizontalFeeder(-0.0);
		intake.RunStarfishFeeder(0.0);
		shooter.SpinFlywheelPID(0.0);
		}
	

	//4-Ball Auto
		//insert code for ball 4
	if(switch3 == true){
		intake.IntakePivot(0,0,0.6);
		frc::Wait(1.2_s);
		intake.IntakePivot(0,0,0.0);
		intake.RunIntake(0.95);
		shooter.SpinFlywheelPID(rpm);
		tankdrive.AutoDriveGyroUSING(35,.75*AutoSpeedUP, 3);
		frc::Wait(0.8_s);
		intake.RunIntake(0.0);

		tankdrive.AutoDriveGyroUSING(10, -.5*AutoSpeedUP, 3);
		intake.RunHorizontalFeeder(-0.8);
		intake.RunStarfishFeeder(0.8);
		frc::Wait(1.8_s);
		intake.RunHorizontalFeeder(0.0);
		intake.RunStarfishFeeder(0.0);
		shooter.FlywheelZero();
		//Going to get third ball
		tankdrive.AutoTurnGyro(105.5,0.7*AutoSpeedUP,4);
		tankdrive.AutoDriveGyroUSING(55,.7*AutoSpeedUP,5);
		intake.RunIntake(0.95);	
		tankdrive.AutoDriveGyroUSING(50, 0.6*AutoSpeedUP, 4);
		intake.RunIntake(0.0);
		tankdrive.AutoDriveGyroUSING(19, 0.7*AutoSpeedUP, 3);
		//going to get third ball
		tankdrive.AutoTurnGyro(-44, 0.7*AutoSpeedUP, 2.5);
		//intake.RunStarfishFeeder(0.8);
		//intake.RunHorizontalFeeder(-0.8);
		//frc::Wait(0.8_s);
		//intake.RunStarfishFeeder(0.0);
		//intake.HorizontalFeederZero();
		tankdrive.AutoDriveGyroUSING(85, 0.7*AutoSpeedUP, 3);
		intake.RunIntake(0.95);
		tankdrive.AutoDriveGyroUSING(15, 0.3*AutoSpeedUP, 3);
		tankdrive.AutoTurnGyro(-0.5,0.5*AutoSpeedUP,4);
		tankdrive.AutoDriveGyroUSING(135, -0.9*AutoSpeedUP, 5);
		intake.RunHorizontalFeeder(-0.8);
		shooter.SpinFlywheelPID(rpm);
		tankdrive.AutoDriveGyroUSING(33, -0.9*AutoSpeedUP, 5);
		intake.RunHorizontalFeeder(-0.8);
		intake.RunStarfishFeeder(0.8);
		frc::Wait(1.5_s);
		intake.RunHorizontalFeeder(-0.0);
		intake.RunStarfishFeeder(0.0);
		shooter.FlywheelZero();
		intake.SetIntakeZero();
		tankdrive.AutoDriveGyroUSING(60, 0.9*AutoSpeedUP, 5);

	}

	//4 ball red
	if(switch4 == true){
		intake.IntakePivot(0,0,0.6);
		frc::Wait(1.2_s);
		intake.IntakePivot(0,0,0.0);
		intake.RunIntake(0.95);
		shooter.SpinFlywheelPID(rpm);
		tankdrive.AutoDriveGyroUSING(35,.8*AutoSpeedUP, 3);
		frc::Wait(1_s);
		tankdrive.AutoDriveGyroUSING(10, -.5*AutoSpeedUP, 3);
		intake.RunHorizontalFeeder(-0.8);
		intake.RunStarfishFeeder(0.8);
		frc::Wait(1.8_s);
		intake.RunHorizontalFeeder(0.0);
		intake.RunStarfishFeeder(0.0);
		shooter.FlywheelZero();
		//Going to get third ball
		tankdrive.AutoTurnGyro(107,0.7*AutoSpeedUP,4);
		tankdrive.AutoDriveGyroUSING(50,.7*AutoSpeedUP,5);
		intake.RunIntake(0.95);	
		tankdrive.AutoDriveGyroUSING(60, 0.7*AutoSpeedUP, 4);
		intake.RunIntake(0.0);
		tankdrive.AutoDriveGyroUSING(22, 0.7*AutoSpeedUP, 3);
		//going to get third ball
		tankdrive.AutoTurnGyro(-43, 0.7*AutoSpeedUP, 2.5);
		//intake.RunStarfishFeeder(0.8);
		//intake.RunHorizontalFeeder(-0.8);
		//frc::Wait(0.8_s);
		//intake.RunStarfishFeeder(0.0);
		//intake.HorizontalFeederZero();
		tankdrive.AutoTurnGyro(3, 0.7*AutoSpeedUP, 2.5);
		tankdrive.AutoDriveGyroUSING(85, 0.7*AutoSpeedUP, 3);
		intake.RunIntake(0.95);
		tankdrive.AutoDriveGyroUSING(16, 0.3*AutoSpeedUP, 3);
		tankdrive.AutoTurnGyro(-0.5,0.5*AutoSpeedUP,4);
		tankdrive.AutoDriveGyroUSING(135, -0.9*AutoSpeedUP, 5);
		intake.RunHorizontalFeeder(-0.8);
		shooter.SpinFlywheelPID(rpm);
		tankdrive.AutoDriveGyroUSING(33, -0.9*AutoSpeedUP, 5);
		intake.RunHorizontalFeeder(-0.8);
		intake.RunStarfishFeeder(0.8);
		frc::Wait(1.5_s);
		intake.RunHorizontalFeeder(-0.0);
		intake.RunStarfishFeeder(0.0);
		shooter.FlywheelZero();
		intake.SetIntakeZero();
		tankdrive.AutoDriveGyroUSING(60, 0.9*AutoSpeedUP, 5);
			*/
	

	
}

void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {
  //ClimberUp = true;

  //Pbool aboveThreshold;
  IntakeUp = true;
  getz1 = -0.20;
  rpm1 = ((1 - (getz1) / 2)) *5700;
	LED = new Spark(3);
} 

void Robot::TeleopPeriodic()
{	
	Range = tankdrive.GetLidarRange();

	//SmartDashboard Requirements
	dash->PutNumber("Robot Speed", tankdrive.GetThrottle());
	dash -> PutNumber("ShooterEncoderReading",shooter.ReturnRPM());
	dash->PutNumber("Shooter Speed (On override)", (1 - stick2.GetZ()) / 2);
	dash->PutNumber("Climber Speed", (1 - stick3.GetZ()) / 2);
	dash->PutNumber("Yaw", tankdrive.GetAngle());
	//dash->PutNumber("Target found", tankdrive.limelight.targetIsFound());
	//dash->PutNumber("X offset", tankdrive.limelight.GetXOffset());
	//dash->PutNumber("Y offset", tankdrive.limelight.GetYOffset());
	//dash->PutNumber("T", DeltaT);
	dash->PutNumber("ultrasonic tank test", tankdrive.GetUSRange());
	dash->PutNumber("ultrasonic(in)", tankdrive.GetUSRange());
	dash->PutNumber("Lidar(in)", tankdrive.GetLidarRange());
	//dash->PutNumber("I", PConstantTeleAim);
	//dash->PutNumber("D", DConstantTeleAim);
	//dash->PutNumber("LeftTDEncoderPos",tankdrive.GetLEncoder());
	//dash->PutNumber("RightTDEncoderPos",tankdrive.GetREncoder());
	//dash ->PutNumber("Ystick1", stick1.GetY());
	//dash -> PutNumber("Ystcik2", stick2.GetY());
	//dash -> PutNumber("gyroangle", tankdrive.GetAngle());
	//dash -> PutNumber("Climber Encoder",intake.IntakePivotEncoderValue());

	if(Range > 115 && Range < 126.0){	
	dash -> PutBoolean("InRange",true);
	dash -> PutBoolean("InRange2",true);

	dash -> PutBoolean("InRange4",true);
	}else{
	dash -> PutBoolean("InRange",false); 
	dash -> PutBoolean("InRange2",false);
	dash -> PutBoolean("InRange4",false);
	
	}

	if(tankdrive.limelight.GetXOffset() > 10){
	dash -> PutBoolean("Right",true);
	dash -> PutBoolean("Left",false);
	}else if(tankdrive.limelight.GetXOffset() < -10){
	dash -> PutBoolean("Right",false);
	dash -> PutBoolean("Left",true);
	}
	else{
	dash -> PutBoolean("Right",false);
	dash -> PutBoolean("Left",false);
	}
//-----------------------------------------------------------------------------------------------------------------

	//Tankdrive
	tankdrive.SetThrottle(stick1.GetZ());
	tankdrive.limelight.Update();

	//LimelightAIM included drive
	/*if ((stick1.GetTrigger()))
	{
				tankdrive.DirectDrive(-1.0,-1.0);
		//tankdrive.;TeleAimLimelight(1,true);
		//tankdrive.TeleAimLimelight(.5, true);
	}
	else if (stick2.GetTrigger()){
		tankdrive.DirectDrive(1.0,1.0);
	}
	else
	{
		tankdrive.TeleAimLimelight(1.0, false);
		tankdrive.Drive(stick1.GetY(), stick2.GetY());
		//cam.EndMotor();
	}
*/
//-----------------------------------------------------------------------------------------------------------------
	//Intake System Mappings

//Intake and Horizontal Belts
	if (stick2.GetRawButton(2) || driverstation.GetRawButton(3))
	{
		intake.RunIntake(IntakeSpeed);
		intake.RunHorizontalFeeder(-HorizontalMotorSpeed);
	}
	else if (stick2.GetRawButton(3) || driverstation.GetRawButton(4))
	{
		intake.RunIntake(-IntakeSpeed);
		intake.RunHorizontalFeeder(HorizontalMotorSpeed);
	}
	else if (stick3.GetTrigger()){
		intake.RunHorizontalFeeder(-HorizontalMotorSpeed);
	}
	else
	{
		intake.SetIntakeZero();
		intake.HorizontalFeederZero();
	}

//Starfished Feeder

	if (stick1.GetRawButton(2) || driverstation.GetRawButton(2))
	{
		
		intake.RunStarfishFeeder(FeederSpeed);

	}
	else if(stick3.GetTrigger()&& shooter.ReturnRPM()>= 2500){
		intake.RunStarfishFeeder(FeederSpeed);
	}
	
	else if (stick1.GetRawButton(3) || driverstation.GetRawButton(6))
	{
		intake.RunStarfishFeeder(-FeederSpeed);
	}
	else
	{
		intake.StarfishZero();
	}

//-----------------------------------------------------------------------------------------------------------------
//Intake Pivot

	bool IntakePivotLimitTop = DIOIntakePivotLimitTop.Get();
	bool IntakePivotLimitBottom = DIOIntakePivotLimitBott.Get();
	dash->PutNumber("Bottom Pivot", DIOIntakePivotLimitTop.Get());
	dash->PutNumber("Top Pivot", DIOIntakePivotLimitBott.Get());
	if ((stick3.GetRawButton(7))){
		intake.IntakePivot(0,0,0.3);
	}
	else if ((stick3.GetRawButton(6))){
		intake.IntakePivot(0,0,-0.5);
	}

	else{
		intake.PivotZero();
	}

	

//-----------------------------------------------------------------------------------------------------------------
	// Shooter On

	 if(driverstation.GetRawButton(7)){
	shooter.SpinFlywheelPID(rpm1);
	}
	else if (stick3.GetRawButton(4)){
		shooter.RunFlywheel((stick3.GetY()*((1-stick3.GetZ())/2)));
		LED -> Set(0.77);
	}
	else
	{
		shooter.FlywheelZero();
		LED -> Set(0.61);
	}

//-----------------------------------------------------------------------------------------------------------------
	//Climber
	
	//Driver Station Black Button Enabled and One Direction Only;


	if (driverstation.GetRawButton(1))
	{
		climber.RunClimber((1 - stick3.GetZ()) / 2 * stick3.GetY());
	}
	//Climber Reset Override
	else if (stick1.GetRawButton(10) && stick1.GetRawButton(11) && stick2.GetRawButton(11)){
		climber.RunClimber(stick3.GetY() *((1 - stick3.GetZ()) / 2));
	}
	//Set Climber Zero
	else
	{
		climber.ZeroClimberSpeed();
	}
//---------------------------------------------------------------------------------------------------------------------

}

void Robot::DisabledInit() {
	/*dash -> PutBoolean("1 Ball:", DIOswitch0.Get());
	dash -> PutBoolean("2 Ball:", DIOswitch1.Get());
	dash -> PutBoolean("3 Ball:", DIOswitch2.Get());
	dash -> PutBoolean("4 Ball:", DIOswitch3.Get());
	dash ->PutBoolean("Alliance Color:", DIOswitch4.Get());
	
	if (DIOswitch4.Get() == true){
		LED -> Set(0.87);
	}
	else{
		LED -> Set(0.61);
	}
*/
}
void Robot::DisabledPeriodic() {}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
