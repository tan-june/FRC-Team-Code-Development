#include "Robot.h"

Robot::Robot():
tankdrive(0),
intake(),
climber(),
bling(),
shooter(),
stick1(0),
stick2(1),
stick3(2),
driverstation(3),
beamBreak0(0),
beamBreak1(1),
lidar(6)
{
dash -> init();
}

void Robot::RobotInit() {
    dash->PutNumber("Top RPM", 0.0);
    dash->PutNumber("Bottom RPM", 0.0);
   
}

void Robot::AutonomousInit() {
    int AutoPathNumber = 0;

    switch(AutoPathNumber){
        case 0:
            shooter.SpinFlywheelsPID(-700, 1100);
            Wait(2.0);
            shooter.SpinFlywheelsOpenLoop(0.0, 0.0);
            tankdrive.AutoDriveGyro(108, -0.2, 5.0, 0.0, false);
            //tankdrive.AutoDriveGyro(108, -0.2, 5.0);
            tankdrive.AutoTurnGyro(20.5, 0.3, 5.0);
            tankdrive.AutoDriveGyro(72, -0.2, 5.0);
            //tankdrive.AutoDriveGyro(96.0, -0.2, 10.0);
            //Wait(0.5);
            //tankdrive.AutoDriveGyro(195.0, -0.5, 5.0);
            //Stop intake
            //Shooting sequence here
            break;
        case 1:
            //Shooting sequence here
            tankdrive.AutoDriveGyro(100.0, 0.5, 2.5);
            //Start intake
            tankdrive.AutoTurnGyro(67.5, 0.5, 1.5);
            tankdrive.AutoDriveGyro(24.0, 0.5, 1.5);
            Wait(0.5);
            tankdrive.AutoDriveGyro(24.0, -0.5, 1.5);
            tankdrive.AutoTurnGyro(-67.5, 0.5, 1.5);
            //Stop intake
            //Shooting sequence here
            break;
    }
}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
    double lidarDist = lidar.GetDistance();



//Driving Code
tankdrive.SetThrottle(stick2.GetZ());

//Tankdrive Throttle
double tankspeed = (1-stick2.GetZ())/2;
dash -> PutNumber("Tankdrive Throttle: ", tankspeed);

//Driver Assistance
if(stick1.GetTrigger()){
    double drivePowerDiff = (stick1.GetY() - stick2.GetY())/4.0;
    tankdrive.Drive(stick1.GetY() - drivePowerDiff, stick2.GetY()-drivePowerDiff);
}
else{
    tankdrive.Drive(-1.0 * stick2.GetY(), -1.0 * stick1.GetY());
}

/*----------------------------------------------------------*/
//                      Intake System Code

if (stick2.GetButton(2) || (stick3.GetButton(8) && stick3.GetButton(11)) || driverstation.GetButton(6)){
//In (put negative sign)
intake.RunIntake(-0.5);
}
else if(stick2.GetButton(3) || (stick3.GetButton(9) && stick3.GetButton(11)) || driverstation.GetButton(2) || driverstation.GetButton(11))
{
//Out (keep positive)
    intake.RunIntake(0.5);
}
else{
    intake.SetIntakeZero();
}

//Pivot Code
if (stick3.GetButton(6)){
//
//(positive sign)
    intake.IntakePivot(0.5);
}
else if(stick3.GetButton(7)){
//
//(negative sign)
    intake.IntakePivot(-0.5);
}
else{
    intake.PivotZero();
}

/*----------------------------------------------------------*/
//                          Climber Code

climberspeed = (1-stick1.GetZ())/2;
double ClimberEncoderPositionVar = climber.EncoderValue();
dash -> PutNumber("Climber Get Encoder Position: ", ClimberEncoderPositionVar);
dash -> PutNumber("Climber Speed:", climberspeed);

//Sending the Climber Up and Down Using Buttons 8 and 9
if (stick1.GetButton(3)){
    //up
    //negative
    climber.ClimberDirection(-1.0);
}
else if (stick1.GetButton(2)){
    //down
    //positive
    climber.ClimberDirection(1.0);
}
else{
    //zero
    climber.ZeroClimberSpeed();
}

/*                     Shooter                              */

shooterspeed = (1-stick3.GetZ())/2;
int encoderDist = 0.0142*lidarDist*lidarDist-13.038*lidarDist+1402.4;

if(driverstation.GetButton(10) && driverstation.GetButton(7)){
    shooter.TeleAimLimelight();
}
else{
    shooter.TurnTurret(0.0);
}

if (driverstation.GetButton(7) && !driverstation.GetButton(10)){
shooter.SpinFlywheelsPID(dash->GetNumber("Top RPM", 0.0), dash->GetNumber("Bottom RPM", 0.0));
    /*if(lidarDist >= 115 && lidarDist <= 310)
         shooter.SpinFlywheelsPID(-700, 1100);
     else if(lidarDist > 310)
         shooter.SpinFlywheelsPID(-1500, 1500);
     else
         shooter.SpinFlywheelsPID(-800, 800);*/
}
    
else{
    shooter.SpinFlywheelsOpenLoop(0.0,0.0);
    }

if (stick3.GetButton(4) || driverstation.GetButton(12)){
   // up 
    shooter.FeederWheel(-0.5);
}
else if (stick3.GetButton(5)){
 //   down
    shooter.FeederWheel(0.5);
}
else{
    shooter.FeederWheel(0.0);
}

dash->PutNumber("Top Velocity", shooter.GetTopFlywheelVelocity());
dash->PutNumber("BottomVelocity", shooter.GetBottomFlywheelVelocity());


if(stick3.GetButton(11))
    shooter.ResetHoodEncoder();


//Firing Using Driverstation Buttons
if((driverstation.GetButton(11))){
    intake.RunTower(-0.5);
    shooter.FeederWheel(-0.5);
}
else if(driverstation.GetButton(5) && beamBreak0.GetValue() < 50 && beamBreak1.GetValue() > 50){
    intake.RunTower(-0.5);
    shooter.FeederWheel(0.0);
}
else if(!driverstation.GetButton(5) && (driverstation.GetButton(3) || stick3.GetButton(2))){
    intake.RunTower(-0.5); 
    shooter.FeederWheel(0.0);
}
else if(!driverstation.GetButton(5) && (driverstation.GetButton(4) || stick3.GetButton(3))){
    intake.RunTower(0.5);
    shooter.FeederWheel(0.0);
}
else{
    intake.RunTower(0);
    shooter.FeederWheel(0.0);
}
    
//Moving Hood Using Driverstation Buttons
if (driverstation.GetButton(8)){
    shooter.MoveHood(0.5);
}
else if (driverstation.GetButton(9)){
    shooter.MoveHood(-0.5);
}
else{
shooter.MoveHood(0.0);

}
if(DEBUG){
    dash->PutNumber("Hood Position", shooter.GetHoodPosition());
    dash->PutNumber("Hood Encoder Target", encoderDist);
    dash->PutNumber("Lidar Distance", lidarDist);
    dash->PutNumber("Beam break 0", beamBreak0.GetValue());
}
}



void Robot::TestInit() {}
void Robot::TestPeriodic() {}


#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif


