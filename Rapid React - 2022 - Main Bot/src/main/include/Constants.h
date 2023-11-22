#pragma once

//Speed Up 4 Ball Auto
#define AutoSpeedUP 1.047

//Motor Ports
#define LeftBackNeo 4
#define LeftFrontNeo 5
#define LeftTopNeo 6
#define RightBackNeo 7
#define RightFrontNeo 8
#define RightTopNeo 9
#define IntakeMotorNo 10
#define BallFeederMotorNo 11
#define HorizontalMotorNo 12
#define PivotMotorNo 13
#define ShooterMotorNo 14
#define ClimberMotorNo 15

//Joystick Ports
#define JoystickPort 0
#define JoystickPort2 1
#define JoystickPort3 2
#define JoystickPort4 3

//Shooter PID Controller Constant
#define SHOOTER_P 0.00001
#define SHOOTER_I 0
#define SHOOTER_D 0
#define SHOOTER_FF 0.00017618
#define SHOOTER_IZONE 300
#define kMinOutput -1
#define kMaxOutput 1

//Ports
#define LidarPort 6
//#define CameraServoPort 2
#define TurretCameraMotor 4
#define JoystickPort 0
#define JoystickPort2 1
#define ServoDegStep 1
#define rotatexspeed 0.5
#define paddingbox 10
#define LIDAR_OFFSET -10
#define USONICPORT 8
#define AUTOVISIONSPEED	0.36		// check this value!!!


#define RPM_SCALE 1.3
//PID Constant(Used in limelight auto aim)
#define TIMEPERIOD 0.00001
#define PCONSTANT -0.006
#define ICONSTANT -0.025	// was 300
#define DCONSTANT -0		// was 0.000125
//Drive Base Limelight Aim PID
#define AIM_P 0.001   // Was 0.012
#define AIM_I 0	// was -0.025
#define AIM_D 0
//PID
#define DeltaT 0.001
#define PConstantTeleAim 0.2
#define IConstantTeleAim 0
#define DConstantTeleAim 0
//temporary PWM motor testing
#define L1 3
#define L2 4
#define L3 5
#define R1 0
#define R2t 1
#define R3 2

//Intake System
#define PivotSpeed 0.5
#define IntakeSpeed 0.85
#define HorizontalMotorSpeed 0.75
#define IntakeSpeedAuto 0.45
#define FeederSpeed .50
#define ROTATION_COUNT_PIVOT 1

//AutoDrive
#define Distance1 5
#define Speed1 -.2
#define timeout1 1
#define rampuptime .4
#define Waitinterrupt .7_s

#define Distance2 4
#define Speed2 -.5
#define timeout2 .5

#define Distance3 15
#define Speed3 -.15
#define timeout3 .5


//Tankdrive 
//Populate with correct values later
#define AUTOGYROCONST -0.012
#define AUTOTIMEMAX 1_s
#define ANGTOLERANCE 1
#define DB_SEMI_WIDTH 1
#define ENCODERCONST 1.148722

//was 6.8971
//Drive Base Speed PID
#define DB_FREE_SPEED 5880.0
#define DBS_P -0.004
#define DBS_I -0.0
#define DBS_D 0.0
#define DBS_MAX 1.0
//Drive Base Position PID
#define DBP_P 0.0
#define DBP_I 0.0
#define DBP_D 0.0
#define DBP_MAX 5880
//Curve PID
#define CURVE_P -0.07
#define CURVE_I -0.00005
#define CURVE_D -8.0

#define PATH_DT 0.01

//rev PID tank drive

#define DRIVE_P 0.01
#define DRIVE_I 1e-4
#define DRIVE_D 1
#define DRIVE_FF 0
#define DRIVE_IZONE 0
//Climber
#define ROTATION_COUNT_CLIMBER 5 
#define CLIMBER_SPEED .5

//Designated rmp for lights to go green
#define Green_RPM 4853