#pragma once
#include <frc/Timer.h>
#include <frc/DigitalInput.h>
#include <frc/Jaguar.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"
#include "Ultrasonic.h"
#include "OECPigeonIMU.h"
#include "Limelight.h"
#include "OECPIDController.h"
#include "PathReader.h"

using namespace frc;
class Tankdrive
{
public: // for functions
	Tankdrive(unsigned int UsonicPort);
	void Drive(float left, float right);
	void DirectDrive(float left, float right);

	void DriveR(double power);
	void DriveL(double power);

	int DirectDrivePID(float leftRPM, float rightRPM, bool reset); // 0 - PID ran on neither, 1 - PID ran on right only, 2 - PID ran on left only, 3 - PID ran on both
	void DrivePositionPID(float leftPos, float rightPos, float lRPM, float rRPM, bool reset);

	void DrivePath(std::string leftFile, std::string rightFile);

	void TeleAimLimelight(float speed, bool enable);
	int TeleDriveLimelight(float USrange, float speed, float bias, bool enable);

	void SetThrottle(float Ithrottle);
	void SetRawThrottle(float Ithrottle);

	void AutoDriveGyro(float distance, float speed, float TimeOut);
	void AutoDriveGyro(float distance, float speed, float TimeOut, bool startup);
	void AutoDriveGyro(float distance, float speed, float TimeOut, float rampTime, bool stopAtEnd);
	void AutoCurveGyro(float distance, float radius, float speed, float TimeOut, float rampTime, bool stopAtEnd);
	void AutoCurveGyroAngle(float angle, float radius, float speed, float TimeOut, float rampTime, bool stopAtEnd);
	void AutoDriveGyroLimit(float distance, float speed, float TimeOut, DigitalInput& LimitLift, Jaguar& Lift);
	void AutoTurnGyroBoth(float angle, float speed, float TimeOut);
	void AutoTurnGyro(float angle, float speed, float TimeOut);
	void AutoDriveGyroUS(float, float, float);

	int AutoDriveLimelight(float USrange, float speed, float Maxdistance, float TimeOut);

	bool IsLimit();

	double GetREncoder();
	double GetLEncoder();
	void ResetEncoders();
	void ResetGyro();

	double GetAngle();
	void GetUSSample();
	double GetUSRange();
private:
	double rEncoderOffset = 0.0;
	double lEncoderOffset = 0.0;
	rev::CANSparkMax RightF;
	rev::CANSparkMax RightB;
	rev::CANSparkMax RightT;
	rev::CANSparkMax LeftF;
	rev::CANSparkMax LeftB;
	rev::CANSparkMax LeftT;


	


	rev::CANEncoder LWEncoder;
	rev::CANEncoder RWEncoder;

	OECPigeonIMU Gyro;

	Timer AutoTimer;
	Limelight limelight;
	USSensor Usonic;
	
	float VisionX;
	float throttle;
	
	OECPIDController CurveController;

	OECPIDController rdbSpeedController;
	OECPIDController ldbSpeedController;

	OECPIDController ldbPosController;
	OECPIDController rdbPosController;

	Timer RPMTimer;

	PathReader pathReader;

	//Vision:
	int returnC;
	float Sample, LastSample;
	float Integral;
	float Derivative;
	float Turn;
	bool USGood;

//Variables for RPM PIDs:
//Last reocrded position of left and right motors
double rLastPosition;
double lLastPosition;

//Time at the last recorded position of left and right motors
double rTimeLastChange;
double lTimeLastChange;

//Left and right PID correction
double rCorrection = 0.0;
double lCorrection = 0.0;

bool speedIsFirstRun;

double lsP = 0.0;
double lsI = 0.0;
double lsD = 0.0;
double lsLastError = 0.0;

double rsP = 0.0;
double rsI = 0.0;
double rsD = 0.0;
double rsLastError = 0.0;

double avgLRPM = 0.0;
double avgRRPM = 0.0;

};
