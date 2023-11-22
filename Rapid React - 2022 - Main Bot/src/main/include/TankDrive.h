
#pragma once
#include <frc/Timer.h>
#include <frc/motorcontrol/PWMVictorSPX.h>
#include <frc/DigitalInput.h>
#include <frc/motorcontrol/Jaguar.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"
#include "LidarLite.h"
#include "Usonic.h"
#include "OECPigeonIMU.h"
#include "Limelight.h"
#include <units/time.h>
#include "PIDController.h"
#include <rev/SparkMaxRelativeEncoder.h>

using namespace frc;
class Tankdrive
{
public: // for functions
	Tankdrive(int gyroport);
	rev::CANSparkMax LeftF;
	rev::CANSparkMax LeftB;
	rev::CANSparkMax LeftT;
	rev::CANSparkMax RightF;
	rev::CANSparkMax RightB;
	rev::CANSparkMax RightT;

    rev::SparkMaxPIDController * LeftFrontPID;
    rev::SparkMaxPIDController * LeftBackPID;
    rev::SparkMaxPIDController * LeftTopPID;
    rev::SparkMaxPIDController * RightFrontPID;
    rev::SparkMaxPIDController * RightBackPID;
    rev::SparkMaxPIDController * RightTopPID;
	   
	Timer AutoTimer;
	LidarLite lidar;
	LimeLight limelight;
	Usonic usonic;
	OECPigeonIMU gyro;
	PIDcontroller rdbSpeedController,ldbSpeedController,rdbPosController,ldbPosController,autoAimLLPID;


	Timer RPMTimer;
	void Drive(float left, float right);
	void DirectDrive(float left, float right);
	void DirectDriveRPM(float left, float right);

	void DriveR(double power);
	void DriveL(double power);

	int DirectDrivePID(float leftRPM, float rightRPM, bool reset); // 0 - PID ran on neither, 1 - PID ran on right only, 2 - PID ran on left only, 3 - PID ran on both
	void DrivePositionPID(float leftPos, float rightPos, float lRPM, float rRPM, bool reset);

	void DrivePath(std::string leftFile, std::string rightFile);
	
	

	void TeleAimLimelight(float speed, bool enable);
	int TeleDriveLimelight(float USrange, float speed, float bias, bool enable);
	void AutoDriveLimelight(float USrange, float speed, float Maxdistance, float TimeOut);

	void SetThrottle(float Ithrottle);
	void SetRawThrottle(float Ithrottle);
	double GetThrottle();
	//Removed everything related to gyro might go back to later later
	
	void AutoDriveGyroUSING(float distance, float speed, float TimeOut);
	void AutoDriveGyroRPM(float distance, float speed, float TimeOut);
	void AutoDriveGyroNOTUSING(float distance, float speed, float TimeOut, bool startup);
	void AutoDriveGyroNOTUSING(float distance, float speed, units::second_t TimeOut, units::second_t rampTime, bool stopAtEnd);
	void AutoCurveGyroNOTUSING(float distance, float radius, float speed, float TimeOut, float rampTime, bool stopAtEnd);
	void AutoCurveGyroAngle(float angle, float radius, float speed, float TimeOut, float rampTime, bool stopAtEnd);
	void AutoDriveGyroLimit(float distance, float speed, float TimeOut, DigitalInput& LimitLift, Jaguar& Lift);
	void AutoTurnGyroBoth(float angle, float speed, float TimeOut);
	void AutoTurnGyro(float angle, float speed, float TimeOut);
	void AutoDriveGyroUS(float, float, float);
	//units::second_t TIMEPERIOD{1};
	//units::second_t AUTOTIMEMAX{1};
	units::second_t ramptime{1};
	units::second_t timeout{1};
	float TimeOut = timeout.value();
	float rampTime = ramptime.value();

	
	//int AutoDriveLimelight(float USrange, float speed, float Maxdistance, float TimeOut);

	bool IsLimit();

	double GetREncoder();
	double GetLEncoder();
	void ResetEncoders();
	void ResetGyro();
	double GetLidarRange();
	double GetAngle();
	void GetUSSample();
	double GetUSRange();
	rev::SparkMaxRelativeEncoder  *LFEncoder;
	rev::SparkMaxRelativeEncoder  *RFEncoder;
	


private:
	double rEncoderOffset = 0.0;
	double lEncoderOffset = 0.0;
	
	


	


	float VisionX;
	float throttle;
	
	
	
	




	



	//PathReader pathReader;

	//Vision:
	int returnC;
	float Sample, LastSample;
	float Integral;
	float Derivative;
	float Turn;
	bool USGood;
	bool lastenable;

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