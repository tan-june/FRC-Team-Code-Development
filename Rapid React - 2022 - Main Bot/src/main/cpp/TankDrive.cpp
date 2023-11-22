
#include "Tankdrive.h"

// Convencion: Teleob gets joystick vals, AUTO: feed positive vals

Tankdrive::Tankdrive(int gyroport):

//defines the motors
LeftF(LeftFrontNeo, rev::CANSparkMax::MotorType::kBrushless),
LeftB(LeftBackNeo, rev::CANSparkMax::MotorType::kBrushless),
LeftT(LeftTopNeo, rev::CANSparkMax::MotorType::kBrushless),
RightF(RightFrontNeo, rev::CANSparkMax::MotorType::kBrushless),
RightB(RightBackNeo, rev::CANSparkMax::MotorType::kBrushless),
RightT(RightTopNeo, rev::CANSparkMax::MotorType::kBrushless),

//Sensors

AutoTimer(),
lidar(),
limelight(),
usonic(),
gyro(gyroport),

//PID Things

rdbSpeedController(),
ldbSpeedController(),
rdbPosController(),
ldbPosController(),

//grabs encoder values and sets PID constants
RPMTimer()
{
	RPMTimer.Start();
	//Speed controller PID constants
	rdbSpeedController.SetConstants(DBS_P, DBS_I, DBS_D, DBS_MAX);
	ldbSpeedController.SetConstants(DBS_P, DBS_I, DBS_D, DBS_MAX);
	ldbPosController.SetConstants(DBP_P, DBP_I, DBP_D, DBP_MAX);
	rdbPosController.SetConstants(DBP_P, DBP_I, DBP_D, DBP_MAX);
	gyro.ResetAngle();
	throttle = 0.0;
	VisionX = 0.0;
	//Get encoders
	RFEncoder = new rev::SparkMaxRelativeEncoder(RightF.GetEncoder());
	LFEncoder = new rev::SparkMaxRelativeEncoder(LeftF.GetEncoder());
	//Conversion factrs
	LFEncoder->SetPositionConversionFactor(ENCODERCONST);
	RFEncoder->SetPositionConversionFactor(-1.0* ENCODERCONST);
	//Left Frnt PID
	LeftFrontPID = new rev::SparkMaxPIDController(LeftF.GetPIDController());
    LeftFrontPID -> SetP(SHOOTER_P);
    LeftFrontPID ->SetI(SHOOTER_I);
    LeftFrontPID ->SetD(SHOOTER_D);
    LeftFrontPID ->SetFF(SHOOTER_FF);
    LeftFrontPID ->SetIZone(SHOOTER_IZONE);
    LeftFrontPID ->SetOutputRange(kMinOutput, kMaxOutput);
	//Left Back PID
	LeftBackPID = new rev::SparkMaxPIDController(LeftB.GetPIDController());
    LeftBackPID -> SetP(SHOOTER_P);
    LeftBackPID ->SetI(SHOOTER_I);
    LeftBackPID ->SetD(SHOOTER_D);
    LeftBackPID ->SetFF(SHOOTER_FF);
    LeftBackPID ->SetIZone(SHOOTER_IZONE);
    LeftBackPID ->SetOutputRange(kMinOutput, kMaxOutput);
	//Left Top PID
	LeftTopPID = new rev::SparkMaxPIDController(LeftT.GetPIDController());
    LeftTopPID -> SetP(SHOOTER_P);
    LeftTopPID ->SetI(SHOOTER_I);
    LeftTopPID ->SetD(SHOOTER_D);
    LeftTopPID ->SetFF(SHOOTER_FF);
    LeftTopPID ->SetIZone(SHOOTER_IZONE);
    LeftTopPID ->SetOutputRange(kMinOutput, kMaxOutput);
	//Right Front PID
	RightFrontPID = new rev::SparkMaxPIDController(RightF.GetPIDController());
    RightFrontPID -> SetP(SHOOTER_P);
    RightFrontPID ->SetI(SHOOTER_I);
    RightFrontPID ->SetD(SHOOTER_D);
    RightFrontPID ->SetFF(SHOOTER_FF);
    RightFrontPID ->SetIZone(SHOOTER_IZONE);
    RightFrontPID ->SetOutputRange(kMinOutput, kMaxOutput);
	//Right Back PID
	RightBackPID = new rev::SparkMaxPIDController(RightB.GetPIDController());
    RightBackPID -> SetP(SHOOTER_P);
    RightBackPID ->SetI(SHOOTER_I);
    RightBackPID ->SetD(SHOOTER_D);
    RightBackPID ->SetFF(SHOOTER_FF);
    RightBackPID ->SetIZone(SHOOTER_IZONE);
    RightBackPID ->SetOutputRange(kMinOutput, kMaxOutput);
	//Right Top PID
	RightTopPID = new rev::SparkMaxPIDController(RightT.GetPIDController());
    RightTopPID -> SetP(SHOOTER_P);
    RightTopPID ->SetI(SHOOTER_I);
    RightTopPID ->SetD(SHOOTER_D);
    RightTopPID ->SetFF(SHOOTER_FF);
    RightTopPID ->SetIZone(SHOOTER_IZONE);
    RightTopPID ->SetOutputRange(kMinOutput, kMaxOutput);


}

//Variables for position PIDs
int rightCount = 1;
int leftCount = 1;

void Tankdrive::Drive(float left, float right)
{
	// Limit left and right inputs to between -1 and 1
	if(left > 1.0)
		left = 1.0;
	else if(left < -1.0)
		left = -1.0;
	if(right > 1)
		right = 1.0;
	else if(right < -1.0)
		right = -1.0;
	LeftF.Set(left * throttle  * -1.0);
	LeftB.Set(left * throttle  * -1.0);		// becuase joystick values of inversed!!!!
	LeftT.Set(left * throttle  * -1.0);
	RightF.Set(right * throttle);
	RightB.Set(right * throttle);
	RightT.Set(right * throttle);
}
//Takes in power value from -1 to 1 and converts to a scaled RPM
void Tankdrive::DirectDriveRPM(float left, float right){
	//do not use implementation - neo spark max doesn't allow it to be greater than 3000 RPM
	if(left > 1.0)
		left = 1.0;
	else if(left < -1.0)
		left = -1.0;
	if(right > 1.0)
		right = 1.0;
	else if(right < -1.0)
		right = -1.0;

	double leftRPM = 5300*left*RPM_SCALE;
	double rightRPM = 5300*right*-1.0*RPM_SCALE;

	LeftFrontPID->SetReference(leftRPM, rev::CANSparkMax::ControlType::kVelocity, 0, 0.0, rev::SparkMaxPIDController::ArbFFUnits::kVoltage);
	LeftBackPID->SetReference(leftRPM, rev::CANSparkMax::ControlType::kVelocity, 0, 0.0, rev::SparkMaxPIDController::ArbFFUnits::kVoltage);		
	LeftTopPID->SetReference(leftRPM, rev::CANSparkMax::ControlType::kVelocity, 0, 0.0, rev::SparkMaxPIDController::ArbFFUnits::kVoltage);


	RightFrontPID->SetReference(rightRPM, rev::CANSparkMax::ControlType::kVelocity, 0, 0.0, rev::SparkMaxPIDController::ArbFFUnits::kVoltage);
	RightBackPID->SetReference(rightRPM, rev::CANSparkMax::ControlType::kVelocity, 0, 0.0, rev::SparkMaxPIDController::ArbFFUnits::kVoltage);
	RightTopPID->SetReference(rightRPM, rev::CANSparkMax::ControlType::kVelocity, 0, 0.0, rev::SparkMaxPIDController::ArbFFUnits::kVoltage);


}

//directly drives no joystick input, can be used by a computer
void Tankdrive::DirectDrive(float left, float right)
{
	if(left > 1.0)
		left = 1.0;
	else if(left < -1.0)
		left = -1.0;
	if(right > 1.0)
		right = 1.0;
	else if(right < -1.0)
		right = -1.0;

	LeftF.Set(left);
	LeftB.Set(left);		// becuase joystick values of inversed!!!!
	LeftT.Set(left);

	RightF.Set(right*-1.0);
	RightB.Set(right*-1.0);
	RightT.Set(right*-1.0);
}

//gets various vars needed for PID, then accounts for it in the speed, no user input

int Tankdrive::DirectDrivePID(float left, float right, bool reset){
	bool ranl = false; //Variable to tell if the left side PID ran
	bool ranr = false; //Variable to tell if the right side PID ran

	//current time at start of the loop
	double currTime = RPMTimer.Get().value();

	//right and left RPMs
	double rRPM = 0.0;
	double lRPM = 0.0;

	//left and right encoder positions converted from inches to rotations
	double rPosition = Tankdrive::GetREncoder();
	double lPosition = Tankdrive::GetLEncoder();

	//left and right RPM error
	double errorR = 0.0;
	double errorL = 0.0;

	//left and right approximate powers based on free speed and rpm input
	double rPower = right/DB_FREE_SPEED;
	double lPower = left/DB_FREE_SPEED;

	//Resets controllers, encoders, and variables between uses
	if(reset){
		rdbSpeedController.ResetController();
		ldbSpeedController.ResetController();
		lTimeLastChange = currTime;
		rTimeLastChange = currTime;
		rLastPosition = rPosition;
		lLastPosition = lPosition;
		lCorrection = 0.0;
		rCorrection = 0.0;
	}

	//If the right encoder position changed from last time calculate RPM, run the PID, and update variables for next time
	if(rLastPosition != rPosition){
		rRPM = (rPosition - rLastPosition)/(currTime - rTimeLastChange);
		avgRRPM = 0.75;		//half-life = 2.4 cycles
		avgRRPM += 0.25*rRPM;
		ranr = true;
		errorR = avgRRPM - right;

		rsI += errorR*(currTime - rTimeLastChange);
		rsD = (errorR - rsLastError)/(currTime - rTimeLastChange);

		rCorrection = DBS_P*errorR + DBS_I*rsI + DBS_D*rsD;
		rLastPosition = rPosition;
		rTimeLastChange = currTime;
		rsLastError = errorR;
	}

	//If the left encoder position changed from last time calculate the RPM, run the PID, and update variables for next time
	if(lLastPosition != lPosition){
		//Last RPM
		lRPM = (lPosition - lLastPosition)/(currTime - lTimeLastChange)*60.0;
		avgLRPM = 0.75; 
		avgLRPM += 0.25*lRPM;
		ranl = true;
		errorL = avgLRPM - left;
		
		lsI += errorL*(currTime - lTimeLastChange);
		lsD = (errorL - lsLastError)/(currTime - lTimeLastChange);

		lCorrection = DBS_P * errorL + DBS_I * lsI + DBS_D * lsD;

		lLastPosition = lPosition;
		lTimeLastChange = currTime;
		lsLastError = errorL;

	}
	lPower += lCorrection;
	rPower += rCorrection;

	Tankdrive::DirectDrive(lPower, rPower);
	Wait(0.002_s);
	return (int)ranr + (int)ranl * 2; //return a value 0-3 for which PIDs ran. 0 = none 1 = right only 2 = left only 3 = both
}

bool lastEnable = false;
double LastTime = 0.0;
double Time = 0.0;

void Tankdrive::SetThrottle(float Ithrottle)
{
	throttle = (1 - Ithrottle) / 2;
}
void Tankdrive::SetRawThrottle(float Ithrottle)
{
	throttle = Ithrottle;
}

double Tankdrive::GetThrottle()
{
	return throttle;
}

// Already have limelight aiming (future comment will need to intergrate with can)
//uses limelight values for a PID aiming function
 void Tankdrive::TeleAimLimelight(float speed, bool enable){


	if(enable && !lastEnable){
		//Thresholding
		if(speed > 1)
			speed = 1;
		else if(speed < -1)
			speed = -1;

		AutoTimer.Reset();
		AutoTimer.Start();
		lastenable = true;

	}
	if(enable){
		limelight.Update();
		//double Time = AutoTimer.Get().value();
		if (limelight.targetIsFound() == 1 )
		{
			//PID for Vison controller
			Sample = limelight.GetXOffset(); 
   			Integral = Integral + (DeltaT/2)*(Sample+LastSample);
    		Derivative = (Sample-LastSample)/2;
   			Turn = (PConstantTeleAim*Sample) + (IConstantTeleAim*Integral) + (DConstantTeleAim*Derivative);


			Tankdrive::DirectDrive(speed*Turn, -1*speed*Turn);
			Sample = LastSample;
			
		}
		else{
			Tankdrive::DirectDrive(0.0, 0.0);
		}
	}
}

//AUTO DRIVE THINGS
//autodrivegyroUSING is the one we currently use in the auto, it works and has been debugged, the otherones are not debbuged
void Tankdrive::AutoDriveGyroUSING(float distance, float speed, float TimeOut) //Args are distance, speed
{
	//Thresholding
	if(speed > 1){
		speed = 1;
	}
	else if(speed < -1){
		speed = -1;
	}

	AutoTimer.Reset();
	AutoTimer.Start();
	ResetEncoders();
	gyro.ResetAngle();

	Tankdrive::DirectDrive(speed, speed);		//Drives both motors at standard length

//Conditions are to locate if the correct amount of distance has been travled and if the period ot driving straight has ended
	while((((fabs(GetLEncoder()) + fabs(GetREncoder())) / 2) < distance) && AutoTimer.Get().value() < TimeOut)
	{								// was +							was -
		double gyroyaw = gyro.GetYaw();
		//Adjusts dependant on the gyro angle
		Tankdrive::DirectDrive((speed-(fabs(speed))*AUTOGYROCONST*gyroyaw), speed+(fabs(speed))*AUTOGYROCONST*gyroyaw);
		Wait(0.01_s); //adjust this time period
	}
	Tankdrive::DirectDrive(0.0,0.0);
}

void Tankdrive::AutoDriveGyroRPM(float distance, float speed, float TimeOut) //Args are distance, speed
{
	//Thresholding
	if(speed > 1){
		speed = 1;
	}
	else if(speed < -1){
		speed = -1;
	}

	AutoTimer.Reset();
	AutoTimer.Start();
	ResetEncoders();
	gyro.ResetAngle();

	Tankdrive::DirectDriveRPM(speed, speed);		//Drives both motors at standard length

//Conditions are to locate if the correct amount of distance has been travled and if the period ot driving straight has ended
	while((((fabs(GetLEncoder()) + fabs(GetREncoder())) / 2) < distance) && AutoTimer.Get().value() < TimeOut)
	{								// was +							was -
		double gyroyaw = gyro.GetYaw();
		//Adjusts dependant on the gyro angle
		Tankdrive::DirectDriveRPM((speed-(fabs(speed))*AUTOGYROCONST*gyroyaw), speed+(fabs(speed))*AUTOGYROCONST*gyroyaw);
		Wait(0.01_s); //adjust this time period
	}
	Tankdrive::DirectDriveRPM(0.0,0.0);
}

void Tankdrive::AutoDriveGyroNOTUSING(float distance, float speed, float TimeOut, bool startup)
{
	
	const float startfrac = 0.2;
	const int startms = 350;
	float dspeed;

	if(speed > 1)
		speed = 1;
	else if(speed < -1)
		speed = -1;

	AutoTimer.Reset();
	AutoTimer.Start();
	ResetEncoders();
	    //Reset Wheel Encoders
	gyro.ResetAngle();

	Tankdrive::DirectDrive(speed, speed);		//Drives both motors at standard length
	int i = 0;

	//same as other autodrive
	while((((fabs(GetLEncoder()) + fabs(GetREncoder())) / 2) < distance) && AutoTimer.Get().value()<=TimeOut)
	{								// was +							was -
		if( i>=startms )dspeed=speed;
		else dspeed = speed * (startfrac * (1-startfrac)*((float)i/(float)startms));
		Tankdrive::DirectDrive((dspeed-(fabs(dspeed))*AUTOGYROCONST*gyro.GetYaw()), dspeed+(fabs(dspeed))*AUTOGYROCONST*gyro.GetYaw());
		i++;
		Wait(0.01_s); //Adjust this number
	}
	Tankdrive::DirectDrive(0.0,0.0);
}


void Tankdrive::AutoDriveGyroNOTUSING(float distance, float speed,  units::second_t TimeOut, units::second_t rampTime, bool stopAtEnd)
{
	float RampRateR = 0.0;
	float RampRateL = 0.0;

	if(speed > 1)
		speed = 1;
	else if(speed < -1)
		speed = -1;
	//Ramptime rate calculations
	if(rampTime.value() != 0.0){
		RampRateR = (speed+RightF.Get())/rampTime.value();
		RampRateL = (speed-LeftF.Get())/rampTime.value();
	}
	float RISpeed = -1.0*RightF.Get();
	float LISpeed = LeftF.Get();
	float RSpeed = RISpeed;
	float LSpeed = LISpeed;
	ResetEncoders();
	    //Reset Wheel Encoders
	gyro.ResetAngle();
	AutoTimer.Reset();
	AutoTimer.Start();
	//Same as other Autodrive
	while((((fabs(GetLEncoder()) + fabs(GetREncoder())) / 2) < distance) && AutoTimer.Get() <= TimeOut)
	{
	
		if(AutoTimer.Get()<ramptime){
			//Increases speed as aproaches closer to desired speed
			RSpeed = RampRateR * AutoTimer.Get().value()+RISpeed;
			LSpeed = RampRateL *AutoTimer.Get().value()+LISpeed;
			//printf("ramp\n");
		}
		else{
			RSpeed = speed;
			LSpeed = speed;
		}
		DirectDrive(LSpeed-(AUTOGYROCONST * gyro.GetYaw()), RSpeed+(AUTOGYROCONST * gyro.GetYaw()));
		Wait(0.1_s);
	}
	if(stopAtEnd){
		//printf("stopped here\n");
		Tankdrive::DirectDrive(0.0,0.0);
	}
	else{
		//printf("kept going here\n");
		DirectDrive(speed, speed);
	}
}

void Tankdrive::AutoDriveLimelight(float USrange, float speed, float Maxdistance, float TimeOut) //Args are distance, speed
{
	returnC = 0;
	Integral = 0.0;
	LastSample = 0.0;
	

	if(speed > 1)
		speed = 1;
	else if(speed < -1)
		speed = -1;

	AutoTimer.Reset();
	AutoTimer.Start();
	ResetEncoders();
	    //Reset Wheel Encoders

	

	
		limelight.Update();
		if (limelight.targetIsFound() == 1.0)
		{
			//PID control
			Sample = limelight.GetXOffset()*6.0;
			Integral = Integral + (TIMEPERIOD/2)*(Sample+LastSample);
		    Derivative = (Sample - LastSample)/TIMEPERIOD;
		    Turn = PCONSTANT*Sample + ICONSTANT*Integral + DCONSTANT*Derivative;
			Tankdrive::DirectDrive(speed*(1 - Turn), speed*(1 + Turn));
			LastSample = Sample;
		}
		else{
			Tankdrive::DirectDrive(0.0,0.0); //Needed to prevent crash
		Wait( 0.00001_s);
		}
}
	
	


void Tankdrive::AutoTurnGyroBoth(float angle, float speed, float TimeOut)	 //Args are angle, speed
{
    double diff = 0.0;
	AutoTimer.Reset(); AutoTimer.Start();
	ResetEncoders();
	    //Reset Wheel Encoders
	gyro.ResetAngle();
	if(speed>1)speed=1;
	else if(speed<-1)speed=-1;				// was -	was +
	if (angle > 0.0) Tankdrive::DirectDrive(speed, -1.0*speed);
	else Tankdrive::DirectDrive(-1.0*speed, speed);

	while (fabs(gyro.GetYaw()) <= fabs(angle)  && AutoTimer.Get().value() <= TimeOut)	//When the gyroscope gives a reading below/equal to 45
	{	//Angle difference
		diff=(fabs(angle)-fabs(gyro.GetYaw()));

		if(diff/ANGTOLERANCE <= 1.0)
		{		//Changes direction also proportional speed scaling
				if (angle > 0.0) Tankdrive::DirectDrive(speed*diff, -1.0*speed*diff);
				else Tankdrive::DirectDrive(-1.0*speed*diff, speed*diff);
		}
	    Wait(0.001_s);
		
	}
	Tankdrive::DirectDrive(0.0, 0.0);
}

void Tankdrive::AutoTurnGyro(float angle, float speed, float TimeOut)	 //Args are angle, speed
{
//	float diff = 0.0;
	AutoTimer.Reset(); AutoTimer.Start();
	ResetEncoders();
	    //Reset Wheel Encoders
	gyro.ResetAngle();
	if(speed>1)speed=1;
	else if(speed<-1)speed=-1;				// was -	was +
	if (angle * speed > 0.0)
		Tankdrive::DirectDrive(1.0 * speed, 0.0 * speed);
	else if (angle * speed < 0.0)
		Tankdrive::DirectDrive(0.0 * speed, 1.0 * speed);
	else
		return;

	while (fabs(gyro.GetYaw()) <= fabs(angle)  && AutoTimer.Get().value() <= TimeOut)	//When the gyroscope gives a reading below/equal to 45
	{
		double diff;
		if((diff=(fabs(angle)-fabs(gyro.GetYaw()))/ANGTOLERANCE <= 1.0))
		{
			//Similar to previous autoturn Gyro
			if (angle* speed > 0.0) Tankdrive::DirectDrive(0.0, -1.0 * speed * diff);
			else Tankdrive::DirectDrive(-1.0 * speed * diff, 0.0);
		}
		Wait(0.001_s);
	    Wait(0.001_s);
	}
	Tankdrive::DirectDrive(0.0, 0.0);
}

void Tankdrive::AutoDriveGyroUS(float USrange, float speed, float Maxdistance) //Args are distance, speed
{
	for (int i = 0; i < 10; i++)
	 usonic.GetDistance();
	bool USGood=1;
	if(speed>1)speed=1;
	else if(speed<-1)speed=-1;
	AutoTimer.Reset(); AutoTimer.Start();
	ResetEncoders();
	    //Reset Wheel Encoders
	gyro.ResetAngle();
	Tankdrive::DirectDrive(speed, speed);		//Drives both motors at standard length

	if (usonic.GetDistance() < 15) USGood=0;
//Continues based on the range determined by the Usonic
	while(((((fabs(GetLEncoder()) + fabs(GetREncoder())) / 2) < Maxdistance)
			&& (usonic.GetDistance() > USrange  || !USGood)) && AutoTimer.Get().value()<= AUTOTIMEMAX.value())
	{
		Tankdrive::DirectDrive(speed*(1-AUTOGYROCONST*gyro.GetYaw()), speed*(1+AUTOGYROCONST*gyro.GetYaw()));
		usonic.GetDistance();
		Wait(.2_s);
	}
	Tankdrive::DirectDrive(0.0,0.0);
}

double Tankdrive::GetREncoder()
{
	return RFEncoder -> GetPosition()-rEncoderOffset;
}

double Tankdrive::GetLEncoder()
{
	return LFEncoder -> GetPosition()-lEncoderOffset;
}

void Tankdrive::ResetEncoders()
{
	rEncoderOffset = RFEncoder -> GetPosition();
	lEncoderOffset = LFEncoder -> GetPosition();
}

void Tankdrive::ResetGyro()
{
	gyro.ResetAngle();
}

double Tankdrive::GetAngle()
{
	return gyro.GetYaw();
}

void Tankdrive::GetUSSample()
{
	usonic.GetDistance();
}

double Tankdrive::GetUSRange()
{
	return usonic.GetDistance();
}
double Tankdrive::GetLidarRange()
{
	return lidar.GetDistance();
}


