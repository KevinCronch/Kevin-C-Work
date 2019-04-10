#ifndef STARTUPDRIVE_H
#define STARTUPDRIVE_H

#include "StepperMotorDrivetrain.h"
#include "Globals.h"
#include "StartUpDrive.h"
#include "StartUpArm.h"
#include "MPU6050.h"
#include "Arm.h"
#include <Servo.h>

StepperMotorDrivetrain drivetrain;

MPU6050 mpu;

const int MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void checkSettings()
{
	Serial.println();

	Serial.print(" * Sleep Mode:        ");
	Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

	Serial.print(" * Clock Source:      ");
	switch(mpu.getClockSource())
	{
		case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
		case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
		case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
		case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
		case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
		case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
		case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
	}

	Serial.print(" * Gyroscope:         ");
	switch(mpu.getScale())
	{
		case MPU6050_SCALE_2000DPS:        Serial.println("2000 dps"); break;
		case MPU6050_SCALE_1000DPS:        Serial.println("1000 dps"); break;
		case MPU6050_SCALE_500DPS:         Serial.println("500 dps"); break;
		case MPU6050_SCALE_250DPS:         Serial.println("250 dps"); break;
	}

	Serial.print(" * Gyroscope offsets: ");
	Serial.print(mpu.getGyroOffsetX());
	Serial.print(" / ");
	Serial.print(mpu.getGyroOffsetY());
	Serial.print(" / ");
	Serial.println(mpu.getGyroOffsetZ());

	Serial.println();
}

void updateGyro()
{
	/*
	float timeStep = 0.01;

	// Read normalized values
	Vector norm = mpu.readNormalizeGyro();

	// Calculate Pitch, Roll and Yaw
	pitch = pitch + norm.YAxis * timeStep;
	roll = roll + norm.XAxis * timeStep;
	yaw = yaw + norm.ZAxis * timeStep;

	// Output raw
	Serial.print(" Pitch = ");
	Serial.print(pitch);
	Serial.print(" Roll = ");
	Serial.print(roll);
	Serial.print(" Yaw = ");
	Serial.println(yaw);

	GYRO_PITCH = pitch;
	GYRO_YAW = yaw;
	GYRO_ROLL = -roll;
	*/
}

void driveSetup()
{
	/*
	DigitalDevice lowSwitch(LOW_SWITCH, INPUT);
	DigitalDevice highSwitch(HIGH_SWITCH, INPUT);

	fistServo.attach(FIST_PIN1);
	wristServo.attach(WRIST_PIN1);

	lowSwitch.pullUp();
	highSwitch.pullUp();

	rackAndPinion.begin(ARM_MOT_PIN1, ARM_MOT_PIN2);

	arm.begin(fistServo, wristServo, rackAndPinion, lowSwitch, highSwitch);
	*/
	Serial.begin(9600);
	Serial.println("Initialize MPU6050");
	mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
	/*while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
	{
		Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
		delay(500);
	}*/

	

	drivetrain.initFrontLeft(FRONT_LEFT_MOT_PIN1, FRONT_LEFT_MOT_PIN2, FRONT_LEFT_MOT_PIN3, FRONT_LEFT_MOT_PIN4);
	drivetrain.initBackLeft(BACK_LEFT_MOT_PIN1, BACK_LEFT_MOT_PIN2, BACK_LEFT_MOT_PIN3, BACK_LEFT_MOT_PIN4);
	drivetrain.initFrontRight(FRONT_RIGHT_MOT_PIN1, FRONT_RIGHT_MOT_PIN2, FRONT_RIGHT_MOT_PIN3, FRONT_RIGHT_MOT_PIN4);
	drivetrain.initBackRight(BACK_RIGHT_MOT_PIN1, BACK_RIGHT_MOT_PIN2, BACK_RIGHT_MOT_PIN3, BACK_RIGHT_MOT_PIN4);

	//attachInterrupt(digitalPinToInterrupt(2), updateGyro, CHANGE);

	// If you want, you can set gyroscope offsets
	// mpu.setGyroOffsetX(155);
	// mpu.setGyroOffsetY(15);
	// mpu.setGyroOffsetZ(15);

	// Calibrate gyroscope. The calibration must be at rest.
	// If you don't want calibrate, comment this line.
	
	//mpu.calibrateGyro();


	// Set threshold sensitivity. Default 3.
	// If you don't want use threshold, comment this line or set 0.
	//mpu.setThreshold(3);


	// Check settings
	//checkSettings();

}


void smartDrive(int forward, int sideways, int targetDistance, int angle)
{
	//unsigned long timer = 0;
	float iTime = micros();
	float fTime = micros();
	float timeStep = 0;
	float roll = 0;
	
	int currSteps = 0;

	int _forward = 0;

	int _angle = 0;




	if (forward > 127)
	{
		_forward = -1;
	}
	else if (forward > 0)
	{
		_forward = 1;
	}

	int _sideways = 0;

	if (sideways > 127)
	{
		_sideways = -1;
	}
	else if (sideways > 0)
	{
		_sideways = 1;
	}

	if (angle > 127)
	{
		_angle = (256 - angle) * -1;
		_sideways = 0;
		_forward = 0;
	}
	else if (angle > 0)
	{
		_angle = angle;
		_sideways = 0;
		_forward = 0;
	}
	if (_angle == 0)
	{
		while(currSteps < targetDistance)
		{
			iTime = fTime;
			fTime = micros();
			timeStep = fTime - iTime;
			//Vector norm = mpu.readNormalizeGyro();
			//roll = roll + norm.XAxis * timeStep/1000000;
			//GYRO_ROLL = -roll;
			
		    //Serial.println(GYRO_ROLL);
/*
			if(norm.XAxis < .5)
			{
				norm.XAxis = 0;
			}
*/

			/**  CHANGED to adjust every 50 steps aka 1 inch roughly. May change threshold.
			if (currSteps < 50)
			{
				drivetrain.strafe(_forward, _sideways, 1);
				currSteps++;
			}
			*/
			if(abs(GYRO_ROLL) > GYRO_THRESHOLD && currSteps % 20 == 0 && currSteps != 0) // makes sure no weird correction at beginning
			{
				if(GYRO_ROLL > 0)
				{
					drivetrain.steppe(0,1);
				}
				else //if (GYRO_ROLL < 0)
				{
					drivetrain.steppe(1,0);
				}
			}
			else
			{
				drivetrain.strafe(_forward, _sideways, 1);
				currSteps++;
			}

		}
	}
	else
	{
		//timer = millis();

		//while (abs(GYRO_ROLL - _angle) > GYRO_THRESHOLD)
		while (abs(GYRO_ROLL) < abs(_angle) && abs(GYRO_ROLL - _angle) > GYRO_THRESHOLD)
		{
			break; //---------------------------------------------------delete
			//drivetrain.steppe(0, 0);
			//delay(10);
            iTime = fTime;
			fTime = micros();
			timeStep = fTime - iTime;
			//Vector norm = mpu.readNormalizeGyro();
			//roll = roll + norm.XAxis * timeStep/1000000;
			//GYRO_ROLL = -roll;

			//Serial.println(GYRO_ROLL);

			if (_angle > 0)
			{
				drivetrain.steppe(1, -1);
				//delay(10);
			}
			else if (_angle < 0)
			{
				drivetrain.steppe(-1, 1);
				//delay(10);
			}

			//delay((timeStep*1000) - (millis() - timer));
		}
		//delay((timeStep*1000) - (millis() - timer));
	}

	GYRO_ROLL = 0;

	//Serial.println("DONE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

}

#endif
