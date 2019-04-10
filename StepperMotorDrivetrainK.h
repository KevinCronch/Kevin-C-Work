#ifndef STEPPERMOTORDRIVETRAIN_H
#define STEPPERMOTORDRIVETRAIN_H

#include <Arduino.h>
#include <String.h>

#define STEPS_PER_REVOLUTION 200
#define ANGLETHRESHOLD 0

/*
 * (Taken from Arduino Stepper.h)
 * Run through sequence to make a step
 * The sequence of control signals for 4 control wires is as follows:
 *
 * Step C0 C1 C2 C3
 *    1  1  0  1  0
 *    2  1  1  1  0
 *    3  0  1  1  0
 *    4  0  1  1  1
 *    5  0  1  0  1
 *    6  1  1  0  1
 *    7  1  0  0  1
 *    8  1  0  1  1
 */

class StepperMotorDrivetrain
{
	public:
		StepperMotorDrivetrain();
		
		//TODO: reorder the methods in cpp to match this order
		void operator=(const StepperMotorDrivetrain& drivetrain);
		
		//initialize motors
		void initFrontLeft(int in1, int in2, int in3, int in4);
		void initFrontRight(int in1, int in2, int in3, int in4);
		void initBackLeft(int in1, int in2, int in3, int in4);
		void initBackRight(int in1, int in2, int in3, int in4);
		
		//movement
		void step(int left, int right);
		void rest();
		void strafe(int forwardDirection, int sidewayDirection, unsigned int stepsActual);
		bool stepToAngle(float target, float current);
		bool steppe(int left, int right);
		//bool PIDdrive(float targetDistance, float targetAngle, float currentAngle); 
				
		//setters and resetters
		void setRPM(float speed);
		void resetStepCounter();
		
		//getters
		long getFrontLeftSteps();
		long getBackLeftSteps();
		long getFrontRightSteps();
		long getBackRightSteps();
		
		//helpers
		double calculateStepWait(int steps);
		int convertInchesToSteps(float inches);
		
	private:
		//Front left Motor
		int fLeftIN1, fLeftIN2, fLeftIN3, fLeftIN4;
		
		//Back left Motor 2
		int bLeftIN1, bLeftIN2, bLeftIN3, bLeftIN4;
		
		//Front right Motor
		int fRightIN1, fRightIN2, fRightIN3, fRightIN4;
		
		//Back right Motor
		int bRightIN1, bRightIN2, bRightIN3, bRightIN4;
		
		//RPM of all drive motors
		float rpm;
		
		//Step counters
		long frontLeftSteps;
		long backLeftSteps;
		long frontRightSteps;
		long backRightSteps;
		
		int frontLeftCounter, frontRightCounter, backLeftCounter, backRightCounter;
		
		//Stepping functions
		void singleStep(unsigned int stepWait);
		void singleStep_us(unsigned int stepWait);
		
		void sendStepSignalToFrontLeft(int stepID);
		void sendStepSignalToBackLeft(int stepID);
		void sendStepSignalToFrontRight(int stepID);
		void sendStepSignalToBackRight(int stepID);
		bool inRange(float variable, float constant, float range);
};

#endif