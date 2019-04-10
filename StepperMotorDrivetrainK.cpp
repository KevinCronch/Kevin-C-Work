#include "StepperMotorDrivetrain.h"
#include <String.h>

// KEVIN'S MODIFIED VERSION with HalfStep

//constructor
StepperMotorDrivetrain::StepperMotorDrivetrain()
{
	//sets steps and step counter to 0
	this->frontLeftSteps = 0;
	this->frontLeftCounter = 0;
	
	this->backLeftSteps = 0;
	this->backLeftCounter = 0;
	
	this->frontRightSteps = 0;
	this->frontRightCounter = 0;
	
	this->backRightSteps = 0;
	this->backRightCounter = 0;
	
	//Set this to a medium speed
	this->rpm = 25;
}

void StepperMotorDrivetrain::operator=(const StepperMotorDrivetrain& drivetrain)
{
	//sets private variables from each drivetrain equal to one other
	this->fLeftIN1 = drivetrain.fLeftIN1;
	this->fLeftIN2 = drivetrain.fLeftIN2;
	this->fLeftIN3 = drivetrain.fLeftIN3;
	this->fLeftIN4 = drivetrain.fLeftIN4;
	
	this->bLeftIN1 = drivetrain.bLeftIN1;
	this->bLeftIN2 = drivetrain.bLeftIN2;
	this->bLeftIN3 = drivetrain.bLeftIN3;
	this->bLeftIN4 = drivetrain.bLeftIN4;
	
	this->fRightIN1 = drivetrain.fRightIN1;
	this->fRightIN2 = drivetrain.fRightIN2;
	this->fRightIN3 = drivetrain.fRightIN3;
	this->fRightIN4 = drivetrain.fRightIN4;
	
	this->bRightIN1 = drivetrain.bRightIN1;
	this->bRightIN2 = drivetrain.bRightIN2;
	this->bRightIN3 = drivetrain.bRightIN3;
	this->bRightIN4 = drivetrain.bRightIN4;
	
	this->rpm = drivetrain.rpm;
	
	this->frontLeftSteps = drivetrain.frontLeftSteps;
	this->backLeftSteps = drivetrain.backLeftSteps;
	this->frontRightSteps = drivetrain.frontRightSteps;
	this->backRightSteps = drivetrain.backRightSteps;

	this->frontLeftCounter = drivetrain.frontLeftCounter;
	this->backLeftCounter = drivetrain.backLeftCounter;
	this->frontRightCounter = drivetrain.frontRightCounter;
	this->backRightCounter = drivetrain.backRightCounter;
}

//sets pins for front left motors
void StepperMotorDrivetrain::initFrontLeft(int in1, int in2, int in3, int in4)
{
	this->fLeftIN1 = in1;
	this->fLeftIN2 = in2;
	this->fLeftIN3 = in3;
	this->fLeftIN4 = in4;
	
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
}

//sets pins for back left motors
void StepperMotorDrivetrain::initBackLeft(int in1, int in2, int in3, int in4)
{
	this->bLeftIN1 = in1;
	this->bLeftIN2 = in2;
	this->bLeftIN3 = in3;
	this->bLeftIN4 = in4;
	
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
}

//sets pins for front right motors
void StepperMotorDrivetrain::initFrontRight(int in1, int in2, int in3, int in4)
{
	this->fRightIN1 = in1;
	this->fRightIN2 = in2;
	this->fRightIN3 = in3;
	this->fRightIN4 = in4;
	
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
}

//sets pins for back right motors
void StepperMotorDrivetrain::initBackRight(int in1, int in2, int in3, int in4)
{
	this->bRightIN1 = in1;
	this->bRightIN2 = in2;
	this->bRightIN3 = in3;
	this->bRightIN4 = in4;
	
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
}

//sets speed of drivetrain
void StepperMotorDrivetrain::setRPM(float speed)
{
	this->rpm = abs(speed);
}

//moves the drivetrain
void StepperMotorDrivetrain::step(int left, int right)
{
	bool millisecond_interval = false;
	
	//We basically force left and right to be equal here, because they should be.
	//NO CURVE TURNS ALLOWED (Down with tank steer)
	int steps = min(abs(left), abs(right));
	
	
	int leftDirection = left < 0 ? -1 : 1;
	int rightDirection = right < 0 ? -1 : 1;
	
	//if (left = 0) leftDirection = 0;
	//if (right = 0) rightDirection = 0;

	
	//Determine how many microseconds we want to wait, and convert to an integer
	//double totalTime = (static_cast<double>(steps) / STEPS_PER_REVOLUTION) / this->rpm * 60.0 * 1000.0 * 1000.0;
	double T = calculateStepWait(steps);
	
	//Convert to milliseconds if delay would be greater than 5,000 us.
	if(T > 5000)
	{
		T /= 1000;
		millisecond_interval = true;
	}
	
	unsigned long stepWait = static_cast<int>(T);
	
	for(int i = 0; i < steps; ++i)
	{
		this->frontLeftSteps += leftDirection;
		this->backLeftSteps += leftDirection;
		this->frontRightSteps += rightDirection;
		this->backRightSteps += rightDirection;
		
		this->frontLeftCounter += leftDirection;
		this->backLeftCounter += leftDirection;
		this->frontRightCounter += rightDirection;
		this->backRightCounter += rightDirection;
		
		//Constrain the counters to the step boundaries
		//Left
		this->frontLeftCounter = this->frontLeftCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->frontLeftCounter;
		this->frontLeftCounter = this->frontLeftCounter >= STEPS_PER_REVOLUTION ? 0 : this->frontLeftCounter;
		
		this->backLeftCounter = this->backLeftCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->backLeftCounter;
		this->backLeftCounter = this->backLeftCounter >= STEPS_PER_REVOLUTION ? 0 : this->backLeftCounter;
		//Right
		this->frontRightCounter = this->frontRightCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->frontRightCounter;
		this->frontRightCounter = this->frontRightCounter >= STEPS_PER_REVOLUTION ? 0 : this->frontRightCounter;
		
		this->backRightCounter = this->backRightCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->backRightCounter;
		this->backRightCounter = this->backRightCounter >= STEPS_PER_REVOLUTION ? 0 : this->backRightCounter;

		if(millisecond_interval)
		{
			singleStep(stepWait);
		}
		else
		{
			singleStep_us(stepWait);
		}
	}
}

// A really cool function for allowing stepping with a single side for more "accurate" turning
// Best to only use values: 	-1, 0, 1
bool StepperMotorDrivetrain::steppe(int left, int right)
{
	bool millisecond_interval = false;
	
	//We basically force left and right to be equal here, because they should be.
	//NO CURVE TURNS ALLOWED (Down with tank steer)
	int steps = max(abs(left), abs(right));
	
	//int leftDirection = left < 0 ? -1 : 1;
	//int rightDirection = right < 0 ? -1 : 1;
	
	int leftDirection = constrain(left, -1, 1);
	int rightDirection = constrain(right, -1, 1);
	
	//Determine how many microseconds we want to wait, and convert to an integer
	//double totalTime = (static_cast<double>(steps) / STEPS_PER_REVOLUTION) / this->rpm * 60.0 * 1000.0 * 1000.0;
	double T = calculateStepWait(steps);
	
	//Convert to milliseconds if delay would be greater than 5,000 us.
	if(T > 5000)
	{
		T /= 1000;
		millisecond_interval = true;
	}
	
	unsigned long stepWait = static_cast<int>(T);
	Serial.print("stewWait is "); Serial.print(stepWait); Serial.println(" ms");
	for(int i = 0; i < steps; ++i)
	{
		this->frontLeftSteps += leftDirection;
		this->backLeftSteps += leftDirection;
		this->frontRightSteps += rightDirection;
		this->backRightSteps += rightDirection;
		
		this->frontLeftCounter += leftDirection;
		this->backLeftCounter += leftDirection;
		this->frontRightCounter += rightDirection;
		this->backRightCounter += rightDirection;
		
		//Constrain the counters to the step boundaries
		//Left
		this->frontLeftCounter = this->frontLeftCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->frontLeftCounter;
		this->frontLeftCounter = this->frontLeftCounter >= STEPS_PER_REVOLUTION ? 0 : this->frontLeftCounter;
		
		this->backLeftCounter = this->backLeftCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->backLeftCounter;
		this->backLeftCounter = this->backLeftCounter >= STEPS_PER_REVOLUTION ? 0 : this->backLeftCounter;
		//Right
		this->frontRightCounter = this->frontRightCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->frontRightCounter;
		this->frontRightCounter = this->frontRightCounter >= STEPS_PER_REVOLUTION ? 0 : this->frontRightCounter;
		
		this->backRightCounter = this->backRightCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->backRightCounter;
		this->backRightCounter = this->backRightCounter >= STEPS_PER_REVOLUTION ? 0 : this->backRightCounter;

		if(millisecond_interval)
		{
			singleStep(stepWait);
		}
		else
		{
			singleStep_us(stepWait);
		}
	}
}

bool StepperMotorDrivetrain::stepToAngle(float target, float current)
{
	if (target > 0)
	{
		if (inRange(target, current, ANGLETHRESHOLD))
		{
			step(0, 0);
			return true;
		}
		else
		{
			if (current < target) 
			{
				step(-1, 1);
				return false;
			}
			else
			{
				step(1, -1);
				return false;
			}
		}

	}
	else //(target < 0)
	{
		if (inRange(target, current, ANGLETHRESHOLD))
		{
			step(0, 0);
			return true;
		}
		else
		{
			if (current > target)
			{
				step(1, -1);
				return false;
			}
			else
			{
				step(-1, 1);
				return false;
			}
		}
	}
}

//calculate the amount of time the amount of steps will take
// Returns time in microseconds
double StepperMotorDrivetrain::calculateStepWait(int steps) {

	//Determine how many microseconds we want to wait, and convert to an integer	
	double totalTime = (static_cast<double>(steps) / STEPS_PER_REVOLUTION) / this->rpm * 60.0 * 1000.0 * 1000.0;
	double T = (totalTime / steps) / 2;
	
	return T;
}

//resets the step counters to 0
void StepperMotorDrivetrain::resetStepCounter()
{
	frontLeftSteps = 0;
	frontRightSteps = 0;
	backLeftSteps = 0;
	backRightSteps = 0;
}

//getter method for left motor steps
long StepperMotorDrivetrain::getFrontLeftSteps()
{
	return this->frontLeftCounter;
}

//getter method for left motor steps
long StepperMotorDrivetrain::getBackLeftSteps()
{
	return this->backLeftCounter;
}

//getter method for right motor steps
long StepperMotorDrivetrain::getFrontRightSteps()
{
	return this->frontRightCounter;
}

//getter method for right motor steps
long StepperMotorDrivetrain::getBackRightSteps()
{
	return this->backRightCounter;
}

//helper method to convert inches to steps
int StepperMotorDrivetrain::convertInchesToSteps(float inches)
{
	//number of steps / circumference of wheel = ratio
	return static_cast<int>((STEPS_PER_REVOLUTION/(4.0*3.141592653589))*inches * 1.04);
}

//Private Functions
void StepperMotorDrivetrain::singleStep(unsigned int stepWait)
{
	sendStepSignalToFrontLeft(frontLeftCounter % 8);
	sendStepSignalToFrontRight(frontRightCounter % 8);
	sendStepSignalToBackLeft(backLeftCounter % 8);
	sendStepSignalToBackRight(backRightCounter % 8);
	
	delay(stepWait); // Wait
}

//same as above but with a different delay function
void StepperMotorDrivetrain::singleStep_us(unsigned int stepWait)
{

	sendStepSignalToFrontLeft(frontLeftCounter % 8);
	sendStepSignalToFrontRight(frontRightCounter % 8);
	sendStepSignalToBackLeft(backLeftCounter % 8);
	sendStepSignalToBackRight(backRightCounter % 8);
	
    delayMicroseconds(stepWait); // Wait
}

void StepperMotorDrivetrain::sendStepSignalToFrontLeft(int stepID)
{
	switch (stepID) {
		case 0:  // 1010
			digitalWrite(fLeftIN1, HIGH);
			digitalWrite(fLeftIN2, LOW);
			digitalWrite(fLeftIN3, HIGH);
			digitalWrite(fLeftIN4, LOW);
			break;
		case 1:  // 1110
			digitalWrite(fLeftIN1, HIGH);
			digitalWrite(fLeftIN2, HIGH);
			digitalWrite(fLeftIN3, HIGH);
			digitalWrite(fLeftIN4, LOW);
			break;
		case 2:  // 0110
			digitalWrite(fLeftIN1, LOW);
			digitalWrite(fLeftIN2, HIGH);
			digitalWrite(fLeftIN3, HIGH);
			digitalWrite(fLeftIN4, LOW);
			break;
		case 3:  // 0111
			digitalWrite(fLeftIN1, LOW);
			digitalWrite(fLeftIN2, HIGH);
			digitalWrite(fLeftIN3, HIGH);
			digitalWrite(fLeftIN4, HIGH);
			break;
		case 4:  //0101
			digitalWrite(fLeftIN1, LOW);
			digitalWrite(fLeftIN2, HIGH);
			digitalWrite(fLeftIN3, LOW);
			digitalWrite(fLeftIN4, HIGH);
			break;
		case 5:  //1101
			digitalWrite(fLeftIN1, HIGH);
			digitalWrite(fLeftIN2, HIGH);
			digitalWrite(fLeftIN3, LOW);
			digitalWrite(fLeftIN4, HIGH);
			break;
		case 6:  //1001
			digitalWrite(fLeftIN1, HIGH);
			digitalWrite(fLeftIN2, LOW);
			digitalWrite(fLeftIN3, LOW);
			digitalWrite(fLeftIN4, HIGH);
			break;
		
		case 7:  //1011
			digitalWrite(fLeftIN1, HIGH);
			digitalWrite(fLeftIN2, LOW);
			digitalWrite(fLeftIN3, HIGH);
			digitalWrite(fLeftIN4, HIGH);
			break;
    }
}

void StepperMotorDrivetrain::sendStepSignalToBackLeft(int stepID)
{
	switch (stepID) {
		case 0:  // 1010
			digitalWrite(bLeftIN1, HIGH);
			digitalWrite(bLeftIN2, LOW);
			digitalWrite(bLeftIN3, HIGH);
			digitalWrite(bLeftIN4, LOW);
			break;
		case 1:  // 1110
			digitalWrite(bLeftIN1, HIGH);
			digitalWrite(bLeftIN2, HIGH);
			digitalWrite(bLeftIN3, HIGH);
			digitalWrite(bLeftIN4, LOW);
			break;
		case 2:  // 0110
			digitalWrite(bLeftIN1, LOW);
			digitalWrite(bLeftIN2, HIGH);
			digitalWrite(bLeftIN3, HIGH);
			digitalWrite(bLeftIN4, LOW);
			break;
		case 3:  // 0111
			digitalWrite(bLeftIN1, LOW);
			digitalWrite(bLeftIN2, HIGH);
			digitalWrite(bLeftIN3, HIGH);
			digitalWrite(bLeftIN4, HIGH);
			break;
		case 4:  //0101
			digitalWrite(bLeftIN1, LOW);
			digitalWrite(bLeftIN2, HIGH);
			digitalWrite(bLeftIN3, LOW);
			digitalWrite(bLeftIN4, HIGH);
			break;
		case 5:  //1101
			digitalWrite(bLeftIN1, HIGH);
			digitalWrite(bLeftIN2, HIGH);
			digitalWrite(bLeftIN3, LOW);
			digitalWrite(bLeftIN4, HIGH);
			break;
		case 6:  //1001
			digitalWrite(bLeftIN1, HIGH);
			digitalWrite(bLeftIN2, LOW);
			digitalWrite(bLeftIN3, LOW);
			digitalWrite(bLeftIN4, HIGH);
			break;
		case 7:  //1011
			digitalWrite(bLeftIN1, HIGH);
			digitalWrite(bLeftIN2, LOW);
			digitalWrite(bLeftIN3, HIGH);
			digitalWrite(bLeftIN4, HIGH);
			break;
    }
}

void StepperMotorDrivetrain::sendStepSignalToFrontRight(int stepID)
{
	switch (stepID) {
		case 0:  // 1010
			digitalWrite(fRightIN1, HIGH);
			digitalWrite(fRightIN2, LOW);
			digitalWrite(fRightIN3, HIGH);
			digitalWrite(fRightIN4, LOW);
			break;
		case 1:  // 1110
			digitalWrite(fRightIN1, HIGH);
			digitalWrite(fRightIN2, HIGH);
			digitalWrite(fRightIN3, HIGH);
			digitalWrite(fRightIN4, LOW);
			break;
		case 2:  // 0110
			digitalWrite(fRightIN1, LOW);
			digitalWrite(fRightIN2, HIGH);
			digitalWrite(fRightIN3, HIGH);
			digitalWrite(fRightIN4, LOW);
			break;
		case 3:  // 0111
			digitalWrite(fRightIN1, LOW);
			digitalWrite(fRightIN2, HIGH);
			digitalWrite(fRightIN3, HIGH);
			digitalWrite(fRightIN4, HIGH);
			break;
		case 4:  //0101
			digitalWrite(fRightIN1, LOW);
			digitalWrite(fRightIN2, HIGH);
			digitalWrite(fRightIN3, LOW);
			digitalWrite(fRightIN4, HIGH);
			break;
		case 5:  //1101
			digitalWrite(fRightIN1, HIGH);
			digitalWrite(fRightIN2, HIGH);
			digitalWrite(fRightIN3, LOW);
			digitalWrite(fRightIN4, HIGH);
			break;
		case 6:  //1001
			digitalWrite(fRightIN1, HIGH);
			digitalWrite(fRightIN2, LOW);
			digitalWrite(fRightIN3, LOW);
			digitalWrite(fRightIN4, HIGH);
			break;
		case 7:  //1011
			digitalWrite(fRightIN1, HIGH);
			digitalWrite(fRightIN2, LOW);
			digitalWrite(fRightIN3, HIGH);
			digitalWrite(fRightIN4, HIGH);
			break;
    }
}

void StepperMotorDrivetrain::sendStepSignalToBackRight(int stepID)
{
	switch (stepID) {
		case 0:  // 1010
			digitalWrite(bRightIN1, HIGH);
			digitalWrite(bRightIN2, LOW);
			digitalWrite(bRightIN3, HIGH);
			digitalWrite(bRightIN4, LOW);
			break;
		case 1:  // 1110
			digitalWrite(bRightIN1, HIGH);
			digitalWrite(bRightIN2, HIGH);
			digitalWrite(bRightIN3, HIGH);
			digitalWrite(bRightIN4, LOW);
			break;
		case 2:  // 0110
			digitalWrite(bRightIN1, LOW);
			digitalWrite(bRightIN2, HIGH);
			digitalWrite(bRightIN3, HIGH);
			digitalWrite(bRightIN4, LOW);
			break;
		case 3:  // 0111
			digitalWrite(bRightIN1, LOW);
			digitalWrite(bRightIN2, HIGH);
			digitalWrite(bRightIN3, HIGH);
			digitalWrite(bRightIN4, HIGH);
			break;
		case 4:  //0101
			digitalWrite(bRightIN1, LOW);
			digitalWrite(bRightIN2, HIGH);
			digitalWrite(bRightIN3, LOW);
			digitalWrite(bRightIN4, HIGH);
			break;
		case 5:  //1101
			digitalWrite(bRightIN1, HIGH);
			digitalWrite(bRightIN2, HIGH);
			digitalWrite(bRightIN3, LOW);
			digitalWrite(bRightIN4, HIGH);
			break;
		case 6:  //1001
			digitalWrite(bRightIN1, HIGH);
			digitalWrite(bRightIN2, LOW);
			digitalWrite(bRightIN3, LOW);
			digitalWrite(bRightIN4, HIGH);
			break;
		case 7:  //1011
			digitalWrite(bRightIN1, HIGH);
			digitalWrite(bRightIN2, LOW);
			digitalWrite(bRightIN3, HIGH);
			digitalWrite(bRightIN4, HIGH);
			break;
    }
}

void StepperMotorDrivetrain::rest()
{
			digitalWrite(fRightIN1, LOW);
			digitalWrite(fRightIN2, LOW);
			digitalWrite(fRightIN3, LOW);
			digitalWrite(fRightIN4, LOW);
			
			digitalWrite(bRightIN1, LOW);
			digitalWrite(bRightIN2, LOW);
			digitalWrite(bRightIN3, LOW);
			digitalWrite(bRightIN4, LOW);
			
			digitalWrite(fLeftIN1, LOW);
			digitalWrite(fLeftIN2, LOW);
			digitalWrite(fLeftIN3, LOW);
			digitalWrite(fLeftIN4, LOW);
			
			digitalWrite(bLeftIN1, LOW);
			digitalWrite(bLeftIN2, LOW);
			digitalWrite(bLeftIN3, LOW);
			digitalWrite(bLeftIN4, LOW);
}

/* Pass ints for direction
 * 
 * forwardDirection: 0-->None	1-->Forward		-1-->Backwards
 * sidewayDirection: 0-->None	1-->Right 		-1-->Left
 *
 * Use drivetrain.convertInchesToSteps(inches) in your .ino for steps
 */
void StepperMotorDrivetrain::strafe(int forwardDirection, int sidewayDirection, unsigned int stepsActual)
{
	
	//We basically force left and right to be equal here, because they should be.
	//NO CURVE TURNS ALLOWED (Down with tank steer)
	int steps = abs(stepsActual);
			
	//TODO: delete Step and implement it here (not priority)	
	// Straight Forward
	if(forwardDirection == 1 && sidewayDirection == 0)
	{
		Serial.println("Forward Strafe");
		step(steps, steps);
	}
	// Straight Backwards
	else if(forwardDirection == -1 && sidewayDirection == 0)
	{
		Serial.println("Backward Strafe");
		step(-steps, -steps);
	}

	// Non basic movement (strafing)
	else 
	{
		bool millisecond_interval = false;
		
		//Calculating amount of time for function
		double T = calculateStepWait(steps);
		
		//Convert to milliseconds if delay would be greater than 5,000 us.
		if(T > 5000)
		{
			T /= 1000;
			millisecond_interval = true;
		}
		
		unsigned long stepWait = static_cast<int>(T);
		Serial.print("stewWait is "); Serial.print(stepWait); Serial.println(" ms");
		
		// Left Strafe
		if(forwardDirection == 0 && sidewayDirection == -1)
		{
			Serial.println("Left Strafe");
			
			for(int i = 0; i < steps; i++)
			{
				//right motors turn outward, left motors turn inward
				backRightCounter -= 1;
				frontRightCounter += 1;
				backLeftCounter += 1;
				frontLeftCounter -= 1;
				
				//Constrain the counters to the step boundaries
				//Left
				this->frontLeftCounter = this->frontLeftCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->frontLeftCounter;
				this->frontLeftCounter = this->frontLeftCounter >= STEPS_PER_REVOLUTION ? 0 : this->frontLeftCounter;
				
				this->backLeftCounter = this->backLeftCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->backLeftCounter;
				this->backLeftCounter = this->backLeftCounter >= STEPS_PER_REVOLUTION ? 0 : this->backLeftCounter;
				
				//Right
				this->frontRightCounter = this->frontRightCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->frontRightCounter;
				this->frontRightCounter = this->frontRightCounter >= STEPS_PER_REVOLUTION ? 0 : this->frontRightCounter;
				
				this->backRightCounter = this->backRightCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->backRightCounter;
				this->backRightCounter = this->backRightCounter >= STEPS_PER_REVOLUTION ? 0 : this->backRightCounter;
				
				if(millisecond_interval)
				{
					singleStep(stepWait);
				}
				else
				{
					singleStep_us(stepWait);
				}
			}
		}
		// Right Strafe
		else if(forwardDirection == 0 && sidewayDirection == 1)
		{
			Serial.println("Right Strafe");
			
			for(int i = 0; i < steps; i++)
			{
				//right motors turn inward, left motors turn outward				
				backRightCounter += 1;
				frontRightCounter -= 1;
				backLeftCounter -= 1;
				frontLeftCounter += 1;
				
				//Constrain the counters to the step boundaries
				//Left
				this->frontLeftCounter = this->frontLeftCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->frontLeftCounter;
				this->frontLeftCounter = this->frontLeftCounter >= STEPS_PER_REVOLUTION ? 0 : this->frontLeftCounter;
				
				this->backLeftCounter = this->backLeftCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->backLeftCounter;
				this->backLeftCounter = this->backLeftCounter >= STEPS_PER_REVOLUTION ? 0 : this->backLeftCounter;
				
				//Right
				this->frontRightCounter = this->frontRightCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->frontRightCounter;
				this->frontRightCounter = this->frontRightCounter >= STEPS_PER_REVOLUTION ? 0 : this->frontRightCounter;
				
				this->backRightCounter = this->backRightCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->backRightCounter;
				this->backRightCounter = this->backRightCounter >= STEPS_PER_REVOLUTION ? 0 : this->backRightCounter;
				
				if(millisecond_interval)
				{
					singleStep(stepWait);
				}
				else
				{
					singleStep_us(stepWait);
				}
			}
		}
		// Forward Left Strafe
		else if(forwardDirection == 1 && sidewayDirection == -1)
		{
			Serial.println("Forward Left Strafe");
			
			for(int i = 0; i < steps; i++)
			{				
				//frontright turns forward, backright turns forward
				frontLeftCounter += 1;
				backRightCounter += 1;
				
				//Constrain the counters to the step boundaries
				//Left
				this->frontLeftCounter = this->frontLeftCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->frontLeftCounter;
				this->frontLeftCounter = this->frontLeftCounter >= STEPS_PER_REVOLUTION ? 0 : this->frontLeftCounter;
				
				//Right
				this->backRightCounter = this->backRightCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->backRightCounter;
				this->backRightCounter = this->backRightCounter >= STEPS_PER_REVOLUTION ? 0 : this->backRightCounter;
				
				if(millisecond_interval)
				{
					singleStep(stepWait);
				}
				else
				{
					singleStep_us(stepWait);
				}
			}
		}
		// Backward Left Strafe 
		else if(forwardDirection == -1 && sidewayDirection == -1)
		{
			Serial.println("Backward Left Strafe");
			
			for(int i = 0; i < steps; i++)
			{
				//frontright turns backward, backleft turns backward
				frontRightCounter -= 1;
				backLeftCounter -= 1;
				
				//Constrain the counters to the step boundaries
				//Right
				this->frontRightCounter = this->frontRightCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->frontRightCounter;
				this->frontRightCounter = this->frontRightCounter >= STEPS_PER_REVOLUTION ? 0 : this->frontRightCounter;

				
				//Left
				this->backLeftCounter = this->backLeftCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->backLeftCounter;
				this->backLeftCounter = this->backLeftCounter >= STEPS_PER_REVOLUTION ? 0 : this->backLeftCounter;

				
				if(millisecond_interval)
				{
					singleStep(stepWait);
				}
				else
				{
					singleStep_us(stepWait);
				}
			}
		}
		// Forward Right Strafe
		else if(forwardDirection == 1 && sidewayDirection == 1)
		{
			Serial.println("Forward Right Strafe");
			
			for(int i = 0; i < steps; i++)
			{
				//backright turns forward, frontleft turns forward
				backLeftCounter += 1;
				frontRightCounter += 1;
				
				//Constrain the counters to the step boundaries
				//Right
				this->frontRightCounter = this->frontRightCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->frontRightCounter;
				this->frontRightCounter = this->frontRightCounter >= STEPS_PER_REVOLUTION ? 0 : this->frontRightCounter;

				//Left
				this->backLeftCounter = this->backLeftCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->backLeftCounter;
				this->backLeftCounter = this->backLeftCounter >= STEPS_PER_REVOLUTION ? 0 : this->backLeftCounter;
				
				if(millisecond_interval)
				{
					singleStep(stepWait);
				}
				else
				{
					singleStep_us(stepWait);
				}
			}
		}
		// Backward Right Strafe
		else if(forwardDirection == -1 && sidewayDirection == 1)
		{
			Serial.println("Backward Right Strafe");
			
			for(int i = 0; i < steps; i++)
			{				
				//backright turns backward, frontleft turns backward			
				backRightCounter -= 1;
				frontLeftCounter -= 1;
				
				//Constrain the counters to the step boundaries
				//Right
				this->backRightCounter = this->backRightCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->backRightCounter;
				this->backRightCounter = this->backRightCounter >= STEPS_PER_REVOLUTION ? 0 : this->backRightCounter;
				
				//Left
				this->frontLeftCounter = this->frontLeftCounter < 0 ? STEPS_PER_REVOLUTION - 1 : this->frontLeftCounter;
				this->frontLeftCounter = this->frontLeftCounter >= STEPS_PER_REVOLUTION ? 0 : this->frontLeftCounter;
				
				if(millisecond_interval)
				{
					singleStep(stepWait);
				}
				else
				{
					singleStep_us(stepWait);
				}
			}
		}
	}
}

/*
boolPIDdrive(float targetDistance, float targetAngle, float currentAngle)
{;
	float currentSteps = 0;
	
	while(currentSteps < targetDistance)
	{
		strafe(1,0,);
	}
}
*/

bool StepperMotorDrivetrain::inRange(float variable, float constant, float range)
{
	if (abs(variable - constant) < range )
	{
		return true;
	}
	else 
	{
		return false;
	}		
}

