/*
Varun Chauhan, Ryan Bernstein, Suyu Chen, Jerry Chen
Version 1.0
Assumptions: User will follow instructions given during startup, all corners in room are 90 degrees, tape boundaries are always straight lines, 
	     obstacles allow for movement between

Description: Main code for cleaning robot. On startup, user will be asked to set tape colour, number of edges, and duration. After that, robot will 
	     clean the perimeter of the room based on inputted number of edges. After cleaning edges, robot will used a weighted random turn navigation
	     algorithm to clean at least 90% of the room within 5 minutes.
*/

// variables for the motors
tMotor motorLeft = motorA, motorRight = motorD, motorSpray, motorDrum;

// variables for standard speeds for movement and turning
const int fwdSpeed = 50, turnSpeed = 20;

// variable for wheel radius
const float RADIUS = 2.75;

// configures all sensors
void configureAllSensors()
{
	SensorType[S1] = sensorEV3_Touch;
	SensorType[S2] = sensorEV3_Ultrasonic;
	SensorType[S3] = sensorEV3_Color;
	wait1Msec(50);
	SensorMode[S3] = modeEV3Color_Color;
	wait1Msec(50);

	SensorType[S4] = sensorEV3_Gyro;
	wait1Msec(50);
	SensorMode[S4] = modeEV3Gyro_Calibration;
	wait1Msec(100);
	SensorMode[S4] = modeEV3Gyro_RateAndAngle;
	wait1Msec(50);
}

// drive robot at specified power/direction
void drive(int mPower)
{
	motor[motorLeft] = motor[motorD] = mPower;
}

// drive robot a specified distance
void driveDistance(int distance, int mPower)
{
	nMotorEncoder[motorLeft]=0;
	const float CM_TO_DEG = 180/(RADIUS*PI);
	
	if (distance>0)	drive(mPower);
	else	drive(-1 * mPower);

	while (abs(nMotorEncoder[motorLeft])<abs(distance*CM_TO_DEG))
	{}

	drive(0);
}

// rotate robot a specified angle
void rotateRobot(int angle, int mPower)
{
	resetGyro(S4);

	if (angle>0)
	{
		motor[motorLeft] = mPower;
		motor[motorRight] = -1 * mPower;
	}
	else
	{
		motor[motorRight] = mPower;
		motor[motorLeft] = -1 * mPower;
	}

	while (abs(getGyroDegrees(S4))<abs(angle))
	{}

	drive(0);
}

// drives robot along an edge and rotates once a corner is detected
void sweepEdge(int edges, int tapeColour)
{
	for(int counter = 0; counter < edges; counter++)
	{
		drive(fwdSpeed);// drives forward until is detect a corner (of any kind)
		while((SensorValue[sidetouch] == 1 && SensorValue[frontTouch] != 1) || SensorValue[colorSensor] == tapeColour)
		{}

		drive(0);
		driveDistance(-5, fwdSpeed);
		
		// case if inside wall corner
		if(SensorValue[sidetouch] == 1 && SensorValue[frontTouch] == 1)
			rotateRobot(-90);
		
		// case if outside wall corner
		else if(SensorValue[sidetouch] == 0 && SensorValue[frontTouch] == 0)
			rotateRobot(90);
	}
}

// startup sequence
void startup(int *tapeColour, int *edges, float *duration)
{
	// inital text
	displayString("It's roboting time.", 10);
	wait1Msec(5000);

	eraseDisplay();
	
	// initialize value to tapeColour
	tapeColour = (int)colorRed;

	// keeps running until user is satisfied with colour
	while(true)
	{
		displayString("Place Robot on coloured tape for at least 5 seconds: ", 10);
		wait1Msec(100);

		// runs until colour sensor detects same colour for 2 seconds (to avoid detecting random colours in set up)
		while(true)
		{
			tapeColour = getColorName(colourSensor);
			wait1Msec(2000);
			if(getColorName(colourSensor) == tapeColour)	break;
			else	tapeColour = (int)(colorRed);
		}

		displayString("Color Chosen! Accept or retry? (up for accept, down for retry)", 15);

		// runs until either up or down is pressed
		while(!getButtonPress(buttonUp) || !getButtonPress(buttonDown)
		{}

		eraseDisplay();

		// breaks loop if up is pressed
		if(getButtonPress(buttonUp)
		{
			while(getButtonPress(buttonUp))
			{}

			break;
		}
		
		// continues running if down is pressed
		else
		{
			while(getButtonPress(buttonDown)
			{}
		}
	}

	wait1Msec(100);
	edges = 4;
	
	// waits until enter is pressed
	while(!getButtonPress(buttonEnter))
	{
		// displays message and number of edges
		eraseDisplay();
		displayString("Enter number of edges in the room: (up to increment, down to decrement, enter to confirm) ", 10);
		displayString("Number of edges: %d", edges, 15);

		// waits until either button is pressed
		while(!getButtonPress(buttonUp) || !getButtonPress(buttonDown) || !getButtonPress(buttonEnter))
		{}

		// minimum edges can be 4
		if(edges > 3)
		{
			// increment if up is pressed
			if(getButtonPress(buttonUp))
			{
				while(getButtonPress(buttonUp))
				{}
				edges++;
			}
			   
			// decrement if down is pressed
			else
			{
				while(getButtonPress(buttonDown))
				{}
				edges--;
			}
		}
	}

	while(getbuttonPress(buttonEnter))
	{}

	eraseDisplay();

	wait1Msec(100);
	duration = 0.0;
			   
	// waits until enter is pressed
	while(!getButtonPress(buttonEnter))
	{
		// displays message and duration
		eraseDisplay();
		displayString("Enter Duration of Cleaning: (Up for increment of 1 minute, down for decrement, enter to confirm)", 10);
		displayString("Duration: %f", duration, 15);

		// waits until either button is pressed
		while(!getButtonPress(buttonUp) || !getButtonPress(buttonDown) || !getButtonPress(buttonEnter))
		{}
		
		// duration can't be negative
		if(duration > 0)
		{
			// increments if up is pressed
			if(getButtonPress(buttonUp))
			{
				while(getButtonPress(buttonUp))
				{}
				duration++;
			}
			
			// decrements if down is pressed
			else
			{
				while(getButtonPress(buttonDown))
				{}
				duration--;
			}
		}
	}

	while(getbuttonPress(buttonEnter))
	{}

	eraseDisplay();

	// final message in startup
	wait1Msec(100);
	displayString("All configured! Place Robot at starting position and press enter to start.", 10);//Waits for enter button to be pressed then begins cleaning

	wait1Msec(100);
	while(!getButtonPress(buttonEnter))
	{}

	while(getButtonPress(buttonEnter))
	{}

	configureAllSensors();
}

// main method
task main()
{
	// initializes variables
	int tapeColour = (int)colorRed, edges = 4;
	float duration = 1.0;

	// runs startup
	startup(tapeColour, edges, duration);

	// runs sweepEdge
	sweepEdge(edges, tapeColour);

	// navigation algorithm (work in progress)
	if(SensorValue[frontTouch] == 1)
	{
		driveDistance(-5);
		rotateRobot(90 + random(180), turnSpeed);
	}
}
