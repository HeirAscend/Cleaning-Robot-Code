/*
Varun Chauhan, Ryan Bernstein, Suyu Chen, Jerry Chen
Version 1.0
Assumptions: User will follow instructions given during startup, all corners in room are 90 degrees, tape boundaries are always straight lines,
	     obstacles allow for movement between
Description: Main code for cleaning robot. On startup, user will be asked to set tape colour, number of edges, and duration. After that, robot will
	     clean the perimeter of the room based on inputted number of edges. After cleaning edges, robot will used a weighted random turn navigation
	     algorithm to clean at least 90% of the room within 5 minutes.
*/

#include "UW_sensorMux.c"

// variables for the motors
tMotor motorLeft = motorA, motorRight = motorD, motorSpray = motorC, motorDrum = motorB;

#define sbTouch S1
#define gyro S2
#define color S3
#define mplexer S4
#define sfTouch msensor_S4_1
#define flTouch msensor_S4_2
#define frTouch msensor_S4_3



// variables for standard speeds for movement and turning
const int fwdSpeed = -50, turnSpeed = 20;

// variable for wheel radius
const float RADIUS = 4;

// initializes variables
	int tapeColour = (int)colorWhite, edges = 4;
	float duration = 1.0;

// configures all sensors
void configureAllSensors()
{
	SensorType[sbTouch] = sensorEV3_Touch;
	SensorType[gyro] = sensorEV3_Gyro;
	wait1Msec(50);
	SensorMode[gyro] = modeEV3Gyro_Calibration;
	wait1Msec(100);
	SensorMode[gyro] = modeEV3Gyro_RateAndAngle;
	wait1Msec(50);

	SensorType[color] = sensorEV3_Color;
	wait1Msec(50);
	SensorMode[color] = modeEV3Color_Color;
	wait1Msec(50);



	// Configure sensor port
	SensorType[mplexer] = sensorEV3_GenericI2C;
	wait1Msec(100);

	// configure each channel on the sensor mux
	if (!initSensorMux(sfTouch, touchStateBump))
		return;
	if (!initSensorMux(flTouch, touchStateBump))
		return;
	if (!initSensorMux(frTouch, touchStateBump))
		return;
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
void rotateRobot(int angle)
{
	//resetGyro(S2);

	if (angle>0)
	{
		motor[motorLeft] = turnSpeed;
		motor[motorRight] = -1 * turnSpeed;
	}
	else
	{
		displayString(11,"bob");
		motor[motorRight] = turnSpeed;
		motor[motorLeft] = -1 * turnSpeed;
	}

	while (abs(getGyroDegrees(S2))<abs(angle))
	{}

	drive(0);
}

// drives robot along an edge and rotates once a corner is detected
void sweepEdge(int edges, int tapeColour)
{

	bool alongTape = false;

	bool w2w = false;
	bool n2n = false;
	bool w2t = false;
	bool t2w = false;

	bool fTouch = readMuxSensor(flTouch) == 1 && readMuxSensor(frTouch) == 1;
	bool sTouch = SensorValue[sbTouch] == 1 && readMuxSensor(sfTouch) == 1;
	bool colorDet = SensorValue[color] == tapeColour;

	for(int counter = 0; counter < edges; counter++)
	{
		drive(fwdSpeed);

		while(!w2w && !n2n && !w2t && !t2w)
		{
			fTouch = readMuxSensor(flTouch) == 1 && readMuxSensor(frTouch) == 1;
			sTouch = SensorValue[sbTouch] == 1 && readMuxSensor(sfTouch) == 1;
			colorDet = SensorValue[color] == tapeColour;

			w2w = sTouch && fTouch && !colorDet;
			w2t = sTouch && !fTouch && colorDet;
			t2w = !sTouch && fTouch && colorDet;
			n2n = !sTouch && !fTouch && !colorDet;

			displayString(1,"w2w %d", w2w);
			displayString(3,"n2n %d", n2n);
			displayString(5,"w2t %d", w2t);
			displayString(7,"t2w %d", t2w);

			wait1Msec(50);
			eraseDisplay();

		}

		drive(0);
		driveDistance(50, fwdSpeed);

		displayString(1,"w2w %d", w2w);
			displayString(3,"n2n %d", n2n);
			displayString(5,"w2t %d", w2t);
			displayString(7,"t2w %d", t2w);

		// case if inside wall corner
		if(w2w || w2t || t2w)
			rotateRobot(-90);

		// case if outside wall corner
		else if(n2n)
			rotateRobot(90);
	}

}

//// startup sequence
//void startup()
//{
//	// inital text
//	displayString(5, "It's roboting time.");
//	wait1Msec(5000);

//	eraseDisplay();

//	// initialize value to tapeColour
//	//tapeColour = (int)colorRed;

//	// keeps running until user is satisfied with colour
//	while(true)
//	{
//		displayString(5, "Place Robot on coloured tape");
//		displayString(6, "for at least 5 seconds: ");
//		wait1Msec(100);

//		// runs until colour sensor detects same colour for 2 seconds (to avoid detecting random colours in set up)
//		while(true)
//		{
//			tapeColour = SensorValue(color);
//			wait1Msec(2000);
//			if(getColorName(color) == tapeColour)	break;
//			else	tapeColour = (int)(colorRed);
//		}

//		displayString(10, "Color %d Chosen! Accept or retry?", tapeColour);
//		displayString(11, "(up for accept, down for retry)");

//		// runs until either up or down is pressed
//		while(!getButtonPress(buttonAny) || !getButtonPress(buttonDown))
//		{}

//		eraseDisplay();

//		// breaks loop if up is pressed
//		if(getButtonPress(buttonUp)
//		{
//			while(getButtonPress(buttonUp))
//			{}

//			break;
//		}

//		// continues running if down is pressed
//		else
//		{
//			while(getButtonPress(buttonDown)
//			{}
//		}
//	}

//	wait1Msec(100);
//	edges = 4;

//	// waits until enter is pressed
//	while(!getButtonPress(buttonEnter))
//	{
//		// displays message and number of edges
//		eraseDisplay();
//		displayString(10, "Enter number of edges in the room: (up to increment, down to decrement, enter to confirm) ");
//		displayString(15, "Number of edges: %d", edges);

//		// waits until either button is pressed
//		while(!getButtonPress(buttonUp) || !getButtonPress(buttonDown) || !getButtonPress(buttonEnter))
//		{}

//		// minimum edges can be 4
//		if(edges > 3)
//		{
//			// increment if up is pressed
//			if(getButtonPress(buttonUp))
//			{
//				while(getButtonPress(buttonUp))
//				{}
//				edges++;
//			}

//			// decrement if down is pressed
//			else
//			{
//				while(getButtonPress(buttonDown))
//				{}
//				edges--;
//			}
//		}
//	}

//	while(getbuttonPress(buttonEnter))
//	{}

//	eraseDisplay();

//	wait1Msec(100);
//	duration = 0.0;

//	// waits until enter is pressed
//	while(!getButtonPress(buttonEnter))
//	{
//		// displays message and duration
//		eraseDisplay();
//		displayString(10, "Enter Duration of Cleaning: (Up for increment of 1 minute, down for decrement, enter to confirm)");
//		displayString(15, "Duration: %f", duration);

//		// waits until either button is pressed
//		while(!getButtonPress(buttonUp) || !getButtonPress(buttonDown) || !getButtonPress(buttonEnter))
//		{}

//		// duration can't be negative
//		if(duration > 0)
//		{
//			// increments if up is pressed
//			if(getButtonPress(buttonUp))
//			{
//				while(getButtonPress(buttonUp))
//				{}
//				duration++;
//			}

//			// decrements if down is pressed
//			else
//			{
//				while(getButtonPress(buttonDown))
//				{}
//				duration--;
//			}
//		}
//	}

//	while(getbuttonPress(buttonEnter))
//	{}

//	eraseDisplay();

//	// final message in startup
//	wait1Msec(100);
//	displayString(10,"All configured! Place Robot at starting position and press enter to start.");//Waits for enter button to be pressed then begins cleaning

//	wait1Msec(100);
//	while(!getButtonPress(buttonEnter))
//	{}

//	while(getButtonPress(buttonEnter))
//	{}

//	configureAllSensors();
//}

// main method
task main()
{

	configureAllSensors();

	// runs startup
	//startup();

	// runs sweepEdge
	sweepEdge(edges, tapeColour);

	// navigation algorithm (work in progress)
	//if(SensorValue[frontTouch] == 1)
	//{
	//	driveDistance(-5);
	//	rotateRobot(90 + random(180), turnSpeed);
	//}
}
