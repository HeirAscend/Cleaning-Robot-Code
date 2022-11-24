/*
Varun Chauhan, Ryan Bernstein, Suyu Chen, Jerry Chen
Version 1.0
Assumptions: User will follow instructions given during startup, all corners in room are 90 degrees, tape boundaries are always straight lines,
		 obstacles allow for movement between
Description: Main code for cleaning robot. On startup, user will be asked to set tape colour, number of edges, and duration. After that, robot will
		 clean the perimeter of the room based on inputted number of edges. After cleaning edges, robot will used a weighted random turn navigation
		 algorithm to clean at least 90% of the room within 5 minutes.
*/

#include <UW_sensorMux.c>

// variables for the motors
tMotor motorLeft = motorA, motorRight = motorD, motorSpray = motorC, motorDrum = motorB;

#define ultrasonic S1
#define gyro S2
#define color S3
#define mplexer S4
#define sfTouch msensor_S4_1
#define flTouch msensor_S4_2
#define frTouch msensor_S4_3

const int FWD_SPEED = 50, TURN_SPEED = 20; // variables for standard speeds for movement and turning
const float RADIUS = 4;					  // variable for wheel radius

int tapeColour = (int)colorWhite
int edges = 4;
float duration = 1.0;

/**
 * @brief Configures all sensors
 * 
 */
void configureAllSensors()
{
	SensorType[ultrasonic] = sensorEV3_Ultrasonic;
	wait1Msec(100);
	SensorType[gyro] = sensorEV3_Gyro;
	wait1Msec(100);
	SensorMode[gyro] = modeEV3Gyro_Calibration;
	wait1Msec(150);
	SensorMode[gyro] = modeEV3Gyro_RateAndAngle;
	wait1Msec(100);

	SensorType[color] = sensorEV3_Color;
	wait1Msec(100);
	SensorMode[color] = modeEV3Color_Color;
	wait1Msec(100);

	// Configure sensor port
	SensorType[mplexer] = sensorEV3_GenericI2C;
	wait1Msec(200);

	// configure each channel on the sensor mux
	if (!initSensorMux(sfTouch, touchStateBump))
		return;
	if (!initSensorMux(flTouch, touchStateBump))
		return;
	if (!initSensorMux(frTouch, touchStateBump))
		return;
}

/**
 * @brief Drive robot at specified power/direction
 * 
 * @param mPower motor power (-100 to 100) negative for driving in reverse
 */
void drive(int mPower)
{
	motor[motorLeft] = motor[motorRight] = -mPower; // negative because motor orientation is reversed on robot
}

/**
 * @brief Drive robot a specified distance
 * 
 * @param distance distance for robot to drive in cm
 * @param mPower motor power (positive number 0-100)
 */
void driveDistance(int distance, int mPower)
{
	const float CM_TO_DEG = 180 / (RADIUS * PI);
	nMotorEncoder[motorLeft] = 0;
	
	if (distance > 0)
		drive(-mPower);   // negative because motor orientation is reversed on robot
	else
		drive(mPower);

	while (abs(nMotorEncoder[motorLeft]) < abs(distance * CM_TO_DEG));

	drive(0);
}

/**
 * @brief Rotate robot a specified angle
 * 
 * @param angle angle to turn robot
 */
void rotateRobot(int angle)
{
	resetGyro(gyro);
	motor[motorDrum] = 0;

	if (angle > 0)
	{
		motor[motorLeft] = TURN_SPEED;
		motor[motorRight] = -1 * TURN_SPEED;
	}
	else
	{
		motor[motorRight] = TURN_SPEED;
		motor[motorLeft] = -1 * TURN_SPEED;
	}

	while (abs(getGyroDegrees(gyro)) < abs(angle));

	drive(0);
}

/**
 * @brief Startup sequence to receive information from user
 * 
 */
void startup()
{
	// inital text
	displayString(5, "It's roboting time.");
	wait1Msec(5000);

	eraseDisplay();

	// initialize value to tapeColour
	// tapeColour = (int)colorRed;

	// keeps running until user is satisfied with colour
	while (true)
	{
		displayString(5, "Place Robot on coloured tape");
		displayString(6, "for at least 5 seconds: ");
		wait1Msec(100);

		// runs until colour sensor detects same colour for 2 seconds (to avoid detecting random colours in set up)
		while (true)
		{
			tapeColour = SensorValue(color);
			wait1Msec(2000);
			if (getColorName(color) == tapeColour)
				break;
			else
				tapeColour = (int)(colorRed);
		}

		displayString(9, "Color %d Chosen!", tapeColour);
		displayString(10, "Accept or retry?");
		displayString(12, "Up = Accept, Down = Retry");

		// runs until either up or down is pressed
		while (!(getButtonPress(buttonUp) || getButtonPress(buttonDown)));

		// breaks loop if up is pressed
		if (getButtonPress(buttonUp))
		{
			while (getButtonPress(buttonUp));
			eraseDisplay();
			break;
		}

		// continues running if down is pressed
		else
		{
			while (getButtonPress(buttonDown));
			eraseDisplay();
		}
	}

	wait1Msec(100);
	edges = 4;

	// waits until enter is pressed
	while (!getButtonPress(buttonEnter))
	{
		// displays message and number of edges
		eraseDisplay();
		displayString(3, "Enter number of edges in room");
		displayString(4, "- Up to increment");
		displayString(5, "- Down for decrement");
		displayString(6, "- Enter to confirm");
		displayString(10, "Number of edges: %d", edges);

		// waits until either button is pressed
		while (!(getButtonPress(buttonUp) || getButtonPress(buttonDown) || getButtonPress(buttonEnter)));

		if (getButtonPress(buttonUp)) // increment if up is pressed
		{
			while (getButtonPress(buttonUp));
			edges++;
		}
		else // decrement if down is pressed
		{
			while (getButtonPress(buttonDown));
			if (edges > 4)
				edges--;
		}
	}

	while (getButtonPress(buttonEnter));
	eraseDisplay();
	wait1Msec(100);
	duration = 0.0;

	// waits until enter is pressed
	while (!getButtonPress(buttonEnter))
	{
		// displays message and duration
		eraseDisplay();
		displayString(3, "Enter Duration of Cleaning:");
		displayString(4, "- Up to increment");
		displayString(5, "- Down for decrement");
		displayString(6, "- Enter to confirm");
		displayString(10, "Duration: %d mins", (int)duration);

		// waits until either button is pressed
		while (!(getButtonPress(buttonUp) || getButtonPress(buttonDown) || getButtonPress(buttonEnter)));

		// increments if up is pressed
		if (getButtonPress(buttonUp))
		{
			while (getButtonPress(buttonUp));
			duration++;
		}

		// decrements if down is pressed
		else
		{
			while (getButtonPress(buttonDown));
			if (duration > 0)
				duration--;
		}
	}

	// wait until enter is released then erase display
	while (getButtonPress(buttonEnter));
	eraseDisplay();

	// final message in startup
	wait1Msec(100);

	// Waits for enter button to be pressed then begins cleaning
	displayString(6, "All configured! Place Robot at");
	displayString(7, "starting position and press");
	displayString(8, "enter to start.");

	wait1Msec(100);
	while (!getButtonPress(buttonEnter));
	while (getButtonPress(buttonEnter));

	configureAllSensors();
}

/**
 * @brief Drives robot along an edge and rotates once a corner is detected
 * 
 * @param edges Number of edges
 * @param tapeColour Color of border tape
 */
void sweepEdge(int edges, int tapeColour)
{
	const int ULTRASONIC_WALL_DIST = 10;
	bool alongTape = false;
	bool fTouch = false;
	bool colorDet = false;

	for (int counter = 0; counter < edges; counter++)
	{
		drive(FWD_SPEED);

		while (!(readMuxSensor(flTouch)==1 || readMuxSensor(frTouch)==1 || SensorValue[ultrasonic] > ULTRASONIC_WALL_DIST))
		{
			fTouch = readMuxSensor(flTouch) == 1 || readMuxSensor(frTouch) == 1;
			
			colorDet = SensorValue[color] == tapeColour;

			wait1Msec(50);
			eraseDisplay();
		}

		drive(0);
		driveDistance(50, FWD_SPEED);

		displayString(1, "w2w %d", w2w);
		displayString(3, "n2n %d", n2n);
		displayString(5, "w2t %d", w2t);
		displayString(7, "t2w %d", t2w);

		// case if inside wall corner
		if (w2w || w2t || t2w)
			rotateRobot(-90);

		// case if outside wall corner
		else if (n2n)
			rotateRobot(90);
	}
}

void navigate(float duration)
{
	//TODO
}

/**
 * @brief Main program
 * 
 */
task main()
{
	configureAllSensors();
	startup();

	motor[motorDrum] = 100;
	motor[motorSpray] = 100;

	// runs sweepEdge
	// sweepEdge(edges, tapeColour);

	// navigation algorithm (work in progress)
	while (true)
	{
		displayString(7, "Cleaning ... ");
		drive(FWD_SPEED);
		if (readMuxSensor(flTouch) == 1 || readMuxSensor(frTouch) == 1 || SensorValue[color] == tapeColour)
		{
			drive(0);
			drive(-FWD_SPEED / 2);
			wait1Msec(2000);
			rotateRobot(90 + rand() % 180);
		}
		wait1Msec(100);
	}
}