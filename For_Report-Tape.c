/*
Varun Chauhan, Ryan Bernstein, Suyu Chen, Jerry Chen
Version 1.0
Assumptions: User will follow instructions given during startup, all corners in room 
are 90 degrees, tape boundaries are always straight lines,
Description: Main code for cleaning robot. On startup, user will be asked to set 
tape colour, number of edges, and duration. After that, robot will clean the 
perimeter of the room based on inputted number of edges. After cleaning edges, robot 
will used a weighted random turn navigation algorithm to clean at least 90% of the 
room.
*/

#include <UW_sensorMux.c>

// Motor ports
tMotor motorLeft = motorA;
tMotor motorRight = motorD;
tMotor motorSpray = motorC;
tMotor motorDrum = motorB;

// Sensor ports
#define ultrasonic S1
#define gyro S2
#define color S3
#define mplexer S4
#define sTouch msensor_S4_1
#define lTouch msensor_S4_2
#define rTouch msensor_S4_3

// constants
#define FWD_SPEED 30	// standard movement speed
#define TURN_SPEED 10 	// standard turn speed
#define RADIUS 4		//  wheel radius
#define DRUM_SPRAY_SPEED 60

/**
 * @brief Configures all sensors
 *
 */
void configureAllSensors()
{
	SensorType[ultrasonic] = sensorEV3_Ultrasonic;
	wait1Msec(100);

	SensorType[gyro] = sensorEV3_Gyro;
	wait1Msec(150);
	SensorMode[gyro] = modeEV3Gyro_Calibration;
	wait1Msec(150);
	SensorMode[gyro] = modeEV3Gyro_RateAndAngle;
	wait1Msec(150);

	SensorType[color] = sensorEV3_Color;
	wait1Msec(150);
	SensorMode[color] = modeEV3Color_Color;
	wait1Msec(150);

	// Configure sensor port
	SensorType[mplexer] = sensorEV3_GenericI2C;
	wait1Msec(200);

	// configure each channel on the sensor mux
	if (!initSensorMux(sTouch, touchStateBump))
		return;
	if (!initSensorMux(lTouch, touchStateBump))
		return;
	if (!initSensorMux(rTouch, touchStateBump))
		return;
}

/**
 * @brief Drive robot at specified power/direction
 *
 * @param mPower motor power (-100 to 100) negative for driving in reverse
 */
void drive(int mPower)
{
	// negative because motor orientation is reversed on robot
	motor[motorLeft] = motor[motorRight] = -mPower; 
}

/**
 * @brief Drive robot a specified distance
 *
 * @param distance distance for robot to drive in cm (negative for backwards)
 * @param mPower motor power (positive number 0-100)
 */
void driveDistance(int distance, int mPower)
{
	const float CM_TO_DEG = 180 / (RADIUS * PI);
	nMotorEncoder[motorLeft] = 0;
	
	if (distance > 0)
		drive(mPower);
	else
		drive(-mPower);

	while (abs(nMotorEncoder[motorLeft]) < abs(distance * CM_TO_DEG));

	drive(0);
}

/**
 * @brief Rotate robot with collision detection
 * @author Ryan Bernstein
 * @param angle target angle to turn in degrees
 * @return true if turn completed, false if collision
 */
bool smartRotateRobot(int angle, int tapeColour)
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

	while (abs(getGyroDegrees(gyro)) < abs(angle))
	{
		if (readMuxSensor(lTouch) == 1 || readMuxSensor(rTouch) == 1 ||
			readMuxSensor(sTouch) == 1 || SensorValue[color] == tapeColour)
		{
			drive(0);
			return false;
		}
	}
	drive(0);
	motor[motorDrum] = DRUM_SPRAY_SPEED;
	return true;
}

/**
 * @brief Turn robot using only one motor driving forwards for a wider turn
 * @author Ryan Bernstein
 * @param angle angle to turn in degrees
 */
void rotateRobotWide(int angle)
{
	resetGyro(gyro);
	motor[motorDrum] = 0;
	if (angle > 0)
		motor[motorRight] = -1 * TURN_SPEED;
	else
		motor[motorLeft] = -1 * TURN_SPEED;
	while (abs(getGyroDegrees(gyro)) < abs(angle));
	motor[motorDrum] = DRUM_SPRAY_SPEED;
	drive(0);
}

/**
 * @brief Turn robot using only one motor driving backward for a wider turn
 * @author Ryan Bernstein
 * @param angle angle to turn
 */
void rotateRobotBackwardsWide(int angle)
{
	resetGyro(gyro);
	motor[motorDrum] = 0;
	if (angle > 0)
		motor[motorLeft] = TURN_SPEED;
	else
		motor[motorRight] = TURN_SPEED;
	while (abs(getGyroDegrees(gyro)) < abs(angle));
	motor[motorDrum] = DRUM_SPRAY_SPEED;
	drive(0);
}

/**
 * @brief Display the splash screen
 * @author Varun Chauhan
 */
void splashScreen()
{
	displayString(5, "It's roboting time.");
	wait1Msec(2000);
}

/**
 * @brief Set tape colour for user
 * @author Varun Chauhan
 * @return int the tape colour code
 */
int getTapeColour()
{
	eraseDisplay();
	int tapeColour = 0;

	// keeps running until user is satisfied with colour
	while (true)
	{
		displayString(5, "Place Robot on coloured tape");
		displayString(6, "for at least 5 seconds: ");
		wait1Msec(500);

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
	return tapeColour;
}

/**
 * @brief Get # of edges in room from user
 * @author Varun Chauhan
 */
int getEdges()
{
	eraseDisplay();
	int edges = 4;

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
		while (!(getButtonPress(buttonUp) || getButtonPress(buttonDown) ||
				 getButtonPress(buttonEnter)));

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
	return edges;
}

/**
 * @brief Get cleaning duration from user
 * @author Varun Chauhan
 */
float getDuration()
{
	eraseDisplay();
	float duration = 0.0;

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
		while (!(getButtonPress(buttonUp) || getButtonPress(buttonDown) ||
				 getButtonPress(buttonEnter)));

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
	wait1Msec(100);
	return duration;
}

/**
 * @brief Display instructions to user and waits for user to press enter to start the
 *  	  robot
 * @author Varun Chauhan
 */
void waitForStartConfirmation()
{
	displayString(6, "All configured! Place Robot at");
	displayString(7, "starting position and press");
	displayString(8, "enter to start.");
	wait1Msec(100);
	while (!getButtonPress(buttonEnter));
	while (getButtonPress(buttonEnter));
}

/**
 * @brief Drives robot along an edge and rotates once a corner is detected
 * @author Suyu Chen
 * @param edges Number of edges
 * @param tapeColour Color of border tape
 */
void sweepEdge(int edges, int tapeColour)
{
	const int ULTRASONIC_WALL_DIST = 20;
	bool alongTape = false;
	int cornerType = 0; // 0 = none, 1 = inside corner, 2 = outside corner,  
						// 3 = wall to tape

	for (int counter = 0; counter < edges; counter++)
	{
		cornerType = 0;
		drive(FWD_SPEED);

		while (cornerType == 0)
		{
			if (readMuxSensor(lTouch) == 1 || readMuxSensor(rTouch) == 1)
			{
				cornerType = 1;
				displayString(11, "inside corner  ");
			}
			else if (!alongTape && SensorValue[ultrasonic] > ULTRASONIC_WALL_DIST)
			{
				cornerType = 2;
				displayString(11, "outside corner ");
			}
			else if (SensorValue[color] == tapeColour)
			{
				cornerType = 3;
				displayString(11, "tape corner    ");
			}

			displayString(10, "Dist: %d", SensorValue[ultrasonic]);
			wait1Msec(20);
		}

		drive(0);
		eraseDisplay();
		displayString(10, "corner type %d", cornerType);
		wait1Msec(1000);

		if (cornerType == 2) // outside corners
		{
			driveDistance(5, FWD_SPEED);
			rotateRobotWide(-90);
			driveDistance(10, FWD_SPEED);
		}
		else // inside corners
		{
			driveDistance(-15, FWD_SPEED);
			rotateRobotWide(90);
			driveDistance(5, FWD_SPEED);
			rotateRobotBackwardsWide(45);
			driveDistance(-5, FWD_SPEED);
			rotateRobotBackwardsWide(-45);
		}
		
		if(cornerType == 3)	
			alongTape = true;
		else
			alongTape = false;
	}
}

/**
 * @brief Randomly moves around room to clean room
 * @author Ryan Bernstein
 */
void randomClean(float duration, int tapeColour)
{
	bool rotationCollision = false;
	while (time100[T1] < duration * 600)
	{
		displayString(7, "Cleaning ... ");
		drive(FWD_SPEED);
		if (readMuxSensor(rTouch) == 1 || readMuxSensor(lTouch) == 1 ||
			readMuxSensor(sTouch) == 1 || rotationCollision)
		{
			drive(0);
			drive(-FWD_SPEED / 2);
			wait1Msec(2000);
			rotationCollision = !smartRotateRobot(90 + rand() % 180, tapeColour);
		}
		wait1Msec(100);
	}
}

/**
 * @brief Play end chime
 * @author Jerry Chen
 */
void endChime()
{
	const float NOTE_C = 554.37;
	const float NOTE_D = 587.33;
	const float NOTE_E = 659.25;
	const float NOTE_F = 739.99;
	const float NOTE_G = 783.99;
	const float NOTE_A = 880;
	
	eraseDisplay();
	displayString(7, "Roboting Complete");

	int beatLength = 25;
	int wait = 200;

	const int NUM_NOTES = 20;
	float music[NUM_NOTES] = {NOTE_G, NOTE_G, NOTE_G, NOTE_G, NOTE_G, NOTE_G, NOTE_A,
							  NOTE_G, NOTE_G, NOTE_F, NOTE_F, NOTE_F, NOTE_F, NOTE_E, 
							  NOTE_E, NOTE_E, NOTE_E, NOTE_E, NOTE_D, NOTE_D};
	int beatLengths[NUM_NOTES] = {1, 2, 1, 1, 2, 3, 2, 1, 3, 8, 1, 1, 2, 1, 8, 1, 1,
								  2, 1, 1};

	for (int note = 0; note < NUM_NOTES; note++)
	{
		playTone(music[note], beatLength * beatLengths[note]);
		wait1Msec(wait);
	}

	sleep(10000);
}

/**
 * @brief Main program
 *
 */
task main()
{
	int edges = 4;
	float duration = 1.0;
	int tapeColour = 0;

	configureAllSensors();
	splashScreen();
	tapeColour = getTapeColour();
	edges = getEdges();
	duration = getDuration();
	waitForStartConfirmation();

	motor[motorDrum] = DRUM_SPRAY_SPEED;
	motor[motorSpray] = DRUM_SPRAY_SPEED;

	time100[T1] = 0;

	sweepEdge(edges, tapeColour);
	randomClean(duration, tapeColour);

	motor[motorDrum] = 0;
	motor[motorSpray] = 0;

	endChime();
}