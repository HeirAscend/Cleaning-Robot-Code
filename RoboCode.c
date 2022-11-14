tMotor motorLeft = motorA, motorRight = motorD, motorSpray, motorDrum;

int fwdSpeed = 50, turnSpeed = 20;

const float RADIUS = 2.75

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

//Drive robot at specified power/direction
void drive(int mPower)
{
	motor[motorLeft] = motor[motorD] = mPower;
}

//Drive robot a specified distance
void driveDistance(int distance, int mPower) //drives robot straight for given distance based on motor encoders and nominal wheel diameter. Positive distance is forward
{
	nMotorEncoder[motorLeft]=0;
	const float CM_TO_DEG = 180/(RADIUS*PI);
	if (distance>0)
	{
		drive(mPower);
	}
	else
	{
		drive(-1 * mPower);
	}

	while (abs(nMotorEncoder[motorLeft])<abs(distance*CM_TO_DEG))
	{}

	drive(0);
}

//Rotate robot a specified angle
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

//Drives robot along an edge and rotates once a corner is detected
void sweepEdge(int edges, int tapeColour)
{
	for(int counter = 0; counter < edges; counter++)
	{
		drive(fwdSpeed);//Drives forward while side touch sensor activated, and front touch sensor not activated or colour sensor detects tape border
		while((SensorValue[sidetouch] == 1 && SensorValue[frontTouch] != 1) || SensorValue[colorSensor] == tapeColour)
		{}

		drive(0);
		driveDistance(-5);

		if(SensorValue[sidetouch] == 1 && SensorValue[frontTouch] == 1)//If both side and front touch sensors activated, rotate 90 degrees counterclockwise
		{
			rotateRobot(-90);
		}
		else if(SensorValue[sidetouch] == 0 && SensorValue[frontTouch] == 0)//If both side and front touch sensores not activated, rotate 90 degrees clockwise
		{
			rotateRobot(90);
		}
	}
}

//Start up sequence and user input
void startup(int *tapeColour, int *edges, float *duration)
{
	displayString("Epic cool robot time.", 10);
	wait1Msec(5000);

	eraseDisplay();

	tapecolour = (int)colorRed;

	while(true)//Detects colour of tape border and sets that as the border colour
	{
		displayString("Place Robot on coloured tape for at least 5 seconds: ", 10);
		wait1Msec(100);

		while(true)
		{
			tapeColour = getColorName(colourSensor);
			wait1Msec(2000);
			if(getColorName(colourSensor) == tapeColour)	break;
			else	tapeColour = (int)(colorRed);
		}

		displayString("Color Chosen! Accept or retry? (up for accept, down for retry)", 15);

		while(!getButtonPress(buttonUp) || !getButtonPress(buttonDown)
		{}

		eraseDisplay();

		if(getButtonPress(buttonUp)
		{
			while(getButtonPress(buttonUp))
			{}

			break;
		}
		else
		{
			while(getButtonPress(buttonDown)
			{}
		}
	}

	wait1Msec(100);
	displayString("Enter number of edges in the room: (up to increment, down to decrement, enter to confirm) ", 10);
	edges = 4;
	while(!getButtonPress(buttonEnter))//User inputs number of edges of the area to clean
	{
		eraseDisplay();
		displayString("Number of edges: %d", edges, 15);

		while(!getButtonPress(buttonUp) || !getButtonPress(buttonDown)
		{}

		if(edges > 0)
		{
			if(getButtonPress(buttonUp)
			{
				while(getButtonPress(buttonUp))
				{}
				edges++;
			}
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
	displayString("Enter Duration of Cleaning: (Up for increment of 1 minute, down for decrement, enter to confirm)", 10);

	duration = 0.0;
	while(!getButtonPress(buttonEnter))//User inputs duration for robot to clean
	{
		eraseDisplay();
		displayString("Duration: %f", duration, 15);

		while(!getButtonPress(buttonUp) || !getButtonPress(buttonDown)
		{}

		if(duration > 0)
		{
			if(getButtonPress(buttonUp)
			{
				while(getButtonPress(buttonUp))
				{}
				duration++;
			}
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

	wait1Msec(100);
	displayString("All configured! Place Robot at starting position and press enter to start.", 10);//Waits for enter button to be pressed then begins cleaning

	wait1Msec(100);
	while(!getButtonPress(buttonEnter))
	{}

	while(getButtonPress(buttonEnter))
	{}

	configureAllSensors();
}

task main()
{
	int tapeColour = (int)colorRed, edges = 4;
	float duration = 1.0;

	startup(tapeColour, edges, duration);

	sweepEdge(edges, tapeColour);

	if(SensorValue[frontTouch] == 1)//random thing, maybe?
	{
		driveDistance(-5);
		rotateRobot(random(180)+90, 20);
	}
}
