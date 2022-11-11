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

void drive(int mPower)
{
	motor[motorLeft] = motor[motorD] = mPower;
}

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

void sweepEdge(int edges, int tapeColour)
{
	for(int counter = 0; counter < edges; counter++)
	{
		drive(fwdSpeed);
		while((SensorValue[sidetouch] == 1 && SensorValue[frontTouch] != 1) || SensorValue[colorSensor] == tapeColour)
		{}

		drive(0);
		driveDistance(-5);

		if(SensorValue[sidetouch] == 1 && SensorValue[frontTouch] == 1)
		{
			rotateRobot(-90);
		}
		else if(SensorValue[sidetouch] == 0 && SensorValue[frontTouch] == 0)
		{
			rotateRobot(90);
		}
	}
}

task main()
{
	displayString("Epic cool robot time.", 10);
	wait1Msec(5000);

	eraseDisplay();

	int tapecolour = (int)colorRed;

	while(true)
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
	int edges = 4;
	while(!getButtonPress(buttonEnter))
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

	float duration = 0.0;
	while(!getButtonPress(buttonEnter))
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
	displayString("All configured! Place Robot at starting position and press enter to start.", 10);

	wait1Msec(100);
	while(!getButtonPress(buttonEnter))
	{}

	while(getButtonPress(buttonEnter))
	{}

	sweepEdge(edges, tapeColour);
}
