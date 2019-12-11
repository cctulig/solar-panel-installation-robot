/*
 * StudentsRobot.cpp
 *
 *  Created on: Dec 28, 20
 *      Author: hephaestus
 *      Author: GarettR
 * 		Author: Conrad Tulig
 *		Author: Dhionis Zhidro
 *
 */

#include "StudentsRobot.h"

StudentsRobot::StudentsRobot(PIDMotor * motor1, PIDMotor * motor2,
		PIDMotor * motor3, Servo * servo) {
	Serial.println("StudentsRobot::StudentsRobot constructor called here ");
	this->servo = servo;
	this->motor1 = motor1;
	this->motor2 = motor2;
	this->motor3 = motor3;

	// Set the PID Clock gating rate. The PID must be 10 times slower than the motors update rate
	motor1->myPID.sampleRateMs = 1; //
	motor2->myPID.sampleRateMs = 1; //
	motor3->myPID.sampleRateMs = 1;  // 10khz H-Bridge, 0.1ms update, 1 ms PID

	// Set default P.I.D gains
	motor1->myPID.setpid(0.025, 0.02, -0.6);
	motor2->myPID.setpid(0.025, 0.02, -0.6);
	motor3->myPID.setpid(0.025, 0.02, -0.6);

	motor1->velocityPID.setpid(0.1, 0, 0);
	motor2->velocityPID.setpid(0.1, 0, 0);
	motor3->velocityPID.setpid(0.1, 0, 0);
	// compute ratios and bounding
	double motorToWheel = 3;
	motor1->setOutputBoundingValues(-255, //the minimum value that the output takes (Full reverse)
			255, //the maximum value the output takes (Full forward)
			0, //the value of the output to stop moving
			125, //a positive value subtracted from stop value to creep backward
			125, //a positive value added to the stop value to creep forwards
			16.0 * // Encoder CPR
					50.0 * // Motor Gear box ratio
					motorToWheel * // motor to wheel stage ratio
					(1.0 / 360.0) * // degrees per revolution
					2, // Number of edges that are used to increment the value
			480, // measured max degrees per second
			150 // the speed in degrees per second that the motor spins when the hardware output is at creep forwards
			);
	motor2->setOutputBoundingValues(-255, //the minimum value that the output takes (Full reverse)
			255, //the maximum value the output takes (Full forward)
			0, //the value of the output to stop moving
			125, //a positive value subtracted from stop value to creep backward
			125, //a positive value added to the stop value to creep forwards
			16.0 * // Encoder CPR
					50.0 * // Motor Gear box ratio
					motorToWheel * // motor to wheel stage ratio
					(1.0 / 360.0) * // degrees per revolution
					2, // Number of edges that are used to increment the value
			480, // measured max degrees per second
			150	// the speed in degrees per second that the motor spins when the hardware output is at creep forwards
			);
	motor3->setOutputBoundingValues(-255, //the minimum value that the output takes (Full reverse)
			255, //the maximum value the output takes (Full forward)
			0, //the value of the output to stop moving
			125, //a positive value subtracted from stop value to creep backward
			125, //a positive value added to the stop value to creep forwards
			16.0 * // Encoder CPR
					50.0 * // Motor Gear box ratio
					1.0 * // motor to arm stage ratio
					(1.0 / 360.0) * // degrees per revolution
					2, // Number of edges that are used to increment the value
			1400, // measured max degrees per second
			50 // the speed in degrees per second that the motor spins when the hardware output is at creep forwards
			);
	// Set up the line tracker
	pinMode(MIDDLE_LEFT, ANALOG);
	pinMode(MIDDLE_RIGHT, ANALOG);
	pinMode(FAR_LEFT, ANALOG);
	pinMode(FAR_RIGHT, ANALOG);

	pinMode(LIMIT_SWITCH, INPUT_PULLUP);

	pinMode(H_BRIDGE_ENABLE, OUTPUT);
	chassis = new DrivingChassis(motor1, motor2, 225, 25);
}
/**
 * Seperate from running the motor control,
 * update the state machine for running the final project code here
 */
void StudentsRobot::updateStateMachine() {
	long now = millis();
	switch (status) {
	case StartupRobot:
		//Do this once at startup
		status = StartRunning;
		Serial.println("StudentsRobot::updateStateMachine StartupRobot here ");
		break;
	case StartRunning:
		Serial.println("Start Running");

		digitalWrite(H_BRIDGE_ENABLE, 1);
		// Start an interpolation of the motors
		//motor1->startInterpolationDegrees(720, 1000, SIN);
		//motor2->startInterpolationDegrees(-720, 1000, SIN);
		//motor3->startInterpolationDegrees(motor3->getAngleDegrees(), 1000, SIN);

		//chassis->turnDegrees(360, 5000);
		Serial.println("We made it here");

		motor3->setVelocityDegreesPerSecond(100);
		status = WAIT_FOR_ARM_TO_ZERO; // set the state machine to wait for the motors to finish
		nextStatus = Running; // the next status to move to when the motors finish

		startTime = now + 1000; // the motors should be done in 1000 ms
		nextTime = startTime + 1000; // the next timer loop should be 1000ms after the motors stop
		break;
	case Running:
		// Set up a non-blocking 1000 ms delay
		status = WAIT_FOR_TIME;
		nextTime = nextTime + 1000; // ensure no timer drift by incremeting the target
		// After 1000 ms, come back to this state
		lineState = 0;
		nextStatus = Running;

		servo->write(0);
		//motor1->setVelocityDegreesPerSecond(-260);
		//motor2->setVelocityDegreesPerSecond(260);

		if (myCommandsStatus == Heading_to_pickup) {
			status = GoToFirstPickup;
			//nextStatus = GoToFirstPickup;
			roofLines = checkBoardSide(dropoffAngle);
			updateRoofSideDir(dropoffAngle, dropoffPosition);
		}

		//Motor Angle Prints
		//Serial.println("motor 1 " + String(motor1->getAngleDegrees()));
		//Serial.println("motor 2 " + String(motor2->getAngleDegrees()));

		//Line Follow Prints
		/*Serial.print("Far left: ");
		 Serial.print(analogRead(FAR_LEFT));
		 Serial.print(",   Middle left: ");
		 Serial.print(analogRead(MIDDLE_LEFT));
		 Serial.print(",   Middle right: ");
		 Serial.print(analogRead(MIDDLE_RIGHT));
		 Serial.print(",   Far right: ");
		 Serial.println(analogRead(FAR_RIGHT));
		 */
		// Do something
		if (!digitalRead(0))
			Serial.println(
					" Running State Machine " + String((now - startTime)));
		break;
	case GoToFirstPickup:
		chassis->resetMotorPos();
		startPos = (motor1->getAngleDegrees() + motor2->getAngleDegrees()) / 2;
		distanceToTravel = 320;
		direction = 1;
		status = LINE_FOLLOW_FOR_DISTANCE;
		nextStatus = Pickup_Plate;
		myCommandsStatus = Waiting_for_approval_to_pickup;
		if (dropoffAngle == 45) {
			liftarmdeg = -2350;
		} else if (dropoffAngle == 25) {
			liftarmdeg = -2500;
		}

		break;
	case Pickup_Plate:
		if (myCommandsStatus == Picking_up) {
			servo->write(105);
			motor3->startInterpolationDegrees(liftarmdeg, 3000, SIN); //TODO delay
			nextTime = millis() + 3000;
			status = WAIT_FOR_TIME;
			nextStatus = DriveToRoofTest;
			myCommandsStatus = Heading_to_Dropoff;
		}

		break;
	case DriveToRoofTest:
		Serial.println("Driving to roof test");
		if (dropoffAngle == 25 && dropoffPosition == 2)
			turnAdjustment = -1;
		if (DriveToRoof(dropoffPosition, roofLines, DriveToRoofTest,
				LineUpToPlate2)) {

		}
		break;
	case LineUpToPlate2:
		chassis->resetMotorPos();
		startPos = (motor1->getAngleDegrees() + motor2->getAngleDegrees()) / 2;
		if (dropoffAngle == 45) {
			distanceToTravel = 0;
		} else if (dropoffAngle == 25) {
			distanceToTravel = 10; //TODO
		}
		status = LINE_FOLLOW_FOR_DISTANCE;
		nextStatus = WaitForDropoff;
		myCommandsStatus = Waiting_for_approval_to_dropoff;
		break;
	case WaitForDropoff:
		if (myCommandsStatus == Dropping_off) {
			status = DropoffPlate;
			nextStatus = GoToSafe;
			myCommandsStatus = Heading_to_safe_zone;
		}
		break;
	case DropoffPlate:
		servo->write(0);
		status = WAIT_FOR_TIME;
		nextTime = millis() + 2000;
		myCommandsStatus = Heading_to_safe_zone;
		break;
	case GoToSafe:
		chassis->resetMotorPos();
		chassis->driveForward(500, 1500);
		status = WAIT_FOR_MOTORS_TO_FINNISH;
		nextStatus = DriveToPickupPlatePt1;

		break;
	case DriveToPickupPlatePt1:
		if (dropoffAngle == 45 && dropoffPosition == 1)		//todo here maybe?
			turnAdjustment = -2;
		if (dropoffAngle == 45 && dropoffPosition == 2)		//todo here maybe?
			turnAdjustment = 1;
//		if(dropoffAngle == 45)
//			turnAdjustment = 1;

		if (DriveToRoof(1, dropoffPosition, DriveToPickupPlatePt1,
				DriveToPickupPlatePt2)) {
			updateRoofSideDir(dropoffAngle,
					getOppositeRoofSide(dropoffPosition));
			myCommandsStatus = Heading_to_pickup;
			turnAdjustment = 0;
		}
		break;
	case DriveToPickupPlatePt2:
		if (dropoffAngle == 45)
			motor3->startInterpolationDegrees(-1700, 1000, SIN);
		if (dropoffAngle == 25)
			motor3->startInterpolationDegrees(-2500, 1000, SIN);		//TODO
		DriveToRoof(getOppositeRoofSide(dropoffPosition), roofLines - 1,
				DriveToPickupPlatePt2, LineUpToPlate);

		break;
	case LineUpToPlate:
		chassis->resetMotorPos();
		startPos = (motor1->getAngleDegrees() + motor2->getAngleDegrees()) / 2;
		if (dropoffAngle == 45) {
			distanceToTravel = 60;
		} else if (dropoffAngle == 25) {
			distanceToTravel = 90; //TODO
		}
		status = LINE_FOLLOW_FOR_DISTANCE;
		nextStatus = WaitForPickup;
		myCommandsStatus = Waiting_for_approval_to_pickup;
		break;
	case WaitForPickup:
		if (myCommandsStatus == Picking_up) {
			servo->write(105);
			status = WAIT_FOR_TIME;
			nextTime = millis() + 2000;
			nextStatus = GoToSafe2;
			if (dropoffAngle == 45)
				motor3->startInterpolationDegrees(-2400, 1000, SIN);
			if (dropoffAngle == 25)
				motor3->startInterpolationDegrees(-2200, 1000, SIN);	//TODO
		}
		break;
	case GoToSafe2:

		chassis->resetMotorPos();
		chassis->driveForward(500, 1500);
		status = WAIT_FOR_MOTORS_TO_FINNISH;
		nextStatus = DriveToPlateDropoff1;
		dropoffPosition = getOppositeRoofSide(dropoffPosition);
		myCommandsStatus = Heading_to_Dropoff;

		break;
	case DriveToPlateDropoff1:
		if (dropoffAngle == 25 && dropoffPosition == 1)
			turnAdjustment = -1;
		if (dropoffAngle == 25 && dropoffPosition == 2)		//TODO
			turnAdjustment = -1;
		if (dropoffAngle == 45 && dropoffPosition == 2)
			turnAdjustment = -1;

		if (DriveToRoof(1, dropoffPosition, DriveToPlateDropoff1,
				DriveToPlateDropoff2)) {

			turnAdjustment = 0;
		}
		break;
	case DriveToPlateDropoff2:
		motor3->startInterpolation(0, 3000, SIN);
		//		if (DriveToRoof(getOppositeRoofSide(dropoffPosition), roofLines-1,
		//				DriveToPlateDropoff2, LowerPlate)) {
		//			Serial.println("Why are we still here?");
		//			myCommandsStatus = Ready_for_new_task;
		//		}
		chassis->resetMotorPos();
		chassis->driveForward(50, 1500);
		status = LowerPlate;
		myCommandsStatus = Waiting_for_approval_to_dropoff;

		break;
	case LowerPlate:
		if (myCommandsStatus == Dropping_off) {
			servo->write(0);
			//u9motor3->startInterpolationDegrees(-100, 500, SIN);
			chassis->resetMotorPos();
			chassis->driveForward(50, 1500);

			status = WAIT_FOR_MOTORS_TO_FINNISH;
			if (totalTrips == 0)
				nextStatus = SwitchSidesPt1;
			else {
				myCommandsStatus = Ready_for_new_task;
				status = Halting;
			}

		}

		break;
	case SwitchSidesPt1:
		if (DriveToRoof(2, 4, SwitchSidesPt1, SwitchSidesPt2)) {
			turn180Dir = 0;
			turnDirection = -turnDirection;
		}
		break;
	case SwitchSidesPt2:
		if (DriveToRoof(4, 1, SwitchSidesPt2, Halting)) {
			turn180Dir = 1;
			myCommandsStatus = Ready_for_new_task;
			totalTrips++;

		}
		break;
	case LINE_FOLLOW_UNTIL_BLACK_LINE:
		if (analogRead(MIDDLE_LEFT) >= lineThreashold
				&& analogRead(MIDDLE_RIGHT) >= lineThreashold
				&& analogRead(FAR_LEFT) >= lineThreashold
				&& analogRead(FAR_RIGHT) >= lineThreashold) {
			Serial.println("Stop");
			motor1->setVelocityDegreesPerSecond(0);
			motor2->setVelocityDegreesPerSecond(0);

			status = nextStatus;
		} else {
			FollowLine(direction);
		}

		//status = WAIT_FOR_MOTORS_TO_FINNISH;
		//nextStatus = LineFollow;
		break;
		/*
		 * startPos= (motor1->getAngleDegrees() + motor2->getAngleDegrees())/2;
		 *	distanceToTravel= 500;
		 *	direction = -1;
		 */
	case LINE_FOLLOW_FOR_DISTANCE:
		Serial.println("Starting distance linefollow");
		Serial.print("Distance traveled: ");
		Serial.print(chassis->distanceTraveled(startPos));
		Serial.print(" Distance Wanted: ");
		Serial.println(distanceToTravel);
		if (chassis->distanceTraveled(startPos) * direction
				< distanceToTravel) {

			Serial.print("Distance traveled: ");
			Serial.print(chassis->distanceTraveled(startPos));
			Serial.print(" Distance Wanted: ");
			Serial.println(distanceToTravel);
			FollowLine(direction);
		} else {
			Serial.println("Stop");
			motor1->setVelocityDegreesPerSecond(0);
			motor2->setVelocityDegreesPerSecond(0);

			status = nextStatus;
		}

		break;
	case LINE_FOLLOW_FOR_DISTANCE_BACKWARDS:
		if (-(chassis->distanceTraveled(startPos)) < distanceToTravel) {

			//Serial.print("Distance traveled: ");
			//Serial.print(chass
		} else {
			Serial.println("Stop");
			motor1->setVelocityDegreesPerSecond(0);
			motor2->setVelocityDegreesPerSecond(0);

			status = nextStatus;
		}
		break;
		//	case ZERO_ARM:

		//	break;
	case WAIT_FOR_TIME:
		// Check to see if enough time has elapsed
		if (nextTime <= millis()) {
			// if the time is up, move on to the next state
			status = nextStatus;
		}
		break;
	case WAIT_FOR_ARM_TO_ZERO:
		Serial.println(digitalRead(LIMIT_SWITCH));
		if (digitalRead(LIMIT_SWITCH) == 0) {
			motor3->setVelocityDegreesPerSecond(0);
			motor3->overrideCurrentPosition(0);

			status = nextStatus;
		}
		break;
	case WAIT_FOR_MOTORS_TO_FINNISH:
		if (motor1->isInterpolationDone() && motor2->isInterpolationDone()
				&& motor3->isInterpolationDone()) {
			status = nextStatus;
		}
		break;
	case ALIGN_ON_LINE:
		//Serial.println("Aligning");
		AlignOnLine(Halting);
		break;

	case Halting:
		// save state and enter safe mode
		Serial.println("Halting State machine");
		//digitalWrite(H_BRIDGE_ENABLE, 0); // Disable and idle motors
		motor3->stop();
		motor2->stop();
		motor1->stop();

		status = Halt;
		break;
	case Halt:
		// in safe mode
		break;

	}
}

/**
 * This is run fast and should return fast
 *
 * You call the PIDMotor's loop function. This will update the whole motor control system
 * This will read from the concoder and write to the motors and handle the hardware interface.
 * Instead of allowing this to be called by the controller you may call these from a timer interrupt.
 */
void StudentsRobot::pidLoop() {
	motor1->loop();
	motor2->loop();
	motor3->loop();
}
/**
 * Approve
 *
 * @param buffer A buffer of floats containing nothing
 *
 * the is the event of the Approve button pressed in the GUI
 *
 * This function is called via coms.server() in:
 * @see RobotControlCenter::fastLoop
 */
void StudentsRobot::Approve(float * buffer) {
	// approve the procession to new state
	Serial.println("StudentsRobot::Approve");

	if (myCommandsStatus == Waiting_for_approval_to_pickup) {
		myCommandsStatus = Picking_up;
	} else if (myCommandsStatus == Waiting_for_approval_to_dropoff) {
		myCommandsStatus = Dropping_off;
	} else {
		myCommandsStatus = Ready_for_new_task;
	}
}
/**
 * ClearFaults
 *
 * @param buffer A buffer of floats containing nothing
 *
 * this represents the event of the clear faults button press in the gui
 *
 * This function is called via coms.server() in:
 * @see RobotControlCenter::fastLoop
 */
void StudentsRobot::ClearFaults(float * buffer) {
	// clear the faults somehow
	Serial.println("StudentsRobot::ClearFaults");
	myCommandsStatus = Ready_for_new_task;
	status = Running;
}

/**
 * EStop
 *
 * @param buffer A buffer of floats containing nothing
 *
 * this represents the event of the EStop button press in the gui
 *
 * This is called whrn the estop in the GUI is pressed
 * All motors shuld hault and lock in position
 * Motors should not go idle and drop the plate
 *
 * This function is called via coms.server() in:
 * @see RobotControlCenter::fastLoop
 */
void StudentsRobot::EStop(float * buffer) {
	// Stop the robot immediatly
	Serial.println("StudentsRobot::EStop");
	myCommandsStatus = Fault_E_Stop_pressed;
	status = Halting;

}
/**
 * PickOrder
 *
 * @param buffer A buffer of floats containing the pick order data
 *
 * buffer[0]  is the material, aluminum or plastic.
 * buffer[1]  is the drop off angle 25 or 45 degrees
 * buffer[2]  is the drop off position 1, or 2
 *
 * This function is called via coms.server() in:
 * @see RobotControlCenter::fastLoop
 */
void StudentsRobot::PickOrder(float * buffer) {
	pickupMaterial = buffer[0];
	dropoffAngle = buffer[1];
	dropoffPosition = buffer[2];
	Serial.println(
			"StudentsRobot::PickOrder Recived from : " + String(pickupMaterial)
					+ " " + String(dropoffAngle) + " "
					+ String(dropoffPosition));
	myCommandsStatus = Heading_to_pickup;
}

void StudentsRobot::FollowLine(int dir) { //1 forward, -1 backwards
	if (analogRead(MIDDLE_LEFT) >= lineThreashold
			&& analogRead(MIDDLE_RIGHT) >= lineThreashold) {
		motor1->setVelocityDegreesPerSecond(-260 * speedAdjustment * dir);
		motor2->setVelocityDegreesPerSecond(260 * speedAdjustment * dir);
		//Serial.println("Driving Forward");
		offSide = Center;
	} else if ((analogRead(MIDDLE_LEFT) < lineThreashold || offSide == Left)
			&& offSide != Right) {
		//Serial.println("Turning right");
		motor1->setVelocityDegreesPerSecond(-200 * speedAdjustment * dir);
		motor2->setVelocityDegreesPerSecond(260 * speedAdjustment * dir);
		offSide = Left;

	} else if (analogRead(MIDDLE_RIGHT) < lineThreashold || offSide == Right) {
		//Serial.println("Turning left");
		motor1->setVelocityDegreesPerSecond(-260 * speedAdjustment * dir);
		motor2->setVelocityDegreesPerSecond(200 * speedAdjustment * dir);
		offSide = Right;
	}
}
void StudentsRobot::FollowLineBackwards() { //1 forward, -1 backwards
	if (analogRead(MIDDLE_LEFT) >= lineThreashold
			&& analogRead(MIDDLE_RIGHT) >= lineThreashold) {
		motor1->setVelocityDegreesPerSecond(220);
		motor2->setVelocityDegreesPerSecond(-220);
		//Serial.println("Driving Forward");
		offSide = Center;
	} else if ((analogRead(MIDDLE_LEFT) < lineThreashold || offSide == Left)
			&& offSide != Right) {
		//Serial.println("Turning right");
		motor1->setVelocityDegreesPerSecond(-160);
		motor2->setVelocityDegreesPerSecond(220);
		offSide = Left;

	} else if (analogRead(MIDDLE_RIGHT) < lineThreashold || offSide == Right) {
		//Serial.println("Turning left");
		motor1->setVelocityDegreesPerSecond(-220);
		motor2->setVelocityDegreesPerSecond(160);
		offSide = Right;
	}
}
bool StudentsRobot::TurnTargetDeg(int smallTurn, int fullTurn, int turnDir,
		int time, RobotStateMachine currentState) {
	switch (turnState) {
	case 0:
		chassis->resetMotorPos();
		chassis->turnDegrees(smallTurn * turnDir, time);
		status = WAIT_FOR_MOTORS_TO_FINNISH;
		nextStatus = currentState;
		turnState = 1;
		break;
	case 1:
		if (analogRead(MIDDLE_LEFT) > lineThreashold
				&& analogRead(MIDDLE_RIGHT) > lineThreashold) {
			motor1->setVelocityDegreesPerSecond(0);
			motor2->setVelocityDegreesPerSecond(0);
			turnState = 0;
			return true;
		} else {
			motor1->setVelocityDegreesPerSecond(250 * turnDir);
			motor2->setVelocityDegreesPerSecond(250 * turnDir);
		}
		break;
	}
	return false;
}
void StudentsRobot::DriveToTargetLine(RobotStateMachine currentState,
		int endState) {
	if (lineNum > 0) {
		Serial.print("lineNum: ");
		Serial.println(lineNum);

		switch (lineState) {
		case 0:
			Serial.println("lineState 0");

			chassis->resetMotorPos();
			startPos = (motor1->getAngleDegrees() + motor2->getAngleDegrees())
					/ 2;
			distanceToTravel = 25;
			direction = 1;
			status = LINE_FOLLOW_FOR_DISTANCE;
			nextStatus = currentState;
			lineState = 1;
			break;

		case 1:
			Serial.println("lineState 1");

			lineNum--;
			status = LINE_FOLLOW_UNTIL_BLACK_LINE;
			nextStatus = currentState;
			lineState = 0;
			break;

		}
	} else {
		lineState = 0;
		roofState = endState;
	}
}

void StudentsRobot::DriveToTargetLineAndTurn90(RobotStateMachine currentState,
		RobotStateMachine endState) {
	switch (lineState) {
	case 0:
		Serial.println("lineState 0");

		status = LINE_FOLLOW_UNTIL_BLACK_LINE;
		nextStatus = currentState;
		lineState = 1;
		break;
	case 1:
		Serial.println("lineState 1");

		chassis->resetMotorPos();
		startPos = (motor1->getAngleDegrees() + motor2->getAngleDegrees()) / 2;
		distanceToTravel = 100;
		direction = 1;
		status = LINE_FOLLOW_FOR_DISTANCE;
		nextStatus = currentState;
		lineState = 2;
		break;
	case 2:
		Serial.println("lineState 3");

		chassis->resetMotorPos();
		chassis->turnDegrees(90, 3000);
		status = WAIT_FOR_MOTORS_TO_FINNISH;
		status = endState;
		lineState = 0;
		break;
	}

}
void StudentsRobot::AlignOnLine(RobotStateMachine endState) {
	//Serial.println("linestate"+String(lineState));
	switch (lineState) {
	case 0:
		Serial.println("Forward 100");
		chassis->resetMotorPos();
		startPos = 0;
		distanceToTravel = 40;
		direction = 1;
		status = LINE_FOLLOW_FOR_DISTANCE;
		nextStatus = ALIGN_ON_LINE;
		lineState = 1;
		Serial.println(status);
		break;
	case 1:

		if (analogRead(MIDDLE_LEFT) >= lineThreashold
				&& analogRead(MIDDLE_RIGHT) >= lineThreashold
				&& analogRead(FAR_LEFT) >= lineThreashold
				&& analogRead(FAR_RIGHT) >= lineThreashold) {
			Serial.println("Stop");
			motor1->setVelocityDegreesPerSecond(0);
			motor2->setVelocityDegreesPerSecond(0);

			status = endState;
			break;
		} else if (analogRead(FAR_LEFT) >= lineThreashold) {
			Serial.println("Left hit");
			motor2->setVelocityDegreesPerSecond(0);
			motor1->setVelocityDegreesPerSecond(180);
			break;
		} else if (analogRead(FAR_RIGHT) >= lineThreashold) {
			Serial.println("Right Hit");
			motor1->setVelocityDegreesPerSecond(0);
			motor2->setVelocityDegreesPerSecond(-180);
			break;
		}

		else {
			//Serial.println("LineFollowBackwards");
			FollowLineBackwards();
			break;
		}

		break;
	}

}

bool StudentsRobot::DriveToRoof(int roofSide, int roofLine,
		RobotStateMachine currentState, RobotStateMachine endState) {
	switch (roofState) {
	case 0:
		/*chassis->resetMotorPos();
		 chassis->turnDegrees((180 + turnAdjustment * 10) * turn180Dir, 6000);
		 status = WAIT_FOR_MOTORS_TO_FINNISH; */

		if (TurnTargetDeg(120, 180, turn180Dir, 2500, currentState)) {
			nextStatus = currentState;
			roofState = 1;
			lineNum = roofSide;
		}
		break;
	case 1:
		DriveToTargetLine(currentState, 2);
		break;
	case 2:
		Serial.println("lineState 1");

		chassis->resetMotorPos();
		startPos = (motor1->getAngleDegrees() + motor2->getAngleDegrees()) / 2;
		distanceToTravel = 100;
		direction = 1;
		status = LINE_FOLLOW_FOR_DISTANCE;
		nextStatus = currentState;
		roofState = 3;
		turnDirection = -turnDirection;
		break;
	case 3:

		//chassis->resetMotorPos();
		//chassis->turnDegrees(90 * turnDirection, 4000);
		//status = WAIT_FOR_MOTORS_TO_FINNISH;

		if (TurnTargetDeg(70, 90, turnDirection, 1000, currentState)) {
			nextStatus = currentState;
			roofState = 4;
			lineNum = roofLine;
		}

		break;
	case 4:
		DriveToTargetLine(currentState, 5);
		break;
	case 5:
		roofState = 0;
		status = endState;
		return true;
	}
	return false;
}

int StudentsRobot::checkBoardSide(int angle) {
	if (angle == 45) {
		turnDirection = -1;
		return 2;
	} else if (angle == 25) {
		turnDirection = 1;
		return 2;
	}
	return -1;
}

void StudentsRobot::updateRoofSideDir(int angle, int side) {
	if ((angle == 25 && side == 1) || (angle == 45 && side == 2))
		turn180Dir = -1;
	else
		turn180Dir = 1;
}

int StudentsRobot::getOppositeRoofSide(int side) {
	if (side == 1) {
		return 2;
	} else if (side == 2) {
		return 1;
	}
	return -1;
}

//TODO arm lift method
