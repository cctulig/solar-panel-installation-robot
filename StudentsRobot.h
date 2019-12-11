/*
 * StudentsRobot.h
 *
 *  Created on: Dec 28, 2018
 *      Author: hephaestus
 */

#ifndef STUDENTSROBOT_H_
#define STUDENTSROBOT_H_
#include "config.h"
#include <Arduino.h>
#include "src/pid/ServoEncoderPIDMotor.h"
#include "src/pid/HBridgeEncoderPIDMotor.h"
#include "src/pid/ServoAnalogPIDMotor.h"
#include <ESP32Servo.h>

#include "DriveChassis.h"

/**
 * @enum RobotStateMachine
 * These are sample values for a sample state machine.
 * Feel free to add ot remove values from here
 */
enum RobotStateMachine {
	StartupRobot = 0,
	StartRunning = 1,
	Running = 2,
	GoToFirstPickup = 100, //TODO fix state numbers
	Pickup_Plate = 3,
	DriveToRoofTest = 4,
	WaitForDropoff = 5,
	DropoffPlate = 6,
	GoToSafe = 7,
	DriveToPickupPlatePt1 = 8,
	DriveToPickupPlatePt2 = 9,
	LineUpToPlate = 200, //TODO
	WaitForPickup = 10,
	GoToSafe2 = 11,
	DriveToPlateDropoff1 = 12,
	DriveToPlateDropoff2 = 13,
	LowerPlate = 15,
	SwitchSidesPt1 = 16,
	SwitchSidesPt2 = 17,
	Halting = 20,
	Halt = 21,
	WAIT_FOR_MOTORS_TO_FINNISH = 22,
	WAIT_FOR_TIME = 23,
	LINE_FOLLOW_UNTIL_BLACK_LINE = 24,
	LINE_FOLLOW_FOR_DISTANCE = 25,
	ALIGN_ON_LINE = 26,
	LINE_FOLLOW_FOR_DISTANCE_BACKWARDS = 27,
	WAIT_FOR_ARM_TO_ZERO = 28,
	LineUpToPlate2 = 29


};
/*
 * @enum OffCourseSide
 * Records the side that the robot went off of last
 */
enum OffCourseSide {
	Center = 0, Left = 1, Right = 2

};
/**
 * @enum ComStackStatusState
 * These are values for the communications stack
 * Don't add any more or change these. This is how you tell the GUI
 * what state your robot is in.
 */
enum ComStackStatusState {
	Ready_for_new_task = 0,
	Heading_to_pickup = 1,
	Waiting_for_approval_to_pickup = 2,
	Picking_up = 3,
	Heading_to_Dropoff = 4,
	Waiting_for_approval_to_dropoff = 5,
	Dropping_off = 6,
	Heading_to_safe_zone = 7,
	Fault_failed_pickup = 8,
	Fault_failed_dropoff = 9,
	Fault_excessive_load = 10,
	Fault_obstructed_path = 11,
	Fault_E_Stop_pressed = 12
};
/**
 * @class StudentsRobot
 */
class StudentsRobot {
private:
	PIDMotor * motor1;
	PIDMotor * motor2;
	PIDMotor * motor3;
	Servo * servo;
	float lsensorVal = 0;
	float rsensorVal = 0;
	long nextTime = 0;
	long startTime = 0;
	RobotStateMachine nextStatus = StartupRobot;
	OffCourseSide offSide = Center;
	DrivingChassis * chassis;
	int lineThreashold = 2000;
	float startPos = 0;
	float distanceToTravel = 0;
	int direction = 1;
	int lineNum = 0;
	int lineState = 0;
	int roofState = 0;
	float pickupMaterial;
	float dropoffAngle;
	float dropoffPosition;
	int turnDirection = 1;
	int turn180Dir = 1;
	int turnAdjustment =0;
	float speedAdjustment = 1;
	int roofLines = 0;
	int liftarmdeg= 100;
	int totalTrips=0;
	int turnState = 0;

public:
	/**
	 * Constructor for StudentsRobot
	 *
	 * attach the 4 actuators
	 *
	 * these are the 4 actuators you need to use for this lab
	 * all 4 must be attached at this time
	 * DO NOT reuse pins or fail to attach any of the objects
	 *
	 */
	StudentsRobot(PIDMotor * motor1, PIDMotor * motor2, PIDMotor * motor3,
			Servo * servo);
	/**
	 * Command status
	 *
	 * this is sent upstream to the Java GUI to notify it of current state
	 */
	ComStackStatusState myCommandsStatus = Ready_for_new_task;
	/**
	 * This is internal data representing the runtime status of the robot for use in its state machine
	 */
	RobotStateMachine status = StartupRobot;
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
	void Approve(float * buffer);
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
	void ClearFaults(float * buffer);
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
	void EStop(float * buffer);
	/**
	 * PickOrder
	 *
	 * @param buffer A buffer of floats containing the pick order data
	 *
	 * buffer[0]  is the material, aluminum or plastic.
	 *
	 * buffer[1]  is the drop off angle 25 or 45 degrees
	 *
	 * buffer[2]  is the drop off position 1, or 2
	 *
	 * This function is called via coms.server() in:
	 * @see RobotControlCenter::fastLoop
	 */
	void PickOrder(float * buffer);

	/**
	 * pidLoop This functoion is called to let the StudentsRobot controll the running of the PID loop functions
	 *
	 * The loop function on all motors needs to be run when this function is called and return fast
	 */
	void pidLoop();
	/**
	 * updateStateMachine use the stub state machine as a starting point.
	 *
	 * the students state machine can be updated with this function
	 */
	void updateStateMachine();

	void FollowLine(int dir);

	void FollowLineBackwards();

	void DriveToTargetLine(RobotStateMachine currentState, int endState);

	void DriveToTargetLineAndTurn90(RobotStateMachine currentState,
			RobotStateMachine endState);

	bool DriveToRoof(int roofSide, int roofLine, RobotStateMachine currentState,
			RobotStateMachine endState);
	void AlignOnLine(RobotStateMachine endState);

	int checkBoardSide(int angle);

	void updateRoofSideDir(int angle, int side);

	int getOppositeRoofSide(int side);

	bool TurnTargetDeg(int smallTurn, int fullTurn, int turnDir,int time, RobotStateMachine currentState);
};

#endif /* STUDENTSROBOT_H_ */
