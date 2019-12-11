/*
 * DrivingChassis.cpp
 *
 *  Created on: Jan 31, 2019
 *      Author: hephaestus
 */

#include "DriveChassis.h"

/**
 * Compute a delta in wheel angle to traverse a specific distance
 *
 * arc length	=	2*	π*	R*	(C/360)
 *
 * C  is the central angle of the arc in degrees
 * R  is the radius of the arc
 * π  is Pi
 *
 * @param distance a distance for this wheel to travel in MM
 * @return the wheel angle delta in degrees
 */
float DrivingChassis::distanceToWheelAngle(float distance) {
	return distance / (2 * pi * mywheelRadiusMM) * 360;
}

/**
 * Compute the arch length distance the wheel needs to travel through to rotate the base
 * through a given number of degrees.
 *
 * arc length	=	2*	π*	R*	(C/360)
 *
 * C  is the central angle of the arc in degrees
 * R  is the radius of the arc
 * π  is Pi
 *
 * @param angle is the angle the base should be rotated by
 * @return is the linear distance the wheel needs to travel given the this CHassis's wheel track
 */
float DrivingChassis::chassisRotationToWheelDistance(float angle) {
	return angle / 360 * (mywheelTrackMM * pi);
}

DrivingChassis::~DrivingChassis() {
	// do nothing
}

/**
 * DrivingChassis encapsulates a 2 wheel differential steered chassis that drives around
 *
 * @param left the left motor
 * @param right the right motor
 * @param wheelTrackMM is the measurment in milimeters of the distance from the left wheel contact point to the right wheels contact point
 * @param wheelRadiusMM is the measurment in milimeters of the radius of the wheels
 */
DrivingChassis::DrivingChassis(PIDMotor * left, PIDMotor * right,
		float wheelTrackMM, float wheelRadiusMM) {
	myleft = left;
	myright = right;
	mywheelTrackMM = wheelTrackMM;
	mywheelRadiusMM = wheelRadiusMM;
}

/**
 * Start a drive forward action
 *
 * @param mmDistanceFromCurrent is the distance the mobile base should drive forward
 * @param msDuration is the time in miliseconds that the drive action should take
 *
 * @note this function is fast-return and should not block
 */
void DrivingChassis::driveForward(float mmDistanceFromCurrent, int msDuration) {

	float deg = this->distanceToWheelAngle(mmDistanceFromCurrent);
	myleft->startInterpolationDegrees(deg, msDuration, SIN);
	myright->startInterpolationDegrees(-deg, msDuration, SIN);

}

/**
 * Start a turn action
 *
 * This action rotates the robot around the center line made up by the contact points of the left and right wheels.
 * Positive angles should rotate to the left
 *
 * This rotation is a positive rotation about the Z axis of the robot.
 *
 * @param degreesToRotateBase the number of degrees to rotate
 * @param msDuration is the time in miliseconds that the drive action should take
 *
 *  @note this function is fast-return and should not block
 */
void DrivingChassis::turnDegrees(float degreesToRotateBase, int msDuration) {
	float motorDeg = this->distanceToWheelAngle(
			this->chassisRotationToWheelDistance(degreesToRotateBase));

	myleft->startInterpolationDegrees(motorDeg, msDuration, SIN);
	myright->startInterpolationDegrees(motorDeg, msDuration, SIN);
}

/**
 * Check to see if the chassis is performing an action
 *
 * @return false is the chassis is driving, true is the chassis msDuration has elapsed
 *
 *  @note this function is fast-return and should not block
 */
bool DrivingChassis::isChassisDoneDriving() {
	if (myleft->isInterpolationDone() && myright->isInterpolationDone())
		return true;
	return false;
}

float DrivingChassis::distanceTraveled(float angle) {
	float angleDif = (myleft->getAngleDegrees() -myright->getAngleDegrees()) / 2 - angle;
	float distance = angleDif * 2 * pi * mywheelRadiusMM / 360;


	return -distance;
}

void DrivingChassis::resetMotorPos() {
	myleft->overrideCurrentPosition(0);
	myright->overrideCurrentPosition(0);
}
