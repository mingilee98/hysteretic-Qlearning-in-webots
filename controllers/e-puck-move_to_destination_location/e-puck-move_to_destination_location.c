/*
 * Copyright 2021 Albert Alfrianta
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * 
 * Created on: 2021-08, Bogor, Indonesia
 * 
 * Contact: albert.alfrianta@gmail.com or https://www.linkedin.com/in/albert-alfrianta/
 * 
 * Description:
 * 	Please read the header file for the method explanations.
 */


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/compass.h>

#define GPS_SAMPLING_PERIOD 1 //in ms
WbDeviceTag gps;

#define COMPASS_SAMPLING_PERIOD 1 //in ms
WbDeviceTag compass;


/* 
 * in real world, there must be gps noise so the coordinate is not accurate
 * so to check if two coordinate are equal, we cannot check with formula: coordinate1==coordinate2
 * we must use a threshold accuracy
 * */
#define COORDINATE_MATCHING_ACCURACY 0.01 //in meter

/* 
 * in real world, there must be compass noise so the heading is not accurate
 * so to check if two heading are equal, we cannot check with formula: heading1==heading2
 * we must use a threshold accuracy
 * */
#define THETA_MATCHING_ACCURACY 1 //in degrees


double * cartesianConvertVec3fToCartesianVec2f(const double coordinate3f[3]) {
    double *coordinate2f = malloc(2);
    coordinate2f[0] = coordinate3f[0];
    coordinate2f[1] = -coordinate3f[2];
    return coordinate2f;
}

double * cartesianConvertCartesianVec2fToVec3f(const double coordinate2f[2]) {
    double *coordinate3f = malloc(3);
    coordinate3f[0] = coordinate2f[0];
    coordinate3f[2] = -coordinate2f[1];
    return coordinate3f;
}

double cartesianConvertCompassBearingToHeading(double heading) {
    /* 
	 * in webots, heading increasement is rotate in clockwise
	 * in cartesian, heading increasement is rotate in counterclockwise
	 * so headingInCartesian = 360-headingInWebots
	 * */
    heading = 360-heading;
    
    /* 
	 * in webots, heading is 0 if robot faced to y+ axis
	 * in cartesian, heading is 0 if robot face to x+ axis
	 * so headingInCartesian = headingInWebots+90
	 * */
    heading = heading + 90;
    if (heading > 360.0)
        heading = heading - 360.0;

    return heading;
}

bool cartesianIsCoordinateEqual(const double coordinate1[2], const double coordinate2[2])
{
    if (fabs(coordinate1[0]-coordinate2[0]) < COORDINATE_MATCHING_ACCURACY &&
            fabs(coordinate1[1]-coordinate2[1]) < COORDINATE_MATCHING_ACCURACY) {
        return true;
    }
    else {
        return false;
    }
}

bool cartesianIsCoordinateVectorEqual(const double coordinateVector1, const double coordinateVector2)
{
    if (fabs(coordinateVector1-coordinateVector2) < COORDINATE_MATCHING_ACCURACY) {
        return true;
    }
    else {
        return false;
    }
}

bool cartesianIsThetaEqual(const double theta, const double theta2)
{
    if (fabs(theta - theta2) < THETA_MATCHING_ACCURACY)
    {
        return true;
    } else
    {
        return false;
    }
}

double cartesianCalcDestinationThetaInDegrees(const double currentCoordinate[2], const double destinationCoordinate[2]) {
    return atan2(destinationCoordinate[1] - currentCoordinate[1], destinationCoordinate[0] - currentCoordinate[0]) * 180 / M_PI;
}

double cartesianCalcThetaDot(double heading, double destinationTheta) {
    double theta_dot = destinationTheta - heading;

    if (theta_dot > 180)
        theta_dot = -(360-theta_dot);
    else if (theta_dot < -180)
        theta_dot = (360+theta_dot);

    return theta_dot;
}

double cartesianCalcRotatedThetaByThetaDot(double theta, double theta_dot)
{
	if (theta_dot == 0)
	{
		return theta;
	}

	theta += theta_dot;

	/*
	 * if theta negative or more than 360, then convert it to normal degree
	 * */
	
	if (theta < 0)
	{
		theta = theta + 360;
	}
	else if (theta >= 360)
	{
		theta = theta - 360;
	}
	
	return theta;
}

double cartesianCalcDistance(const double currentCoordinate[2], const double destinationCoordinate[2]) {
    return sqrt(pow(destinationCoordinate[0]-currentCoordinate[0], 2) + pow(destinationCoordinate[1]-currentCoordinate[1], 2));
}
#define MAX_SPEED 6.28 //angular speed in rad/s
static WbDeviceTag left_motor, right_motor;

void motorControllerInit(int time_step)
{
    // get a handler to the motors and set target position to infinity (speed control).
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);
}

void motorStop()
{
    wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 0);
}

void motorMoveForward()
{
    wb_motor_set_velocity(left_motor, MAX_SPEED);
    wb_motor_set_velocity(right_motor, MAX_SPEED);
}

void motorRotateLeft()
{
    wb_motor_set_velocity(left_motor, -MAX_SPEED);
    wb_motor_set_velocity(right_motor, MAX_SPEED);
}

void motorRotateRight()
{
    wb_motor_set_velocity(left_motor, MAX_SPEED);
    wb_motor_set_velocity(right_motor, -MAX_SPEED);
}

void positioningControllerInit(int time_step)
{
    // get a handler to the gps
    gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, GPS_SAMPLING_PERIOD);
	
    // get a handler to the compass
	compass = wb_robot_get_device("compass");
    wb_compass_enable(compass, COMPASS_SAMPLING_PERIOD);
}

/*
 * copied from: https://cyberbotics.com/doc/reference/compass
 * */
double getRobotBearing()
{
    /* calculate bearing angle in degrees */
    const double *north = wb_compass_get_values(compass);
    double rad = atan2(north[0], north[2]);
    double bearing = (rad - 1.5708) / M_PI * 180.0;
    if (bearing < 0.0) {
        bearing = bearing + 360.0;
	}
	
    return bearing;
}

double * positioningControllerGetRobotCoordinate()
{
	return cartesianConvertVec3fToCartesianVec2f(wb_gps_get_values(gps));
}

double positioningControllerGetRobotHeading()
{
	return cartesianConvertCompassBearingToHeading(getRobotBearing());
}

double positioningControllerCalcDistanceToDestination(const double destinationCoordinate[2])
{
	const double *currentCoordinate = positioningControllerGetRobotCoordinate();
	return cartesianCalcDistance(currentCoordinate, destinationCoordinate);
}

double positioningControllerCalcThetaDotToDestination(const double destinationCoordinate[2])
{
	const double *currentCoordinate = positioningControllerGetRobotCoordinate();
	double robotHeading = positioningControllerGetRobotHeading();
	double destinationTheta = cartesianCalcDestinationThetaInDegrees(currentCoordinate, destinationCoordinate);
	return cartesianCalcThetaDot(robotHeading, destinationTheta);
}
// tangensial/linear speed in m/s. 
// Tangensial speed = angular speed * wheel radius 
// Tangensial speed = 6.28 rad * 2.05 cm = 0.12874 m/s
#define TANGENSIAL_SPEED 0.12874

// Speed of robot to spinning in place (in cycles per second)
// 1 cycle = 360 degrees. 
// Robo t rotational speed = tangensial speed / (phi * axle length) 
// note: axle length is distance between wheels
// Robot rotational speed = 0.12874 / (phi*0.052) = 0.787744755
#define ROBOT_ROTATIONAL_SPEED 0.772881647

// Speed of robot to spinning in place (in degrees per second)
// Robot angular speed in degrees = robot rotational speed * 360 
// Robot angular speed in degrees = 0.787744755*360 = 283.588111888 
#define ROBOT_ANGULAR_SPEED_IN_DEGREES 278.237392796

int time_step;

/*
Method to get simulator time step.
Webots have simulator time step. 
The basic time step is the time step increment used by Webots to advance the virtual time and perform physics simulation.
*/
int getTimeStep()
{
    static int time_step = -1;
    if (time_step == -1)
        time_step = (int)wb_robot_get_basic_time_step();
    return time_step;
}

/*
This command is to perform simulation steps. 
This needed for the controller time step. 
The controller time step is the time increment of time executed at each iteration of the control loop of a controller. 
We must call this to synchronize our program and the simulator condition. 
It will return -1 if the simulation is stopped. 
If we not call this command, the robot will do nothing. 
For example the wb_motor_set_velocity(left_motor, MAX_SPEED) only set the motor speed value. 
So we need to call and looping the wb_robot_step(time_step) command to make the robot move.
*/
void step()
{
    if (wb_robot_step(time_step) == -1)
    {
        wb_robot_cleanup();
        exit(EXIT_SUCCESS);
    }
}

static void init()
{
	time_step = getTimeStep();
	
	motorControllerInit(time_step);
	
	positioningControllerInit(time_step);
	
    step();
}

void rotateHeading(const double thetaDot)
{
	// if thetaDot is zero
	if (!cartesianIsThetaEqual(thetaDot, 0))
	{
		// the duration required for the robot to rotate the body by the specified thetaDot
		double duration = abs(thetaDot) / ROBOT_ANGULAR_SPEED_IN_DEGREES;
		printf("duration to face the destination: %.5f\n", duration);

		// if thetaDot > 0, robot will rotate to left
		if (thetaDot > 0)
		{
			// set robot motor to rotate left
			motorRotateLeft();
		}
		// if thetaDot < 0, robot will rotate to right
		else if (thetaDot < 0)
		{
			// set robot motor to rotate right
			motorRotateRight();
		}

		// run the simulator
		double start_time = wb_robot_get_time();
		do
		{
			step();
		}
		while (wb_robot_get_time() < start_time + duration);
	}
}

void moveForward(double distance)
{
	// the duration required for the robot to move by the specified distance
	double duration = distance / TANGENSIAL_SPEED;
	printf("duration to reach target location: %.5f\n", duration);

	// set robot motor to move forward
	motorMoveForward();

	// run the simulator
	double start_time = wb_robot_get_time();
	do
	{
		step();	
	}
	while (wb_robot_get_time() < start_time + duration);
	
	// stop the motor
    motorStop();
	step();
}

void moveToDestination(const double destinationCoordinate[2])
{
	double * currentCoordinate = positioningControllerGetRobotCoordinate();
	printf("Initial Coordinate: %.5f %.5f\n", currentCoordinate[0], currentCoordinate[1]);

	printf("Destination Coordinate: %.5f %.5f\n", destinationCoordinate[0], destinationCoordinate[1]);
	
	// if the robot is already at the destination location
	if (cartesianIsCoordinateEqual(positioningControllerGetRobotCoordinate(), destinationCoordinate))
	{
		printf("Robot is already at the destination location\n");
		return;
	}

	// thetaDot is the degree of rotation needed by the robot to face the destination
	// thetaDot is zero if robot is facing the destination
	double thetaDotToDestination = positioningControllerCalcThetaDotToDestination(destinationCoordinate);
	printf("thetaDotToDestination: %.5f\n", thetaDotToDestination);

	rotateHeading(thetaDotToDestination);

	// the distance needed for the robot to reach its destination
	double distanceToDestination = positioningControllerCalcDistanceToDestination(destinationCoordinate);
	printf("distanceToDestination: %.5f\n", distanceToDestination);
	
	moveForward(distanceToDestination);

	currentCoordinate = positioningControllerGetRobotCoordinate();
	printf("Stop Coordinate: %.5f %.5f\n", currentCoordinate[0], currentCoordinate[1]);
}

int main(int argc, char **argv)
{
    wb_robot_init();

    init();
	
    const double destinationCoordinate[2] = {0.35, -0.35};
    
    moveToDestination(destinationCoordinate);
    
	wb_robot_cleanup();
    return EXIT_SUCCESS;
}