package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerData;
import ca.mcgill.ecse211.ultrasonic.UltrasonicController;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is used to drive the robot on the demo floor while avoiding
 * obstacles
 */

// navigation class should not extends thread
public class NavigationAvoid extends Thread implements UltrasonicController {
	private static final int FORWARD_SPEED = 100;
	private static final int ROTATE_SPEED = 50;
	private static final double TILE_SIZE = 30.48;
	private int ODOMETER_PERIOD = 25;

	private Odometer odometer = Lab3.odometer;
	private EV3LargeRegulatedMotor leftMotor = Lab3.leftMotor;
	private EV3LargeRegulatedMotor rightMotor = Lab3.rightMotor;
	private double track = Lab3.TRACK;
	private double wr = Lab3.WHEEL_RAD;
	private static double OFF_CONST=1.1;
	private double xCurrent;
	private double yCurrent;
	private double thetaCurrent;
	boolean isFinished = false;
	boolean isAvoided = false;
	private int distance;
	private int xDirCompensation = -10000;

	NavigationAvoid() {
		this.isFinished = false;
	}

	// thread behavior
	public void run() {
		long updateStart, updateEnd;
		while (true) {
			updateStart = System.currentTimeMillis();
			// the thread fetches current x, y and theta
			getCurrentPos();

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
	}

	public int halfWall(int distance, int angle) {
		int halfWall = (int) (Math.sin(Math.toRadians(angle)) * distance);
		return halfWall;
	}

	/*
	 * this method drives the robot to a position specified as x and y
	 */
	public void travelTo(double xDest, double yDest) {
		// convert coordinate to length
		xDest = xDest * TILE_SIZE;
		yDest = yDest * TILE_SIZE;
		double dX = xDest - xCurrent;
		double dY = yDest - yCurrent;
		double linearDistance = getLinearDistance(dX, dY);
		/*
		 * Calculate the angle using tangent
		 */
		double angularDistance = Math.atan2(dX, dY) - Math.toRadians(thetaCurrent);

		/*
		 * If needs to turn more than 180 degree turn the other way instead
		 */
		if (angularDistance > Math.PI) {
			// the angular disance will be negaive
			angularDistance = angularDistance - 2 * Math.PI;
		} else if (angularDistance < (-Math.PI)) {
			// if needs to turn more than -180 degree
			angularDistance = angularDistance + 2 * Math.PI;
		}

		// synchronous turn to the desired degree
		turnTo(angularDistance/OFF_CONST);
		// after turn we detect distance
		if (distance < linearDistance) {
			// we will move until the wall
			Sound.beep();
			moveForward(distance - 5);
			xDirCompensation = distance - 5;
			System.out.println("Dist: " + distance);
			System.out.println("Comp: " + xDirCompensation);
			avoidObstacle();
		}
		leftMotor.setAcceleration(300);
		rightMotor.setAcceleration(300);
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		isFinished = true;
		
		if(xDirCompensation!=-10000) {
			System.out.println("Compensate!");
			leftMotor.rotate(convertDistance(wr, linearDistance - xDirCompensation-45), true);
			rightMotor.rotate(convertDistance(wr, linearDistance - xDirCompensation-45), false);
			//reset
			xDirCompensation=-10000;
		}else {
			// else moves as normal
			leftMotor.rotate(convertDistance(wr, linearDistance), true);
			rightMotor.rotate(convertDistance(wr, linearDistance), false);
		}
		
		isFinished = false;
	}

	/**
	 * this method supplements the travelTo method it turns the robot in place into
	 * the right direction before moving
	 */
	public void turnTo(double thetaDest) {
		/*
		 * Set acceleration to 3000 the default speed is 6000 this gives a smooth
		 * acceleration
		 */
		leftMotor.setAcceleration(300);
		rightMotor.setAcceleration(300);
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		thetaDest = Math.toDegrees(thetaDest);
		int rotationAngle = convertAngle(wr, track, thetaDest);
		/*
		 * Initialize a synchronous action to avoid slip
		 */
		leftMotor.rotate(rotationAngle, true);
		rightMotor.rotate(-rotationAngle);
	}

	/**
	 * this method is only used to avoid obstacles it moves the robot forward in
	 * current direction 10cm
	 */
	public void moveForward(double linearDist) {
		/*
		 * Set acceleration to 3000 the default speed is 6000 this gives a smooth
		 * acceleration
		 */
		leftMotor.setAcceleration(300);
		rightMotor.setAcceleration(300);
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(wr, linearDist), true);
		rightMotor.rotate(convertDistance(wr, linearDist), false);
	}

	/**
	 * this method fetches the x and y position from odometer
	 */
	private void getCurrentPos() {
		/*
		 * changed the visibility of lock in odometer class to public here the method
		 * locks up the thread before accessing data
		 */
		synchronized (OdometerData.lock) {
			this.thetaCurrent = odometer.getXYT()[2];
			this.xCurrent = odometer.getXYT()[0];
			this.yCurrent = odometer.getXYT()[1];
		}
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	private static double getLinearDistance(double x, double y) {
		return Math.hypot(x, y);
	}

	/*
	 * This class helps return from the thread
	 */
	public boolean isFinished() {
		return isFinished;
	}

	public void avoidObstacle() {
		turnTo(Math.toRadians(90));
		moveForward(25);
		turnTo(-Math.toRadians(90));
		moveForward(40);
		turnTo(-Math.toRadians(90));
		moveForward(25);
		turnTo(Math.toRadians(90));
		System.out.println("Done avoid");
		// we set the gate back to normal
		isAvoided = true;
	}

	@Override
	public void processUSData(int distance) {
		this.distance = distance;
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}