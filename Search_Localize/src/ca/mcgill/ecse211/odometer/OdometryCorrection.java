package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.model.Robot;
import lejos.hardware.Sound;

public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private static final int BLACK_THRESHOLD = 300;
	private static double offsetOrigin; // this is how far the robot is from the (0,0) in y
	private double offset = 0;
	public static boolean isRunnable = true;
	private Odometer odometer; // Odometer object
	private float lightVal; // value of single sensor data
	private double theta;
	private int xLine;
	private int yLine;

	/**
	 * This is the default class constructor. An existing instance of the odometer
	 * is used. This is to ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection(double offsetOrigin) throws OdometerExceptions {
		this.odometer = Odometer.getOdometer();
		xLine=Robot.xStartingOffset;
		yLine=Robot.yStartingOffset;
	}

	/**
	 * This method corrects the robot's position based on line it crosses, it cannot
	 * handle if robot is not driving parallel to x or y line!
	 * 
	 * @throws OdometerExceptions
	 */
	// run method (required for Thread)
	public void run() {
		while (true) {
			long correctionStart, correctionEnd;
			correctionStart = System.currentTimeMillis();
			// fetch color from Sample Provider thread
			lightVal = Robot.getFloorColor();
			System.out.println("lightval: " + lightVal);
			// if robot is not on the line light sensor in red mode should output value less
			// than 10
			if (lightVal <= BLACK_THRESHOLD) {
				Sound.beep();
				// getting the theta value from odometer class
				theta = odometer.theta;
				// check the postion of the robot
				if ((340 < theta && theta <= 0) || (theta > 0 && theta < 20)) {
					// x is incrementing
					xLine++;
					offset = xLine * Robot.TILE_SIZE;
					odometer.setY(offset);
				} else if (70 <= theta && theta <= 110) {
					yLine++;
					offset = yLine * Robot.TILE_SIZE;
					odometer.setX(offset);
				} else if (160 < theta && theta <= 200) {
					xLine--;
					offset = xLine * Robot.TILE_SIZE;
					odometer.setY(offset);
				} else if (250 < theta && theta <= 290) {
					yLine--;
					offset = yLine * Robot.TILE_SIZE;
					odometer.setX(offset);
				}
				System.out.println("Corrected");
			}
			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
	}

	/**
	 * This method checks which direction the robot is moving
	 * 
	 * @param theta,
	 *            angle in degrees
	 * @return int: 0: moving west, 1: moving north, 2: moving east, 3: moving south
	 */
	public static int isMovingX(double theta) {
		if ((315 < theta && theta <= 0) || (theta > 0 && theta < 45)) {
			return 1;
		} else if (45 <= theta && theta <= 135) {
			return 2;
		} else if (135 < theta && theta <= 225) {
			return 3;
		} else if (225 < theta && theta <= 315) {
			return 4;
		}
		return -999;
	}

}