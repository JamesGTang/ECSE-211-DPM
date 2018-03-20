package ca.mcgill.ecse211.model;

import ca.mcgill.ecse211.ultrasonic.UltrasonicController;
import ca.mcgill.ecse211.util.*;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * 
 * @author jamestang
 * This class defines the robot as a model
 * Specify sensor methods, drive methods
 * Define feedback mechanism
 *
 */
public class Robot {
	// basic data measurement of robot-specific data
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 15.8;
	public static final double TILE_SIZE = 30.48;
	private static final int FORWARD_SPEED = 50;
	private static final int ROTATE_SPEED = 50;
	public static double lsOffset=2.0; // distance of light sensor to wheel axis
	public static int usMotorAngle=0;
	private static double OFF_CONST=1.02;
	
	// locType and state of the robot
	public enum LocalizationCategory {
		FALLING_EDGE,
		RISING_EDGE;
	};
	public static LocalizationCategory loc;
	
	// define motors
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor usMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	
	/**
	 * This method turns the robot to an angle specified by thetaDest
	 * @author jamestang
	 * @param thetaDest : the angle needs to be turned in radian
	 */
	public static void turnTo(double thetaDest) {
		/*
		 * Set acceleration to 3000 the default speed is 6000 this gives a smooth
		 * acceleration
		 */
		thetaDest=thetaDest/OFF_CONST;
		leftMotor.setAcceleration(50);
		rightMotor.setAcceleration(50);
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		thetaDest = Math.toDegrees(thetaDest);
		int rotationAngle = Util.convertAngle(WHEEL_RAD, TRACK, thetaDest);
		/*
		 * Initialize a synchronous action to avoid slip
		 */
		leftMotor.rotate(rotationAngle, true);
		rightMotor.rotate(-rotationAngle);
	}
	/**
	 * This method drives robot to a specific location
	 * @author jamestang
	 * @param xDest: x coordinate in cm
	 * @param yDest: y coordinate in cm
	 */
	public static void travelTo(double linearDistance) {
		// move the linear distance possible improvement here
		leftMotor.setAcceleration(50);
		rightMotor.setAcceleration(50);
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(WHEEL_RAD, linearDistance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, linearDistance), false);
	}
	/**
	 * This method drives robot to a specific location
	 * @author jamestang
	 * @param xDest: x coordinate in cm
	 * @param yDest: y coordinate in cm
	 */
	public static void travelTo(double xCurrent,double yCurrent,double thetaCurrent,double xDest,double yDest) {
		// convert coordinate to length
		System.out.println("In the model: xDest,yDest"+xDest+" "+yDest);
		double dX=xDest-xCurrent;
		double dY=yDest-yCurrent;
		double linearDistance=getLinearDistance(dX, dY);
		/*
		 * Calculate the angle using tangent
		 */
		double angularDistance=Math.atan2(dX,dY)-Math.toRadians(thetaCurrent);

		System.out.println("Angle before conversion: "+angularDistance);
		/*
		 * If needs to turn more than 180 degree
		 * turn the other way instead
		 */
		if(angularDistance>Math.PI) {
			// the angular disance will be negaive
			angularDistance = angularDistance-2*Math.PI;
		}else if(angularDistance < (-Math.PI)) {
			// if needs to turn more than -180 degree
			angularDistance = angularDistance+2*Math.PI;
		}
		turnTo(angularDistance);
		// move the linear distance possible improvement here
		leftMotor.setAcceleration(50);
		rightMotor.setAcceleration(50);
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(WHEEL_RAD, linearDistance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, linearDistance), false);
	}
	
	// define Ultrasonic sensor
	public static SensorModes usSensor = new EV3UltrasonicSensor(SensorPort.S4); // the instance
	public static SampleProvider usDistance = usSensor.getMode("Distance"); // provides samples from this instance
	public static float[] usData = new float[usDistance.sampleSize()];
	public static UltrasonicController usController;
	
	// define light sensor
	public static EV3ColorSensor colorSensor=new EV3ColorSensor(SensorPort.S2);
	public static SampleProvider colorProvider=colorSensor.getRedMode();
	
	// define textLCD
	public static TextLCD lcd = LocalEV3.get().getTextLCD();
	
	// define localization type of the robot
	
	public static TextLCD getLCD() {
		return lcd;
	}
	
	/**
	 * Run diagonostic  on the robot
	 * @return integer, 0: normal, 1: faulty
	 */
	public static int runDiagonistic() {
		int ret=1;
		if(leftMotor.isStalled()) {
			System.out.println("Left motor is stalled");
			ret=1;
		}else if (rightMotor.isStalled()) {
			System.out.println("Right motor is stalled");
			ret=1;
		}else if(usMotor.isStalled()) {
			System.out.println("US motor is stalled");
		}
		return ret;		
	}
	
	public void clearLCD() {
		lcd.clear();
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	private static double getLinearDistance(double x,double y) {
		  return Math.hypot(x, y);
	  }
	
	
	
}