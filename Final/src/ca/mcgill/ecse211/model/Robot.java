package ca.mcgill.ecse211.model;

import ca.mcgill.ecse211.display.Display;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
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
 * 	Specify sensor methods, drive methods, define feedback mechanism
 *	======== Set up =========== 
 *	Sensors:
 *	Ultrasonic sensor: Sensor Port S4
 *	Block light sensor (forward facing): Sensor Port S2
 *	Floor light sensor (downward facing): Sensor Port S3
 *	
 *	Motors:
 *	Left motor: Motor Port D
 *	Right motor: Motor Port B
 *	Ultrasonic base motor: Motor Port A
 */
public class Robot {
	// robot drive system related data
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 15.8;
	public static final double TILE_SIZE = 30.48;
	private static int FORWARD_SPEED = 150;
	private static int ACCELERATION_SPEED=100;
	private static final int ROTATE_SPEED = 100;
	
	// robot sensor placement data
	public static double floorSensorOffset=2.0; // distance of light sensor to wheel axis
	public static double usSensorOffset=7;
	public static double usSensorAngle=15;  // angle for ultrasonic sensor detection, which correlates to distance
	public static final double forwardLightSensorOffset=1.5; // how far the forward facing light sensor is from the ultrasonic sensor
	public static int usMotorAngle=0;
	private static double OFF_CONST=1.02;
	
	// robot state related data
	public static DriveState driveState;
	public static LocalizationCategory loc;
	public static int Starting_Corner=-1;
	
	// odometry related object
	public static Odometer odometer;
	public static Display odometryDisplay;
	public static int xStartingOffset=1; //this is starting offset for odometer corrrection depends on SC, number of tiles already passed when robot arrives at starting postion
	public static int yStartingOffset=1;
	/**
	 * This method initialize the robot into original state
	 * @throws OdometerExceptions 
	 */
	public static int init(int startingPoint) throws OdometerExceptions {
		usMotor.setSpeed(100);
		usMotor.setAcceleration(50);
		//System.out.println("Robot initialized with odometer, odometer display");
		driveState=DriveState.STOP;
		loc=LocalizationCategory.NONE;
		Starting_Corner=startingPoint;
		
		// Odometer related objects
		odometer = Odometer.getOdometer(Robot.leftMotor, Robot.rightMotor, Robot.TRACK, Robot.WHEEL_RAD);
		odometryDisplay = new Display(Robot.lcd);
		
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();		
		return 1;
	}
	
	// locType and state of the robot
	public enum LocalizationCategory {
		NONE,
		FALLING_EDGE,
		RISING_EDGE;
	};
	// the state of the robot's drive system
	public enum DriveState {
		STOP,
		FORWARD,
		BACKWARD,
		TURN,
		TRAVEL;
	};
	
	// define motors
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3LargeRegulatedMotor usMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	
	/**
	 * This method put the robot in a fixed speed drive forward
	 */
	public static void driveForward() {
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.setAcceleration(ACCELERATION_SPEED);
		rightMotor.setAcceleration(ACCELERATION_SPEED);
		leftMotor.forward();
		rightMotor.forward();
	}
	
	/**
	 * This method put the robot in an assigned speed drive forward
	 */
	public static void driveForwardWithSpeed(int fwd_speed,int acceleration_speed) {
		leftMotor.setSpeed(fwd_speed);
		rightMotor.setSpeed(fwd_speed);
		leftMotor.setAcceleration(acceleration_speed);
		rightMotor.setAcceleration(acceleration_speed);
		leftMotor.forward();
		rightMotor.forward();
	}
	
	/**
	 * This method put the robot in a fixed speed drive backward
	 */
	public static void driveBackward() {
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.setAcceleration(ACCELERATION_SPEED);
		rightMotor.setAcceleration(ACCELERATION_SPEED);
		leftMotor.forward();
		rightMotor.forward();
	}
	
	/**
	 * This method turn the robot continuously in place
	 * @param int direction: -1: turn left, 1: turn right
	 */
	public static void turn(String direction) {
		Robot.leftMotor.setSpeed(FORWARD_SPEED);
		Robot.rightMotor.setSpeed(FORWARD_SPEED);
		Robot.leftMotor.setAcceleration(ACCELERATION_SPEED);
		Robot.rightMotor.setAcceleration(ACCELERATION_SPEED);
		
		if(direction=="LEFT") {
			Robot.leftMotor.forward();
			Robot.rightMotor.backward();
		}else if(direction=="RIGHT") {
			Robot.leftMotor.backward();
			Robot.rightMotor.forward();
		}
	}
	
	/**
	 * This method puts the robot to a synchronized stop
	 */
	public static void stop() {
		Robot.leftMotor.stop(true);
		Robot.rightMotor.stop(false);
	}
	
	/**
	 * This method turns the robot to an angle specified by thetaDest
	 * @param thetaDest : the angle needs to be turned in radian
	 */
	public static void turnTo(double thetaDest) {
		/*
		 * If needs to turn more than 180 degree
		 * turn the other way instead
		 */
		if(thetaDest>Math.PI) {
			// the angular disance will be negaive
			thetaDest = thetaDest-2*Math.PI;
		}else if(thetaDest < (-Math.PI)) {
			// if needs to turn more than -180 degree
			thetaDest = thetaDest+2*Math.PI;
		}
		thetaDest=thetaDest/OFF_CONST;
		leftMotor.setAcceleration(ACCELERATION_SPEED);
		rightMotor.setAcceleration(ACCELERATION_SPEED);
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		thetaDest = Math.toDegrees(thetaDest);
		int rotationAngle = robotUtil.convertAngle(WHEEL_RAD, TRACK, thetaDest);
		/*
		 * Initialize a synchronous action to avoid slip
		 */
		leftMotor.rotate(rotationAngle, true);
		rightMotor.rotate(-rotationAngle);
	}
	
	/**
	 * This method drives robot straight for a distance
	 * @param linearDistance the distance to be covered
	 */
	public static void travelTo(double linearDistance) {
		// move the linear distance possible improvement here
		leftMotor.setAcceleration(ACCELERATION_SPEED);
		rightMotor.setAcceleration(ACCELERATION_SPEED);
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(robotUtil.convertDistance(WHEEL_RAD, linearDistance), true);
		rightMotor.rotate(robotUtil.convertDistance(WHEEL_RAD, linearDistance), false);
	}
	/**
	 * This method drives robot to a specific location using euclidean distance for shortest path
	 * @param xDest: x coordinate in cm
	 * @param yDest: y coordinate in cm
	 */
	public static void travelTo(double xDest,double yDest) {
		// convert coordinate to length
		double xCurrent=odometer.getX();
		double yCurrent=odometer.getY();
		double thetaCurrent=odometer.getTheta();
		System.out.println("Xdest,YDest: "+xDest+" "+yDest);
		System.out.println("In the model: xCurrent,yCurrent,Theta"+xCurrent+" "+yCurrent+" "+thetaCurrent);
		double dX=xDest-xCurrent;
		double dY=yDest-yCurrent;
		double linearDistance=robotUtil.getLinearDistance(dX, dY);
		/*
		 * Calculate the angle using tangent
		 */
		double angularDistance=Math.atan2(dX,dY)-Math.toRadians(thetaCurrent);

		System.out.println("Angle before conversion: "+angularDistance);
		turnTo(angularDistance);
		// move the linear distance possible improvement here
		travelTo(linearDistance);
	}
	/**
	 * This method drives robot to the destination point (x,y) selecting only paths that
	 * are perpendicular to x axis or y axis, it helps to avoid cross the search zone if the search zone is obstructing
	 * the robot from starting corner to start of search zone
	 * @param xDest: destination x
	 * @param yDest: destination y
	 */
	public static void squareTravelTo(double xDest,double yDest) {
		double xCurrent=odometer.getX();
		double yCurrent=odometer.getY();
		System.out.println("In the sqaure travel: xCurrent,yCurrent,Theta"+xCurrent+" "+yCurrent+" "+odometer.getTheta());
		// convert coordinate to length
		double dX=xDest-xCurrent;
		double dY=yDest-yCurrent;
		System.out.println("Covering dy to starting point");
		Robot.travelTo(dY);
		// calculate the angle to be turned before covering dx. first find the heading angle
		double thetaCurrent=odometer.getTheta();
		System.out.println("theta current: "+thetaCurrent);
		double angularDistance;
		if(xDest>=odometer.getX()) angularDistance=Math.toRadians(90);
		else angularDistance=Math.toRadians(-90);
		turnTo(angularDistance);
		System.out.println("Covering dx to starting point");
		travelTo(dX);
		// turn back to align with y axis
		turnTo(-angularDistance); 
		System.out.println("In the sqaure travel: xCurrent,yCurrent,Theta"+odometer.getX()+" "+odometer.getY()+" "+odometer.getTheta());
	}
	
	
	
	// define Ultrasonic sensor
	public static SensorModes usSensor = new EV3UltrasonicSensor(SensorPort.S4); // the instance
	public static SampleProvider usDistance = usSensor.getMode("Distance"); // provides samples from this instance
	public static float[] usData = new float[usDistance.sampleSize()];
	public static UltrasonicController usController;
	
	/**
	 * This method fetch the distance value from the ultrasonic sensor
	 * @return float: the distance from light sensor
	 */
	public static double getDistance() {
		Robot.usDistance.fetchSample(usData, 0);
		double distance=usData[0]*100;
		return distance;
	}
	
	
	// define light sensor
	public static EV3ColorSensor colorSensor=new EV3ColorSensor(SensorPort.S2);
	public static SampleProvider colorProvider=colorSensor.getRGBMode();
	private static float[] color=new float[Robot.colorProvider.sampleSize()];
	
	// define floor light sensor
	public static EV3ColorSensor floorColorSensor=new EV3ColorSensor(SensorPort.S3);
	public static SampleProvider floorColorProvider=floorColorSensor.getRedMode();
	private static float[] floorColor=new float[Robot.floorColorProvider.sampleSize()];
	
	/**
	 * This method fetch the color value from the forward light sensor
	 * @return float: the value of light from light sensor
	 */
	public static float[] getColor() {
		Robot.colorProvider.fetchSample(color, 0);
		return color;
	}
	
	/**
	 * This method fetch the color value from floor light sensor
	 * @return float: the value of light from light sensor
	 */
	public static float getFloorColor() {
		Robot.floorColorProvider.fetchSample(floorColor, 0);
		float lightVal=floorColor[0]*1000;
		return lightVal;
	}
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
	/**
	 * This method sets the odometer based on which starting corner
	 * the robot is assigned to
	 * important: the robot has x and y offset of one tile length from the origin after localization
	 */
	public static void setSCOdometer() {
		if(Starting_Corner!=-1) {
			switch (Starting_Corner) {
			case 0:
				// the robot is starting at lower left
				odometer.setXYT(TILE_SIZE , TILE_SIZE , 0);
				System.out.println("Robot starting from: "+Starting_Corner);
				break;
			case 1:
				// the robot is starting at lower right
				odometer.setXYT(TILE_SIZE*7, TILE_SIZE , 0);
				System.out.println("Robot starting from: "+Starting_Corner);
				break;
			case 2:
				// the robot is starting at upper right
				odometer.setXYT(TILE_SIZE*7 , TILE_SIZE*7 , 180);
				System.out.println("Robot starting from: "+Starting_Corner);
				break;
			case 3:
				// the robot is starting at upper right
				odometer.setXYT(TILE_SIZE*7 , TILE_SIZE, 180);
				System.out.println("Robot starting from: "+Starting_Corner);
				break;
			default:
				break;
			}
		}else {
			// starting corner is not set
		}
	}
	/**
	 * Method to alter the speed of the robot
	 * DRIVE: fastest, for navigation to a destination, 
	 * SEARCH: medium, for searching and scanning the area
	 * COR: slowest, for correcting robot's postion
	 * 
	 * @param type
	 */
	public static void alterSpeed(String type) {
		if(type=="DRIVE") {
			FORWARD_SPEED = 125;
			ACCELERATION_SPEED=75;
		}else if(type=="SEARCH") {
			FORWARD_SPEED = 150;
			ACCELERATION_SPEED=75;
		}else if(type=="COR") {
			FORWARD_SPEED = 50;
			ACCELERATION_SPEED=50;
		}else if(type=="FAST") {
			FORWARD_SPEED = 200;
			ACCELERATION_SPEED=150;
		}
	}
	public void clearLCD() {
		lcd.clear();
	}

}