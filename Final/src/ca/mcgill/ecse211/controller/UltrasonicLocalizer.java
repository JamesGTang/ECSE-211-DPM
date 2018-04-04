package ca.mcgill.ecse211.controller;

import ca.mcgill.ecse211.model.Robot;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.ultrasonic.UltrasonicController;
import ca.mcgill.ecse211.util.robotUtil;
import lejos.hardware.Sound;

/**
 * This class provides methods that localizes the robot using ultrasonic sensor
 * @author jamestang
 *
 */
public class UltrasonicLocalizer implements UltrasonicController{
	private double dist=50;
	// assume of robot is placed at original, the maximum usable distance used for localizalition
	private double maxD=30;
	// this offset is to compensate our the stiffness robot's right wheel 
	private double OFFSET_CONST=1.12;
	// these offset help correct motor power issue
	private double COR_OFFSET=20;
	private double MOTOR_OFFSET=3;
	private double lAngle=0;
	private double rAngle=0;
	private double xBefore=0,xAfter=0;
	
	private Odometer odometer;
	// this array holds all 
	public UltrasonicLocalizer(Odometer odometer) {
		this.odometer=odometer;
	}
	/**
	 * This method localizes the robot using ultrasonic sensor
	 */
	public void localize() {
		System.out.println("Localization started");
		Robot.alterSpeed("SEARCH");
		// setAcceleration and speed to very slow for accurate reading
		// falling edge: robot is facing away from the wall
		Robot.usMotor.setSpeed(100);
		Robot.usMotor.rotateTo(0);
		if(Robot.loc==Robot.LocalizationCategory.FALLING_EDGE) {
			// the robot detects a falling edge, switch direction and detect another falling edge
			System.out.println("Falling edge selected");
			// trap process until valid data is received
			while(dist>maxD) {
				// find the left wall
				Robot.turn("RIGHT");
			}
			Robot.stop();
			
			// beep once for signaling
			Sound.beep();
			lAngle=this.odometer.getTheta();
			System.out.println("Left Turn finished "+lAngle);
			//revert 90 degree
			System.out.println("Start turning right now");
			Robot.turnTo(Math.toRadians(-180));
			// try to find the other falling edge
			while(dist>maxD) {
				// find the left wall
				Robot.turn("LEFT");
			}
			Robot.stop();
			
			// beep once for signaling
			Sound.beep();
			rAngle=this.odometer.getTheta();			
			System.out.println("Right Turn finished "+rAngle);
			// after both angle has been found
			Robot.turnTo(Math.toRadians((lAngle - rAngle) / 2 +135));
			System.out.println("Finished localization");
			System.out.println("Compensation: " + ((lAngle - rAngle) / 2 +135));
			// correct heading here
			if(Robot.runDiagonistic()==1) {
				verifyCorrection();
			}
			System.out.println("Finished localization");
			
		}else if(Robot.loc==Robot.LocalizationCategory.RISING_EDGE) {
			// the robot detects a falling edge, switch direction and detect another falling edge
			System.out.println("Rising edge selected");
			// trap process until valid data is received
			Robot.alterSpeed("SEARCH");
			while (dist < maxD) {
				// find the left wall
				Robot.turn("LEFT");
			}
			Robot.stop();
			
			// beep once for signaling
			Sound.beep();
			lAngle = this.odometer.getTheta();
			System.out.println("Left Turn finished " + lAngle);
			// revert 90 degree
			System.out.println("Start left right now");
			Robot.alterSpeed("SEARCH");
			Robot.turnTo(Math.toRadians(90));
			Robot.alterSpeed("SEARCH");
			// try to find the other falling edge
			while (dist < maxD) {
				// find the left wall
				Robot.turn("RIGHT");
			}
			Robot.leftMotor.stop(true);
			Robot.rightMotor.stop(false);
			// beep once for signaling
			Sound.beep();
			rAngle = this.odometer.getTheta();
			System.out.println("Right Turn finished " + rAngle);
			// after both angle has been found
			Robot.turnTo(Math.toRadians((rAngle - lAngle) / 2 +45));
			System.out.println("Finished localization");
			System.out.println("Compensation: " + ((rAngle - lAngle) / 2 +45));
			// correct heading here
			if(Robot.runDiagonistic()==1) {
				verifyCorrection();
			}
		}else {
			// the category is not set, abort
			Robot.lcd.clear();
			Robot.lcd.drawString("Loc check failed, abort", 0, 0);
		}
		//verifyLocalization();
		Sound.beep();
	}
	
	@Override
	public void processUSData(double distance) {
		// test: passed distance is normal
		//System.out.println("Distance: "+distance);
		this.dist=distance;
	}
	
	/**
	 * @return ultrasonic sensor reading
	 */
	@Override
	public double readUSDistance() {
		return this.dist;
	}
	/**
	 * @param none
	 * @return none
	 * This method verifies the falling edge and rising edge is applied correct and check if any mechanical error
	 */
	public void verifyCorrection() {		
		Robot.alterSpeed("FAST");
		// turn usMotor 90 degree to face west wall
		Robot.usMotor.setSpeed(150);
		Robot.usMotor.setAcceleration(1000);
		Robot.usMotor.rotateTo(90);
		// get distance at this point 
		xBefore=dist;
		System.out.println("Dist to wall before"+dist);
		// move forward 5cm
		Robot.travelTo(COR_OFFSET);
		xAfter=dist;
		Robot.travelTo(-(COR_OFFSET));
		System.out.println("Dist to wall after"+dist);
		if(xBefore>xAfter) {
			// the robot is to the left
			double angleCor=Math.toRadians((Math.toDegrees(Math.atan((xBefore-xAfter)/COR_OFFSET))+MOTOR_OFFSET));;
			System.out.println("Angle correction for left"+angleCor);
			Robot.turnTo(angleCor);
		}else if (xBefore<xAfter){
			double angleCor=Math.toRadians((Math.toDegrees(Math.atan((xAfter-xBefore)/COR_OFFSET))+MOTOR_OFFSET));
			System.out.println("Angle correction for right"+angleCor);
			Robot.turnTo(-angleCor);
		}else {
			//do nothing here
		}
		Robot.alterSpeed("FAST");
		Robot.usMotor.rotate(-90);
		
	}
}
