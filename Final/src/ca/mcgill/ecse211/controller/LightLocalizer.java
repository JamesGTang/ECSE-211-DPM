package ca.mcgill.ecse211.controller;

import ca.mcgill.ecse211.model.Robot;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
/**
 * This class provides methods for localizing the robot to starting corner using light sensor
 * @author jamestang
 *
 */
public class LightLocalizer {
	private Odometer odometer;
	
	private float lightVal=100;
	private static final int BLACK_THRESHOLD=300;// the threshold is 280 when light sensor is 1cm from ground
	private double dX,dY;
	
	public LightLocalizer(Odometer odometer) {
		this.odometer=odometer;
	}
	/**
	 * This method uses light sensor to localize the robot to starting corner
	 */
	public void localize() {
		// reset odometer
		this.odometer.setXYT(0, 0, 0);
		System.out.println("Verify odometer reset"+odometer.getXYT().toString());
		lightVal=Robot.getFloorColor();
		System.out.println("Light val at beginnign"+lightVal);
		while(lightVal>BLACK_THRESHOLD) {
			Robot.driveForward();
			lightVal=Robot.getFloorColor();
		}
		Robot.stop();
		// beep once for update
		Sound.beep();
		// stop immediately
		
		dY=odometer.getXYT()[1];
		System.out.println("dY"+dY);
		// revert back to original state
		Robot.travelTo(-dY);
		// fetch sensor data again
		lightVal=Robot.getFloorColor();
		// turn to check the other line
		System.out.println("Turn 90");
		Robot.turnTo(Math.toRadians(90));
		// we will turn counter clockwise until cross another line
		while(lightVal>BLACK_THRESHOLD) {
			Robot.driveForward();
			lightVal=Robot.getFloorColor();
		}
		Robot.stop();
		// beep once for update
		Sound.beep();
		// stop immediately
		dX=odometer.getXYT()[0];
		// revert back to original state
		System.out.println("dX"+dX);
		Robot.travelTo(-dX);
		// add correction here
		Robot.travelTo(-dX , dY);
		System.out.println("Aligning"+(270-odometer.getTheta()));
		Robot.turnTo(Math.toRadians(-odometer.getTheta()));
		
	}	
}