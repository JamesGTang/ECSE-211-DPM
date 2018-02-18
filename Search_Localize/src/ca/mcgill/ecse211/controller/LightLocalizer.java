package ca.mcgill.ecse211.controller;

import ca.mcgill.ecse211.model.Robot;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;

public class LightLocalizer {
	private Odometer odometer;
	
	private float lightVal=100;
	private static final int BLACK_THRESHOLD=300;// the threshold is 280 when light sensor is 1cm from ground
	private double dX,dY;
	
	public LightLocalizer(Odometer odometer) {
		this.odometer=odometer;
	}
	
	public void localize() {
		// reset odometer
		this.odometer.setXYT(0, 0, 0);
		System.out.println("Verify odometer reset"+odometer.getXYT().toString());
		lightVal=Robot.getColor();
		System.out.println("Light val at beginnign"+lightVal);
		while(lightVal>BLACK_THRESHOLD) {
			Robot.driveForward();
			lightVal=Robot.getColor();
		}
		Robot.stop();
		// beep once for update
		Sound.beep();
		// stop immediately
		
		dX=odometer.getXYT()[0];
		System.out.println("dX"+dX);
		// revert back to original state
		Robot.travelTo(dX);
		// fetch sensor data again
		lightVal=Robot.getColor();
		// turn to check the other line
		System.out.println("Turn 90");
		Robot.turnTo(Math.toRadians(90));
		// we will turn counter clockwise until cross another line
		while(lightVal>BLACK_THRESHOLD) {
			Robot.driveForward();
			lightVal=Robot.getColor();
		}
		Robot.stop();
		// beep once for update
		Sound.beep();
		// stop immediately
		
		dY=odometer.getXYT()[1];
		// revert back to original state
		System.out.println("dy"+dY);
		Robot.travelTo(-dY);
		// reset odometer
		odometer.setXYT(0, 0, 0);
		// add correction here
		System.out.println("dX: "+dX+"dY: "+dY);
		Robot.travelTo(0,0, 0,dX , dY);
		System.out.println("Aligning"+(270-odometer.getTheta()));
		Robot.turnTo(Math.toRadians(270-odometer.getTheta()));
		// reset odometer
		odometer.setXYT(0, 0, 0);
	}
	
	
}
