package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerData;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

// navigation class should not extends thread
public class Navigation extends Thread{
	private static final int FORWARD_SPEED = 200;
	private static final int ROTATE_SPEED = 150;
	private static final double TILE_SIZE = 30.48;
	private int ODOMETER_PERIOD= 25;
	
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor,rightMotor;
	private double track;
	private double wr;
	private static double OFF_CONST=1.1;
	
	private double xCurrent;
	private double yCurrent;
	private double thetaCurrent=0;
	private boolean isFinished;
	
	Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
		  double track,double wr) {
		this.odometer=odometer;
		this.rightMotor=rightMotor;
		this.leftMotor=leftMotor;
		this.track=track;
		this.wr=wr;
		isFinished=false;
		}
	
	// thread behavior
	public void run() {
		long updateStart, updateEnd;
		while (true) {
			updateStart = System.currentTimeMillis();
			getCurrentPos();
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
	/* this method drives the robot to a position specified
	 * as x and y
	 */
	public void travelTo(double xDest,double yDest) {
		// convert coordinate to length
		xDest=xDest * TILE_SIZE;
		yDest=yDest * TILE_SIZE;
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
		turnTo(angularDistance/OFF_CONST);
		// move the linear distance possible improvement here
		leftMotor.setAcceleration(300);
		rightMotor.setAcceleration(300);
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		isFinished=true;
		leftMotor.rotate(convertDistance(wr, linearDistance), true);
		rightMotor.rotate(convertDistance(wr, linearDistance), false);
		isFinished=false;
	}
	
	/* this method supplements the travelTo method
	 * it turns the robot in place into the right direction before moving
	 */
	public void turnTo(double thetaDest) {
		/*
		 * Set acceleration to 300
		 * this gives a smooth acceleration
		 */
		leftMotor.setAcceleration(300);
		rightMotor.setAcceleration(300);
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		thetaDest=Math.toDegrees(thetaDest);
		int rotationAngle=convertAngle(wr, track, thetaDest);
		/*
		 * Initialize a synchronous action to avoid slip
		 */
		leftMotor.rotate(rotationAngle,true);
		rightMotor.rotate(-rotationAngle,false);
		isFinished=false;
	}
	
	/*
	 * this method fetches the x and y position from odometer
	 */
	private void getCurrentPos() {
		/* 
		 * changed the visibility of lock in odometer class to public
		 * here the method locks up the thead before accessing data
		 */
		synchronized (OdometerData.lock) {
			this.thetaCurrent=odometer.getXYT()[2];
			this.xCurrent=odometer.getXYT()[0];
			this.yCurrent=odometer.getXYT()[1];
		}
	}

  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  private static int convertAngle(double radius, double width, double angle) {
	return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
  
  private static double getLinearDistance(double x,double y) {
	  return Math.hypot(x, y);
  }
  
  public boolean isFinished() {
	  return isFinished;
  }
}
