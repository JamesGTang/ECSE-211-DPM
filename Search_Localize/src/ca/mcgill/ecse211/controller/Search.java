package ca.mcgill.ecse211.controller;

import java.util.ArrayList;

import ca.mcgill.ecse211.data.ColorBlock;
import ca.mcgill.ecse211.data.LocalizationData;
import ca.mcgill.ecse211.model.Robot;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.ultrasonic.UltrasonicController;

public class Search implements UltrasonicController{
	private double distance;
	// records which side of the search zone the robot is heading now
	// 1: west 2: north 3: east 4:south -1: invalid 
	private int direction=-1;
	private double turnPoint=0; // the x or y value at the point where the robot turned
	private boolean isMovingX;
	private Odometer odometer;
	private int tb;
	// maximun allowed distance for block discovery in that quadrant
	private double xQuadrantDistance;
	private double yQuadrantDistance;
	// this holds all the color block found
	ArrayList<ColorBlock> colorBlockList=new ArrayList<ColorBlock>();
	
	public Search(int tb) throws OdometerExceptions {
		this.tb=tb;
		this.odometer=Odometer.getOdometer();
		// rotate the ultrasonic sensor so it is facing the side
		Robot.usMotor.rotate(-90);
		xQuadrantDistance=(LocalizationData.getURx()-LocalizationData.getLLx())/2*Robot.TILE_SIZE;
		yQuadrantDistance=(LocalizationData.getURy()-LocalizationData.getLLy())/2*Robot.TILE_SIZE;
		System.out.println("Valid x/y: "+xQuadrantDistance+" "+yQuadrantDistance);
	}
	
	public void SearchTarget() {
		for(int i=1;i<=4;i++) {
			// set the direction at start
			direction=i;
			// drive one vertice at a time
			if(direction==1 || direction==3) {
				isMovingX=false;
				System.out.println("Moving alone y, vertice:  "+direction);
				// robot heading alone y
				double linearDistance=(LocalizationData.getURy()-LocalizationData.getLLy()+1)*Robot.TILE_SIZE;
				Robot.travelTo(linearDistance);
				Robot.turnTo(90);
			}else if(direction==2||direction==4) {
				isMovingX=true;
				System.out.println("Moving alone x, vertice:  "+direction);
				// robot heading alone x
				double linearDistance=(LocalizationData.getURx()-LocalizationData.getLLx()+1)*Robot.TILE_SIZE;
				Robot.travelTo(linearDistance);
				Robot.turnTo(90);
			}
		}
	}
	
	
	@Override
	public void processUSData(double distance) {
		// interrupt the drive process if we found a block
		// below is for data collection only, should be removed before demo
		System.out.println(distance);
		// check which direction the robot is moving
		double validDistance;
		// indicate the allowed range for block detection on each vertice
		if(isMovingX) validDistance=yQuadrantDistance;
		else validDistance=xQuadrantDistance;
		
		if(distance<validDistance) {
			// there is a block 
			//ToDo: add a method to take a few sample data and confirm distance
			// ToDo: verify if this block has been searched already
			if(!isBlockSearched()) {
				// if block not searched, stop the wheel
				Robot.stop();
				// record the path for robot to get back to
				if(isMovingX) turnPoint=odometer.getY();
				else turnPoint=odometer.getX();
				// rotate robot 90 degree
				Robot.turnTo(Math.toRadians(90));
				// rotate us motor 90 degree to face forward
				Robot.usMotor.rotate(-90);
				// approach the block slowly
				Robot.driveForwardWithSpeed(75, 50);
				while(Robot.getDistance()>6) {
					// keep driving until the robot is 6 cm away, and the light sensor is 4.5cm away
				}
				// move even slower
				Robot.driveForwardWithSpeed(30,25);
				// ToDo: keep moving until a color is detected, if color block is not the target block ,add to list, else beep and terminate
				int aColor=1;
				
				Robot.stop();
				if(isMovingX) {
					double yTB=odometer.getY()+Robot.forwardLightSensorOffset+Robot.usSensorOffset+1.5+5;
					ColorBlock newColorBlock=new ColorBlock(odometer.getX(),odometer.getY()+yTB, aColor);
					colorBlockList.add(newColorBlock);
					Robot.travelTo(turnPoint-odometer.getY());
				}else {
					double xTB=odometer.getX()+Robot.forwardLightSensorOffset+Robot.usSensorOffset+1.5+5;
					ColorBlock newColorBlock=new ColorBlock(odometer.getX()+xTB,odometer.getY(), aColor);
					colorBlockList.add(newColorBlock);
					Robot.travelTo(turnPoint-odometer.getX());
				}
				// turn back and continue with the path
				Robot.turnTo(Math.toRadians(-90));		
			}
		}		
	}

	@Override
	public double readUSDistance() {
		return this.distance;
	}
	
	public boolean isBlockSearched() {
		return false;
	}

}
