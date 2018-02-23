package ca.mcgill.ecse211.controller;

import java.util.ArrayList;
import java.util.Iterator;

import ca.mcgill.ecse211.data.ColorBlock;
import ca.mcgill.ecse211.data.LocalizationData;
import ca.mcgill.ecse211.model.Robot;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.ultrasonic.UltrasonicController;
import lejos.hardware.Sound;

public class SearchTargetBlock implements UltrasonicController{
	private double distance;
	// records which side of the search zone the robot is heading now
	// 1: west 2: north 3: east 4:south -1: invalid 
	private int direction=-1;
	private double turnPoint=0; // the x or y value at the point where the robot turned
	private boolean isMovingX;
	private Odometer odometer;
	private int tb;
	private static double[] targetValue = new double [Robot.colorProvider.sampleSize()]; 
	private static float[] color=new float[Robot.colorProvider.sampleSize()];
	// maximun allowed distance for block discovery in that quadrant
	private double xQuadrantDistance;
	private double yQuadrantDistance;
	// this holds all the color block found
	ArrayList<ColorBlock> colorBlockList=new ArrayList<ColorBlock>();
	int counter = 0; //count the number of blocks in the colorBlockList

	public SearchTargetBlock(int tb) throws OdometerExceptions {
		this.tb=tb;
		this.odometer=Odometer.getOdometer();
		// rotate the ultrasonic sensor so it is facing the side
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
				Robot.turnTo(Math.toRadians(90));
			}else if(direction==2||direction==4) {
				isMovingX=true;
				System.out.println("Moving alone x, vertice:  "+direction);
				// robot heading alone x
				double linearDistance=(LocalizationData.getURx()-LocalizationData.getLLx()+1)*Robot.TILE_SIZE;
				Robot.travelTo(linearDistance);
				Robot.turnTo(Math.toRadians(90));
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
		boolean ifDistanceConfirmed;
		//indicates if the target block is found
		boolean ifFound = false;
		//indicates if the not target block is added to the search list
		boolean ifAddBlock = false;
		//the color of the not target block
		int TBColor=0;  // 1: Red, 2: Blue, 3: Yellow, 4: White -99: noise
		//indicates whether a block is in front or not 
		boolean ifBlockDetected = false;
		//the array of light value which increase the RGB value by 10^3 for calculation purpose
		double lightVal[]=new double[3];


		if(!ifFound) {
		if(distance < validDistance) {
			// there is a block, verify distance
			System.out.println("Found a block here");
			Robot.stop();
				ifDistanceConfirmed = verifyDistance(distance);
				if (ifDistanceConfirmed) {
					// check if block is searched already
					if (!isBlockSearched(distance)) {
						// if block not searched, stop the wheel
						Robot.stop();
						Robot.turnTo(Math.toRadians(90));
						// rotate us motor 90 degree to face forward
						Robot.usMotor.rotate(90);
						
						// record the path for robot to get back to
						if (isMovingX) turnPoint = odometer.getY();
						else turnPoint = odometer.getX();
						
						// approach the block slowly
						Robot.alterSpeed("SEARCH");
						Robot.driveForward();
						while (Robot.getDistance() > 6) {
							// keep driving until the robot is 6 cm away, and the light sensor is 4.5cm away
						}
						// distance is less than 6, move even slower
						Robot.alterSpeed("COR");
						// set the block to be searched
						setTargetBlock(tb);
						
						// ToDo: keep moving until a color is detected,
						while (!ifBlockDetected) {
							color = Robot.getColor();

							lightVal[0] = color[0] * 1000.0; // R value
							lightVal[1] = color[1] * 1000.0; // G value
							lightVal[2] = color[2] * 1000.0; // B value

							double differencewithTarget = 100;

							if ((lightVal[0]+lightVal[1]+lightVal[2])/3 < 5) {
								ifBlockDetected = false;
							} else {
								ifBlockDetected = true;
								Robot.stop();
								differencewithTarget = Math.sqrt(Math.pow((lightVal[0] - targetValue[0]), 2)
										+ Math.pow((lightVal[1] - targetValue[1]), 2)
										+ Math.pow((lightVal[2] - targetValue[2]), 2));

								if (differencewithTarget < 12) {
									ifFound = true;
								} else {
									ifFound = false;

									// determine the block if not target block
									double differencewithRed, differencewithYellow = 0;
									double differencewithBlue, differencewithWhite = 0;

									differencewithRed = Math.sqrt(Math.pow((lightVal[0] - 22), 2)
											+ Math.pow((lightVal[1] - 2), 2) + Math.pow((lightVal[2] - 2), 2));
									differencewithYellow = Math.sqrt(Math.pow((lightVal[0] - 39), 2)
											+ Math.pow((lightVal[1] - 28), 2) + Math.pow((lightVal[2] - 4), 2));
									differencewithBlue = Math.sqrt(Math.pow((lightVal[0] - 5), 2)
											+ Math.pow((lightVal[1] - 12), 2) + Math.pow((lightVal[2] - 17), 2));
									differencewithWhite = Math.sqrt(Math.pow((lightVal[0] - 45), 2)
											+ Math.pow((lightVal[1] - 47), 2) + Math.pow((lightVal[2] - 35), 2));

									// 1: Red, 2: Blue, 3: Yellow, 4: White -99: noise
									if ((differencewithRed < 10)) {
										TBColor = 1;
									} else if ((differencewithYellow < 10)) {
										TBColor = 2;
									} else if ((differencewithBlue < 5)) {
										TBColor = 3;
									} else if ((differencewithWhite < 10)) {
										TBColor = 4;
									}
								}
							}
						}
				
				if(ifFound == true) {   //target block is found beep twice
					Sound.twoBeeps();
				}else if(ifFound == false ) {  //beep once
					Sound.beep();
					// addBlockToSearchList(TBColor);
				}
				
				if(direction==1) {
					double xTB=odometer.getX()+Robot.forwardLightSensorOffset+Robot.usSensorOffset+1.5+5;
					colorBlockList.add(new ColorBlock(odometer.getX()+xTB,odometer.getY(), TBColor));
					Robot.travelTo(turnPoint-odometer.getX());
				}else if(direction==3) {
					double xTB=odometer.getX()-Robot.forwardLightSensorOffset-Robot.usSensorOffset-1.5-5;
					colorBlockList.add(new ColorBlock(odometer.getX()+xTB,odometer.getY(), TBColor));
					Robot.travelTo(turnPoint-odometer.getX());
				}else if(direction==2) {
					double yTB=odometer.getY()-Robot.forwardLightSensorOffset-Robot.usSensorOffset-1.5-5;
					colorBlockList.add(new ColorBlock(odometer.getX(),odometer.getY()+yTB, TBColor));
					Robot.travelTo(turnPoint-odometer.getY());
				}else if(direction==4) {
					double yTB=odometer.getY()+Robot.forwardLightSensorOffset+Robot.usSensorOffset+1.5+5;
					colorBlockList.add(new ColorBlock(odometer.getX(),odometer.getY()+yTB, TBColor));
					Robot.travelTo(turnPoint-odometer.getY());
				}
				// turn back and continue with the path
				Robot.turnTo(Math.toRadians(-90));
				Robot.usMotor.rotate(90);
			}
		}
		}
		}
	}

	@Override
	public double readUSDistance() {
		return this.distance;
	}
	/**
	 * This method verifies the distance detected by polling 5 data and calculate the average
	 * @param distance
	 * @return true if the distance detected is a valid detection
	 */
	public boolean verifyDistance(double distance) {
		double totalDistance = 0;
		boolean ifDistanceConfirmed = false;
		
		for(int i = 0; i < 5; i++) {
			totalDistance+= Robot.getDistance();
		}
		
		if(Math.abs( totalDistance/5 - distance) < 5) {
			ifDistanceConfirmed = true;
		}
		System.out.println("Verify distance: "+ifDistanceConfirmed);
		return ifDistanceConfirmed;
	}
	/**
	 * This method adds color block to the list that stores all the color block found
	 * @param int TBColor, the color of the block found
	 */
	public boolean addBlockToSearchList(int TBColor) {
		
		boolean ifAddedBlock = false;
		double sensorX = Robot.odometer.getX();
		double sensorY = Robot.odometer.getY();
		double blockX;
		double blockY;
		
		if(direction == 1) {     //same y sensor has smaller x
			blockY = sensorY;
			blockX = sensorX + 14;  //6 should be changed to an offset variable specified in class Robot
			ColorBlock aColorBlock = new ColorBlock(blockX, blockY, TBColor);
			colorBlockList.add(aColorBlock);
		}else if(direction == 2) {   //same x sensor has larger y
			blockY = sensorY - 14;
			blockX = sensorX;  //6 should be changed to an offset variable specified in class Robot
			ColorBlock aColorBlock = new ColorBlock(blockX, blockY, TBColor);
			colorBlockList.add(aColorBlock);
		}else if(direction == 3) {   //same y sensor has larger x
			blockY = sensorY;
			blockX = sensorX - 14;  //6 should be changed to an offset variable specified in class Robot
			ColorBlock aColorBlock = new ColorBlock(blockX, blockY, TBColor);
			colorBlockList.add(aColorBlock);
		}else if(direction == 4) {   //same x sensor has smaller y
			blockY = sensorY + 14;
			blockX = sensorX;  //6 should be changed to an offset variable specified in class Robot
			ColorBlock aColorBlock = new ColorBlock(blockX, blockY, TBColor);
			colorBlockList.add(aColorBlock);
		}
		System.out.println("Block added to list, total: "+colorBlockList.toString());
		return ifAddedBlock;
	}
	/*
	 * This method determines if the block just found is already searched
	 * @param distance, the distance detected by ultrasonic sensor
	 */
	public boolean isBlockSearched(double distance) {
		boolean isBlockSearched = false;
		
		double sensorX = Robot.odometer.getX();
		double sensorY = Robot.odometer.getY();
		double predictedblockX=-99;
		double predictedblockY=-99;
		
		if(direction == 1) {     //same y sensor has smaller x
			predictedblockY = sensorY;
			predictedblockX = sensorX + distance;  //6 should be changed to an offset variable specified in class Robot
		}else if(direction == 2) {   //same x sensor has larger y
			predictedblockY = sensorY - distance;
			predictedblockY = sensorX;  //6 should be changed to an offset variable specified in class Robot
		}else if(direction == 3) {   //same y sensor has larger x
			predictedblockY = sensorY;
			predictedblockY = sensorX - distance;  //6 should be changed to an offset variable specified in class Robot
		}else if(direction == 4) {   //same x sensor has smaller y
			predictedblockY = sensorY + distance;
			predictedblockY = sensorX;  //6 should be changed to an offset variable specified in class Robot
		}
		Iterator<ColorBlock> cbIterator=colorBlockList.iterator();
		while (cbIterator.hasNext()) {
			ColorBlock aBlock=cbIterator.next();
			if(Math.abs(aBlock.getX()-predictedblockX)<=5&&Math.abs(aBlock.getY()-predictedblockY)<=5) {
				isBlockSearched=true;
			}
		}
		System.out.println("Block searched: "+isBlockSearched);
		return isBlockSearched;
	}
	/**
	 * This method sets the target block using color
	 * @param int tb, the color of the target block
	 */
	public static void setTargetBlock(int tb) {
		// set the RGB value of the target block
		if (tb == 1) {
			targetValue[0] = 22;
			targetValue[1] = 2;
			targetValue[2] = 2;
		} else if (tb == 2) {
			targetValue[0] = 5;
			targetValue[1] = 12;
			targetValue[2] = 17;
		} else if (tb == 3) {
			targetValue[0] = 39;
			targetValue[1] = 28;
			targetValue[2] = 4;
		} else if (tb == 4) {
			targetValue[0] = 45;
			targetValue[1] = 47;
			targetValue[2] = 35;
		}
	}

}