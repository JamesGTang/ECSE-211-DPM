package ca.mcgill.ecse211.controller;

import java.util.ArrayList;
import java.util.Iterator;

import javax.xml.xpath.XPath;

import ca.mcgill.ecse211.data.ColorBlock;
import ca.mcgill.ecse211.data.LocalizationData;
import ca.mcgill.ecse211.model.Robot;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.util.robotUtil;
import lejos.hardware.Sound;

public class SearchTargetBlock {
	private double distance;
	// records which side of the search zone the robot is heading now
	// 1: west 2: north 3: east 4:south -1: invalid
	private int direction = -1;
	private double xBefore = 0; // the x or y value at the point where the robot turned
	private double yBefore=0;
	
	private boolean isMovingX; // records which direction robot is moving at the moment
	private int tb; // color of the target block
	private static double[] targetValue = new double[Robot.colorProvider.sampleSize()];
	private static float[] color = new float[Robot.colorProvider.sampleSize()];
	// maximun allowed distance for block discovery in that quadrant
	private double xRange;
	private double yRange;
	private double searchRange;
	private Odometer odometer;
	// this holds all the color block found
	ArrayList<ColorBlock> colorBlockList = new ArrayList<ColorBlock>();
	int counter = 0; // count the number of blocks in the colorBlockList
	private boolean isTargetBlockFound = false; // this tracks if TB is found
	double trueOffset;
	
	public SearchTargetBlock(int tb) throws OdometerExceptions {
		this.tb = tb;
		odometer = Odometer.getOdometer();
		// ToDo: this line must be taken out before demo! this is used during testing to
		// substitute for falling edge correction
		odometer.setXYT(45.72, 45.72, 0);
		xRange = (LocalizationData.getURx() - LocalizationData.getLLx()) / 2 * Robot.TILE_SIZE;
		yRange = (LocalizationData.getURy() - LocalizationData.getLLy()) / 2 * Robot.TILE_SIZE;
		//System.out.println("Valid x/y: " + xRange + " " + yRange);
		System.out.println("Setting target block");
		setTargetBlock(tb);

	}

	public void SearchTarget() throws InterruptedException {
		// Robot.driveForward();
		for (int i = 1; i <= 4; i++) {
			// set the direction at start
			Robot.alterSpeed("SEARCH");
			Robot.driveForward();
			direction = i;
			if (isMovingX)
				searchRange = yRange;
			else
				searchRange = xRange;
			// drive one vertice at a time
			if (direction == 1 && !isTargetBlockFound) {
				// robot heading alone y
				isMovingX = false;
				System.out.println("Moving alone y, vertice:  " + direction);
				double linearOffset = (LocalizationData.getURy() + 0.5) * Robot.TILE_SIZE;
				// keep driving until the robot's y covers search zone y
				while (odometer.getY() <= linearOffset) {
					// if a block is detected
					distance = Robot.getDistance();
					System.out.println("Distance: "+distance);
					// System.out.println(distance+"|"+searchRange);
					if (distance < searchRange && !isBlockSearched(distance)) {
						/*
						if(isBlockSearched(distance)) {
							// if block is searched, move forward
							Robot.travelTo(15); 
						}*/
						Robot.stop();
						if (verifyDistance(distance)) {
							discoverBlock(Robot.getDistance());
							Robot.driveForward();
						} else {
							// do nothing the distance is false postive
						}
						Robot.driveForward();
					}
					Thread.sleep(200);
				}

			} else if (direction == 2 && !isTargetBlockFound) {
				// robot heading alone x
				isMovingX = true;
				System.out.println("Moving alone x, vertice:  " + direction);
				double linearOffset = (LocalizationData.getURx() + 0.5) * Robot.TILE_SIZE;
				// keep driving until the robot's y covers search zone y
				while (odometer.getX() <= linearOffset) {
					// if a block is detected
					distance = Robot.getDistance();
					if( distance < searchRange && !isBlockSearched(distance)) {
						Robot.stop();
						
						if (verifyDistance(distance)) {
							discoverBlock(Robot.getDistance());
							Robot.driveForward();
						} else {
							// dothing the distance is false postive
						}
						Robot.driveForward();
					}
				}
			} else if (direction == 3 && !isTargetBlockFound) {
				// robot heading alone y
				isMovingX = false;
				System.out.println("Moving alone y, vertice:  " + direction);
				double linearOffset = (LocalizationData.getLLy() - 0.5) * Robot.TILE_SIZE;
				// keep driving until the robot's y covers search zone y
				while (odometer.getY() >= linearOffset) {
					// if a block is detected
					distance = Robot.getDistance();
					if (distance < searchRange && !isBlockSearched(distance)) {
						Robot.stop();
						if (verifyDistance(distance)) {
							discoverBlock(Robot.getDistance());
							Robot.driveForward();
						} else {
							// dothing the distance is false postive
						}
						Robot.driveForward();
					}
				}

			} else if (direction == 4 && !isTargetBlockFound) {
				// robot heading alone x
				isMovingX = true;
				System.out.println("Moving alone x, vertice:  " + direction);
				double linearOffset = (LocalizationData.getLLx() - 0.5) * Robot.TILE_SIZE;
				// keep driving until the robot's y covers search zone y
				while (odometer.getX() >= linearOffset) {
					// if a block is detected
					distance = Robot.getDistance();
					if (distance < searchRange && !isBlockSearched(distance)) {
						Robot.stop();
						if (verifyDistance(distance)) {
							discoverBlock(Robot.getDistance());
							Robot.driveForward();
						} else {
							// dothing the distance is false postive
						}
						Robot.driveForward();
					}
				}
			}
			Robot.stop();
			Robot.turnTo(Math.toRadians(90));
			// ToDo: exit loop if found
		}
	}

	public void discoverBlock(double distance) {
		// indicates if the color block is found
		boolean ifFound = false;
		// the color of the not target block
		int BlockColor = 0; // 1: Red, 2: Blue, 3: Yellow, 4: White -99: noise
		// indicates whether a block is in front or not
		boolean ifBlockDetected = false;
		// the array of light value which increase the RGB value by 10^3 for calculation  purpose
		double lightVal[] = new double[3];
		// since us sensor has an angle of detection, we will need to use distance to find out where exactly is the block
		trueOffset=Math.tan(Math.toRadians(Robot.usSensorAngle))*distance+5;
		boolean isOverDrove=false;
		double xPrev,yPrev;
		//System.out.println("Distance: "+distance);
		//System.out.println("Odometer here: "+odometer.getX()+" "+odometer.getY());
		//System.out.println("True offset according to angle is: "+trueOffset);
		int angleToPerf=0;
		
		if (!ifFound) {
			Robot.stop();
			// there is a block, verify distance
			System.out.println("Found a block here");
			// check if block is searched already
			Sound.beep();
			// if block not searched, stop the wheel
			Robot.stop();
			// align axis with the block
			Robot.travelTo(trueOffset);
			//System.out.println("Turn to face the block");
			Robot.turnTo(Math.toRadians(90));
			// rotate us motor 90 degree to face forward
			Robot.usMotor.rotate(90);
			// verify the which kind of block position it is: special scenario, block is faced 45 degree to the x axis
			int i=0;
			int lEdge=0;
			int rEdge=0;
			
			double radar[]=new double[18];
			// radar the distance of 50 degree
			
			while(i<=10) {
				// rotate 5 degree at a time to the right
				Robot.usMotor.rotate(-(i+1)*5);
				radar[i]=Robot.getDistance();
				i=i+1;
				System.out.println("radar: "+radar[i]);
			}
			
			for(int k=0;k<=10;k++) {
				System.out.println("Find left edge");
				if(Math.abs(radar[k]-distance)<=10) {
					lEdge=k*5;
					System.out.println("ledge: "+lEdge);
					break;
				}
			}
			
			for(int k=0;k<=10;k++) {
				System.out.println("Find left edge");
				if(Math.abs(radar[9-k]-distance)<=10) {
					rEdge=k*5;
					System.out.println("redge: "+rEdge);
					break;
				}
			}
			*/
			// record the x and y value now
			xPrev=odometer.getX();
			yPrev=odometer.getY();
			
			//Robot.usMotor.rotate(25);
			angleToPerf=Math.abs(rEdge-lEdge);
			System.out.println("Turn: "+(rEdge-lEdge));
			Robot.turnTo(Math.toRadians(angleToPerf));
			
			// approach the block slowly
			Robot.alterSpeed("SEARCH");
			System.out.println("Discovering the block by driving toward");
			Robot.driveForward();
			while (Robot.getDistance() > 6&&!isOverDrove) {
				// keep driving until the robot is 6 cm away, and the light sensor is 4.5cm away
				if(robotUtil.getLinearDistance(odometer.getX()-xPrev, odometer.getY()-yPrev)>=distance) {
					isOverDrove=true;
					System.out.println("Distance overdrove");
				}
			}
			// distance is less than 6, move even slower
			Robot.alterSpeed("COR");
			// set the block to be searched
			// ToDo: keep moving until a color is detected,
			while (!ifBlockDetected&&!isOverDrove) {
				if(robotUtil.getLinearDistance(odometer.getX()-xPrev, odometer.getY()-yPrev)>=distance) {
					isOverDrove=true;
					System.out.println("Color sensor overdrove");
				}
				System.out.println("This block is detected");
				Robot.stop();
				color = Robot.getColor();
				lightVal[0] = color[0] * 1000.0; // R value
				lightVal[1] = color[1] * 1000.0; // G value
				lightVal[2] = color[2] * 1000.0; // B value

				double differencewithTarget = 100;

				if ((lightVal[0] + lightVal[1] + lightVal[2]) / 3 < 5) {
					ifBlockDetected = false;
				} else {
					ifBlockDetected = true;
					Robot.stop();
					differencewithTarget = Math.sqrt(
							Math.pow((lightVal[0] - targetValue[0]), 2) + Math.pow((lightVal[1] - targetValue[1]), 2)
									+ Math.pow((lightVal[2] - targetValue[2]), 2));

					if (differencewithTarget < 12) {
						ifFound = true;
					} else {
						ifFound = false;

						// determine the block if not target block
						double differencewithRed, differencewithYellow = 0;
						double differencewithBlue, differencewithWhite = 0;

						differencewithRed = Math.sqrt(Math.pow((lightVal[0] - 22), 2) + Math.pow((lightVal[1] - 2), 2)
								+ Math.pow((lightVal[2] - 2), 2));
						differencewithYellow = Math.sqrt(Math.pow((lightVal[0] - 39), 2)
								+ Math.pow((lightVal[1] - 28), 2) + Math.pow((lightVal[2] - 4), 2));
						differencewithBlue = Math.sqrt(Math.pow((lightVal[0] - 5), 2) + Math.pow((lightVal[1] - 12), 2)
								+ Math.pow((lightVal[2] - 17), 2));
						differencewithWhite = Math.sqrt(Math.pow((lightVal[0] - 45), 2)
								+ Math.pow((lightVal[1] - 47), 2) + Math.pow((lightVal[2] - 35), 2));

						// 1: Red, 2: Blue, 3: Yellow, 4: White -99: noise
						if ((differencewithRed < 15)) {
							BlockColor = 1;
						} else if ((differencewithYellow < 15)) {
							BlockColor = 2;
						} else if ((differencewithBlue < 15)) {
							Sound.beepSequenceUp();
							BlockColor= 3;
						} else if ((differencewithWhite < 15)) {
							BlockColor = 4;
						}
						System.out.println("The color block is: "+color);
					}
				}
			}

			if (ifFound == true) { // target block is found beep twice
				isTargetBlockFound=true;
				Sound.twoBeeps();
				moveToFinalLocation();
				// ToDo: better handling of exit
				System.exit(0);
			} else if (ifFound == false) { // beep once
				Sound.beep();
				// addBlockToSearchList(TBColor);
			}
			Robot.alterSpeed("DRIVE");
			if (direction == 1) {
				//System.out.println("odometer before adding block: " + odometer.getX() + "|" + odometer.getY());
				double xTB = odometer.getX() + Robot.forwardLightSensorOffset + Robot.usSensorOffset + 1.5 + 5;
				colorBlockList.add(new ColorBlock(xTB, odometer.getY(), BlockColor));
			} else if (direction == 3) {
				double xTB = odometer.getX() - Robot.forwardLightSensorOffset - Robot.usSensorOffset - 1.5 - 5;
				colorBlockList.add(new ColorBlock(xTB, odometer.getY(), BlockColor));
			} else if (direction == 2) {
				double yTB = odometer.getY() - Robot.forwardLightSensorOffset - Robot.usSensorOffset - 1.5 - 5;
				colorBlockList.add(new ColorBlock(odometer.getX(), yTB, BlockColor));
			} else if (direction == 4) {
				double yTB = odometer.getY() + Robot.forwardLightSensorOffset + Robot.usSensorOffset + 1.5 + 5;
				colorBlockList.add(new ColorBlock(odometer.getX(), yTB, BlockColor));
			}
			System.out.println("Blocked added: " + colorBlockList.toString());
			// turn back and continue with the path
			returnToPath(xPrev,yPrev,angleToPerf);
		}
	}
	
	public void returnToPath(double xPrev,double yPrev,int angleToPerf) {
		/*
		if (direction == 1) {
			Robot.travelTo(turnPoint - odometer.getX());
		} else if (direction == 3) {
			Robot.travelTo(odometer.getX()-turnPoint);
		} else if (direction == 2) {
			System.out.println("Move in 2: "+(odometer.getY()-turnPoint));
			Robot.travelTo(odometer.getY()-turnPoint);
		} else if (direction == 4) {
			Robot.travelTo(odometer.getY()-turnPoint);
		}
		*/
		// turn back and continue with the path
		System.out.println("Turn back to search path");
		Robot.travelTo(xPrev, yPrev);
		Robot.usMotor.rotate(-90);
		Robot.turnTo(Math.toRadians(-90-angleToPerf));
		Robot.travelTo(trueOffset*2); // get out of us sensor's range to avoid detecting the same block again
		Robot.alterSpeed("SEARCH");
	}
	/**
	 * This method verifies the distance detected by polling 5 data and calculate
	 * the average
	 * 
	 * @param distance
	 * @return true if the distance detected is a valid detection
	 */
	public boolean verifyDistance(double distance) {
		double totalDistance = 0;
		boolean isDistanceValid = false;
		
		for (int i = 0; i < 5; i++) {
			totalDistance += Robot.getDistance();
			System.out.println("average distance: "+totalDistance);
		}

		if (Math.abs(totalDistance / 5 - distance) < 5) {
			isDistanceValid = true;
		}
		System.out.println("Detected distance: " + isDistanceValid);
		System.out.println("Verify distance: " + isDistanceValid);
		return isDistanceValid;
	}

	/**
	 * This method adds color block to the list that stores all the color block
	 * found
	 * 
	 * @param int
	 *            TBColor, the color of the block found
	 */
	public boolean addBlockToSearchList(int TBColor) {

		boolean ifAddedBlock = false;
		double sensorX = odometer.getX();
		double sensorY = odometer.getY();
		double blockX;
		double blockY;

		if (direction == 1) { // same y sensor has smaller x
			blockY = sensorY;
			blockX = sensorX + 14; // 6 should be changed to an offset variable specified in class Robot
			ColorBlock aColorBlock = new ColorBlock(blockX, blockY, TBColor);
			colorBlockList.add(aColorBlock);
		} else if (direction == 2) { // same x sensor has larger y
			blockY = sensorY - 14;
			blockX = sensorX; // 6 should be changed to an offset variable specified in class Robot
			ColorBlock aColorBlock = new ColorBlock(blockX, blockY, TBColor);
			colorBlockList.add(aColorBlock);
		} else if (direction == 3) { // same y sensor has larger x
			blockY = sensorY;
			blockX = sensorX - 14; // 6 should be changed to an offset variable specified in class Robot
			ColorBlock aColorBlock = new ColorBlock(blockX, blockY, TBColor);
			colorBlockList.add(aColorBlock);
		} else if (direction == 4) { // same x sensor has smaller y
			blockY = sensorY + 14;
			blockX = sensorX; // 6 should be changed to an offset variable specified in class Robot
			ColorBlock aColorBlock = new ColorBlock(blockX, blockY, TBColor);
			colorBlockList.add(aColorBlock);
		}
		System.out.println("Block added to list, total: " + colorBlockList.toString());
		return ifAddedBlock;
	}

	/*
	 * This method determines if the block just found is already searched
	 * 
	 * @param distance, the distance detected by ultrasonic sensor
	 */
	public boolean isBlockSearched(double distance) {
		boolean isBlockSearched = false;

		double sensorX = odometer.getX();
		double sensorY = odometer.getY();
		// System.out.println("in IsBlocke, x/y"+sensorX+" "+sensorY);
		double predictedblockX = -99;
		double predictedblockY = -99;
		System.out.println("Direction when searching for blocklist: "+direction+" "+sensorX+" "+sensorY);
		trueOffset=Math.tan(Math.toRadians(Robot.usSensorAngle))*distance+6;
		
		if (direction == 1) { // same y sensor has smaller x
			predictedblockY = sensorY + 8;
			predictedblockX = sensorX + distance + 5; // 6 should be changed to an offset variable specified in class
														// Robot
		} else if (direction == 2) { // same x sensor has larger y
			predictedblockY = sensorY - distance - 5;
			predictedblockX = sensorX + 8; // 6 should be changed to an offset variable specified in class Robot
		} else if (direction == 3) { // same y sensor has larger x
			predictedblockY = sensorY - 8;
			predictedblockY = sensorX - distance - 5; // 6 should be changed to an offset variable specified in class
														// Robot
		} else if (direction == 4) { // same x sensor has smaller y
			predictedblockY = sensorY + distance + 5;
			predictedblockY = sensorX - 8; // 6 should be changed to an offset variable specified in class Robot
		}
		Iterator<ColorBlock> cbIterator = colorBlockList.iterator();
		while (cbIterator.hasNext()) {
			ColorBlock aBlock = cbIterator.next();
			System.out.println("block iterator x, y, trueoffset,blockx,blocky,predx,predy" + sensorX + "|" + sensorY + "|"+trueOffset+"|"+
					+ aBlock.getX() + "|" + aBlock.getY() + "|" + predictedblockX + "|" + predictedblockY);
			if (Math.abs(aBlock.getX() - predictedblockX)+trueOffset <= 15 && Math.abs(aBlock.getY() - predictedblockY)+trueOffset <= 15) {
				isBlockSearched = true;
				System.out.println("Block is searched!!");
			}else {
				System.out.println("Block is not  searched!!");
			}
		}
		// System.out.println("Block searched: " + isBlockSearched);
		return isBlockSearched;
	}

	/**
	 * This method sets the target block using color
	 * 
	 * @param int
	 *            tb, the color of the target block
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
	/*
	 * This method moves the robot to final postion
	 * 
	 */
	public static void moveToFinalLocation() {
		System.out.println("Moving to ending postion");
		Robot.alterSpeed("FAST");
		Sound.beepSequenceUp();
		Robot.travelTo((LocalizationData.getURx()+1)*Robot.TILE_SIZE, (LocalizationData.getURy()+1)*Robot.TILE_SIZE);
	}

}