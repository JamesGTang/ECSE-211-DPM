package ca.mcgill.ecse211.controller;

import java.util.ArrayList;
import java.util.Iterator;


import ca.mcgill.ecse211.data.ColorBlock;
import ca.mcgill.ecse211.data.LocalizationData;
import ca.mcgill.ecse211.model.Robot;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.util.robotUtil;
import lejos.hardware.Sound;
import lejos.hardware.Sounds;

public class SearchTargetBlock {
	private double distance;
	// records which side of the search zone the robot is heading now
	// 1: west 2: north 3: east 4:south -1: invalid
	private int direction = -1;

	private boolean isMovingX; // records which direction robot is moving at the moment

	int colorTable[][];
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
	int tb;
	public double redBlockMean[] = new double[] { 91, 11, 0 };
	public double blueBlockMean[] = new double[] { 9.74, 20.35, 23.38 };
	public double yellowBlockMean[] = new double[] { 149.89, 98.01, 8.705 };
	public double whiteBlockMean[] = new double[] { 109.49, 132.15, 65.35 };

	public double redBlockDev[] = new double[] { 21.16945, 3.307641, 3 };
	public double blueBlockDev[] = new double[] { 3.272273, 10.27079, 4.102944 };
	public double yellowBlockDev[] = new double[] { 58.75809, 32.7203, 3.307891 };
	public double whiteBlockDev[] = new double[] { 71.93577, 52.65776, 22.49775 };
	long startTime;

	public SearchTargetBlock(int tb) throws OdometerExceptions {
		odometer = Odometer.getOdometer();
		xRange = (LocalizationData.getURx() - LocalizationData.getLLx()) / 2 * Robot.TILE_SIZE+10;
		yRange = (LocalizationData.getURy() - LocalizationData.getLLy()) / 2 * Robot.TILE_SIZE+10;
		System.out.println("Setting target block");
		this.tb = tb;
		colorTable = setTargetBlock();
		startTime = System.currentTimeMillis();
	}

	public void SearchTarget() throws InterruptedException {
		// Robot.driveForward();
		
		for (int i = 1; i <= 4; i++) {
			if(System.currentTimeMillis()-startTime>=21000) {
				moveToFinalLocation();
				System.out.println("Killed");
				System.exit(0);
			}
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
				double linearOffset = (LocalizationData.getURy()+ 0.5) * Robot.TILE_SIZE;
				// keep driving until the robot's y covers search zone y
				while (odometer.getY() <= linearOffset) {
					// if a block is detected
					distance = Robot.getDistance();
					// System.out.println(distance+"|"+searchRange);
					if (distance < searchRange && !isBlockSearched(distance)) {
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
					Thread.sleep(200);
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
					Thread.sleep(200);
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
					Thread.sleep(200);
				}
			}
			Robot.stop();
			Robot.turnTo(Math.toRadians(90));
		}
	}

	public void discoverBlock(double distance) {
		// indicates if the color block is found
		boolean ifFound = false;
		// the color of the not target block
		int BlockColor = 0; // 1: Red, 2: Blue, 3: Yellow, 4: White -99: noise
		// indicates whether a block is in front or not
		boolean ifBlockDetected = false;
		// the array of light value which increase the RGB value by 10^3 for calculation
		// purpose
		double lightVal[] = new double[3];
		// since us sensor has an angle of detection, we will need to use distance to
		// find out where exactly is the block
		trueOffset = Math.tan(Math.toRadians(Robot.usSensorAngle)) * distance + 5;
		boolean isOverDrove = false;
		double xPrev, yPrev;
		int angleToPerf = 0;

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
<<<<<<< HEAD
			Robot.turnTo(Math.toRadians(90));
			// rotate us motor 90 degree to face forward
			Robot.usMotor.rotate(90);
=======
			
>>>>>>> 73317d1b279c5edb453d770091ef72a1935f37dd
			// record the x and y value now
			xPrev = odometer.getX();
			yPrev = odometer.getY();
			
			// System.out.println("Turn to face the block");
			Robot.turnTo(Math.toRadians(90));
			// rotate us motor 90 degree to face forward
			Robot.usMotor.rotate(120);
			// verify the which kind of block position it is: special scenario, block is
			// faced 45 degree to the x axis
			int i = 0;
			int lEdge = 0;
			int rEdge = 0;

			double radar[] = new double[18];
			// radar the distance of 50 degree

			while (i < 12) {
				// rotate 5 degree at a time to the right
				Robot.usMotor.rotate(-5);
				radar[i] = Robot.getDistance();
				System.out.println("radar: " + radar[i]);
				i = i + 1;
			}

			for (int k = 0; k <= 12; k++) {
				System.out.println("Find left edge");
				if (Math.abs(radar[k] - distance) <= 10) {
					lEdge = k * 5 + 60;
					System.out.println("ledge: " + lEdge);
					break;
				}
			}

			for (int k = 0; k <= 12; k++) {
				System.out.println("Find right edge");
				if (Math.abs(radar[12 - k] - distance) <= 10) {
					rEdge = 120 - k * 5;
					System.out.println("redge: " + rEdge);
					break;
				}
			}

			Robot.usMotor.rotate(+30);

			if ((rEdge - lEdge) / 2 + lEdge > 90) {
				// needs to turn right slightly
				angleToPerf = (rEdge - lEdge) / 2 + lEdge - 90;
			} else if ((rEdge - lEdge) / 2 + lEdge < 90) {
				angleToPerf = (rEdge - lEdge) / 2 + lEdge - 90;
			} else {
				angleToPerf = 0;
			}
			System.out.println("Turn: " + angleToPerf);
			Robot.turnTo(Math.toRadians(angleToPerf));

			// approach the block slowly
			Robot.alterSpeed("FAST");
			System.out.println("Discovering the block by driving toward");

			Robot.driveForward();
			while (Robot.getDistance() > 6 && !isOverDrove) {
				// keep driving until the robot is 6 cm away, and the light sensor is 4.5cm away
				if (robotUtil.getLinearDistance(odometer.getX() - xPrev, odometer.getY() - yPrev) >= distance) {
					isOverDrove = true;
					System.out.println("Distance overdrove");
					returnToPath(xPrev, yPrev, angleToPerf);
					return;
				}
			}
			// distance is less than 6, move even slower
			Robot.alterSpeed("COR");
<<<<<<< HEAD

			// ToDo: keep moving until a color is detected,
			while (!ifBlockDetected&&!isOverDrove) {
				if(robotUtil.getLinearDistance(odometer.getX()-xPrev, odometer.getY()-yPrev)>=distance) {
					isOverDrove=true;
					System.out.println("Distance overdrove");
				}
				System.out.println("Color block detected");
=======
			Robot.travelTo(0);
			System.out.println("Touching block and Calculate color");
			color = Robot.getColor();
			lightVal[0] = color[0] * 1000.0; // R value
			lightVal[1] = color[1] * 1000.0; // G value
			lightVal[2] = color[2] * 1000.0; // B value
			Sound.beep();
			System.out.println("Light val: " + lightVal[0] + "|" + lightVal[1] + "|" + lightVal[2]);
			// 1: Red, 2: Blue, 3: Yellow, 4: White -99: noise
			if (isRed(lightVal)) {
				BlockColor = 1;
			} else if (isYellow(lightVal)) {
				BlockColor = 3;
			} else if (isWhite(lightVal)) {
				BlockColor = 4;
			} else if (isBlue(lightVal)) {
				BlockColor = 2;
			} else {
				Sound.beep();
				BlockColor = 0;
				Robot.travelTo(1);
				
				System.out.println("Touching block again and Calculate color");
>>>>>>> 73317d1b279c5edb453d770091ef72a1935f37dd
				color = Robot.getColor();
				lightVal[0] = color[0] * 1000.0; // R value
				lightVal[1] = color[1] * 1000.0; // G value
				lightVal[2] = color[2] * 1000.0; // B value
				
				if (isRed(lightVal)) {
					BlockColor = 1;
				} else if (isYellow(lightVal)) {
					BlockColor = 3;
				} else if (isWhite(lightVal)) {
					BlockColor = 4;
				} else if (isBlue(lightVal)) {
					BlockColor = 2;
				} else {
					BlockColor = 0;
				}
			}
			
			// if block is still 0, there is no color detected
			if (BlockColor == 0) {
				ifBlockDetected = false;
			} else {
				ifBlockDetected = true;
				Robot.stop();

				if (BlockColor == tb) {
					ifFound = true;
					isTargetBlockFound = true;
					System.out.println("Light val: " + lightVal.toString());
					System.out.println("Target found: " + BlockColor);
					Sound.twoBeeps();
					moveToFinalLocation();
					// ToDo: better handling of exit
					System.exit(0);
				} else {
					ifFound = false;
					System.out.println("Light val: " + lightVal[0] + "|" + lightVal[1] + "|" + lightVal[2]);
					//System.out.println("Color table: " + colorTable[BlockColor]);
					System.out.println("The color block is: " + BlockColor);
					Sound.beep();
				}
			}
			
			Robot.alterSpeed("DRIVE");
			if (direction == 1) {
				// System.out.println("odometer before adding block: " + odometer.getX() + "|" +
				// odometer.getY());
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
			returnToPath(xPrev, yPrev, angleToPerf);
		}
	}

	public void returnToPath(double xPrev, double yPrev, int angleToPerf) {

		// turn back and continue with the path
		Robot.stop();
		Robot.alterSpeed("FAST");
		System.out.println("Turn back to search path");
		System.out.println(xPrev+"|"+yPrev);
		Robot.travelTo(xPrev, yPrev);
		Robot.usMotor.rotate(-90);
		Robot.turnTo(Math.toRadians(90 - angleToPerf));
		Robot.alterSpeed("SEARCH");
		Robot.travelTo(trueOffset * 2); // get out of us sensor's range to avoid detecting the same block again
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
			System.out.println("average distance: " + totalDistance);
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
		System.out.println("Direction when searching for blocklist: " + direction + " " + sensorX + " " + sensorY);
		trueOffset = Math.tan(Math.toRadians(Robot.usSensorAngle)) * distance + 6;

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
<<<<<<< HEAD
			System.out.println("block iterator x, y, blockx,blocky,predx,predy" + sensorX + "|" + sensorY + "|"
					+ aBlock.getX() + "|" + aBlock.getY() + "|" + predictedblockX + "|" + predictedblockY);
			if (Math.abs(aBlock.getX() - predictedblockX-5) <= 15 && Math.abs(aBlock.getY() - predictedblockY-5) <= 15) {
=======
			System.out.println("block iterator x, y, trueoffset,blockx,blocky,predx,predy" + sensorX + "|" + sensorY
					+ "|" + trueOffset + "|" + +aBlock.getX() + "|" + aBlock.getY() + "|" + predictedblockX + "|"
					+ predictedblockY);
			if (Math.abs(aBlock.getX() - predictedblockX) + trueOffset <= 15
					&& Math.abs(aBlock.getY() - predictedblockY) + trueOffset <= 15) {
>>>>>>> 73317d1b279c5edb453d770091ef72a1935f37dd
				isBlockSearched = true;
				System.out.println("Block is searched!!");
			} else {
				System.out.println("Block is not  searched!!");
			}
		}
		return isBlockSearched;
	}

	/**
	 * This method returns the color lookup table
	 * 
	 * @param none
	 */
	public static int[][] setTargetBlock() {
		// set the RGB value of the target block
		RGB rgbColor = new RGB();
		return rgbColor.targetValue;
	}

	/*
	 * This method moves the robot to final postion
	 * 
	 */
	public static void moveToFinalLocation() {
		System.out.println("Moving to ending postion");
		Robot.alterSpeed("FAST");
		Sound.beepSequenceUp();
		Robot.travelTo((LocalizationData.getURx() + 0.5) * Robot.TILE_SIZE,
				(LocalizationData.getURy() + 0.5) * Robot.TILE_SIZE);
	}
	/*
	public static void  timeUp() {
		long startTime =System.nanoTime();
		methodToTime();
		
	}*/

	private boolean isRed(double lightval[]) {
		
		if (lightval[0] < redBlockMean[0] + 2 * redBlockDev[0]
				&& lightval[0] > redBlockMean[0] - 2 * redBlockDev[0]
				&& lightval[1] < redBlockMean[1] + 2 * redBlockDev[1]
				&& lightval[1] > redBlockMean[1] - 2 * redBlockDev[1]
				&& lightval[2] < redBlockMean[2] + 2 * redBlockDev[2]
				&& lightval[2] > redBlockMean[2] - 2 * redBlockDev[2]
		) {
			return true;
		} else
			return false;
	}

	private boolean isBlue(double lightval[]) {
		if (lightval[0] < blueBlockMean[0] + 2 * blueBlockDev[0] 
				&& lightval[0] > blueBlockMean[0] - 2 * blueBlockDev[0]
				&& lightval[1] < blueBlockMean[1] + 2 * blueBlockDev[1]
				&& lightval[1] > blueBlockMean[1] - 2 * blueBlockDev[1]
				&& lightval[2] < blueBlockMean[2] + 2 * blueBlockDev[2]
				&& lightval[2] > blueBlockMean[2] - 2 * blueBlockDev[2])
		{
			return true;
		} else
			return false;
	}

	private boolean isYellow(double lightval[]) {
		if (lightval[0] < yellowBlockMean[0] + 2 * yellowBlockDev[0]
				&& lightval[0] > yellowBlockMean[0] - 2 * yellowBlockDev[0]
				&& lightval[1] < yellowBlockMean[1] + 2 * yellowBlockDev[1]
				&& lightval[1] > yellowBlockMean[1] - 2 * yellowBlockDev[1]
				&& lightval[2] < yellowBlockMean[2] + 2 * yellowBlockDev[2]
				&& lightval[2] > yellowBlockMean[2] - 2 * yellowBlockDev[2])
		{
			return true;
		} else
			return false;
	}

	private boolean isWhite(double lightval[]) {
		if (lightval[0] < whiteBlockMean[0] + 2 * whiteBlockDev[0]
				&& lightval[0] > whiteBlockMean[0] - 2 * whiteBlockDev[0]
				&& lightval[1] < whiteBlockMean[1] + 2 * whiteBlockDev[1]
				&& lightval[1] > whiteBlockMean[1] - 2 * whiteBlockDev[1]
				&& lightval[2] < whiteBlockMean[2] + 2 * whiteBlockDev[2]
				&& lightval[2] > whiteBlockMean[2] - 2 * whiteBlockDev[2]) 
		{
			return true;
		} else
			return false;
	}

}