package ca.mcgill.ecse211.lightsensor;

import ca.mcgill.ecse211.display.Display;

import ca.mcgill.ecse211.model.Robot;
import lejos.hardware.Sound;

public class ColorTest extends Thread {
	private LightSensorController cont;
	private static float[] usData = new float[Robot.usSensor.sampleSize()];
	public static Display colorDisplay;

	
	// constructor here
	public ColorTest() {
	}
	
	public void run() {
		Robot.lcd.clear();
		while (true) {
			//Robot.colorProvider.fetchSample(color, 0); // acquire data
			//Robot.usSensor.fetchSample(usData, 0);
			
			// print to RSV file format up to two floating point precision
			//System.out.printf("%.2f,%.2f,%.2f,%.2f,%s\n",lightVal[0],lightVal[1],lightVal[1],DISTANCE,EXPECTED_COLOR);
			int ifFound = findColor(Robot.getColor());
			
			if(ifFound == 1) {
				Sound.beep();
				Robot.lcd.drawString("Object detected", 0, 0);
				Robot.lcd.drawString("RED", 0, 1);
				Robot.lcd.drawInt((int)usData[0],0,2);
			}else if(ifFound == 2) {
				Sound.beep();
				Robot.lcd.drawString("Object detected", 0, 0);
				Robot.lcd.drawString("YELLOW", 0, 1);
				Robot.lcd.drawInt((int)usData[0],0,2);
			}else if(ifFound == 3) {
				Sound.beep();
				Robot.lcd.drawString("Object detected", 0, 0);
				Robot.lcd.drawString("BLUE", 0, 1);
				Robot.lcd.drawInt((int)usData[0],0,2);
			}else if(ifFound == 4) {
				Sound.beep();
				Robot.lcd.drawString("Object detected", 0, 0);
				Robot.lcd.drawString("WHITE", 0, 1);
				Robot.lcd.drawInt((int)usData[0],0,2);
			}
			// return value only when a color block is detected
			
			try {
				// 10hz refresh rate
				Thread.sleep(200);
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}
	
	/**
	 * This method checks if the block is a color
	 * @param int: color of the block
	 */
	public boolean isColorBlock(int tb) {
		if(tb!=-1) {
			return true;
		}
		else 
			return false;
	}
	/**
	 * This method calculates the color of the block using RGB value
	 * @return
	 */
	public int findColor(float color[]) {
		// 1: Red, 2: Blue, 3: Yellow, 4: White -99: noise 
		int ifFound = 0;
		double lightVal[]=new double[3];
		
		double differencewithRed, differencewithYellow, differencewithBlue, differencewithWhite = 0;
		
		lightVal[0] = color[0] * 1000.0; // R value
		lightVal[1] = color[1] * 1000.0; // G value
		lightVal[2] = color[2] * 1000.0; // B value
		if(lightVal[2] <= 5) { //when B value is less than 5 there is nothing in front of the robot
			ifFound = 0;	
		}else {
			differencewithRed = Math.sqrt(Math.pow((lightVal[0] - 22), 2) + Math.pow((lightVal[1] - 2), 2) + Math.pow((lightVal[2] - 2), 2));
			differencewithYellow = Math.sqrt(Math.pow((lightVal[0] - 39), 2) + Math.pow((lightVal[1] - 28), 2) + Math.pow((lightVal[2] - 4), 2));
			differencewithBlue = Math.sqrt(Math.pow((lightVal[0] - 5), 2) + Math.pow((lightVal[1] - 12), 2) + Math.pow((lightVal[2] - 17), 2));
			differencewithWhite = Math.sqrt(Math.pow((lightVal[0] - 45), 2) + Math.pow((lightVal[1] - 47), 2) + Math.pow((lightVal[2] - 35), 2));
			
			if((differencewithRed < 10)) {
				ifFound = 1;
				Robot.lcd.drawInt((int)lightVal[0], 0, 2);
				Robot.lcd.drawInt((int)lightVal[1], 0, 3);
				Robot.lcd.drawInt((int)lightVal[2], 0, 4);
				Robot.lcd.drawInt((int)usData[0],0,5);
			}else if( (differencewithYellow < 10)){
				ifFound = 2;
				Robot.lcd.drawInt((int)lightVal[0], 0, 2);
				Robot.lcd.drawInt((int)lightVal[1], 0, 3);
				Robot.lcd.drawInt((int)lightVal[2], 0, 4);
				Robot.lcd.drawInt((int)usData[0],0,5);
			}else if((differencewithBlue < 5)) {
				ifFound = 3;
				Robot.lcd.drawInt((int)lightVal[0], 0, 2);
				Robot.lcd.drawInt((int)lightVal[1], 0, 3);
				Robot.lcd.drawInt((int)lightVal[2], 0, 4);
				Robot.lcd.drawInt((int)usData[0],0,5);
			}else if((differencewithWhite < 10)) {
				ifFound = 4;
				Robot.lcd.drawInt((int)lightVal[0], 0, 2);
				Robot.lcd.drawInt((int)lightVal[1], 0, 3);
				Robot.lcd.drawInt((int)lightVal[2], 0, 4);
				Robot.lcd.drawInt((int)usData[0],0,5);
			}			
		}

		return ifFound;
	}
}