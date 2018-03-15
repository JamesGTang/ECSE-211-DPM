package ca.mcgill.ecse211.lightsensor;

import ca.mcgill.ecse211.display.Display;

import ca.mcgill.ecse211.model.Robot;
import lejos.hardware.Button;
import lejos.hardware.Sound;

public class ColorTest extends Thread {
	private LightSensorController cont;
	private static float[] usData = new float[Robot.usSensor.sampleSize()];
	public static Display colorDisplay;
	public double redBlockMean[] = new double[] { 91, 11, 0 };
	public double blueBlockMean[] = new double[] { 9.74, 20.35, 23.38 };
	public double yellowBlockMean[] = new double[] { 149.89, 98.01, 8.705 };
	public double whiteBlockMean[] = new double[] { 109.49, 132.15, 65.35 };

	public double redBlockDev[] = new double[] { 21.16945, 3.307641, 3 };
	public double blueBlockDev[] = new double[] { 3.272273, 10.27079, 4.102944 };
	public double yellowBlockDev[] = new double[] { 58.75809, 32.7203, 3.307891 };
	public double whiteBlockDev[] = new double[] { 71.93577, 52.65776, 22.49775 };
	int BlockColor;
	
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
			float color[]=Robot.getColor();
			double lightVal[] = new double[3];
			lightVal[0] = color[0] * 1000.0; // R value
			lightVal[1] = color[1] * 1000.0; // G value
			lightVal[2] = color[2] * 1000.0; // B value
			
			if(isRed(lightVal)) {
				Sound.beep();
				Robot.lcd.clear();
				Robot.lcd.drawString("Object detected", 0, 0);
				Robot.lcd.drawString("RED", 0, 1);
			}else if(isYellow(lightVal)) {
				Sound.beep();
				Robot.lcd.clear();
				Robot.lcd.drawString("Object detected", 0, 0);
				Robot.lcd.drawString("YELLOW", 0, 1);
			}else if(isBlue(lightVal)) {
				Sound.beep();
				Robot.lcd.clear();
				Robot.lcd.drawString("Object detected", 0, 0);
				Robot.lcd.drawString("BLUE", 0, 1);
			}else if(isWhite(lightVal)) {
				Sound.beep();
				Robot.lcd.clear();
				Robot.lcd.drawString("Object detected", 0, 0);
				Robot.lcd.drawString("WHITE", 0, 1);
			}
			// return value only when a color block is detected
			
			try {
				// 10hz refresh rate
				Thread.sleep(400);
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