package ca.mcgill.ecse211.lightsensor;

import ca.mcgill.ecse211.display.Display;

import ca.mcgill.ecse211.model.Robot;
import lejos.hardware.Button;
import lejos.hardware.Sound;

public class ColorTest extends Thread {
	private LightSensorController cont;
	private static float[] usData = new float[Robot.usSensor.sampleSize()];
	public static Display colorDisplay;
	public double redBlockMean[] = new double[] { 97, 12, 5.6 };
	public double blueBlockMean[] = new double[] { 12.65, 57, 25.9 };
	public double yellowBlockMean[] = new double[] { 114.4, 80.8, 8.33 };
	public double whiteBlockMean[] = new double[] { 119, 114.15, 57.5 };

	public double redBlockDev[] = new double[] { 5.57, 1.47, 0.939 };
	public double blueBlockDev[] = new double[] { 1.63, 3.096235822, 1.860715489 };
	public double yellowBlockDev[] = new double[] { 8.942279329, 7.408652396, 1.067993805 };
	public double whiteBlockDev[] = new double[] { 7.280961704, 6.421787875, 2.607317883 };
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