package ca.mcgill.ecse211.lightsensor;

import ca.mcgill.ecse211.model.Robot;

public class LightSensor extends Thread {
		private LightSensorController cont;
		private static float[] color=new float[Robot.colorProvider.sampleSize()];
	
	// constructor here
	public LightSensor(LightSensorController cont) {
		this.cont=cont;
	}
	
	public void run() {
		double lightVal[]=new double[3];
		while (true) {
			Robot.colorProvider.fetchSample(color, 0); // acquire data
			lightVal[0] = color[0] * 1000.0; // get R value
			lightVal[1] = color[1] * 1000.0; // get G value
			lightVal[2] = color[2] * 1000.0; // get B value
			int tb=findColor();
			// check if there us an object in front
			if(isColorBlock(tb)) {
				// return value only when a color block is detected
				cont.processLightData(tb);
			}else {
				// its not a color block, just some random noise or the ambient light
			}
			
			try {
				// 10hz refresh rate
				Thread.sleep(100);
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
	public int findColor() {
		// 1: Red, 2: Blue, 3: Yellow, 4: White -1: noise 
		int color=0;
		//ToDO: algorithm that determines the color
		
		return color;
	}
}
