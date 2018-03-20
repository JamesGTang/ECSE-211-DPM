/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.Sound;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private static final double TILE_LENGTH=30.48;
  private static final int BLACK_THRESHOLD=10;
  
  private Odometer odometer; // Odometer object
  private EV3ColorSensor colorSensor; // sensor object
  private SampleProvider colorProvider; // sensor thread to get data
  private float[] color;	// sensor data in array
  private float lightVal; // value of single sensor data
  private double theta;
  private int xLine=0;
  private int yLine=0;
  private static double sensorOffset = 0.0;
  private double xOffset=0;
  private double yOffset=0;
  private boolean isMovingX=true;
  

  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();
    // Improvement: use getColorMode(0
    this.colorSensor=new EV3ColorSensor(SensorPort.S4);
    colorProvider=colorSensor.getRedMode();
    color=new float[colorProvider.sampleSize()];
  }

  /**
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;

    while (true) {
      correctionStart = System.currentTimeMillis();
      // fetch color from Sample Provider thread
      colorProvider.fetchSample(color, 0);
      lightVal=color[0]*1000;
      // if robot is not on the line
      // light sensor in red mode should output value less than 10
      if(lightVal <=BLACK_THRESHOLD){
    	  	// getting the theta value from odometer class
    	  	theta=odometer.theta;
    	  	double offset;
    	  	// if robot is moving in x direction
    	  	if(isMovingX){
    	  		// beep once when in x direction
    	  		Sound.beep();
    	  		// increment xLine and mod it by 3
    	  		// calculate crossed xLine in that quadrant
    	  		xLine++;
    	  		if(xLine==1 || xLine==4){
    	  			// no corretion to be made
    	  			yOffset=TILE_LENGTH*0;
    	  		}else if(xLine==2||xLine==5){
    	  			yOffset=TILE_LENGTH*1.0;
    	  		}else if(xLine==3||xLine==6){
    	  			// set isMovingX to false since robot is now moving y
    	  			yOffset=TILE_LENGTH*2.0;
    	  			isMovingX=false;
    	  		}
    	  		// determine offset sign
    	  		if(xLine<=3) {
    	  			// positive correction
    	  			offset=yOffset;
    	  		}
    	  		else if(xLine!=6) { 
    	  			// on the way back
    	  			offset=2*TILE_LENGTH-yOffset;
    	  		}else{
    	  			offset=2*TILE_LENGTH-yOffset-4;
    	  		}
    	  		odometer.setY(offset);
    	  	}else{
    	  		// beep twice when crossing in y direction
    	  		Sound.twoBeeps();
    	  		yLine++;
    	  		if(yLine==1 || yLine==4){
    	  			// no corretion to be made
    	  			xOffset=TILE_LENGTH*0;
    	  		}else if(yLine==2||yLine==5){
    	  			xOffset=TILE_LENGTH*1.0;
    	  		}else if(yLine==3||yLine==6){
    	  			xOffset=TILE_LENGTH*2.0;
    	  			isMovingX=true;
    	  		}
    	  		// determine offset sign
    	  		if(yLine<=3) {
    	  			// positive correction
    	  			offset=xOffset;
    	  		}
    	  		else { 
    	  			// on the way back
    	  			offset=2*TILE_LENGTH-xOffset;
 
    	  		}
    	  		odometer.setX(offset);
    	  	}
      }
      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }

  // return offset of x due to sensor placement
  private double getsensorOffsetX(){
	  return Math.sin(odometer.theta*sensorOffset);
  }
  // retrun offset of y due to sensor placement
  private double getsensorOffsetY(){
	  return Math.cos(odometer.theta*sensorOffset);
  }
}


