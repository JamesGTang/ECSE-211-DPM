package ca.mcgill.ecse211.ultrasonic;

import lejos.robotics.SampleProvider;

/**
 * 	Recommended Ultrasonic Sensor Range
 * 	After testing is: 3cm- 50cm
 * 	At 70-80 cm, the reading is no longer accurate for sampling
 */
public class UltrasonicPoller extends Thread {
  private SampleProvider us;
  private UltrasonicController cont;
  private float[] usData;
  private boolean isRun=true;
  public UltrasonicPoller(SampleProvider us, float[] usData, UltrasonicController cont) {
    this.us = us;
    this.cont = cont;
    this.usData = usData;
  }

  /*
   * Sensors now return floats using a uniform protocol. Need to convert US result to an integer
   * [0,255] (non-Javadoc)
   * 
   * @see java.lang.Thread#run()
   */
  public void run() {
    double distance;
    while (isRun) {
      us.fetchSample(usData, 0); // acquire data
      distance = usData[0] * 100.0;
      if(isDistanceValid(distance)==true) {
    	  cont.processUSData(distance); 
      }
      else {
  		//distance=prevDistance;
      }
      try {
        Thread.sleep(200);
      } catch (Exception e) {
      }
    }
  }

  /**
   * This method evaluates if the sensor data is valid
   *  based on observation and testing in lab
   *  value above 20000 is due to sensor error
   * @param distance
   * @return boolean
   */
  public boolean isDistanceValid(double distance) {
	  if(distance==2147483647) {
		  return true;
	  }
	  if(distance>=255) {
		  return false;	  
	  }
	  return true;  
  }
  
  public void setRun(boolean status) {
	  isRun=status;
  }
}