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
    int distance;
    while (true) {
      us.fetchSample(usData, 0); // acquire data
      distance = (int) (usData[0] * 100.0);
      if(isDistanceValid(distance)==true) {
    	  cont.processUSData(distance); 
      }
      else {
  		//distance=prevDistance;
      }
      try {
        Thread.sleep(50);
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
  public boolean isDistanceValid(int distance) {
	  if(distance==2147483647) {
		  return true;
	  }
	  if(distance>=255) {
		  return false;	  
	  }
	  return true;  
  }
}