package ca.mcgill.ecse211.wallfollowing;

import lejos.robotics.SampleProvider;

/**
 * Control of the wall follower is applied periodically by the UltrasonicPoller thread. The while
 * loop at the bottom executes in a loop. Assuming that the us.fetchSample, and cont.processUSData
 * methods operate in about 20mS, and that the thread sleeps for 50 mS at the end of each loop, then
 * one cycle through the loop is approximately 70 mS. This corresponds to a sampling rate of 1/70mS
 * or about 14 Hz.
 */
/*
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
      //cont.processUSData(distance); // now take action depending on value
      try {
        Thread.sleep(100);
      } catch (Exception e) {
      }
    }
  }
  /**
   *  based on obserrvation and testing in lab
   *  value above 20000 is due to sensor error
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