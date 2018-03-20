// Lab2.java
package ca.mcgill.ecse211.lab3;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.ultrasonic.UltrasonicController;
import ca.mcgill.ecse211.ultrasonic.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab3 {

  // Motor Objects, and Robot related parameters
  public static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
  public static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  public static final EV3LargeRegulatedMotor usMotor=
		  new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  public static final double WHEEL_RAD = 2.2;
  public static final double TRACK = 15.8;
  
	public static SensorModes usSensor = new EV3UltrasonicSensor(SensorPort.S4); // the instance
	static SampleProvider usDistance = usSensor.getMode("Distance"); // provides samples from this instance
	static float[] usData = new float[usDistance.sampleSize()];
	static UltrasonicController usController;
	
	public static Odometer odometer;
	public static Display odometryDisplay;

  // the coordinates for the robot to follow
  public static int[] xPos= {1,0,2,2,1};
  public static int[] yPos= {1,2,2,1,0};  
  public static void main(String[] args) throws OdometerExceptions {
	  
    int buttonChoice;
    // Odometer related objects
    odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation
    odometryDisplay = new Display(lcd); // No need to change
	
    System.out.println("Remote user connected");
    do {
      // clear the display
      lcd.clear();
      // ask the user whether the motors should drive with obstacle avoidance
      lcd.drawString("< Left | Right >", 0, 0);
      lcd.drawString("L: drive, R: avoid", 0, 1); 

      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    if (buttonChoice == Button.ID_LEFT) {
      // Display changes in position as wheels are (manually) moved
    	  System.out.println("Start Nav w/o avoidance");
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();
      // start a new navigation thread
      Navigation newNav=new Navigation(odometer, leftMotor, rightMotor, TRACK, WHEEL_RAD);
      newNav.start();
      // navigate according to coordinates
      for (int i=0;i<5;i++) {
    	  	System.out.println("Travelling "+i);
    	  	newNav.travelTo(xPos[i], yPos[i]);
    	  	while(newNav.isFinished()) {
    	  		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    	        System.out.println("Nav finished");
	    		System.exit(0);
    	  	}
      }
      Sound.twoBeeps();
      lcd.clear();
      lcd.drawString("Nav Complete", 0, 0);
      
    } else {
      // clear the display
      lcd.clear();
      // ask the user whether odometery correction should be run or not
      // left is w/o correction, r is w correction
      lcd.drawString("Avoidng obstables", 0, 0);
      System.out.println("Start Nav w avoidance");
      // Start odometer and display threads
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();
      //start obstacle avoidance thread
      final NavigationAvoid newNavigationAvoid=new NavigationAvoid();
      UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, newNavigationAvoid);
      usPoller.start();
      newNavigationAvoid.start();
      
      System.out.println("Starting nav avoid thread");
      for (int i=0;i<5;i++) {
    	  	System.out.println("Travelling "+i);
  	  	newNavigationAvoid.travelTo(xPos[i], yPos[i]);
  	  	while(newNavigationAvoid.isFinished()) {
	  		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
	  		System.out.println("Nav finished");
	  		System.exit(0);
	  	}
      } 
      Sound.twoBeeps();
    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}
