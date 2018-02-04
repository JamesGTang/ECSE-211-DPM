// Lab2.java
package ca.mcgill.ecse211.lab3;

import java.util.ArrayList;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

public class Lab3 {

  // Motor Objects, and Robot related parameters
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  private static final EV3MediumRegulatedMotor usMotor=
		  new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
  
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  public static final double WHEEL_RAD = 2.2;
  public static final double TRACK =15.8;
  
  // the coordinates for the robot to follow
  ArrayList<Integer> x=new ArrayList<Integer>();
  ArrayList<Integer> y=new ArrayList<Integer>();
  public static int[] xPos= {0,1,0,1,2};
  public static int[] yPos= {0,1,2,1,0};
		  
  public static void main(String[] args) throws OdometerExceptions {
	  
    int buttonChoice;
    
    // Odometer related objects
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation
    OdometryCorrection odometryCorrection = new OdometryCorrection(); // TODO Complete  // implementation
    Display odometryDisplay = new Display(lcd); // No need to change

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
      
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();
      // start a new navigation thread
      Navigation newNav=new Navigation(odometer, leftMotor, rightMotor, WHEEL_RAD, TRACK);
      // navigate according to coordinates
      for (int i=0;i<5;i++) {
    	  	newNav.travelTo(xPos[i], yPos[i]);
      }
      lcd.clear();
      lcd.drawString("Nav Complete", 0, 0);
      

    } else {
      // clear the display
      lcd.clear();

      // ask the user whether odometery correction should be run or not
      // left is w/o corection, r is w correction
      lcd.drawString("Avoidng obstables", 0, 0);

      // Start odometer and display threads
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();
      //start obstacle avoidance thread
      NavigationAvoid newNavigationAvoid=new NavigationAvoid(odometer, leftMotor, rightMotor, usMotor, WHEEL_RAD, TRACK);
      for (int i=0;i<5;i++) {
  	  	newNavigationAvoid.travelTo(xPos[i], yPos[i]);
      }
      // spawn a new Thread to avoid SquareDriver.drive() from blocking
      
    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}
