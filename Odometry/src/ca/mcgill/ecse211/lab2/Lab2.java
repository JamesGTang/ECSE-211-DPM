// Lab2.java
package ca.mcgill.ecse211.lab2;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Lab2 {

  // Motor Objects, and Robot related parameters
// ToDo: change parameter here	
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
 // ToDo: change wheel constant here
  public static final double WHEEL_RAD = 2.2;
  public static final double TRACK =15.8;

  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;

    // Odometer related objects
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation
    OdometryCorrection odometryCorrection = new OdometryCorrection(); // TODO Complete  // implementation
    Display odometryDisplay = new Display(lcd); // No need to change

    do {
      // clear the display
      lcd.clear();

      // ask the user whether the motors should drive in a square or float
      // left is float, right is square
      lcd.drawString("< Left | Right >", 0, 0);
      lcd.drawString("L: float, R: square", 0, 1); 

      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    if (buttonChoice == Button.ID_LEFT) {
      // set the motors into float 
    	 // motor will stop the motor without braking and the position of the motor will not be maintained.
      leftMotor.forward();
      leftMotor.flt();
      rightMotor.forward();
      rightMotor.flt();

      // Display changes in position as wheels are (manually) moved
      
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();

    } else {
      // clear the display
      lcd.clear();

      // ask the user whether odometery correction should be run or not
      // left is w/o corection, r is w correction
      lcd.drawString("< Left | Right >", 0, 0);
      lcd.drawString("L: W/O Correction",0,1); 
      lcd.drawString("R: w Correction",  0, 2); 
      
      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)

      // Start odometer and display threads
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();

      // Start correction if right button was pressed
      if (buttonChoice == Button.ID_RIGHT) {
        Thread odoCorrectionThread = new Thread(odometryCorrection);
        odoCorrectionThread.start();
      }

      // spawn a new Thread to avoid SquareDriver.drive() from blocking
      (new Thread() {
        public void run() {
          SquareDriver.drive(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK);
        }
      }).start();
    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}
