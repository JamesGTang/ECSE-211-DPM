/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.odometer;

import java.util.LinkedList;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  // Motors and related variables
  // ToDo: implement as queue data structure  
  LinkedList<Integer> leftMotorTachoCount;
  LinkedList<Integer> rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  // constant information about the robot design
  private final double TRACK;
  private final double WHEEL_RAD;
  
  // delta variables
  double theta;

  /*
   * dLeft: left wheel change
   * dRight: right wheel change
   * dTheta: change in angle
   * dPostion:  change in postion
   */
  

  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    
    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);
    this.leftMotorTachoCount=new LinkedList<Integer>();
    this.rightMotorTachoCount=new LinkedList<Integer>();
    // Add first tachocount
    leftMotorTachoCount.addFirst(0);
    rightMotorTachoCount.addFirst(0);
    //this.rightMotorTachoCount.clear();

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;

  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      return odo;
    }
  }

  /**
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   * 
   * @return error if no previous odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");

    }
    return odo;
  }

  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   */
  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();
      double deltaTheta;
      double deltaDist;
      double dLeft,dRight, dX,dY;
      // adds new count to the end of the list
      leftMotorTachoCount.addLast(leftMotor.getTachoCount());
      rightMotorTachoCount.addLast(rightMotor.getTachoCount());
      dLeft=Math.PI*WHEEL_RAD*(leftMotorTachoCount.getLast()-leftMotorTachoCount.removeFirst())/180;
      dRight=Math.PI*WHEEL_RAD*(rightMotorTachoCount.getLast()-rightMotorTachoCount.removeFirst())/180;
      deltaTheta=(dLeft-dRight)/TRACK;
      deltaDist=(dLeft+dRight)*0.5;
      // calcualte new x, y and theta displacement
      theta=(theta+deltaTheta);
      //if theta is over 360, reset theta
      if(Math.toDegrees(theta)>=360){
    	  	theta=Math.toRadians(theta-360);
      }
      dX=deltaDist*Math.sin(theta);
      dY=deltaDist*Math.cos(theta);
      // update odometer value
      odo.update(dX, dY,deltaTheta*180/Math.PI);
      deltaTheta=0;
      dLeft=0;
      dRight=0;
      

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }

}
