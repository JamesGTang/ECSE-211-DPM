package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.*;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 20;
  public static final double PROPCONST = 2; // Proportionality constant
  public static final int FWDSPEED = 200; // Forward speed (deg/sec)
  public static final int MAXCORRECTION = 50; 
  
  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl=0;
  
  //define class variables
  public static final int allowedDistance = 45;
  public static final int allowedDeviation =3;
  public static final int stopDistance = 15;
  public int cor=0;
  public int distError=0;
  
  //define lcd object
  public static TextLCD t = LocalEV3.get().getTextLCD();

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;
    // Initalize motor rolling forward
    /*
    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
    */
  }

  @Override
  public void processUSData(int distance) {
	/*
	// validate distance and mare sure the distance is not invalid
	// if distance is above tolerence, use last usable data instead
	// test 0: status: not tested
	// situation: general
	// response: no distance is above 255
	 * 
	 */ 
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }
	// measure how far off the sensor is from the allowedDistance
	distError = allowedDistance - distance;
	
	// apply corrections using speed
    if (distance<=stopDistance) {
    		/*
    		// apply emergency course alteration to avoid crash
    		// test 1: status: not tested
    		// situation: sensor value <= 15 response: 
    		// response: right Motor rotates backward 2 circles
    		 *
    		 */
    	WallFollowingLab.leftMotor.stop();
		WallFollowingLab.rightMotor.stop();
		WallFollowingLab.rightMotor.rotate(-120);
		WallFollowingLab.leftMotor.rotate(30);
		updateStatus("Avoiding crash", distance,-1,-2);
		//System.out.println("Avoiding crash. "+distance+"-1 -2");
  	}else if(Math.abs(distError)<=allowedDeviation) {
  		/*
  		// maintain current course
		// test 2: status: not tested
		// situation: sensor value between 27 and 33 
		// response: move straight
		 * 
		 */
    	WallFollowingLab.leftMotor.setSpeed(FWDSPEED);
    	WallFollowingLab.rightMotor.setSpeed(FWDSPEED);
    	WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
        updateStatus("Go straight", distance,FWDSPEED,FWDSPEED);
        //System.out.println("Go Straight. "+distance+"-1 -2");
    		//Printer.updateLCD("Distance; "+distance+"Turn right");
    }else if(distError >0) {
    		/*	
    		// turn right
		// test 3: status: not tested
		// situation: sensor value between 15 and 27 
		// response: turn right
		 */
    		cor = calcDeviation(distError);
    		WallFollowingLab.leftMotor.setSpeed(FWDSPEED+cor);
    		WallFollowingLab.rightMotor.setSpeed(FWDSPEED-cor);
    		WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
        updateStatus("Turn right", distance,(FWDSPEED+cor),(FWDSPEED-cor));
        
		//Printer.updateLCD("Distance; "+distance+"Move Straight");
        //System.out.println("Turn right. "+distance+" "+(FWDSPEED+cor)+" "+(FWDSPEED-cor));
    		//Printer.updateLCD("Distance; "+distance+"Move Straight");
    }else if(distError<=0){
    		/*
    		// turn left
		// test 3: status: not tested
		// situation: sensor value >=33 
		// response: turn left
		 */
    	cor = calcDeviation(distError);
    	WallFollowingLab.leftMotor.setSpeed(FWDSPEED-cor);
		WallFollowingLab.rightMotor.setSpeed(FWDSPEED+cor);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
        updateStatus("Turn left", distance,FWDSPEED-cor,FWDSPEED+cor);
        //System.out.println("Turn left. "+distance+" "+(FWDSPEED-cor)+" "+(FWDSPEED+cor));
    		//Printer.updateLCD("Distance; "+distance+"Turn Left");
    }
}
  
  @Override
  public int readUSDistance() {
    return this.distance;
  }
  /* 
   * This method calculates porportion and return correction
   */
  int calcDeviation(int err) {
	  int correction;
	  err = Math.abs(err);
	  
	  correction = (int) (PROPCONST* (double) err);
	  if(correction >=FWDSPEED) {
		  /* 
		   * to prevent one wheel from stalling the speed o
		   * must not go below 50
		   * test : status:not tested
		   * situation: general
		   * output:not wheel speed is below 50
		   */
		  correction = MAXCORRECTION;
	  }
	  // print correction to screen
	  t.drawString("Correction: "+correction, 0, 5);
	  return correction;
  }
  /* 
   * a class that prints to LCD
   */
  public static void updateStatus(String update,int distance2,int lSpeed,int rSpeed) {
	  t.clear();
	  t.drawString("Status: "+update, 0, 0);
	  t.drawString("Distance: "+distance2, 0, 1);
	  t.drawString("Left Speed: "+lSpeed, 0, 2);
	  t.drawString("Right Speed: "+rSpeed, 0,3);
	  t.drawString("------------------", 0,4);
  }
}