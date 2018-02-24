package ca.mcgill.ecse211.controller;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.data.LocalizationData;
import ca.mcgill.ecse211.lightsensor.ColorTest;
import ca.mcgill.ecse211.lightsensor.LightSensorController;
import ca.mcgill.ecse211.model.*;

import ca.mcgill.ecse211.ultrasonic.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.Sound;

public class Lab5 {


	public static OdometryCorrection odometryCorrection;
	// defines data for search area {LLx,LLy,URx,URy,TB,SC} for lab 5 the SC will be set to 0
	public static int coordinates[]= {2,2,6,6,1,0};
	
	@SuppressWarnings("deprecation")
	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;
		// start all threads and all robot in-class services
		Robot.init(coordinates[5]);
		odometryCorrection=new OdometryCorrection(0);
		//System.out.println("Console output directed to terminal");
		
		do {
			// clear the display
			Robot.lcd.clear();
			// ask the user whether the motors should drive with obstacle avoidance
			Robot.lcd.drawString("< Left | Right >", 0, 0);
			Robot.lcd.drawString("L: Color test, ", 0, 1);
			Robot.lcd.drawString("R: Search, ", 0, 2);
			Robot.lcd.drawString("D: Color test, ", 0, 3);
			Robot.lcd.drawString("Please select", 0, 4);
			// Record choice (left or right press)
			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT&&buttonChoice!=Button.ID_DOWN);
		
		if (buttonChoice == Button.ID_LEFT) {
			// when press down, enter color data collection
						//System.out.println("Start color sensor data collection");
						Sound.beepSequenceUp();
						LightSensorController newCont=new LightSensorController() {
							
							@Override
							public int readLightData() {
								return 0;
							}
							
							@Override
							public void processLightData(int tb) {
								
							}
						};
						Robot.lcd.clear();
						ColorTest colorTest=new ColorTest();
						colorTest.start();
						// exit the system on button press
						while (Button.waitForAnyPress() != Button.ID_ESCAPE);
						System.exit(0);
						
		} else if(buttonChoice==Button.ID_RIGHT) {

			// do not start correction yet
			Thread odocorrectionThread=new Thread(odometryCorrection);		
			UltrasonicLocalizer usLocal = new UltrasonicLocalizer(Odometer.getOdometer());
			UltrasonicPoller usPoller = new UltrasonicPoller(Robot.usDistance, Robot.usData, usLocal);
			// run ultrasonic thread last to save resources
			usPoller.start();
			/*
			// update the robot model
			Robot.loc=Robot.LocalizationCategory.FALLING_EDGE;
			// temporary alter speed to slowest for accurate correction
			Robot.alterSpeed("COR");
			System.out.println("US localization: "+Robot.loc);
			// do something with localizer
			System.out.println("Start localization method");
			usLocal.localize();
			// beep twice when the localization finishes
			Sound.twoBeeps();
			System.out.println("Complete Ultrasonic Loc");
			
			LightLocalizer newLightLoc=new LightLocalizer(Odometer.getOdometer());
			newLightLoc.localize();
			Sound.twoBeeps();
			System.out.println("Complete Light Loc");
			// ToDo: implement search method
			//p1: navigate to the point of the starting point
			// set the odometer data accordingly*/
			Robot.setSCOdometer();
			
			if(LocalizationData.setAll(coordinates)) {
				// setting is success
				System.out.println("Coordinate updated!"+LocalizationData.print());
			}else {
				// setting is failed
				System.out.println("Coordinate update failed");
			}
			/**move to the starting coordinate of the search zone, use only travelTo() to avoid cross the search zone unintentionally
			 * 	move to center of tile before going to search zone
			 * ToDo: here we should use odometry correction to make sure the odometer is in sync with robot's actual location
			 */
			// we start the odometer correction thread first
			// change the speed back for small turn
			
			//below if for 
			/*Robot.alterSpeed("SEARCH");
			System.out.println("Travel to center of tile for start");
			Robot.turnTo(Math.toRadians(-45));
			Robot.travelTo(21.55);
			Robot.turnTo(Math.toRadians(45));// move in y direction a distance of dy
			System.out.println("After going to middle of tile x/y"+Odometer.getOdometer().getXYT()[0]+"|"+Odometer.getOdometer().getXYT()[1]);
			System.out.println("Go to starting point of search");
			odocorrectionThread.start();
			// change the speed back for small turn
			Robot.alterSpeed("DRIVE");
			Robot.squareTravelTo((LocalizationData.getLLx()-0.5)*Robot.TILE_SIZE,(LocalizationData.getLLy()-1.5)*Robot.TILE_SIZE);
			System.out.println("Arriving at SC, x/y/theta"+Odometer.getOdometer().getXYT()[0]+"|"+Odometer.getOdometer().getXYT()[1]+"|"+Odometer.getOdometer().getTheta());
			System.out.println("Stopping odometer correction thread");
			// ToDo: use in thread stop machanism to stop the thread
			// turn the sensor in right direction
			Robot.usMotor.rotate(-180);
			odocorrectionThread.stop();*/
			// now start search
			Robot.alterSpeed("SEARCH");
			SearchTargetBlock newSearch=new SearchTargetBlock(coordinates[4]);
			// UltrasonicPoller usSearchPoller = new UltrasonicPoller(Robot.usDistance, Robot.usData,newSearch);	
			// run ultrasonic thread last to save resources
			// usSearchPoller.start();
			newSearch.SearchTarget();
			System.out.println("Search finished");	
			Robot.lcd.drawString("Finished", 0, 1);
			while (Button.waitForAnyPress() != Button.ID_ESCAPE);
			System.exit(0);
		}else if(buttonChoice==Button.ID_DOWN){
			
		}
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
