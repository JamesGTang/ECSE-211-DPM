package ca.mcgill.ecse211.controller;

import ca.mcgill.ecse211.odometer.*;

import ca.mcgill.ecse211.data.GameData;
import ca.mcgill.ecse211.data.Wifi;
import ca.mcgill.ecse211.lightsensor.ColorTest;
import ca.mcgill.ecse211.model.*;

import ca.mcgill.ecse211.ultrasonic.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.Sound;
/**
 * Main class of the robot
 * @author jamestang
 *
 */
public class Main {

	public static OdometryCorrection odometryCorrection;

	public static void main(String[] args) throws OdometerExceptions, InterruptedException {

		int buttonChoice;
		// start all threads and all robot in-class services
		Robot.init();
		odometryCorrection=new OdometryCorrection(0);
		do {
			// clear the display
			Robot.lcd.clear();
			// ask the user whether the motors should drive with obstacle avoidance
			Robot.lcd.drawString("< Left | Right >", 0, 0);
			Robot.lcd.drawString("L: Color test, ", 0, 1);
			Robot.lcd.drawString("R: Beta Demo, ", 0, 2);
			Robot.lcd.drawString("Please select", 0, 4);
			// Record choice (left or right press)
			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT&&buttonChoice!=Button.ID_DOWN);
		
		if (buttonChoice == Button.ID_LEFT) {
			Sound.beepSequenceUp();
			Robot.lcd.clear();
			/*
			ColorTest colorTest = new ColorTest();
			colorTest.start();
			// exit the system on button press
			while (Button.waitForAnyPress() != Button.ID_ESCAPE);
			System.exit(0);
			*/
			while(true) {
				Robot.correctLocation(0, 0);
				Button.waitForAnyPress();
			}
		} else if(buttonChoice==Button.ID_RIGHT) {	
			/*-------- Get data from server----------*/
			int ret=Wifi.getParameter();
			if(ret==-1) {
				// beep two times if server connection failed
				Sound.beep();
				System.out.println("Cannot connect to server");
			}else if(ret==0) {
				Sound.beepSequenceUp();
				System.out.println("Connected to server");
			}else if(ret==1) {
				Sound.twoBeeps();
				System.out.println("Unable to fetch data, using local variable");
			}
			
			/*-------- Wait for game to start----------*/
			
			/*-------- Localizations----------*/
			UltrasonicLocalizer usLocal = new UltrasonicLocalizer(Odometer.getOdometer());
			UltrasonicPoller usPoller = new UltrasonicPoller(Robot.usDistance, Robot.usData, usLocal);
			// run ultrasonic thread last to save resources
			usPoller.start();
			// update the robot model,use falling edge or rising edge
			if(Robot.getDistance()>30) {
				Robot.loc=Robot.LocalizationCategory.FALLING_EDGE;
			}else {
				Robot.loc=Robot.LocalizationCategory.RISING_EDGE;
			}				
			// temporary alter speed to slowest for accurate correction
			Robot.alterSpeed("COR");
			System.out.println("US localization: "+Robot.loc);
			// perform ultrasonic localization
			System.out.println("Start localization method");
			usLocal.localize();
			System.out.println("Complete Ultrasonic Loc");
			Robot.alterSpeed("DRIVE");
			LightLocalizer newLightLoc=new LightLocalizer(Odometer.getOdometer());
			newLightLoc.localize();
			System.out.println("Complete Light Loc");
			System.out.println("Stop US thread");
			usPoller.setRun(false);
			// set the odometer data accordingly
			Robot.Starting_Corner=GameData.GreenCorner;
			Robot.setBetaSCOdometer();
			
			Navigation navigation = new Navigation();
			/*-------- Navigation to Bridge or tunnel entrance----------*/
			navigation.NavtoTunnelAsGT();
			/*-------- Cross bridge/tunnel----------*/
			Robot.correctLocation((GameData.TN_LL_x + 0.5) * Robot.TILE_SIZE,
					(GameData.TN_LL_y - 0.5) * Robot.TILE_SIZE);
			Robot.odometer.setTheta(0);
			navigation.CrossTunnelAsGT();
			/*-------- Navigation back to Bridge or tunnel entrance-----------*/
			navigation.NavtoBridgeAsGT();
			/*-------- Cross bridge or tunnel----------*/
			Robot.correctLocation((GameData.SR_UR_x - 0.5) * Robot.TILE_SIZE,
					(GameData.SR_LL_y - 0.5) * Robot.TILE_SIZE);
			Robot.odometer.setTheta(180);
			// this should be removed in final competition, turn robot backward
			navigation.CrossBridgeAsGT();
			/*-------- Navigation to ending point----------*/
			navigation.NavtoSCAsGT();
			
			while (Button.waitForAnyPress() != Button.ID_ESCAPE);
			System.exit(0);
		}
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
