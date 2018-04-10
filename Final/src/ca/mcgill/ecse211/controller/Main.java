package ca.mcgill.ecse211.controller;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.data.GameData;
import ca.mcgill.ecse211.data.Wifi;
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
		odometryCorrection = new OdometryCorrection(0);
		do {
			// clear the display
			Robot.lcd.clear();
			// ask the user whether the motors should drive with obstacle avoidance
			Robot.lcd.drawString("< Left | Right >", 0, 0);
			Robot.lcd.drawString("L: Test, ", 0, 1);
			Robot.lcd.drawString("R: Demo, ", 0, 2);
			Robot.lcd.drawString("DPM Team 5", 0, 3);
			Robot.lcd.drawString("CHIP", 0, 4);
			Robot.lcd.drawString("By JT@jamesgtang.com", 0, 5);
			Robot.lcd.drawString("Xirui. Z", 0, 6);
			
			// Record choice (left or right press)
			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT && buttonChoice != Button.ID_DOWN);

		if (buttonChoice == Button.ID_LEFT) {
			Sound.beepSequenceUp();
			Robot.lcd.clear();
			/*
			 * ColorTest colorTest = new ColorTest(); colorTest.start(); // exit the system
			 * on button press while (Button.waitForAnyPress() != Button.ID_ESCAPE);
			 * System.exit(0);
			 */
			
			/*
			boolean test=true;
			for(int i=0;i<10;i++){
				Robot.centerRobot(15.2,15.2);
				Robot.odometer.setXYT(0, 0, 0);
				Button.waitForAnyPress();
			}*/
			Robot.odometer.setXYT(Robot.TILE_SIZE/2, Robot.TILE_SIZE/2, 0);
			SearchTargetBlock searchTargetBlock=new SearchTargetBlock(2, 4, 4, 1, 1);
			searchTargetBlock.SearchTarget();
			
		} else if (buttonChoice == Button.ID_RIGHT) {
			/*-------- Get data from server----------*/
			int ret = Wifi.getParameter();
			if (ret == -1) {
				// beep two times if server connection failed
				Sound.beep();
				System.out.println("Cannot connect to server");
			} else if (ret == 0) {
				Sound.beepSequenceUp();
				System.out.println("Connected to server");
			} else if (ret == 1) {
				Sound.twoBeeps();
				System.out.println("Unable to fetch data, using local variable");
			}

			/*-------- Localizations----------*/
			UltrasonicLocalizer usLocal = new UltrasonicLocalizer(Odometer.getOdometer());
			UltrasonicPoller usPoller = new UltrasonicPoller(Robot.usDistance, Robot.usData, usLocal);
			// run ultrasonic thread last to save resources
			usPoller.start();
			// update the robot model,use falling edge or rising edge
			if (Robot.getDistance() > 30) {
				Robot.loc = Robot.LocalizationCategory.FALLING_EDGE;
			} else {
				Robot.loc = Robot.LocalizationCategory.RISING_EDGE;
			}
			// temporary alter speed to slowest for accurate correction
			Robot.alterSpeed("COR");
			System.out.println("US localization: " + Robot.loc);
			// perform ultrasonic localization
			System.out.println("Start localization method");
			usLocal.localize();
			System.out.println("Complete Ultrasonic Loc");
			Robot.alterSpeed("DRIVE");
			LightLocalizer newLightLoc = new LightLocalizer(Odometer.getOdometer());
			newLightLoc.localize();
			System.out.println("Complete Light Loc");
			System.out.println("Stop US thread");
			usPoller.setRun(false);
			//if we are playing as green team
			if (GameData.GreenTeam == 5) {
				// set the odometer data accordingly
				Robot.Starting_Corner = GameData.GreenCorner;
				// Important TODO: this must be changed!
				Robot.setBetaSCOdometer();
				// Robot.setSCOdometer();
				Navigation navigation = new Navigation();
				/*-------- Navigation to Tunnel entrance----------*/
				navigation.NavtoTunnelAsGT();
				/*-------- Cross bridge/tunnel----------*/
				Robot.centerRobot((GameData.TN_LL_x + 0.5) * Robot.TILE_SIZE,
						(GameData.TN_LL_y - 0.5) * Robot.TILE_SIZE);
				Robot.odometer.setTheta(0);
				navigation.CrossTunnelAsGT();
				/*-------- Navigation to Search zone-----------*/
				navigation.NavtoSearchZoneAsGT();
				/*-------- Start Searching-----------*/
				SearchTargetBlock searchTB = new SearchTargetBlock(GameData.OR, GameData.SR_UR_x, GameData.SR_UR_y,
						GameData.SR_LL_x, GameData.SR_LL_y);
				//searchTB.SearchTarget();
				/*-------- Navigation back to Bridge or tunnel entrance-----------*/
				navigation.NavtoBridgeAsGT();
				/*-------- Cross bridge or tunnel----------*/
				Robot.centerRobot((GameData.BR_UR_x - 0.5) * Robot.TILE_SIZE,
						(GameData.BR_LL_y + 0.5) * Robot.TILE_SIZE);
				Robot.odometer.setTheta(180);
				navigation.CrossBridgeAsGT();
				/*-------- Navigation to ending point----------*/
				navigation.NavtoSCAsGT();
			// if we are playing as red team	
			} else if (GameData.RedTeam == 5) {
				// set the odometer data accordingly
				Robot.Starting_Corner=GameData.RedCorner;
				// Important TODO: this must be changed!
				Robot.setBetaSCOdometer();
				// Robot.setSCOdometer();
				Navigation navigation = new Navigation();
				/*-------- Navigation to Bridge entrance----------*/
				navigation.NavtoBridgeAsRT();
				/*-------- Cross bridge----------*/
				Robot.centerRobot((GameData.BR_UR_x - 0.5) * Robot.TILE_SIZE,
						(GameData.BR_UR_y + 0.5) * Robot.TILE_SIZE);
				Robot.odometer.setTheta(180);
				navigation.CrossBridgeAsRT();
				/*-------- Navigation to Search zone-----------*/
				navigation.NavtoSearchZoneAsRT();
				/*-------- Start Searching-----------*/
				SearchTargetBlock searchTB = new SearchTargetBlock(GameData.OG, GameData.SG_UR_x, GameData.SG_UR_y,
						GameData.SG_LL_x, GameData.SG_LL_y);
				//searchTB.SearchTarget();
				/*-------- Navigation back to tunnel entrance-----------*/
				navigation.NavtoTunnelAsRT();
				/*-------- Cross Tunnel----------*/
				Robot.centerRobot((GameData.TN_LL_x + 0.5) * Robot.TILE_SIZE,
						(GameData.TN_LL_y - 0.5) * Robot.TILE_SIZE);
				Robot.odometer.setTheta(0);
				navigation.CrossTunnelAsRT();
				/*-------- Navigation to ending point----------*/
				navigation.NavtoSCAsRT();
			}

			while (Button.waitForAnyPress() != Button.ID_ESCAPE);
			System.exit(0);
		}
		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}
}
