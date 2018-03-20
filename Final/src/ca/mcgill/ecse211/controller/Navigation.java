package ca.mcgill.ecse211.controller;

import ca.mcgill.ecse211.data.GameData;
import ca.mcgill.ecse211.model.Robot;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
/**
 * This class provides navigations methods that corresponds to the state of the robot
 * @author jamestang
 *
 */
public class Navigation {
	/**
	 * Empty Contructor
	 */
	public Navigation() {
	}
	/**
	 * This method navigates the robot as green team to the entrance of the tunnel as green team
	 * @throws OdometerExceptions 
	 */
	public void NavtoTunnelAsGT() throws OdometerExceptions {
		// change the speed back for small turn	
		System.out.println("Going to Tunnel");
		Robot.alterSpeed("DRIVE");
		Robot.squareTravelTo((GameData.TN_LL_x-0.5)*Robot.TILE_SIZE,(GameData.TN_LL_y-0.5)*Robot.TILE_SIZE);
		System.out.println("Arrived at Tunnel Entrance, x/y/theta"+Odometer.getOdometer().getXYT()[0]+"|"+Odometer.getOdometer().getXYT()[1]+"|"+Odometer.getOdometer().getTheta());
	}
	/**
	 * This method lets the robot as green team cross the tunnel
	 * @throws OdometerExceptions 
	 */
	public void CrossTunnelAsGT() throws OdometerExceptions {
		// change the speed back for small turn	
		System.out.println("Crossing Tunnel");
		Robot.alterSpeed("DRIVE");
		Robot.travelTo((GameData.TN_UR_y-GameData.TN_LL_y+1)*Robot.TILE_SIZE);
		System.out.println("Crossed tunnel, x/y/theta"+Odometer.getOdometer().getXYT()[0]+"|"+Odometer.getOdometer().getXYT()[1]+"|"+Odometer.getOdometer().getTheta());
	}
	
	/**
	 * This method moves the robot as green team to starting position of the search zone
	 * @throws OdometerExceptions 
	 */
	public void NavtoSearchZoneAsGT() throws OdometerExceptions {
		// change the speed back for small turn	
		System.out.println("Going to search zone");
		Robot.alterSpeed("DRIVE");
		Robot.squareTravelTo((GameData.SR_UR_x-0.5)*Robot.TILE_SIZE,(GameData.SR_LL_y-0.5)*Robot.TILE_SIZE);
		System.out.println("Arrived searchzone, x/y/theta"+Odometer.getOdometer().getXYT()[0]+"|"+Odometer.getOdometer().getXYT()[1]+"|"+Odometer.getOdometer().getTheta());
	}
	/**
	 * This method navigates the robot as green team to the entrance of the bridge as green team
	 * @throws OdometerExceptions 
	 */
	public void NavtoBridgeAsGT() throws OdometerExceptions {
		// change the speed back for small turn	
		System.out.println("Going to Bridge");
		Robot.alterSpeed("DRIVE");
		Robot.squareTravelTo((GameData.BR_UR_x-0.5)*Robot.TILE_SIZE,(GameData.BR_UR_y+0.5)*Robot.TILE_SIZE);
		System.out.println("Arrived at Tunnel Entrance, x/y/theta"+Odometer.getOdometer().getXYT()[0]+"|"+Odometer.getOdometer().getXYT()[1]+"|"+Odometer.getOdometer().getTheta());
	}
	/**
	 * This method lets the robot as green team cross bridge
	 * @throws OdometerExceptions 
	 */
	public void CrossBridgeAsGT() throws OdometerExceptions {
		// change the speed back for small turn	
		System.out.println("Crossing Bridge");
		Robot.alterSpeed("DRIVE");
		Robot.travelTo((GameData.BR_UR_y-GameData.BR_LL_y+1)*Robot.TILE_SIZE);
		System.out.println("Crossed bridge, x/y/theta"+Odometer.getOdometer().getXYT()[0]+"|"+Odometer.getOdometer().getXYT()[1]+"|"+Odometer.getOdometer().getTheta());
	}
	/**
	 * This method navigates the robot as green team to the starting point
	 * @throws OdometerExceptions 
	 */
	public void NavtoSCAsGT() throws OdometerExceptions {
		// change the speed back for small turn	
		System.out.println("Going to SC");
		Robot.alterSpeed("DRIVE");
		Robot.squareTravelTo(Robot.startingX,Robot.startingY);
		System.out.println("Arrived at Tunnel Entrance, x/y/theta"+Odometer.getOdometer().getXYT()[0]+"|"+Odometer.getOdometer().getXYT()[1]+"|"+Odometer.getOdometer().getTheta());
	}
	/**
	 * This method navigates the robot as red team to the entrance of the bridge
	 * @throws OdometerExceptions 
	 */
	public void NavtoBridgeAsRT() throws OdometerExceptions{
		
	}
	/**
	 * This method lets the robot as red team cross the bridge
	 * @throws OdometerExceptions 
	 */
	public void CrossBridgeAsRT() throws OdometerExceptions{
		
	}
	
	/**
	 * This method moves the robot as red team to starting position of the search zone
	 * @throws OdometerExceptions 
	 */
	public void NavToSearchZoneAsRT() throws OdometerExceptions{
		
	}
	/**
	 * This method navigates the robot as red team to the entrance of the tunnel as green team
	 * @throws OdometerExceptions 
	 */
	public void NavtoTunnelAsRT() throws OdometerExceptions {

	}
	/**
	 * This method lets the robot as red team cross tunnel
	 * @throws OdometerExceptions 
	 */
	public void CrossTunnelAsRT() throws OdometerExceptions {
		
	}
	/**
	 * This method navigates the robot as red team to the starting point
	 * @throws OdometerExceptions 
	 */
	public void NavtoSCAsRT() throws OdometerExceptions {
	
	}
}