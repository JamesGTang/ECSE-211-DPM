package ca.mcgill.ecse211.util;

public class Util {
	
	/**
	 * Converts travel distance to number of degrees of rotation of robot's wheel.
	 * 
	 * @param radius - radius of robot's wheel (cm)
	 * @param distance - distance needed to be traveled (cm)
	 * @return number of degrees of wheel rotation (degrees)
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	/**
	 *  Calculate robot rotation to number of degrees each wheel must turn
	 * 
	 * @param radius - robot wheel radius (cm)
	 * @param width - robot axle width (cm)
	 * @param angle - amount of robot rotation (radians)
	 * @return number of degrees of wheel rotation (degrees)
	 */
	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
