package ca.mcgill.ecse211.data;

/**
 * This class holds the data structure of the color block
 * @author jamestang
 *
 */
public class ColorBlock {
	static final int width = 10;
	static final int length = 10;
	final int height = 10;
	// ColorBlock Attributes
	private double x;
	private double y;
	private int color;
	
	/**
	 * 
	 * @param aX the x position
	 * @param aY the y position
	 * @param aColor the color
	 */
	public ColorBlock(double aX, double aY, int aColor) {
		x = aX;
		y = aY;
		color = aColor;
	}
	/**
	 * 
	 * @param aX the x position
	 * @return true if set
	 */
	public boolean setX(double aX) {
		boolean wasSet = false;
		x = aX;
		wasSet = true;
		return wasSet;
	}
	/**
	 * 
	 * @param aY the y postion
	 * @return true if set
	 */
	public boolean setY(double aY) {
		boolean wasSet = false;
		y = aY;
		wasSet = true;
		return wasSet;
	}
	/**
	 * 
	 * @param aColor the color
	 * @return true if set
	 */
	public boolean setColor(int aColor) {
		boolean wasSet = false;
		color = aColor;
		wasSet = true;
		return wasSet;
	}
	/**
	 * 
	 * @return the x position
	 */
	public double getX() {
		return x;
	}
	/**
	 * 
	 * @return the y postion
	 */
	public double getY() {
		return y;
	}
	/**
	 * 
	 * @return the color
	 */
	public int getColor() {
		return color;
	}
	/**
	 * Convert the data to a string
	 */

	public String toString() {
		return super.toString() + "[" + "x" + ":" + getX() + "," + "y" + ":" + getY() + "," + "color" + ":" + getColor()
				+ "]";
	}
}