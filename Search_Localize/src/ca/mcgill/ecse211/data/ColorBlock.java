package ca.mcgill.ecse211.data;

public class ColorBlock {
	static final int width = 10;
	static final int length = 10;
	final int height = 10;
	// ColorBlock Attributes
	private double x;
	private double y;
	private int color;

	public ColorBlock(double aX, double aY, int aColor) {
		x = aX;
		y = aY;
		color = aColor;
	}
	
	public boolean setX(double aX) {
		boolean wasSet = false;
		x = aX;
		wasSet = true;
		return wasSet;
	}

	public boolean setY(double aY) {
		boolean wasSet = false;
		y = aY;
		wasSet = true;
		return wasSet;
	}

	public boolean setColor(int aColor) {
		boolean wasSet = false;
		color = aColor;
		wasSet = true;
		return wasSet;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public int getColor() {
		return color;
	}

	public void delete() {
	}

	public String toString() {
		return super.toString() + "[" + "x" + ":" + getX() + "," + "y" + ":" + getY() + "," + "color" + ":" + getColor()
				+ "]";
	}
}