package ca.mcgill.ecse211.data;

public class RGB {
	public int targetValue[][]=new int[5][3];
	
	public RGB() {
		// default contructor
		
		// red data=1
		targetValue[1][0] = 150;
		targetValue[1][1] = 15;
		targetValue[1][2] = 0;
		// yellow data=2
		targetValue[2][0] = 250;
		targetValue[2][1] = 140;
		targetValue[2][2] = 10;
		//blue data=3
		targetValue[3][0] = 20;
		targetValue[3][1] = 35;
		targetValue[3][2] = 30;
		//white data=4
		targetValue[4][0] = 230;
		targetValue[4][1] = 180;
		targetValue[4][2] = 90;
	}
}
