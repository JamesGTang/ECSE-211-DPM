package ca.mcgill.ecse211.data;

public class GameData
{
  //LocalizationData Attributes
  public static int RedTeam; // RedTeam (i=1,20) – Team starting out from red zone
  public static int GreenTeam; // GreenTeam (i=1,20) – Team starting out from green zone
  public static int RedCorner; // RedCorner (i=1,4) – Starting corner for red team
  public static int GreenCorner; //GreenCorner (i=1,4) – Starting corner for green team

  public static int OG; // OG (i=1,5) – color of green opponent flag
  public static int OR; // OR (i=1,5) – color of red opponent flag
  public static int Red_LL_x; // Red_LL (x,y) – lower left hand corner of Red Zone
  public static int Red_LL_y; // Red_LL (x,y) – lower left hand corner of Red Zone
  
  public static int Red_UR_x; // Red_UR (x,y) – upper right hand corner of Red Zone
  public static int Red_UR_y; // Red_UR (x,y) – upper right hand corner of Red Zone
  public static int Green_LL_x; // Green_LL (x,y) – lower left hand corner of Green Zone
  public static int Green_LL_y; // Green_LL (x,y) – lower left hand corner of Green Zone
  
  public static int Green_UR_x; // Green_UR (x,y) – upper right hand corner of Green Zone
  public static int Green_UR_y; // Green_UR (x,y) – upper right hand corner of Green Zone
  public static int TN_LL_x; // TN_LL (x,y) – lower left hand corner of the tunnel footprint
  public static int TN_LL_y; // TN_LL (x,y) – lower left hand corner of the tunnel footprint
  
  public static int TN_UR_x; // TN_UR (x,y) – upper right hand corner of the tunnel footprint
  public static int TN_UR_y; // TN_UR (x,y) – upper right hand corner of the tunnel footprint
  public static int BR_LL_x; // BR_LL (x,y) – lower left hand corner of the bridge footprint
  public static int BR_LL_y; // BR_LL (x,y) – lower left hand corner of the bridge footprint
  
  public static int BR_UR_x; // BR_UR (x,y) – upper right hand corner of the bridge footprint
  public static int BR_UR_y; // BR_UR (x,y) – upper right hand corner of the bridge footprint
  public static int SR_LL_x; // SR_LL (x,y) – lower left hand corner of search region in red player zone
  public static int SR_LL_y; // SR_LL (x,y) – lower left hand corner of search region in red player zone
  
  public static int SR_UR_x; // SR_UR (x,y) – upper right hand corner of search region in red player zone
  public static int SR_UR_y; // SR_UR (x,y) – upper right hand corner of search region in red player zone
  public static int SG_LL_x; // SG_LL (x,y) – lower left hand corner of search region in green player zone
  public static int SG_LL_y; // SG_LL (x,y) – lower left hand corner of search region in green player zone
  
  public static int SG_UR_x; // SG_UR (x,y) – upper right hand corner of search region in green player zone
  public static int SG_UR_y; // SG_UR (x,y) – upper right hand corner of search region in green player zone
  
  /**
   * A method to print out all vairables in the class
   */
  public static void printString() {
	  System.out.println("RedTeam: "+RedTeam);
	  System.out.println("GreenTeam: "+GreenTeam);
	  System.out.println("RedCorner: "+RedCorner);
	  System.out.println("GreenCorner: "+GreenCorner);
	  
	  System.out.println("OG: "+OG);
	  System.out.println("OR"+OR);
	  System.out.println("Red_LL_x: "+Red_LL_x);
	  System.out.println("Red_LL_y: "+Red_LL_y);
	  
	  System.out.println("Red_UR_x: "+Red_UR_x);
	  System.out.println("Red_UR_y: "+Red_UR_y);	  
	  System.out.println("Green_LL_x: "+Green_LL_x);
	  System.out.println("Green_LL_y: "+Green_LL_y);
	  
	  System.out.println("Green_UR_x: "+Green_UR_x);
	  System.out.println("Green_UR_y: "+Green_UR_y);
	  System.out.println("TN_LL_x: "+TN_LL_x);
	  System.out.println("TN_LL_y: "+TN_LL_y);
	  
	  System.out.println("TN_UR_x: "+TN_UR_x);
	  System.out.println("TN_UR_y: "+TN_UR_y);
	  System.out.println("BR_LL_x: "+BR_LL_x);
	  System.out.println("BR_LL_y: "+BR_LL_y);
	  
	  System.out.println("BR_UR_x: "+BR_UR_x);
	  System.out.println("BR_UR_y: "+BR_UR_y);
	  System.out.println("SR_LL_x: "+SR_LL_x);
	  System.out.println("SR_LL_y: "+SR_LL_y);
	  
	  System.out.println("SR_UR_x: "+SR_UR_x);
	  System.out.println("SR_UR_y: "+SR_UR_y);
	  System.out.println("SG_LL_x: "+SG_LL_x);
	  System.out.println("SG_LL_y: "+SG_LL_y);
	  
	  System.out.println("SG_UR_x;: "+SG_UR_x);
	  System.out.println("SG_UR_y: "+SG_UR_y);
  }
}