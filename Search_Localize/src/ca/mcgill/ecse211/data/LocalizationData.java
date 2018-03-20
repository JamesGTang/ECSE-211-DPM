package ca.mcgill.ecse211.data;

public class LocalizationData
{
  //LocalizationData Attributes
  private static int LLx;
  private static int LLy;
  private static int URx;
  private static int URy;
  private static int TB;
  private static int SC;
  
  // empty constructor
  public LocalizationData() {
	  
  }
  // constructor overload
  public LocalizationData(int aLLx, int aLLy, int aURx, int aURy, int aTB, int aSC)
  {
    LLx = aLLx;
    LLy = aLLy;
    URx = aURx;
    URy = aURy;
    TB = aTB;
    SC = aSC;
  }
  
  public static boolean setAll(int coordinates[]) {
	  	boolean wasSet = false;
	    LLx=coordinates[0];
	    LLy=coordinates[1];
	    URx=coordinates[2];
	    URy=coordinates[3];
	    TB=coordinates[4];
	    SC=coordinates[5];
	    wasSet = true;
	    return wasSet;
  }
  public static boolean setLLx(int aLLx)
  {
    boolean wasSet = false;
    LLx = aLLx;
    wasSet = true;
    return wasSet;
  }

  public static boolean setLLy(int aLLy)
  {
    boolean wasSet = false;
    LLy = aLLy;
    wasSet = true;
    return wasSet;
  }

  public static boolean setURx(int aURx)
  {
    boolean wasSet = false;
    URx = aURx;
    wasSet = true;
    return wasSet;
  }

  public static boolean setURy(int aURy)
  {
    boolean wasSet = false;
    URy = aURy;
    wasSet = true;
    return wasSet;
  }

  public static boolean setTB(int aTB)
  {
    boolean wasSet = false;
    TB = aTB;
    wasSet = true;
    return wasSet;
  }

  public static boolean setSC(int aSC)
  {
    boolean wasSet = false;
    SC = aSC;
    wasSet = true;
    return wasSet;
  }

  public static int getLLx()
  {
    return LLx;
  }

  public static int getLLy()
  {
    return LLy;
  }

  public static int getURx()
  {
    return URx;
  }

  public static int getURy()
  {
    return URy;
  }

  public static int getTB()
  {
    return TB;
  }

  public static int getSC()
  {
    return SC;
  }

  public static void delete()
  {}


  public static String print()
  {
    return "["+
            "LLx" + ":" + getLLx()+ "," +
            "LLy" + ":" + getLLy()+ "," +
            "URx" + ":" + getURx()+ "," +
            "URy" + ":" + getURy()+ "," +
            "TB" + ":" + getTB()+ "," +
            "SC" + ":" + getSC()+ "]";
  }
}