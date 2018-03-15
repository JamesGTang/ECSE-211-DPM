package ca.mcgill.ecse211.data;

import lejos.hardware.Button;
import java.util.Map;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.data.GameData;

public class Wifi {
	// ToDo: this must be changed before demo
	private static final String SERVER_IP = "192.168.2.3";
	private static final int TEAM_NUMBER = 1;
	
	// LocalizationData Attributes only used for testing and in case the wifi connection returns null object failed
	public static int RedTeam = 1; // RedTeam (i=1,20) – Team starting out from red zone
	public static int GreenTeam = 3; // GreenTeam (i=1,20) – Team starting out from green zone
	public static int RedCorner = 1; // RedCorner (i=1,4) – Starting corner for red team
	public static int GreenCorner = 3; // GreenCorner (i=1,4) – Starting corner for green team

	public static int OG = 3; // OG (i=1,5) – color of green opponent flag
	public static int OR = 4; // OR (i=1,5) – color of red opponent flag
	public static int Red_LL_x = 0; // Red_LL (x,y) – lower left hand corner of Red Zone
	public static int Red_LL_y = 7; // Red_LL (x,y) – lower left hand corner of Red Zone

	public static int Red_UR_x = 8; // Red_UR (x,y) – upper right hand corner of Red Zone
	public static int Red_UR_y = 12; // Red_UR (x,y) – upper right hand corner of Red Zone
	public static int Green_LL_x = 4; // Green_LL (x,y) – lower left hand corner of Green Zone
	public static int Green_LL_y = 0; // Green_LL (x,y) – lower left hand corner of Green Zone

	public static int Green_UR_x = 12; // Green_UR (x,y) – upper right hand corner of Green Zone
	public static int Green_UR_y = 5; // Green_UR (x,y) – upper right hand corner of Green Zone
	public static int TN_LL_x = 3; // TN_LL (x,y) – lower left hand corner of the tunnel footprint
	public static int TN_LL_y = 5; // TN_LL (x,y) – lower left hand corner of the tunnel footprint

	public static int TN_UR_x = 4; // TN_UR (x,y) – upper right hand corner of the tunnel footprint
	public static int TN_UR_y = 7; // TN_UR (x,y) – upper right hand corner of the tunnel footprint
	public static int BR_LL_x = 7; // BR_LL (x,y) – lower left hand corner of the bridge footprint
	public static int BR_LL_y = 5; // BR_LL (x,y) – lower left hand corner of the bridge footprint

	public static int BR_UR_x = 8; // BR_UR (x,y) – upper right hand corner of the bridge footprint
	public static int BR_UR_y = 7; // BR_UR (x,y) – upper right hand corner of the bridge footprint
	public static int SR_LL_x = 1; // SR_LL (x,y) – lower left hand corner of search region in red player zone
	public static int SR_LL_y = 9; // SR_LL (x,y) – lower left hand corner of search region in red player zone

	public static int SR_UR_x = 2; // SR_UR (x,y) – upper right hand corner of search region in red player zone
	public static int SR_UR_y = 11; // SR_UR (x,y) – upper right hand corner of search region in red player zone
	public static int SG_LL_x = 9; // SG_LL (x,y) – lower left hand corner of search region in green player zone
	public static int SG_LL_y = 1; // SG_LL (x,y) – lower left hand corner of search region in green player zone

	public static int SG_UR_x = 11; // SG_UR (x,y) – upper right hand corner of search region in green player zone
	public static int SG_UR_y = 2; // SG_UR (x,y) – upper right hand corner of search region in green player zone
	// Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

	@SuppressWarnings("rawtypes")
	public static void getParameter() {
		System.out.println("Running..");
		// Initialize WifiConnection class
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);
		// Connect to server and get the data, catching any errors that might occur
		try {
			/*
			 * getData() will connect to the server and wait until the user/TA presses the
			 * "Start" button in the GUI on their laptop with the data filled in. Once it's
			 * waiting, you can kill it by pressing the upper left hand corner button
			 * (back/escape) on the EV3. getData() will throw exceptions if it can't connect
			 * to the server (e.g. wrong IP address, server not running on laptop, not
			 * connected to WiFi router, etc.). It will also throw an exception if it
			 * connects but receives corrupted data or a message from the server saying
			 * something went wrong. For example, if TEAM_NUMBER is set to 1 above but the
			 * server expects teams 17 and 5, this robot will receive a message saying an
			 * invalid team number was specified and getData() will throw an exception
			 * letting you know.
			 */
			Map data = conn.getData();
			if(data==null) {
				// if getting data from server failed, will use local data ( this is useful for testing )
				System.out.println("Unable to fetch data from server, using local data");
				GameData.RedTeam = RedTeam;
				GameData.GreenTeam = GreenTeam;
				GameData.RedCorner = RedCorner;
				GameData.GreenTeam = GreenCorner;

				GameData.OG = OG;
				GameData.OR = OR;
				GameData.Red_LL_x = Red_LL_x;
				GameData.Red_LL_y = Red_LL_y;

				GameData.Red_UR_x = Red_UR_x;
				GameData.Red_UR_y = Red_UR_y;
				GameData.Green_LL_x = Green_LL_x;
				GameData.Green_LL_y = Green_LL_y;

				GameData.Green_UR_x = Green_UR_x;
				GameData.Green_UR_y = Green_UR_y;
				GameData.TN_LL_y =TN_LL_x;
				GameData.TN_LL_y = TN_LL_y;

				GameData.TN_UR_x = TN_UR_x;
				GameData.TN_UR_y = TN_UR_y;
				GameData.BR_LL_x = BR_LL_x;
				GameData.BR_LL_y = BR_LL_y;

				GameData.BR_UR_x = BR_UR_x;
				GameData.BR_UR_y = BR_UR_y;
				GameData.SR_LL_x = SR_LL_x;
				GameData.SR_LL_y = SR_LL_y;

				GameData.SR_UR_x = SR_UR_x;
				GameData.SR_UR_y = SR_UR_y;
				GameData.SG_LL_x = SG_LL_x;
				GameData.SG_LL_y = SG_LL_y;

				GameData.SG_UR_x = SG_UR_x;
				GameData.SG_UR_y = SG_UR_y;
				
			}
			System.out.println("Raw data from server: " + data);

			// Example 2 : Print out specific values
			GameData.RedTeam = ((Long) data.get("RedTeam")).intValue();
			GameData.GreenTeam = ((Long) data.get("GreenTeam")).intValue();
			GameData.RedCorner = ((Long) data.get("RedCorner")).intValue();
			GameData.GreenTeam = ((Long) data.get("GreenCorner")).intValue();

			GameData.OG = ((Long) data.get("OG")).intValue();
			GameData.OR = ((Long) data.get("OR")).intValue();
			GameData.Red_LL_x = ((Long) data.get("Red_LL_x")).intValue();
			GameData.Red_LL_y = ((Long) data.get("Red_LL_y")).intValue();

			GameData.Red_UR_x = ((Long) data.get("Red_UR_x")).intValue();
			GameData.Red_UR_y = ((Long) data.get("Red_UR_y")).intValue();
			GameData.Green_LL_x = ((Long) data.get("Green_LL_x")).intValue();
			GameData.Green_LL_y = ((Long) data.get("Green_LL_y")).intValue();

			GameData.Green_UR_x = ((Long) data.get("Green_UR_x")).intValue();
			GameData.Green_UR_y = ((Long) data.get("Green_UR_y")).intValue();
			GameData.TN_LL_y = ((Long) data.get("TN_LL_x")).intValue();
			GameData.TN_LL_y = ((Long) data.get("TN_LL_y")).intValue();

			GameData.TN_UR_x = ((Long) data.get("TN_UR_x")).intValue();
			GameData.TN_UR_y = ((Long) data.get("TN_UR_y")).intValue();
			GameData.BR_LL_x = ((Long) data.get("BR_LL_x")).intValue();
			GameData.BR_LL_y = ((Long) data.get("BR_LL_y")).intValue();

			GameData.BR_UR_x = ((Long) data.get("BR_UR_x")).intValue();
			GameData.BR_UR_y = ((Long) data.get("BR_UR_y")).intValue();
			GameData.SR_LL_x = ((Long) data.get("SR_LL_x")).intValue();
			GameData.SR_LL_y = ((Long) data.get("SR_LL_y")).intValue();

			GameData.SR_UR_x = ((Long) data.get("SR_UR_x")).intValue();
			GameData.SR_UR_y = ((Long) data.get("SR_UR_y")).intValue();
			GameData.SG_LL_x = ((Long) data.get("SG_LL_x")).intValue();
			GameData.SG_LL_y = ((Long) data.get("SG_LL_y")).intValue();

			GameData.SG_UR_x = ((Long) data.get("SG_UR_x")).intValue();
			GameData.SG_UR_y = ((Long) data.get("SG_UR_y")).intValue();
			System.out.println("Stored data to GameData class");
			GameData.printString();

		} catch (Exception e) {
			System.err.println("Error: " + e.getMessage());
		}

		// Wait until user decides to end program
		Button.waitForAnyPress();
	}
}
