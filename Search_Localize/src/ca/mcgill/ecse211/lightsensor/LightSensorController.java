package ca.mcgill.ecse211.lightsensor;

public interface LightSensorController {
	public void processLightData(int tb);

	public int readLightData();
}
