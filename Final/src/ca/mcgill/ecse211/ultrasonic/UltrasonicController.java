package ca.mcgill.ecse211.ultrasonic;

public interface UltrasonicController {

  public void processUSData(double distance);

  public double readUSDistance();
}
