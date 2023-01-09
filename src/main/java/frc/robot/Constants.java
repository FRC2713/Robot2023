package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
  // in case you need a zero :)
  public static final int ZERO = 0;

  public static class Elevator {
    public static final double CARRIAGE_MASS_KG = Units.lbsToKilograms(20.0);
    public static final double ELEVATOR_DRUM_RADIUS_METERS = Units.inchesToMeters(1.0);
    public static final double ELEVATOR_MIN_HEIGHT_METERS = Units.inchesToMeters(0.0);
    public static final double ELEVATOR_MAX_HEIGHT_METERS = Units.inchesToMeters(40.0);
  }

}
