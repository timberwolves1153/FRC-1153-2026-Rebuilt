package frc.robot.interpolation;

import java.util.function.DoubleSupplier;

public class LauncherTable {
  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> hoodMap =
      new InterpolatingTreeMap<>();

  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> flywheelMap =
      new InterpolatingTreeMap<>();

  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> flightTimeMap =
      new InterpolatingTreeMap<>();

  static {
    /* Hood
    Key = distance from hub in meters
    Value = hood angle setpoint */

    /* Flywheel
    Key = distance from hub in meters
    Value = flywheel velocity setpoint */

    /* Flight Time
    Key = distance from hub in meters
    Value = flight time seconds */

  }

  public DoubleSupplier getValueSupplier(double distance) {
    return () -> hoodMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }

  public double getValue(double distance) {
    return hoodMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }
}
