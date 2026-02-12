package frc.robot.interpolation;

import java.util.function.DoubleSupplier;

public class HoodTable {
  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> hoodMap =
      new InterpolatingTreeMap<>();

  static {
    /* Key = distance from hub in meters
     * Value = hood angle setpoint */
  } // TODO: populate map

  public DoubleSupplier getSetpointSupplier(double distance) {
    return () -> hoodMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }

  public double getSetpoint(double distance) {
    return hoodMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }
}
