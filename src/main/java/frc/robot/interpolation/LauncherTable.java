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

    // 3V Feeder
    // hoodMap.put(new InterpolatingDouble(1.676), new InterpolatingDouble(-0.01));
    // hoodMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(-0.08));
    // hoodMap.put(new InterpolatingDouble(2.35), new InterpolatingDouble(-0.5));
    // hoodMap.put(new InterpolatingDouble(3.1), new InterpolatingDouble(-1.25));
    // hoodMap.put(new InterpolatingDouble(3.786), new InterpolatingDouble(-1.35));
    // hoodMap.put(new InterpolatingDouble(4.0), new InterpolatingDouble(-1.5));
    // hoodMap.put(new InterpolatingDouble(5.135), new InterpolatingDouble(-1.85));

    // 6V Feeder
    hoodMap.put(new InterpolatingDouble(1.676), new InterpolatingDouble(-0.01));
    hoodMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(-0.08));
    hoodMap.put(new InterpolatingDouble(2.35), new InterpolatingDouble(-0.5));
    hoodMap.put(new InterpolatingDouble(3.1), new InterpolatingDouble(-1.25));
    hoodMap.put(new InterpolatingDouble(3.786), new InterpolatingDouble(-1.35));
    hoodMap.put(new InterpolatingDouble(4.0), new InterpolatingDouble(-1.5));
    hoodMap.put(new InterpolatingDouble(5.135), new InterpolatingDouble(-1.85));

    /* Flywheel
    Key = distance from hub in meters
    Value = flywheel velocity setpoint */

    // 3V Feeder
    // flywheelMap.put(new InterpolatingDouble(1.676), new InterpolatingDouble(-28.5));
    // flywheelMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(-32.5));
    // flywheelMap.put(new InterpolatingDouble(2.35), new InterpolatingDouble(-35.0));
    // flywheelMap.put(new InterpolatingDouble(3.1), new InterpolatingDouble(-38.5));
    // flywheelMap.put(new InterpolatingDouble(3.786), new InterpolatingDouble(-40.0));
    // flywheelMap.put(new InterpolatingDouble(4.0), new InterpolatingDouble(-43.0));
    // flywheelMap.put(new InterpolatingDouble(5.135), new InterpolatingDouble(-47.0));

    // 9V Feeder
    flywheelMap.put(new InterpolatingDouble(1.676), new InterpolatingDouble(-30.0));
    flywheelMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(-32.5));
    flywheelMap.put(new InterpolatingDouble(2.35), new InterpolatingDouble(-35.0));
    flywheelMap.put(new InterpolatingDouble(3.1), new InterpolatingDouble(-38.5));
    flywheelMap.put(new InterpolatingDouble(3.786), new InterpolatingDouble(-41.0));
    flywheelMap.put(new InterpolatingDouble(4.0), new InterpolatingDouble(-43.0));
    flywheelMap.put(new InterpolatingDouble(5.1), new InterpolatingDouble(-47.25));

    /* Flight Time
    Key = distance from hub in meters
    Value = flight time seconds */

  }

  public DoubleSupplier getHoodValueSupplier(double distance) {
    return () -> hoodMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }

  public double getHoodValue(double distance) {
    return hoodMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }

  public DoubleSupplier getFlywheelValueSupplier(double distance) {
    return () -> flywheelMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }

  public double getFlywheelValue(double distance) {
    return flywheelMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }
}
