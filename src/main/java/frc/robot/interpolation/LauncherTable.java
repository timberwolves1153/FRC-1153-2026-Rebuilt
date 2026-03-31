package frc.robot.interpolation;

import java.util.function.DoubleSupplier;

public class LauncherTable {
  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> hoodMap =
      new InterpolatingTreeMap<>();

  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> flywheelShootingMap =
      new InterpolatingTreeMap<>();

  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> flywheelPassingMap =
      new InterpolatingTreeMap<>();

  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> flightTimeMap =
      new InterpolatingTreeMap<>();

  static {

    /* Hood
    Key = distance from hub in meters
    Value = hood angle setpoint */

    hoodMap.put(new InterpolatingDouble(1.5), new InterpolatingDouble(-0.2));
    hoodMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(-0.45));
    hoodMap.put(new InterpolatingDouble(2.5), new InterpolatingDouble(-0.75));
    hoodMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(-0.8));
    hoodMap.put(new InterpolatingDouble(3.5), new InterpolatingDouble(-0.95));
    hoodMap.put(new InterpolatingDouble(4.0), new InterpolatingDouble(-1.1));
    hoodMap.put(new InterpolatingDouble(4.5), new InterpolatingDouble(-1.35));
    hoodMap.put(new InterpolatingDouble(5.0), new InterpolatingDouble(-1.5));

    /* Flywheel Hub Shot
    Key = distance from hub in meters
    Value = flywheel velocity setpoint */

    flywheelShootingMap.put(new InterpolatingDouble(1.5), new InterpolatingDouble(-27.0));
    flywheelShootingMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(-28.5));
    flywheelShootingMap.put(new InterpolatingDouble(2.5), new InterpolatingDouble(-29.5));
    flywheelShootingMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(-32.0));
    flywheelShootingMap.put(new InterpolatingDouble(3.5), new InterpolatingDouble(-34.0));
    flywheelShootingMap.put(new InterpolatingDouble(4.0), new InterpolatingDouble(-36.0));
    flywheelShootingMap.put(new InterpolatingDouble(4.5), new InterpolatingDouble(-36.75));
    flywheelShootingMap.put(new InterpolatingDouble(5.0), new InterpolatingDouble(-37.75));

    /*Flywheel Passing */
    flywheelPassingMap.put(new InterpolatingDouble(4.457), new InterpolatingDouble(-35.0));
    flywheelPassingMap.put(new InterpolatingDouble(8.71), new InterpolatingDouble(-50.0));
    flywheelPassingMap.put(new InterpolatingDouble(9.71), new InterpolatingDouble(-55.0));
    flywheelPassingMap.put(new InterpolatingDouble(11.131), new InterpolatingDouble(-60.0));
    flywheelPassingMap.put(new InterpolatingDouble(13.184), new InterpolatingDouble(-70.0));
    flywheelPassingMap.put(new InterpolatingDouble(15.0), new InterpolatingDouble(-72.0));

    /* Flight Time
    Key = distance from hub in meters
    Value = flight time seconds */
    flightTimeMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(1.0));
    flightTimeMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(1.02));
    flightTimeMap.put(new InterpolatingDouble(4.0), new InterpolatingDouble(1.15));
    flightTimeMap.put(new InterpolatingDouble(5.0), new InterpolatingDouble(1.2));
  }

  public DoubleSupplier getHoodValueSupplier(double distance) {
    return () -> hoodMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }

  public double getHoodValue(double distance) {
    return hoodMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }

  public DoubleSupplier getFlywheelShootingValueSupplier(double distance) {
    return () -> flywheelShootingMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }

  public double getFlywheelShootingValue(double distance) {
    return flywheelShootingMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }

  public DoubleSupplier getFlywheelPassingValueSupplier(double distance) {
    return () -> flywheelPassingMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }

  public double getFlywheelPassingValue(double distance) {
    return flywheelPassingMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }

  public DoubleSupplier getTimeofFlightSupplier(double distance) {
    return () -> flightTimeMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }

  public double getTimeofFlightSeconds(double distance) {
    return flightTimeMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }
}
