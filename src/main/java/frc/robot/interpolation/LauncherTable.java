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

    // 3V Feeder
    // hoodMap.put(new InterpolatingDouble(1.676), new InterpolatingDouble(-0.01));
    // hoodMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(-0.08));
    // hoodMap.put(new InterpolatingDouble(2.35), new InterpolatingDouble(-0.5));
    // hoodMap.put(new InterpolatingDouble(3.1), new InterpolatingDouble(-1.25));
    // hoodMap.put(new InterpolatingDouble(3.786), new InterpolatingDouble(-1.35));
    // hoodMap.put(new InterpolatingDouble(4.0), new InterpolatingDouble(-1.5));
    // hoodMap.put(new InterpolatingDouble(5.135), new InterpolatingDouble(-1.85));

    // 9V Feeder
    hoodMap.put(new InterpolatingDouble(1.5), new InterpolatingDouble(-0.2));
    hoodMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(-0.4));
    hoodMap.put(new InterpolatingDouble(2.5), new InterpolatingDouble(-0.65));
    hoodMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(-0.8));
    hoodMap.put(new InterpolatingDouble(3.5), new InterpolatingDouble(-0.95));
    hoodMap.put(new InterpolatingDouble(4.0), new InterpolatingDouble(-1.1));
    hoodMap.put(new InterpolatingDouble(4.5), new InterpolatingDouble(-1.35));
    hoodMap.put(new InterpolatingDouble(5.0), new InterpolatingDouble(-1.5));

    /* Flywheel Hub Shot
    Key = distance from hub in meters
    Value = flywheel velocity setpoint */

    // 3V Feeder
    // flywheelShootingMap.put(new InterpolatingDouble(1.676), new InterpolatingDouble(-28.5));
    // flywheelShootingMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(-32.5));
    // flywheelShootingMap.put(new InterpolatingDouble(2.35), new InterpolatingDouble(-35.0));
    // flywheelShootingMap.put(new InterpolatingDouble(3.1), new InterpolatingDouble(-38.5));
    // flywheelShootingMap.put(new InterpolatingDouble(3.786), new InterpolatingDouble(-40.0));
    // flywheelShootingMap.put(new InterpolatingDouble(4.0), new InterpolatingDouble(-43.0));
    // flywheelShootingMap.put(new InterpolatingDouble(5.135), new InterpolatingDouble(-47.0));

    // 9V Feeder --> 0.1905 accounts for turret offset from robot center
    flywheelShootingMap.put(new InterpolatingDouble(1.5 - 0.1905), new InterpolatingDouble(-28.0));
    flywheelShootingMap.put(new InterpolatingDouble(2.0 - 0.1905), new InterpolatingDouble(-30.0));
    flywheelShootingMap.put(new InterpolatingDouble(2.5 - 0.1905), new InterpolatingDouble(-30.5));
    flywheelShootingMap.put(new InterpolatingDouble(3.0 - 0.1905), new InterpolatingDouble(-32.0));
    flywheelShootingMap.put(new InterpolatingDouble(3.5 - 0.1905), new InterpolatingDouble(-34.0));
    flywheelShootingMap.put(new InterpolatingDouble(4.0 - 0.1905), new InterpolatingDouble(-36.0));
    flywheelShootingMap.put(new InterpolatingDouble(4.5 - 0.1905), new InterpolatingDouble(-36.75));
    flywheelShootingMap.put(new InterpolatingDouble(5.0 - 0.1905), new InterpolatingDouble(-37.75));

    /*Flywheel Passing */
    flywheelPassingMap.put(new InterpolatingDouble(4.457), new InterpolatingDouble(-35.0));
    flywheelPassingMap.put(new InterpolatingDouble(8.71), new InterpolatingDouble(-50.0));
    flywheelPassingMap.put(new InterpolatingDouble(9.71), new InterpolatingDouble(-55.0));
    flywheelPassingMap.put(new InterpolatingDouble(11.131), new InterpolatingDouble(-60.0));
    flywheelPassingMap.put(new InterpolatingDouble(13.184), new InterpolatingDouble(-70.0));
    flywheelPassingMap.put(new InterpolatingDouble(15.0), new InterpolatingDouble(-75.0));

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
