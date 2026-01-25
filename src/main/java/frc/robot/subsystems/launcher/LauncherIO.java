package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {
  @AutoLog
  public static class LauncherIOInputs {
    /* Flywheel */

    public double leadCurrent = 0.0;
    public double leadAppliedVoltage = 0.0;
    public double leadTemp = 0.0;

    public double followerCurrent = 0.0;
    public double followerAppliedVoltage = 0.0;
    public double followerTemp = 0.0;

    /* Hood */

    public double hoodAppliedVoltage = 0.0;
    public double hoodCurrent = 0.0;
    public double hoodPosition = 0.0;
    public double hoodTemp = 0.0;

    /* Turret */

    public double turretAppliedVoltage = 0.0;
    public double turretCurrent = 0.0;
    public double turretPositionRad = 0.0;
    public double turretTemp = 0.0;
  }

  public default void updateInputs(LauncherIOInputs inputs) {}

  /* Flywheel */

  public default void setVoltageLeader(double volts) {}

  public default void setVoltageFollower(double volts) {}

  public default void stopLauncher() {}

  /* Hood */

  // public default void setPositionHood(double hoodDegrees) {}

  /* Turret */

  public default void setPositionTurretRad(double turretSetpointRad) {}

  public default void setTurretPosition(double rotations) {}

  public default void setVoltageTurret(double volts) {}

  public default void stopTurret() {}
}
