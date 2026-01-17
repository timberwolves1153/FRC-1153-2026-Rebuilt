package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {
  @AutoLog
  public static class LauncherIOInputs {
    public double leadCurrent = 0.0;
    public double leadAppliedVoltage = 0.0;
    public double leadTemp = 0.0;

    public double followerCurrent = 0.0;
    public double followerAppliedVoltage = 0.0;
    public double followerTemp = 0.0;
  }

  public default void updateInputs(LauncherIOInputs inputs) {}

  public default void runVolts(double volts) {}

  // public default void runDutyCycle(double output) {}
}
