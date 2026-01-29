package frc.robot.subsystems.launcher.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public double leadCurrent = 0.0;
    public double leadVelocity = 0.0;
    public double leadAppliedVoltage = 0.0;
    public double leadTemp = 0.0;

    public double followerCurrent = 0.0;
    public double followerVelocity = 0.0;
    public double followerAppliedVoltage = 0.0;
    public double followerTemp = 0.0;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setVoltageLeader(double volts) {}

  public default void setVoltageFollower(double volts) {}

  public default void setVelocityLeader(double volts) {}

  public default void setVelocityFollower(double volts) {}

  public default void stopFlywheel() {}
}
