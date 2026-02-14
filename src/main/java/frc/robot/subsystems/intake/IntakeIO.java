// IntakeIO

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeInputs {
    public double deployAppliedVolts = 0.0; // input
    public double deployCurrentAmps = 0.0;

    public double collectAppliedVolts = 0.0; // input
    public double collectCurrentAmps = 0.0;
  }

  public default void setPositionIntake(double rotations) {}

  public default void updateInputs(IntakeInputs inputs) {}

  public default void setDeployVoltage(double volts) {}

  public default void resetDeployEncoder() {}

  public default void setCollectVoltage(double volts) {}
}
