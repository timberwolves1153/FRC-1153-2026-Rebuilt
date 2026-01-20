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

  public default void updateInputs(IntakeInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setDeployVoltage(double volts) {}

  /** Stop in open loop. */
  public default void stopDeploy() {}

  public default void resetDeployEncoder() {}

  public default void setCollectVoltage(double volts) {}
  /** Stop in open loop. */
  public default void stopCollect() {}

  public default void resetCollectEncoder() {}
}
