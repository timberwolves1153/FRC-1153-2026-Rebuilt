package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double spinAppliedVolts = 0.0;
    public double spinCurrentAmps = 0.0;

    public double feedAppliedVolts = 0.0;
    public double feedCurrentAmps = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  /** Runs the Spin motor to serialize game pieces. */
  public default void runSpin(double volts) {}

  /** Runs the Feed motor to push piece into shooter. */
  public default void runFeeder(double volts) {}

  /** Stops both motors. */
  public default void stopAll() {
    stopSpin();
    stopFeeder();
  }

  public default void stopSpin() {}

  public default void stopFeeder() {}
}
